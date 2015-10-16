#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/soc/qcom/smd.h>
#include <linux/suspend.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#define	MAX_SMD_TTY_PORTS	32
#define	CHANNEL_PROBE_TIMEOUT	60000

static struct tty_driver *smd_tty_driver;

struct msm_smd_port_info_s {
	char	*name;
	u32	flags;
	int	index;
	int	smd_tty_info_index;
};

#define	SMD_DROP_ONE_TX_BYTE	(1)
#define	SMD_TTY_ACL		(1 << 1)
#define	SMD_TTY_CMD		(1 << 2)

static struct msm_smd_port_info_s msm_smd_port_info[] = {
	{
		.name	= "DUMMY_SMD_TTY0",
		.flags	= 0,
		.index	= 0,
		.smd_tty_info_index = 0,
	}, {
		.name	= "DUMMY_SMD_TTY1",
		.flags	= 0,
		.index	= 1,
		.smd_tty_info_index = 1,
	}, {
		.name	= "APPS_RIVA_BT_ACL",
		.flags	= SMD_DROP_ONE_TX_BYTE | SMD_TTY_ACL,
		.index	= 2,
		.smd_tty_info_index = 3,
	}, {
		.name	= "APPS_RIVA_BT_CMD",
		.flags	= SMD_DROP_ONE_TX_BYTE | SMD_TTY_CMD,
		.index	= 3,
		.smd_tty_info_index = 3,
	},
};

struct msm_smd_port_info_s *match_smd_port_info(char *name)
{
	int i;
	struct msm_smd_port_info_s *pinfo = NULL;

	for (i = 0; i < ARRAY_SIZE(msm_smd_port_info); i++) {
		if (!strcmp(name, msm_smd_port_info[i].name)) {
			pinfo = &msm_smd_port_info[i];
			break;
		}
	}

	return pinfo;
}

struct msm_smd_channel_info {
        struct	qcom_smd_device *cmd_sdev;
        struct	qcom_smd_channel *channel;

        struct	completion ack;
        struct	mutex lock;

	u32	flags;
	int	smd_tty_info_index;
};

struct msm_smd_tty_info_s {
	/* For some special case (like BT), out/in channel are differnet.
	 * In most case, they should be same.
	 */
	struct	msm_smd_channel_info *cmd_info;
	struct	msm_smd_channel_info *acl_info;

        struct	tty_port port;
        struct	device *tty;

	struct	mutex open_lock;
	struct	completion probe_done;
	int	opened;
};

void dump_buf(unsigned char *buf, int len)
{
        int i;
        unsigned char *tmp = buf;

        printk("\n\t");

        for (i = 0; i < len; i++) {
                printk("0x%02x ", tmp[i]);
                if (((i + 1) % 16) == 0)
                        printk("\n\t");
        }
}

static struct msm_smd_tty_info_s msm_smd_tty_info[MAX_SMD_TTY_PORTS];

static int qcom_smd_tty_probe(struct qcom_smd_device *sdev)
{
        struct msm_smd_channel_info *ch_info;
	struct msm_smd_port_info_s *pinfo;
	int idx = 0;

        ch_info = devm_kzalloc(&sdev->dev, sizeof(*ch_info), GFP_KERNEL);
        if (!ch_info)
                return -ENOMEM;

        mutex_init(&ch_info->lock);
        init_completion(&ch_info->ack);
        ch_info->channel = sdev->channel;

	pinfo = match_smd_port_info(ch_info->channel->name);
	if (!pinfo) {
		pr_err("Can't match channel %s\n", ch_info->channel->name);
		kfree(ch_info);
		return -EINVAL;
	}

	ch_info->flags = pinfo->flags;
	idx = ch_info->smd_tty_info_index = pinfo->smd_tty_info_index;

	if (pinfo->flags & SMD_TTY_ACL) {
		msm_smd_tty_info[idx].acl_info = ch_info;
	} else if (pinfo->flags & SMD_TTY_CMD) {
		msm_smd_tty_info[idx].cmd_info = ch_info;
	} else {
		msm_smd_tty_info[idx].cmd_info =
		msm_smd_tty_info[idx].acl_info = ch_info;
	}

	/* We use all here to prevent the maltiple port open
	 * blocking.
	 */
	complete_all(&msm_smd_tty_info[pinfo->index].probe_done);

        dev_set_drvdata(&sdev->dev, ch_info);
	return 0;
}

static void qcom_smd_tty_remove(struct qcom_smd_device *sdev)
{
        struct msm_smd_channel_info *ch_info = dev_get_drvdata(&sdev->dev);

	kfree(ch_info);
	return;
}

static int qcom_smd_tty_callback(struct qcom_smd_device *sdev,
                                 const void *data,
                                 size_t count)
{
	unsigned char *ptr, *tmp;
	int buf_len;
        struct msm_smd_channel_info *ch_info;
	struct tty_struct *tty;

	ch_info = dev_get_drvdata(&sdev->dev);
	tty = tty_port_tty_get(
		&msm_smd_tty_info[ch_info->smd_tty_info_index].port);
	if (!tty) {
		pr_err("can't get tty for index: %d\n",
			ch_info->smd_tty_info_index);
		return 0;
	}

	if (ch_info->flags & SMD_DROP_ONE_TX_BYTE)
		buf_len = count + 1;
	else
		buf_len = count;

	if (buf_len > ch_info->channel->fifo_size)
		buf_len = ch_info->channel->fifo_size;

        buf_len = tty_prepare_flip_string(tty->port, &ptr, buf_len);
	if (buf_len <= 0) {
		pr_err("can't get flip buffer for tty index: %d\n",
			ch_info->smd_tty_info_index);
		tty_kref_put(tty);
		return 0;
	}

	tmp = ptr;
	if (ch_info->flags & SMD_TTY_ACL) {
		*ptr++ = HCI_ACLDATA_PKT;
	} else if (ch_info->flags & SMD_TTY_CMD) {
		*ptr++ = HCI_EVENT_PKT;
	}

	memcpy_fromio(ptr, data, count);

	tty_flip_buffer_push(tty->port);
	tty_kref_put(tty);

	return 0;
}

static const struct of_device_id qcom_smd_tty_of_match[] = {
	{ .compatible = "qcom,hci-smd-bt-cmd"},
	{ .compatible = "qcom,hci-smd-bt-acl"},
	{}
};
MODULE_DEVICE_TABLE(of, qcom_smd_tty_of_match);

static struct qcom_smd_driver qcom_smd_tty_driver = {
	.probe = qcom_smd_tty_probe,
	.remove = qcom_smd_tty_remove,
	.callback = qcom_smd_tty_callback,
	.driver  = {
		.name  = "qcom_smd_tty",
		.owner = THIS_MODULE,
		.of_match_table = qcom_smd_tty_of_match,
	},
};

static int smd_tty_open(struct tty_struct *tty, struct file *f)
{
	struct msm_smd_tty_info_s *info = msm_smd_tty_info + tty->index;

	return tty_port_open(&info->port, tty, f);
}

static void smd_tty_close(struct tty_struct *tty, struct file *f)
{
	struct msm_smd_tty_info_s *info = tty->driver_data;

	tty_port_close(&info->port, tty, f);
}

static int smd_tty_write(struct tty_struct *tty, const unsigned char *buf,
									int len)
{
	int ret = 0;
	int avail = len;
	struct msm_smd_tty_info_s *info = tty->driver_data;
	struct msm_smd_channel_info *cinfo;

	if ((info->acl_info->flags & SMD_DROP_ONE_TX_BYTE) ||
		(info->cmd_info->flags & SMD_DROP_ONE_TX_BYTE)) {
		if (HCI_COMMAND_PKT == *buf)
			cinfo = info->cmd_info;
		else
			cinfo = info->acl_info;
		buf++;
		avail--;
	}

        mutex_lock(&cinfo->lock);
        ret = qcom_smd_send(cinfo->channel, buf, avail);
        if (ret) {
                pr_err("smd_tty_write failed on smd tty %d\n", tty->index);
        }
        mutex_unlock(&cinfo->lock);

	return len;
}

static int smd_tty_port_activate(struct tty_port *tport,
                                 struct tty_struct *tty)
{
	int ret = 0;
        unsigned int n = tty->index;
        struct msm_smd_tty_info_s *info;

        if (n >= ARRAY_SIZE(msm_smd_port_info))
                return -ENODEV;

        info = &msm_smd_tty_info[n];

        mutex_lock(&info->open_lock);
        tty->driver_data = info;
	if (info->opened) {
        	mutex_unlock(&info->open_lock);
		return 0;
	}

	/* We'd better to add the channel probe for BT here.
	 * So the open will be blocked till channel is probe.
	 */
	ret = wait_for_completion_interruptible_timeout(
		&info->probe_done, msecs_to_jiffies(CHANNEL_PROBE_TIMEOUT));
	if (ret == 0) {
		pr_err("Wait for smd channel ready timeout\n");
		info->opened--;
		return -ETIMEDOUT;
	} else if (ret < 0) {
		pr_err("Error waiting for smd channel ready\n");
		info->opened--;
		return ret;
	}

	info->opened++;
        mutex_unlock(&info->open_lock);

        return 0;
}

static void smd_tty_port_shutdown(struct tty_port *tport)
{
        struct msm_smd_tty_info_s *info;
        struct tty_struct *tty = tty_port_tty_get(tport);

        info = tty->driver_data;

        mutex_lock(&info->open_lock);
	info->opened--;
	if (info->opened > 0) {
		mutex_unlock(&info->open_lock);
        	tty_kref_put(tty);
		return;
	}
	mutex_unlock(&info->open_lock);
        tty_kref_put(tty);
	return;
}

static const struct tty_port_operations smd_tty_port_ops = {
        .shutdown = smd_tty_port_shutdown,
        .activate = smd_tty_port_activate,
};

static const struct tty_operations smd_tty_ops = {
	.open = smd_tty_open,
	.close = smd_tty_close,
	.write = smd_tty_write,
};

static void smd_tty_device_init(int idx)
{
        struct tty_port *port;

        port = &msm_smd_tty_info[idx].port;
        tty_port_init(port);
	mutex_init(&msm_smd_tty_info[idx].open_lock);
	init_completion(&msm_smd_tty_info[idx].probe_done);

	msm_smd_tty_info[idx].opened = 0;

        port->ops = &smd_tty_port_ops;
        msm_smd_tty_info[idx].tty = tty_port_register_device(port,
			smd_tty_driver, idx, NULL);
	return;
}

static int smd_tty_register_driver(void)
{
	int ret = 0;

	smd_tty_driver = tty_alloc_driver(MAX_SMD_TTY_PORTS,
		TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV |
		TTY_DRIVER_RESET_TERMIOS);
	if (IS_ERR(smd_tty_driver)) {
		ret = PTR_ERR(smd_tty_driver);
		return ret;
	}

	smd_tty_driver->owner = THIS_MODULE;
	smd_tty_driver->driver_name = "smd_tty_driver";
	smd_tty_driver->name = "smd";
	smd_tty_driver->major = 0;
	smd_tty_driver->minor_start = 0;
	smd_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	smd_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	smd_tty_driver->init_termios = tty_std_termios;
	smd_tty_driver->init_termios.c_iflag = 0;
	smd_tty_driver->init_termios.c_oflag = 0;
	smd_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	smd_tty_driver->init_termios.c_lflag = 0;
	tty_set_operations(smd_tty_driver, &smd_tty_ops);

	ret = tty_register_driver(smd_tty_driver);
	if (ret) {
		put_tty_driver(smd_tty_driver);
		pr_err("register tty driver failed (%d) for smd tty\n", ret);
	}

	return ret;
}

static int __init smd_tty_init(void)
{
	int ret = 0;
	int i = 0;
	ret = smd_tty_register_driver();
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(msm_smd_port_info); i++)
		smd_tty_device_init(i);

	qcom_smd_driver_register(&qcom_smd_tty_driver);

	return 0;
}

module_init(smd_tty_init);
