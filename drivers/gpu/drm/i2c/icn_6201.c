/*
 * Copyright (c) 2016  http://www.lemaker.org/
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>


#define ICN6201_DEBUG    1
#define ICN6201_NAME	"icn6201"


struct icn6201_data_t {
	unsigned int irq;
	struct i2c_client *client;
};



struct icn6201_cmd {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};


static struct icn6201_cmd icm6201_init_cmds[] = {

	{0x20, 2, {0x20, 0x00} },

	{0x21, 2, {0x21, 0x58} },
	{0x22, 2, {0x22, 0x24} },

	{0x23, 2, {0x23, 0xE6} },
	{0x24, 2, {0x24, 0x3C} },
	{0x25, 2, {0x25, 0x02} },
	{0x26, 2, {0x26, 0x00} },
	{0x27, 2, {0x27, 0x14} },

	{0x28, 2, {0x28, 0x0A} },
	{0x29, 2, {0x29, 0x14} },
	{0x34, 2, {0x34, 0x80} },	/*buffer*/
	{0x36, 2, {0x36, 0xE6} },	/*buffer*/

	{0xB5, 2, {0xB5, 0xA0} },
	{0x5C, 2, {0x5C, 0xFF} },	/*delay*/

	{0x13, 2, {0x13, 0x10} },	/*8bit, 6bit del 10(8) 00(6)*/

	{0x56, 2, {0x56, 0x90} }, /*0x90 extern clk , 0x92 inter clk*/

	{0x6B, 2, {0x6B, 0x21} }, /* lvds clk*/

	{0x69, 2, {0x69, 0x1D} }, /*lvds clk*/

	{0xB6, 2, {0xB6, 0x20} },

	{0x51, 2, {0x51, 0x20} }, /*pll*/
	{0x09, 2, {0x09, 0x10} }  /*disply on */

};


/*
*icn6201_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int icn6201_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error\n", __func__);
	}
	return ret;
}

/*write data by i2c*/
int ic6201_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}


static void icn6201_write_cmds(struct i2c_client *client,
				struct icn6201_cmd *table, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;
		ic6201_i2c_Write(client, table[i].para_list, table[i].count);
	}

}


static int icn6201_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
	struct device *dev = &client->dev;
	struct icn6201_data_t *icn6201_data;
	unsigned int irq_no = 0;


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	icn6201_data = devm_kzalloc(dev, sizeof(*icn6201_data), GFP_KERNEL);

	if (!icn6201_data) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, icn6201_data);



	icn6201_data->irq = irq_no;
	icn6201_data->client = client;


	mdelay(500);
	icn6201_write_cmds(client, icm6201_init_cmds,
		sizeof(icm6201_init_cmds) / sizeof(struct icn6201_cmd));


	/*make sure ICN6201 already finish startup process */
	msleep(150);


#ifdef ICN6201_DEBUG
	/*get some register information */
	uc_reg_addr = 0x20;
	icn6201_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_dbg(&client->dev, "[FTS] Firmware version = 0x%x\n", uc_reg_value);

	printk(KERN_DEBUG "icn6201_probe succssss!\n");
#endif

	return 0;


exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int icn6201_remove(struct i2c_client *client)
{
	struct icn6201_data_t *icn6201_data;

	icn6201_data = i2c_get_clientdata(client);

	/*free_irq(client->irq, icn6201_data);*/
	/* kfree(icn6201_data);*/
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id icn6201_id[] = {
	{ICN6201_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, icn6201_id);

static const struct of_device_id inc6201_of_match[] = {
	{ .compatible = "ChipOne,icn6201", },
	{ }
};
MODULE_DEVICE_TABLE(of, inc6201_of_match);

static struct i2c_driver icn6201_driver = {
	.probe = icn6201_probe,
	.remove = icn6201_remove,
	.id_table = icn6201_id,
	.driver = {
		   .name = ICN6201_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = inc6201_of_match,
		   },
};

static int __init icn6201_init(void)
{
	int ret;

	ret = i2c_add_driver(&icn6201_driver);
	if (ret) {
		pr_err("Adding icn6201 driver failed errno = %d\n",
		       ret);
	} else {
		pr_info("Successfully added driver %s\n",
			icn6201_driver.driver.name);
	}

	return ret;
}

static void __exit icn6201_exit(void)
{
	i2c_del_driver(&icn6201_driver);
}


module_init(icn6201_init);
module_exit(icn6201_exit);

MODULE_AUTHOR("support@lemaker.org");
MODULE_DESCRIPTION("ICN6201 driver");
MODULE_LICENSE("GPL v2");
