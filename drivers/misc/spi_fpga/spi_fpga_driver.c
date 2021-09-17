#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "ad7768.h"
#include "fpga.h"

int power_gpio = FPGA_POWER_GPIO;
int reset_gpio = FPGA_RESET_GPIO;

module_param(power_gpio, int, S_IRUSR|S_IWUSR);
module_param(reset_gpio, int, S_IRUSR|S_IWUSR);

/* forward declarations */
static int fpga_get_id(void);
static int fpga_get_window_size(void);
static int fpga_set_window_size(uint8_t);

static int fpga_get_test_mode(void);
static int fpga_set_test_mode1(void);
static int fpga_set_test_mode2(void);
static int fpga_set_test_mode_disable(void);

static int fpga_set_pps_enable(void);
static int fpga_set_pps_disable(void);

static int fpga_set_cfg_normal(void);
static int fpga_set_cfg_adc0(void);
static int fpga_set_cfg_adc1(void);

static int fpga_get_soft_reset(void);
static int fpga_get_slices_enabled(void);
static int fpga_assert_soft_reset(void);
static int fpga_release_soft_reset(void);

static int fpga_set_slices_enabled(unsigned char);

static int fpga_get_irq_offset(void);
static int fpga_set_irq_offset(uint8_t);

static int fpga_get_ch_irq_mask_hi(void);
static int fpga_set_ch_irq_mask_hi(uint8_t);
static int fpga_get_ch_irq_mask_low(void);
static int fpga_set_ch_irq_mask_low(uint8_t);

static int fpga_get_ch_overflow_hi(void);
static int fpga_get_ch_overflow_low(void);
static int fpga_get_ch_underflow_hi(void);
static int fpga_get_ch_underflow_low(void);

static int fpga_get_stat(void);
static int fpga_clear_stat(void);

static int fpga_get_adc_reset(void);
static int fpga_adc_reset(uint32_t);
static int fpga_adc_reset_assert(void);
static int fpga_adc_reset_deassert(void);

static int ad7768_set_sampling_freq(unsigned int);
static int ad7768_get_sampling_freq(void);
static int ad7768_set_power_mode(unsigned int);
static int ad7768_get_power_mode(void);
static int ad7768_set_filter_type(unsigned int);
static int ad7768_get_filter_type(void);

static int ad7768_set_channel_standby(unsigned char);
static int ad7768_get_channel_standby(void);

static int ad7768_get_revision(void);
static int ad7768_get_interface_mode(void);

static int ad7768_read_register(uint8_t);


/* static void fpga_spi_data_read(void); */

#define fpga_attr(_name) \
        static struct kobj_attribute _name##_attr = {	\
                .attr	= {				\
                        .name = __stringify(_name),	\
                        .mode = 0644,			\
                },					\
                .show	= _name##_show,			\
                .store	= _name##_store,		\
        }

#define fpga_attr_ro(_name) \
        static struct kobj_attribute _name##_attr = {	\
                .attr	= {				\
                        .name = __stringify(_name),	\
                        .mode = S_IRUGO,		\
                },					\
                .show	= _name##_show,			\
        }

#define fpga_attr_wo(_name) \
        static struct kobj_attribute _name##_attr = {	\
                .attr	= {				\
                        .name = __stringify(_name),	\
                        .mode = S_IWUGO,		\
                },					\
                .store	= _name##_store,		\
        }

dev_t dev = 0;

static struct spi_device *fpga_spi_cfg;

static struct fpga_state st = {
        .clock_rate = 32768000,
        .cfg_mode = FPGA_CFG_MODE_CFG_NORMAL
};

unsigned int sampling_rate = 256000;
char *filter_type = "WIDEBAND";

static struct class *dev_class;
static struct cdev fpga_cdev;
struct kobject *fpga_kobj;

static ssize_t id_show(struct kobject *kobj,
                       struct kobj_attribute *attr,
                       char *buf)
{
        int ret = fpga_get_id();
        if (ret < 0) {
                pr_err("Failed to get fpga id\n");
                return -ENODEV;
        }
        return sprintf(buf, "%02x\n", ret);
}

fpga_attr_ro(id);

static ssize_t test_mode_show(struct kobject *kobj,
                       struct kobj_attribute *attr,
                       char *buf)
{
        int ret = fpga_get_test_mode();
        if (ret < 0) {
                pr_err("Failed to get fpga test mode\n");
                return -ENODEV;
        }
        return sprintf(buf, "test mode: %s, PPS Alignment %s\n",
                       (ret & (FPGA_TEST_MODE2 | FPGA_TEST_MODE1)) == 2 ?
                       "mode2" : (((ret & (FPGA_TEST_MODE2 | FPGA_TEST_MODE1)) == 1)
                        ? "mode1" : "normal"),
                       ((ret & FPGA_TEST_MODE_DEFAULT) >> 4) ?
                       "enabled" : "disabled");
}

static ssize_t test_mode_store(struct kobject *kobj,
                               struct kobj_attribute *attr,const char *buf,
                               size_t count)
{
        int test_mode;
        int ret;
        sscanf(buf,"%d",&test_mode);
        pr_info("test_mode_store %d\n", test_mode);

        if (test_mode == 1) {
                ret = fpga_set_test_mode1();
                if (ret < 0) {
                        pr_err("Failed to get fpga test mode1\n");
                }
        }
        else if (test_mode == 2) {
                ret = fpga_set_test_mode2();
                if (ret < 0) {
                        pr_err("Failed to get fpga test mode2\n");
                }
        }
        else  {
                ret = fpga_set_test_mode_disable();
                if (ret < 0) {
                        pr_err("Failed to get fpga test disable\n");
                }
        }
        return count;
}

fpga_attr(test_mode);

static ssize_t cfg_cfg_show(struct kobject *kobj,
                            struct kobj_attribute *attr,
                            char *buf)
{
        return sprintf(buf, "cfg_cfg %s\n",
                       st.cfg_mode == FPGA_CFG_MODE_CFG_ADC0 ? "adc0"
                       : (st.cfg_mode == FPGA_CFG_MODE_CFG_ADC1 ? "adc1" : "normal"));
}

static ssize_t cfg_cfg_store(struct kobject *kobj, struct kobj_attribute *attr,
                             const char *buf, size_t count)
{
        int config, ret;

        sscanf(buf,"%d",&config);

        if (config == FPGA_CFG_MODE_CFG_ADC0) {
                ret = fpga_set_cfg_adc0();
        }
        else if (config == FPGA_CFG_MODE_CFG_ADC1) {
                ret = fpga_set_cfg_adc1();
        }
        else  {
                ret = fpga_set_cfg_normal();
        }

        if (ret < 0) {
                pr_err("Failed to set config mode %d\n", config);
                return -ENODEV;
        }

        return count;
}

fpga_attr(cfg_cfg);

static ssize_t soft_reset_show(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               char *buf)
{
        int ret = fpga_get_soft_reset();
        if (ret < 0) {
                pr_err("Failed to get fpga soft reset\n");
                return -ENODEV;
        }
        return sprintf(buf, "soft reset: %s\n",
                       (ret & 0x01) ? "asserted" : "de-asserted");
}

static ssize_t soft_reset_store(struct kobject *kobj,
                                struct kobj_attribute *attr,const char *buf,
                                size_t count)
{
        int soft_reset;
        int ret;

        sscanf(buf,"%d",&soft_reset);

        if (soft_reset == 0) {
                ret = fpga_release_soft_reset();
                if (ret < 0) {
                        pr_err("Failed to release soft reset\n");
                        return -ENODEV;
                }
        }
        else  {
                ret = fpga_assert_soft_reset();
                if (ret < 0) {
                        pr_err("Failed to assert soft reset\n");
                        return -ENODEV;
                }
        }
        return count;
}

fpga_attr(soft_reset);

static ssize_t adc_reset_show(struct kobject *kobj,
                              struct kobj_attribute *attr,
                              char *buf)
{
        int ret = fpga_get_adc_reset();
        if (ret < 0) {
                pr_err("Failed to get fpga adc reset\n");
                return -ENODEV;
        }
        return sprintf(buf, "adc reset: %s\n",
                       (ret & 0x02) ? "asserted" : "de-asserted");
}

static ssize_t adc_reset_store(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               const char *buf,
                               size_t count)
{
        int adc_reset;
        int ret;

        sscanf(buf,"%d",&adc_reset);

        if (adc_reset == 0) {
                ret = fpga_adc_reset_deassert();
                if (ret < 0) {
                        pr_err("Failed to assert adc reset\n");
                        return -ENODEV;
                }
        }
        else  {
                ret = fpga_adc_reset_assert();
                if (ret < 0) {
                        pr_err("Failed to de-assert adc reset\n");
                        return -ENODEV;
                }
        }
        return count;
}

fpga_attr(adc_reset);

static ssize_t slices_enable_show(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               char *buf)
{
        int ret = fpga_get_slices_enabled();

        if (ret < 0) {
                pr_err("Failed to get fpga soft reset\n");
                return -ENODEV;
        }
        return sprintf(buf, "slice 3: %s,"
                       " slice 2: %s,"
                       " slice 1: %s,"
                       " slice 0: %s\n",
                       (ret & 0x08) ? "enabled" : "disabled",
                       (ret & 0x04) ? "enabled" : "disabled",
                       (ret & 0x02) ? "enabled" : "disabled",
                       (ret & 0x01) ? "enabled" : "disabled");
}

static ssize_t slices_enable_store(struct kobject *kobj,
                                   struct kobj_attribute *attr,
                                   const char *buf,
                                   size_t count)
{
        int slices_enabled;
        int ret;

        sscanf(buf,"%d", &slices_enabled);

        if (slices_enabled >= 0) {
                ret = fpga_set_slices_enabled(slices_enabled);
                if (ret < 0) {
                        pr_err("Failed to set slices enabled\n");
                return -ENODEV;
                }
        }
        return count;
}

fpga_attr(slices_enable);

static ssize_t window_size_show(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                char *buf)
{
        int ret = fpga_get_window_size();
        if (ret < 0) {
                pr_err("Failed to get fpga window size\n");
                return -ENODEV;
        }
        return sprintf(buf, "%d\n", ret);
}

static ssize_t window_size_store(struct kobject *kobj,
                                 struct kobj_attribute *attr,const char *buf,
                                 size_t count)
{
        int window_size;
        int ret;

        sscanf(buf,"%d",&window_size);
        pr_info("window_size %d\n", window_size);

        if (window_size > 0) {
                ret = fpga_set_window_size(window_size);
                if (ret < 0) {
                        pr_err("Failed to set window size\n");
                        return -ENODEV;
                }
        }
        else  {
                pr_err("Failed to set window size\n");
                return -ENODEV;
        }
        return count;
}

fpga_attr(window_size);

static ssize_t irq_offset_show(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                char *buf)
{
        int ret = fpga_get_irq_offset();
        if (ret < 0) {
                pr_err("Failed to get fpga irq offset\n");
                return -ENODEV;
        }
        return sprintf(buf, "%d\n", ret);
}

static ssize_t irq_offset_store(struct kobject *kobj,
                                struct kobj_attribute *attr,const char *buf,
                                size_t count)
{
        int irq_offset;
        int ret;

        sscanf(buf,"%d",&irq_offset);
        pr_info("irq_offset %d\n", irq_offset);

        if (irq_offset > 0) {
                ret = fpga_set_irq_offset(irq_offset);
                if (ret < 0) {
                        pr_err("Failed to set irq offset\n");
                        return -ENODEV;
                }
        }
        else  {
                pr_err("Failed to set irq offset\n");
                return -ENODEV;
        }
        return count;
}

fpga_attr(irq_offset);

static ssize_t sampling_freq_show(struct kobject *kobj,
                                  struct kobj_attribute *attr,
                                  char *buf)
{
        int ret = ad7768_get_sampling_freq();
        if (ret < 0) {
                pr_err("Failed to get ad7768 samping freq\n");
                return -ENODEV;
        }
        return sprintf(buf, "%u\n", ret);
}

static ssize_t sampling_freq_store(struct kobject *kobj,
                                   struct kobj_attribute *attr,const char *buf,
                                   size_t count)
{
        int sampling_freq, ret, i;

        sscanf(buf,"%d",&sampling_freq);
        pr_info("sampling_freq %d\n", sampling_freq);

        for (i = 0; i < ARRAY_SIZE(ad7768_sampl_freq_avail); i++) {
                pr_info("sampling_freq %d:%d\n", sampling_freq, ad7768_sampl_freq_avail[i]);
                if (sampling_freq == ad7768_sampl_freq_avail[i]) {
                        pr_info("got avail sampling_freq %d\n", sampling_freq);
                        break;
                }
        }

        if (i >= ARRAY_SIZE(ad7768_sampl_freq_avail)) {
                pr_err("Sampling rate is out of bound %d\n", sampling_freq);
                return -ENODEV;
        }

        if (sampling_freq > 0) {
                ret = ad7768_set_sampling_freq(sampling_freq);
                if (ret < 0) {
                        pr_err("Failed to set ad7768 sampling frequency\n");
                        return -ENODEV;
                }
        }
        else  {
                pr_err("Failed to set ad7768 sampling frequency\n");
                return -ENODEV;
        }
        return count;
}

fpga_attr(sampling_freq);

static ssize_t power_mode_show(struct kobject *kobj,
                                  struct kobj_attribute *attr,
                                  char *buf)
{
        int ret;
#if 0
        ret = fpga_get_cfg_cfg();
        if (ret < 0) {
                pr_err("Failed to get fpga cfg cfg\n");
                return -ENODEV;
        }
#endif

        ret = ad7768_get_power_mode();
        if (ret < 0) {
                pr_err("Failed to get ad7768 power mode\n");
                return -ENODEV;
        }
        return sprintf(buf, "%u\n", ret);
}

static ssize_t power_mode_store(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                const char *buf,
                                size_t count)
{
        int power_mode, ret;

        pr_info("power_mode_store\n");
        sscanf(buf,"%d",&power_mode);
        pr_info("power_mode %d\n", power_mode);

#if 0
        ret = fpga_get_cfg_cfg();
        if (ret < 0) {
                pr_err("Failed to get fpga cfg cfg\n");
                return -ENODEV;
        }
        if ((ret != FPGA_CFG_MODE_CFG_ADC0) && (ret != FPGA_CFG_MODE_CFG_ADC1)) {
                pr_err("FPGA is not in passthrough mode\n");
                return -ENODEV;
        }
#endif

        if (power_mode >= 0) {
                ret = ad7768_set_power_mode(power_mode);
                if (ret < 0) {
                        pr_err("Failed to set ad7768 power mode\n");
                        return -ENODEV;
                }
        }
        else  {
                pr_err("Failed to set ad7768 power mode\n");
                return -ENODEV;
        }
        return count;
}

fpga_attr(power_mode);

static ssize_t filter_type_show(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                char *buf)
{
        int ret;
        pr_info("power mode\n");

        ret = ad7768_get_filter_type();
        if (ret < 0) {
                pr_err("Failed to get ad7768 power mode\n");
                return -ENODEV;
        }
        return sprintf(buf, "%02x\n", ret);
}

static ssize_t filter_type_store(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                const char *buf,
                                size_t count)
{
        int filter_type, ret;

        pr_info("filter_type_store\n");
        sscanf(buf,"%d",&filter_type);
        pr_info("filter_type %d\n", filter_type);

        if (filter_type >= 0) {
                ret = ad7768_set_filter_type(filter_type);
                if (ret < 0) {
                        pr_err("Failed to set ad7768 filter type\n");
                        return -ENODEV;
                }
        }
        else  {
                pr_err("Failed to set ad7768 filter type\n");
                return -ENODEV;
        }
        return count;
}

fpga_attr(filter_type);

static ssize_t adc_revision_show(struct kobject *kobj,
                                 struct kobj_attribute *attr,
                                 char *buf)
{
        int ret;

        ret = ad7768_get_revision();
        if (ret < 0) {
                pr_err("Failed to get ad7768 revision\n");
                return -ENODEV;
        }
        return sprintf(buf, "adc %02x\n", ret);
}

fpga_attr_ro(adc_revision);

static ssize_t fpga_stat_show(struct kobject *kobj,
                              struct kobj_attribute *attr,
                              char *buf)
{
        int ret;

        ret = fpga_get_stat();
        if (ret < 0) {
                pr_err("Failed to get fpga stat\n");
                return -ENODEV;
        }
        return sprintf(buf, "fatal: %01x frq error: %01x"
                       " out sync: %01x"
                       " underflow: %01x"
                       " overflow: %01x"
                       " wait pps : %01x\n",
                       (ret & FPGA_STAT_FATAL) >> 7,
                       (ret & FPGA_STAT_FRQ_ERROR) >> 6,
                       (ret & FPGA_STAT_OUT_SYNC) >> 5,
                       (ret & FPGA_STAT_UNDERFLOW) >> 4,
                       (ret & FPGA_STAT_OVERFLOW) >> 3,
                       (ret & FPGA_STAT_WAIT_PPS) >> 0
                       );
}

static ssize_t fpga_stat_store(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               const char *buf,
                               size_t count)
{
        int ret;

        ret = fpga_clear_stat();
        if (ret < 0) {
                pr_err("Failed to clear fpga stat\n");
                return -ENODEV;
        }

        return count;
}

fpga_attr(fpga_stat);

static ssize_t fpga_reset_show(struct kobject *kobj,
                                 struct kobj_attribute *attr,
                                 char *buf)
{
        return sprintf(buf, "reset show\n");
}

static ssize_t fpga_reset_store(struct kobject *kobj,
                               struct kobj_attribute *attr,
                               const char *buf,
                               size_t count)
{

        gpio_direction_output(reset_gpio, 0);
        pr_info("Set reset gpio to %d\n", gpio_get_value(reset_gpio));
        msleep(100);
        gpio_direction_output(reset_gpio, 1);
        pr_info("Set reset gpio to %d\n", gpio_get_value(reset_gpio));

        return count;
}

fpga_attr(fpga_reset);

static ssize_t write_reg_show(struct kobject *kobj,
                        struct kobj_attribute *attr,
                        char *buf)
{
        return sprintf(buf, "send cmd to fpga\n");
}

static ssize_t write_reg_store(struct kobject *kobj,
                              struct kobj_attribute *attr,
                              const char *buf,
                              size_t count)
{

        uint32_t addr = 0, val = 0;

        sscanf(buf,"%x %x",&addr, &val);
        pr_info("send cmd: addr %02x, val %02x\n", addr, val);
        if (addr != 0) {
                uint16_t tx = cpu_to_be16(((addr & 0x7F) << 8) | (0x00ff & val));
                pr_info("send cmd: Data to set be_to_cpu %04x\n", tx);
                pr_info("send cmd: Data to set be_to_cpu 0 address: %02x\n", ((uint8_t *)&tx)[0]);
                pr_info("send cmd: Data to set be_to_cpu 1 value: %02x\n", ((uint8_t *)&tx)[1]);
                spi_write(st.spi_cfg, &tx, sizeof(tx));
                return count;
        }

        else  {
                pr_err("Failed to write to reg %02x value %02x\n", addr, val);
                return -ENODEV;
        }
}

fpga_attr(write_reg);

static ssize_t read_reg_show(struct kobject *kobj,
                        struct kobj_attribute *attr,
                        char *buf)
{
        return sprintf(buf, "read reg fpga\n");
}

static ssize_t read_reg_store(struct kobject *kobj,
                              struct kobj_attribute *attr,
                              const char *buf,
                              size_t count)
{
    uint32_t addr = 0, val = 0;

    sscanf(buf,"%x %x",&addr, &val);

    pr_info("read reg: addr %02x, val %02x\n", addr, val);
    if (addr != 0) {
        uint16_t rx = 0x00;
        uint16_t tx = cpu_to_be16((AD7768_WR_FLAG_MSK(addr) << 8));

        struct spi_transfer t[] = {
            {
                .tx_buf = &tx,
                .len = 2,
                .cs_change = 0, /* do not hold low */
                .bits_per_word = 8,
                .rx_buf = &rx,
            },
        };

        int ret;

        pr_info("read reg: Data to send  %02x %02x\n", ((uint8_t *)&tx)[0], ((uint8_t *)&tx)[1]);

        ret = spi_sync_transfer(st.spi_cfg, t, ARRAY_SIZE(t));
        if (ret < 0)
            return -ENODEV;

        /* #ifdef DEBUG_FPGA_SPI_ */
        pr_info("read reg: Data received rx %04x\n", rx);
        /* printk("Read: Data recieved be_to_cpu %02x\n", st->d16); */
        pr_info("read reg: Data received rx 0: %02x\n", ((uint8_t *)&rx)[0]);
        pr_info("read reg: Data received  val : %02x\n", be16_to_cpu(rx));
        return count;
    }
    else  {
        pr_err("Failed to write to reg %02x value %02x\n", addr, val);
        return -ENODEV;
    }
}

fpga_attr(read_reg);

static ssize_t irq_mask_high_show(struct kobject *kobj,
                                  struct kobj_attribute *attr,
                                  char *buf)
{
        int ret;

        ret = fpga_get_ch_irq_mask_hi();
        if (ret < 0) {
                pr_err("Failed to get fpga irq high\n");
                return -ENODEV;
        }
        return sprintf(buf, "ch15: %01x ch14: %01x"
                       " ch13: %01x"
                       " ch12: %01x"
                       " ch11: %01x"
                       " ch10: %01x"
                       " ch9: %01x"
                       " ch8: %01x\n",
                       (ret & FPGA_IRQ_MSK_HI_CH15) >> 7,
                       (ret & FPGA_IRQ_MSK_HI_CH14) >> 6,
                       (ret & FPGA_IRQ_MSK_HI_CH13) >> 5,
                       (ret & FPGA_IRQ_MSK_HI_CH12) >> 4,
                       (ret & FPGA_IRQ_MSK_HI_CH11) >> 3,
                       (ret & FPGA_IRQ_MSK_HI_CH10) >> 2,
                       (ret & FPGA_IRQ_MSK_HI_CH9) >> 1,
                       (ret & FPGA_IRQ_MSK_HI_CH8) >> 0
                       );
}

static ssize_t irq_mask_high_store(struct kobject *kobj,
                                   struct kobj_attribute *attr,
                                   const char *buf,
                                   size_t count)
{
        int mask;
        int ret;

        sscanf(buf,"%d",&mask);
        pr_info("irq high mask %01x\n", mask);

        if (mask >=0) {
                ret = fpga_set_ch_irq_mask_hi(mask);
                if (ret < 0) {
                        pr_err("Failed to set irq high\n");
                        return -ENODEV;
                }
        }
        else  {
                pr_err("Failed to set irq high\n");
                return -ENODEV;
        }
        return count;
}

fpga_attr(irq_mask_high);

static ssize_t irq_mask_low_show(struct kobject *kobj,
                                 struct kobj_attribute *attr,
                                 char *buf)
{
        int ret;

        ret = fpga_get_ch_irq_mask_low();
        if (ret < 0) {
                pr_err("Failed to get fpga irq high\n");
                return -ENODEV;
        }
        return sprintf(buf, "ch7: %01x ch6: %01x"
                       " ch5: %01x"
                       " ch4: %01x"
                       " ch3: %01x"
                       " ch2: %01x"
                       " ch1: %01x"
                       " ch0: %01x\n",
                       (ret & FPGA_IRQ_MSK_LOW_CH7) >> 7,
                       (ret & FPGA_IRQ_MSK_LOW_CH6) >> 6,
                       (ret & FPGA_IRQ_MSK_LOW_CH5) >> 5,
                       (ret & FPGA_IRQ_MSK_LOW_CH4) >> 4,
                       (ret & FPGA_IRQ_MSK_LOW_CH3) >> 3,
                       (ret & FPGA_IRQ_MSK_LOW_CH2) >> 2,
                       (ret & FPGA_IRQ_MSK_LOW_CH1) >> 1,
                       (ret & FPGA_IRQ_MSK_LOW_CH0) >> 0
                       );
}

static ssize_t irq_mask_low_store(struct kobject *kobj,
                                  struct kobj_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
        int mask;
        int ret;

        sscanf(buf,"%d",&mask);
        pr_info("irq low mask %01x\n", mask);

        if (mask >=0) {
                ret = fpga_set_ch_irq_mask_low(mask);
                if (ret < 0) {
                        pr_err("Failed to set irq low\n");
                        return -ENODEV;
                }
        }
        else  {
                pr_err("Failed to set irq low\n");
                return -ENODEV;
        }
        return count;
}

fpga_attr(irq_mask_low);

static ssize_t pps_show(struct kobject *kobj,
                        struct kobj_attribute *attr,
                        char *buf)
{
        int ret = fpga_get_test_mode();
        if (ret < 0) {
                pr_err("Failed to get fpga test mode\n");
                return -ENODEV;
        }
        return sprintf(buf, "PPS Alignment: %s\n",
                       ((ret & FPGA_TEST_MODE_DEFAULT) >> 4) ?
                       "enabled" : "disabled");
}

static ssize_t pps_store(struct kobject *kobj,
                               struct kobj_attribute *attr,const char *buf,
                               size_t count)
{
        int pps;
        int ret;
        sscanf(buf,"%d",&pps);
        pr_info("pps: %d\n", pps);

        if (pps == 0) {
                ret = fpga_set_pps_disable();
                if (ret < 0) {
                        pr_err("Failed to get disable pps\n");
                }
        }
        else  {
                ret = fpga_set_pps_enable();
                if (ret < 0) {
                        pr_err("Failed to get enable pps\n");
                }
        }
        return count;
}

fpga_attr(pps);

static ssize_t interface_config_show(struct kobject *kobj,
                                     struct kobj_attribute *attr,
                                     char *buf)
{
        int ret;

        ret = ad7768_get_interface_mode();
        if (ret < 0) {
                pr_err("Failed to get ad7768 interface mode\n");
                return -ENODEV;
        }
        return sprintf(buf, "%u\n", ret);
}

static ssize_t interface_config_store(struct kobject *kobj,
                                      struct kobj_attribute *attr,
                                      const char *buf,
                                      size_t count)
{
        int interface_mode;

        sscanf(buf,"%d",&interface_mode);
        pr_info("interface_mode %d\n", interface_mode);

        return count;
}

fpga_attr(interface_config);

static ssize_t adc_reg_show(struct kobject *kobj,
                            struct kobj_attribute *attr,
                            char *buf)
{
        int ret, i;

        char buf_l[512] = {0};
        for (i = 0; i < 0x0a; i++)
        {
                ret = ad7768_read_register(i);
                if (ret < 0) {
                        pr_err("Failed to get ad7768 register %02x\n", i);
                        return -ENODEV;
                }
                sprintf(buf_l + strlen(buf_l), " reg_%02x: %02x", i, ret);
        }
        return sprintf(buf, "%s\n", buf_l);
}

fpga_attr_ro(adc_reg);

static ssize_t adc_channel_standby_show(struct kobject *kobj,
                                        struct kobj_attribute *attr,
                                        char *buf)
{
        int ret;
        pr_info("%s\n", __FUNCTION__ );

        ret = ad7768_get_channel_standby();
        if (ret < 0) {
                pr_err("Failed to get ad7768 power mode\n");
                return -ENODEV;
        }
        return sprintf(buf, "%02x\n", ret);
}

static ssize_t adc_channel_standby_store(struct kobject *kobj,
                                         struct kobj_attribute *attr,
                                         const char *buf,
                                         size_t count)
{
        int standby, ret;

        pr_info("%s\n", __FUNCTION__ );
        sscanf(buf,"%d", &standby);
        pr_info("standby value: %02x\n", standby);

        if (filter_type >= 0) {
                ret = ad7768_set_channel_standby(standby);
                if (ret < 0) {
                        pr_err("Failed to set ad7768 channel standby\n");
                        return -ENODEV;
                }
        }
        else  {
                pr_err("Failed to set ad7768 filter type\n");
                return -ENODEV;
        }
        return count;
}

fpga_attr(adc_channel_standby);

static ssize_t overflow_high_show(struct kobject *kobj,
                                  struct kobj_attribute *attr,
                                  char *buf)
{
        int ret = fpga_get_ch_overflow_hi();
        if (ret < 0) {
                pr_err("Failed to get fpga ch overflow high\n");
                return -ENODEV;
        }
        return sprintf(buf, "%02x\n", ret);
}

fpga_attr_ro(overflow_high);

static ssize_t overflow_low_show(struct kobject *kobj,
                                 struct kobj_attribute *attr,
                                 char *buf)
{
        int ret = fpga_get_ch_overflow_low();
        if (ret < 0) {
                pr_err("Failed to get fpga ch overflow low\n");
                return -ENODEV;
        }
        return sprintf(buf, "%02x\n", ret);
}

fpga_attr_ro(overflow_low);

static ssize_t underflow_high_show(struct kobject *kobj,
                                   struct kobj_attribute *attr,
                                   char *buf)
{
        int ret = fpga_get_ch_underflow_hi();
        if (ret < 0) {
                pr_err("Failed to get fpga ch underflow high\n");
                return -ENODEV;
        }
        return sprintf(buf, "%02x\n", ret);
}

fpga_attr_ro(underflow_high);

static ssize_t underflow_low_show(struct kobject *kobj,
                                  struct kobj_attribute *attr,
                                  char *buf)
{
        int ret = fpga_get_ch_underflow_low();
        if (ret < 0) {
                pr_err("Failed to get fpga ch underflow low\n");
                return -ENODEV;
        }
        return sprintf(buf, "%02x\n", ret);
}

fpga_attr_ro(underflow_low);

static struct attribute * g[] = {
        &id_attr.attr,
        &test_mode_attr.attr,
        &cfg_cfg_attr.attr,
        &soft_reset_attr.attr,
        &window_size_attr.attr,
        &slices_enable_attr.attr,
        &irq_offset_attr.attr,
        &sampling_freq_attr.attr,
        &power_mode_attr.attr,
        &filter_type_attr.attr,
        &adc_revision_attr.attr,
        &fpga_stat_attr.attr,
        &fpga_reset_attr.attr,
        &write_reg_attr.attr,
        &read_reg_attr.attr,
        &irq_mask_high_attr.attr,
        &irq_mask_low_attr.attr,
        &pps_attr.attr,
        &interface_config_attr.attr,
        &adc_reg_attr.attr,
        &adc_reset_attr.attr,
		&adc_channel_standby_attr.attr,
		&overflow_high_attr.attr,
		&overflow_low_attr.attr,
		&underflow_high_attr.attr,
		&underflow_low_attr.attr,
        NULL,
};

static const struct attribute_group attr_group = {
        .attrs = g,
};

static const struct attribute_group *attr_groups[] = {
        &attr_group,
        NULL,
};

static struct file_operations fops =
{
        .owner          = THIS_MODULE,
};

struct spi_board_info fpga_spi_device_cfg_info =
{
        .modalias     = "qed-fpga-spi-cfg",
        .max_speed_hz = 1000000,              // speed your device (slave) can handle
        .bus_num      = FPGA_CFG_SPI_DEV_CFG,  /* SPI 4 on QED, SPI 2 on ADP */
        .chip_select  = FPGA_CFG_SPI_CS,      /* CS 1, CS 0 is spidev */
        .mode         = SPI_MODE_0            // SPI mode 0
};

static struct fpga_state *fpga_get_data(void)
{
        /* struct axiadc_converter *conv; */
        return &st;
}

static int fpga_spi_reg_read(struct fpga_state *st, unsigned int addr,
                             unsigned char *val)
{
        uint16_t rx = 0x00;
        uint16_t tx = cpu_to_be16((AD7768_WR_FLAG_MSK(addr) << 8));

        struct spi_transfer t[] = {
                {
                        .tx_buf = &tx,
                        .len = 2,
                        .cs_change = 0, /* do not hold low */
                        .bits_per_word = 8,
                        .rx_buf = &rx,
                },
        };

        int ret;

        pr_info("Read: Data to send in tx %04x\n", tx);

        ret = spi_sync_transfer(st->spi_cfg, t, ARRAY_SIZE(t));
        if (ret < 0)
                return ret;

        /* #ifdef DEBUG_FPGA_SPI_ */
        pr_info("Read: Data recieved rx %04x\n", rx);
        /* printk("Read: Data recieved be_to_cpu %02x\n", st->d16); */
        pr_info("Read: Data received rx 0: %02x\n", ((uint8_t *)&rx)[0]);
        pr_info("Read: Data received rx 1: %02x\n", ((uint8_t *)&rx)[1]);

        *val = be16_to_cpu(rx);
        pr_info("Read: Data received  val : %02x\n", *val);
        /* #endif */

        return ret;
}

static int fpga_spi_reg_write(struct fpga_state *st,
                              unsigned int addr,
                              unsigned char val)
{
        uint16_t tx = cpu_to_be16(((addr & 0x7F) << 8) | (0x00ff & val));
        printk("Write: Data to set be_to_cpu %04x\n", tx);
        printk("Write: Data to set be_to_cpu 0: %02x\n", ((uint8_t *)&tx)[0]);
        printk("Write: Data to set be_to_cpu 1: %02x\n", ((uint8_t *)&tx)[1]);

        return spi_write(st->spi_cfg, &tx, sizeof(tx));
}

static int fpga_spi_write_mask(struct fpga_state *st,
                               unsigned int addr,
                               unsigned long int mask,
                               unsigned char val)
{
        unsigned char regval;
        int ret;

        pr_info("%s\n", __FUNCTION__);
        ret = fpga_spi_reg_read(st, addr, &regval);
        if (ret < 0) {
                pr_err("%s: failed to read %02x\n", __FUNCTION__, addr);
                return ret;
        }
        pr_info("Write mask: val to write %02x\n", val);
        pr_info("Write mask: val in register %02x\n", regval);

        pr_info("Write mask: mask %02x and ~%02x\n", mask, ~mask);
        regval &= ~mask;
        pr_info("Write mask: regval read with mask  %02x\n", regval);
        regval |= val;
        pr_info("Write mask: regval read with val  %02x\n", regval);

        return fpga_spi_reg_write(st, addr, regval);
}

static int ad7768_spi_reg_read(struct fpga_state *st, unsigned int addr,
                               unsigned int *val)
{
        struct spi_transfer t[] = {
                {
                        .tx_buf = &st->d16,
                        .len = 2,
                        .cs_change = 1, /* hold low */
                        .bits_per_word = 8,
                }, {
                        /* .tx_buf = &tx, */
                        .rx_buf = &st->d16,
                        .len = 2,
                        .bits_per_word = 8,
                },
        };

        int ret;

        st->d16 = cpu_to_be16((AD7768_WR_FLAG_MSK(addr) << 8));
        ((unsigned char*)&st->d16)[1] = 0xaa;
        /* #ifdef DEBUG_FPGA_SPI_ */
        pr_info("ad7768_spi_reg_read: Data to send st->d16 %04x\n", st->d16);
        pr_info("ad7768_spi_reg_read: Data to send st->d16 addr with 1 [0] %02x\n", ((unsigned char*)&st->d16)[0]);
        pr_info("ad7768_spi_reg_read: Data to send st->d16 addr [0] %02x\n", 0x7f & ((unsigned char*)&st->d16)[0]);
        pr_info("ad7768_spi_reg_read: Data to send st->d16 [1] %02x\n", ((unsigned char*)&st->d16)[1]);
        /* #endif */

        ret = spi_sync_transfer(st->spi_cfg, t, ARRAY_SIZE(t));
        if (ret < 0)
                return ret;

        *val = be16_to_cpu(st->d16);
        /* #ifdef DEBUG_FPGA_SPI_ */
        pr_info("ad7768_spi_reg_read: Data recieved %04x\n", st->d16);
        pr_info("ad7768_spi_reg_read: Data recieved be_to_cpu %02x\n", st->d16);
        pr_info("ad7768_spi_reg_read: Data recieved st->d16 [0] %02x\n", ((unsigned char*)&st->d16)[0]);
        pr_info("ad7768_spi_reg_read: Data recieved st->d16 [1] %02x\n", ((unsigned char*)&st->d16)[1]);
        pr_info("ad7768_spi_reg_read: Data recieved be_to_cpu %04x\n", *val);

        /* pr_info("ad7768_spi_reg_read: Data in tx [0] %02x\n", ((unsigned char*)&tx)[0]); */
        /* pr_info("ad7768_spi_reg_read: Data in tx [1] %02x\n", ((unsigned char*)&tx)[1]); */
        /* #endif */
        *val = be16_to_cpu(st->d16);

        return ret;
}

static int ad7768_spi_reg_write(struct fpga_state *st,
                                unsigned int addr,
                                unsigned int val)
{
        st->d16 = cpu_to_be16(((addr & 0x7F) << 8) | val);

        pr_info("ad7768_spi_reg_write:  %04x\n", st->d16);
        pr_info("ad7768_spi_reg_write:  addr: [0] %02x\n", ((unsigned char*)&st->d16)[0]);
        pr_info("ad7768_spi_reg_write:  [1] %02x\n", ((unsigned char*)&st->d16)[1]);
        return spi_write(st->spi_cfg, &st->d16, sizeof(st->d16));
}

static int ad7768_spi_write_mask(struct fpga_state *st,
                                 unsigned int addr,
                                 unsigned long int mask,
                                 unsigned int val)
{
        unsigned int regval;
        int ret;
        short local_mask = ~mask;

        pr_info("%s\n", __FUNCTION__);
        ret = ad7768_spi_reg_read(st, addr, &regval);
        if (ret < 0)
                return ret;

        pr_info("write mask: to %02x data received %04x, mask %02x\n", addr, regval, local_mask);
        regval &= ~mask;
        pr_info("write mask: data masked %04x, mask %02x \n", regval, local_mask);
        regval |= val;
        pr_info("write mask: data to write masked %02x, mask %02x value %02x\n", regval, local_mask, val);

        return ad7768_spi_reg_write(st, addr, regval);
}

int fpga_get_id(void)
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        ret = fpga_spi_reg_read(st, FPGA_ID, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA ID\n");
                return ret;
        }
        pr_info("FPGA ID = %02x\n", regval);
        return regval;
}

int fpga_get_window_size()
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        ret = fpga_spi_reg_read(st, FPGA_WINDOW_SIZE, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA windows size\n");
                return ret;
        }

        regval;
        pr_info( "FPGA window size = %u\n", regval);
        return regval;
}

int fpga_set_window_size(uint8_t size)
{
        struct fpga_state *st = fpga_get_data();
        pr_info( "SET FPGA window size = %u\n", size);
        return fpga_spi_reg_write(st, FPGA_WINDOW_SIZE, size);
}

int fpga_get_irq_offset()
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        ret = fpga_spi_reg_read(st, FPGA_IRQ_OFFSET, &regval);
        if (ret < 0) {
                pr_err( "Failed to read irq offset\n");
                return ret;
        }
        pr_info( "FPGA irq offset: %u\n", regval);
        return regval;
}

int fpga_set_irq_offset(uint8_t offset)
{
        struct fpga_state *st = fpga_get_data();
        pr_info( "Set irq offset: %u\n", offset);
        return fpga_spi_reg_write(st, FPGA_IRQ_OFFSET, offset);
}

int fpga_get_ch_irq_mask_hi()
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        ret = fpga_spi_reg_read(st, FPGA_IRQ_MSK_HI, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA irq mask high\n");
                return ret;
        }
        pr_info( "FPGA irq mask high: %u\n", regval);
        return regval;
}

int fpga_set_ch_irq_mask_hi(uint8_t mask)
{
        struct fpga_state *st = fpga_get_data();
        pr_info( "SET FPGA irq mask high: %u\n", mask);
        return fpga_spi_reg_write(st, FPGA_IRQ_MSK_HI, mask);
}

int fpga_get_ch_irq_mask_low()
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        ret = fpga_spi_reg_read(st, FPGA_IRQ_MSK_LOW, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA irq mask low\n");
                return ret;
        }
        pr_info( "FPGA irq mask low: %u\n", regval);
        return regval;
}

int fpga_set_ch_irq_mask_low(uint8_t mask)
{
        struct fpga_state *st = fpga_get_data();
        pr_info( "SET FPGA irq mask low: %u\n", mask);
        return fpga_spi_reg_write(st, FPGA_IRQ_MSK_LOW, mask);
}

int fpga_get_ch_overflow_hi(void)
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        ret = fpga_spi_reg_read(st, FPGA_OVERFLOW_HI, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA ch overflow high\n");
                return ret;
        }
        pr_info( "FPGA underflow high: %02x\n", regval);
        return regval;
}

int fpga_get_ch_overflow_low(void)
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        ret = fpga_spi_reg_read(st, FPGA_OVERFLOW_LOW, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA ch overlfow low\n");
                return ret;
        }
        pr_info( "FPGA underflow low: %02x\n", regval);
        return regval;
}

int fpga_get_ch_underflow_hi(void)
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        ret = fpga_spi_reg_read(st, FPGA_UNDERFLOW_HI, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA ch underflow high\n");
                return ret;
        }
        pr_info( "FPGA underflow high: %02x\n", regval);
        return regval;
}

int fpga_get_ch_underflow_low(void)
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        ret = fpga_spi_reg_read(st, FPGA_UNDERFLOW_LOW, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA ch underflow low\n");
                return ret;
        }
        pr_info( "FPGA underflow low: %02x\n", regval);
        return regval;
}

int fpga_get_test_mode()
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        ret = fpga_spi_reg_read(st, FPGA_TEST_MODE, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA irq mask low\n");
                return ret;
        }
        pr_info( "FPGA test mode: %u, PPS alignment %u\n",
                 regval & (FPGA_TEST_MODE2 | FPGA_TEST_MODE1),
                 (regval & FPGA_TEST_MODE_DEFAULT) >> 4);
        return regval;
}

int fpga_set_test_mode1()
{
        struct fpga_state *st = fpga_get_data();

        pr_info( "SET FPGA test mode1: %u\n", FPGA_TEST_MODE1);

        /* PPS aligment is to be disabled */
        return fpga_spi_reg_write(st, FPGA_TEST_MODE, FPGA_TEST_MODE1);
}

int fpga_set_test_mode2()
{
        struct fpga_state *st = fpga_get_data();
        pr_info( "SET FPGA test mode1: %u\n", FPGA_TEST_MODE2);
        /* PPS aligment is to be disabled */
        return fpga_spi_reg_write(st, FPGA_TEST_MODE, FPGA_TEST_MODE2);
}

int fpga_set_test_mode_disable()
{
        struct fpga_state *st = fpga_get_data();
        pr_info( "SET FPGA test mode disable: %u\n", FPGA_TEST_MODE_DEFAULT);
        /* PPS aligment is to be enabled */
        return fpga_spi_reg_write(st, FPGA_TEST_MODE, FPGA_TEST_MODE_DEFAULT);
}

int fpga_set_pps_enable()
{
        struct fpga_state *st = fpga_get_data();
        pr_info( "SET FPGA PPS enable: %01x\n", FPGA_TEST_MODE_PPS_SET);
        /* PPS aligment to be enabled */
        return fpga_spi_write_mask(st, FPGA_TEST_MODE, FPGA_TEST_MODE_PPS_MSK, FPGA_TEST_MODE_PPS_SET);
}

int fpga_set_pps_disable()
{
        struct fpga_state *st = fpga_get_data();
        pr_info( "SET FPGA PPS disable: %01x\n", FPGA_TEST_MODE_PPS_UNSET);
        /* PPS aligment is to be disabled, normal mode enabled */
        return fpga_spi_write_mask(st, FPGA_TEST_MODE, FPGA_TEST_MODE_PPS_MSK, FPGA_TEST_MODE_PPS_UNSET);
}

int fpga_get_stat()
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        if (st->cfg_mode != FPGA_CFG_MODE_CFG_NORMAL) {
                return -EAGAIN;
        }

        ret = fpga_spi_reg_read(st, FPGA_STAT, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA irq mask low\n");
                return ret;
        }
        return regval;
}

int fpga_clear_stat()
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        if (st->cfg_mode != FPGA_CFG_MODE_CFG_NORMAL) {
                return -EAGAIN;
        }

        ret = fpga_spi_reg_write(st, FPGA_STAT, 0);
        if (ret < 0) {
                pr_err( "Failed to write FPGA stat clear\n");
                return ret;
        }
        return regval;
}

int fpga_get_soft_reset()
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        if (st->cfg_mode != FPGA_CFG_MODE_CFG_NORMAL) {
                return -EAGAIN;
        }

        ret = fpga_spi_reg_read(st, FPGA_SOFT_RESET, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA soft reset\n");
                return ret;
        }

        pr_info( "FPGA GET SOFT RESET = %02x\n", regval & FPGA_SOFT_RESET_MSK);
        return (regval & FPGA_SOFT_RESET_MSK);
}

int fpga_get_slices_enabled(void)
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        if (st->cfg_mode != FPGA_CFG_MODE_CFG_NORMAL) {
                return -EAGAIN;
        }

        ret = fpga_spi_reg_read(st, FPGA_SOFT_RESET, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA soft reset\n");
                return ret;
        }

        pr_info( "FPGA soft reset register = %02x\n", regval >> 4);
        return (regval >> 4);
}

int fpga_assert_soft_reset()
{
        struct fpga_state *st = fpga_get_data();

        if (st->cfg_mode != FPGA_CFG_MODE_CFG_NORMAL) {
                return -EAGAIN;
        }

        return fpga_spi_write_mask(st, FPGA_SOFT_RESET, FPGA_SOFT_RESET_MSK, FPGA_SOFT_RESET_SET);
}

int fpga_release_soft_reset()
{
        struct fpga_state *st = fpga_get_data();
        if (st->cfg_mode != FPGA_CFG_MODE_CFG_NORMAL) {
                return -EAGAIN;
        }

        return fpga_spi_write_mask(st, FPGA_SOFT_RESET, FPGA_SOFT_RESET_MSK, FPGA_SOFT_RESET_RELEASE);
}

int fpga_set_slices_enabled(unsigned char slices_enabled)
{
        struct fpga_state *st = fpga_get_data();
        int ret;
        unsigned char regval;

        if (st->cfg_mode != FPGA_CFG_MODE_CFG_NORMAL) {
                return -EAGAIN;
        }

        pr_info( "slices enable\n");
        ret = fpga_spi_reg_read(st, FPGA_SOFT_RESET, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA soft reset\n");
                return ret;
        }
        pr_info( "slices enable read %02x\n", regval);

        if (!(regval & 0x1)) {
                pr_err( "Soft reset is deasserted, can't change the slice enabled\n");
                return -EAGAIN;
        }

        pr_info( "slices enable write with mask  val: %02x\n", slices_enabled);
        pr_info( "slices enable write with mask  val << 4: %02x\n", slices_enabled << 4);
        fpga_spi_write_mask(st, FPGA_SOFT_RESET, FPGA_SOFT_RESET_SLICE_MSK, 0);
        return fpga_spi_write_mask(st, FPGA_SOFT_RESET, FPGA_SOFT_RESET_SLICE_MSK, slices_enabled << 4);
}

int fpga_set_cfg_adc0()
{
        struct fpga_state *st = fpga_get_data();
        mutex_lock(&st->lock);
        if (st->cfg_mode != FPGA_CFG_MODE_CFG_ADC0) {
                int ret = fpga_spi_reg_write(st, FPGA_CFG_MODE,
                                             FPGA_CFG_MODE_CFG_ADC0 | FPGA_CFG_MODE_CFG_SPI);
                if (ret < 0) {
                        pr_err("Failed to set cfg mode adc0\n");
                        mutex_unlock(&st->lock);
                        return ret;
                }
                st->cfg_mode = FPGA_CFG_MODE_CFG_ADC0;

        }
        else {
                pr_info("Already in cfg mode adc0\n");
        }

        mutex_unlock(&st->lock);
        return 0;
}

int fpga_set_cfg_adc1()
{
        struct fpga_state *st = fpga_get_data();

        mutex_lock(&st->lock);
        if (st->cfg_mode != FPGA_CFG_MODE_CFG_ADC1) {
                int ret = fpga_spi_reg_write(st, FPGA_CFG_MODE,
                                             FPGA_CFG_MODE_CFG_ADC1 | FPGA_CFG_MODE_CFG_SPI);
                if (ret < 0) {
                        pr_err("Failed to set cfg mode adc1\n");
                        mutex_unlock(&st->lock);
                        return ret;
                }
                st->cfg_mode = FPGA_CFG_MODE_CFG_ADC1;

        }
        else {
                pr_info("Already in cfg mode adc1\n");
        }
        mutex_unlock(&st->lock);
        return 0;
}

int fpga_set_cfg_normal()
{
        struct fpga_state *st = fpga_get_data();
        mutex_lock(&st->lock);
        if (st->cfg_mode != FPGA_CFG_MODE_CFG_NORMAL) {
                int ret = fpga_spi_reg_write(st, FPGA_CFG_MODE,
                                             FPGA_CFG_MODE_CFG_NORMAL | FPGA_CFG_MODE_CFG_SPI);
                if (ret < 0) {
                        pr_err("Failed to set cfg mode normal\n");
                        mutex_unlock(&st->lock);
                        return ret;
                }
                st->cfg_mode = FPGA_CFG_MODE_CFG_NORMAL;

        }
        else {
                pr_info("Already in cfg mode normal\n");
        }
        mutex_unlock(&st->lock);
        return 0;
}

int fpga_get_adc_reset()
{
        struct fpga_state *st = fpga_get_data();
        unsigned char regval;
        int ret;

        if (st->cfg_mode != FPGA_CFG_MODE_CFG_NORMAL) {
                return -EAGAIN;
        }

        ret = fpga_spi_reg_read(st, FPGA_SOFT_RESET, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA soft reset\n");
                return ret;
        }

        pr_info( "FPGA get adc reset = %02x\n", regval & FPGA_ADC_RESET_MSK);
        return regval & FPGA_ADC_RESET_MSK;
}

int fpga_adc_reset(uint32_t reset)
{
        struct fpga_state *st = fpga_get_data();
        int ret;
        unsigned char regval;

        if (st->cfg_mode != FPGA_CFG_MODE_CFG_NORMAL) {
                return -EAGAIN;
        }

        ret = fpga_spi_reg_read(st, FPGA_SOFT_RESET, &regval);
        if (ret < 0) {
                pr_err( "Failed to read FPGA soft reset\n");
                return ret;
        }

        if (!(regval & 0x1)) {
                pr_err( "Soft reset is deasserted, can't change the adc reset\n");
                return -EAGAIN;
        }

        pr_info( "adc reset write with mask value: %02x\n", reset);
        return fpga_spi_write_mask(st, FPGA_SOFT_RESET, FPGA_ADC_RESET_MSK, reset);
}

int fpga_adc_reset_assert()
{
        return fpga_adc_reset(FPGA_ADC_RESET_SET);
}

int fpga_adc_reset_deassert()
{
        return fpga_adc_reset(FPGA_ADC_RESET_RELEASE);
}

static int ad7768_sync(struct fpga_state *st)
{
        int ret;

        if (st->cfg_mode == FPGA_CFG_MODE_CFG_NORMAL) {
                pr_err("Failed to sync ad7768, fpga is in normal mode\n");
                return -EAGAIN;
        }

        ret = ad7768_spi_write_mask(st, AD7768_DATA_CONTROL,
                                    AD7768_DATA_CONTROL_SPI_SYNC_MSK,
                                    AD7768_DATA_CONTROL_SPI_SYNC_CLEAR);
        if (ret < 0) {
                pr_err("Failed to sync clear\n");
                return ret;
        }

        return ad7768_spi_write_mask(st,  AD7768_DATA_CONTROL,
                                     AD7768_DATA_CONTROL_SPI_SYNC_MSK,
                                     AD7768_DATA_CONTROL_SPI_SYNC);
}

static int ad7768_set_clk_divs(struct fpga_state *st,
                               unsigned int mclk_div,
                               unsigned int freq)
{
        unsigned int mclk, dclk_div, dec, div;
        unsigned int result = 0;
        int ret = 0;

        /* TODO: clarify clock rates */
        mclk = st->clock_rate;
        pr_info("clock rate %u\n", mclk);
        /* mclk = clk_get_rate(st->mclk); */

        for (dclk_div = 0; dclk_div < 4 ; dclk_div++) {
                for (dec = 0; dec < ARRAY_SIZE(ad7768_dec_rate); dec++) {
                        div = mclk_div *
                                (1 <<  (3 - dclk_div)) *
                                ad7768_dec_rate[dec];

                        result = DIV_ROUND_CLOSEST_ULL(mclk, div);
                        if (freq == result)
                                break;
                }
        }

        if (freq != result) {
                pr_err("freq != result, %u != %u\n", freq, result);
                return -EINVAL;
        }
        pr_info("clock div: %u\n", dclk_div);

        ret = ad7768_spi_write_mask(st, AD7768_INTERFACE_CFG,
                                    AD7768_INTERFACE_CFG_DCLK_DIV_MSK,
                                    /* AD7768_INTERFACE_CFG_DCLK_DIV_MODE(3 - dclk_div)); */
                                    AD7768_INTERFACE_CFG_DCLK_DIV_MODE(3));
        if (ret < 0)
                return ret;

        ret = ad7768_spi_write_mask(st, AD7768_CH_MODE,
                                     AD7768_CH_MODE_DEC_RATE_MSK,
                                     AD7768_CH_MODE_DEC_RATE_MODE(dec));
        if (ret < 0) {
                pr_err("failed to set decimation rate on ch mode A : %u\n", AD7768_CH_MODE_DEC_RATE_MODE(dec));
                return ret;
        }
        return ad7768_spi_write_mask(st, AD7768_CH_MODE_B,
                                     AD7768_CH_MODE_DEC_RATE_MSK,
                                     AD7768_CH_MODE_DEC_RATE_MODE(dec));
}

int ad7768_set_power_mode(unsigned int mode)
{
        struct fpga_state *st = fpga_get_data();
        unsigned int regval;
        int ret;
        /* Check if this mode supports the current sampling rate */

        if (st->cfg_mode == FPGA_CFG_MODE_CFG_NORMAL) {
                pr_err("Failed to set ad7768 power mode, fpga is in normal mode\n");
                return -EINVAL;
        }

        if (st->sampling_freq > ad7768_sampling_rates[mode][AD7768_MAX_RATE] ||
            st->sampling_freq < ad7768_sampling_rates[mode][AD7768_MIN_RATE]) {
                pr_err("Failed to set ad7768 power mode, sampling freq is out of range %u\n", st->sampling_freq);
                return -EINVAL;
        }

        regval = AD7768_POWER_MODE_POWER_MODE(mode);
        pr_info("setting ad7768 power mode %08x\n", regval);
        ret = ad7768_spi_write_mask(st, AD7768_POWER_MODE,
                                    AD7768_POWER_MODE_POWER_MODE_MSK,
                                    regval);
        if (ret < 0) {
                pr_err("Failed to set ad7768 power mode, spi write mask\n");
                return ret;
        }
        /* The values for the powermode correspond for mclk div */
        ret = ad7768_spi_write_mask(st, AD7768_POWER_MODE,
                                    AD7768_POWER_MODE_MCLK_DIV_MSK,
                                    AD7768_POWER_MODE_MCLK_DIV_MODE(mode));
        if (ret < 0) {
                pr_err("Failed to set ad7768 power mode, spi write div\n");
                return ret;
        }

        ret = ad7768_set_clk_divs(st, ad7768_mclk_divs[mode],
                                  st->sampling_freq);
        if (ret < 0)
                return ret;

        ret = ad7768_sync(st);
        if (ret < 0) {
                pr_err("Failed to set ad7768 power mode, sync\n");
                return ret;
        }

        st->power_mode = mode;
        pr_info("power mode %u\n", mode);

        return ret;
}

int ad7768_get_interface_mode(void)
{
        struct fpga_state *st = fpga_get_data();
        unsigned int regval;
        int ret;

        if (st->cfg_mode == FPGA_CFG_MODE_CFG_NORMAL) {
                pr_err("Failed to read ad7768 power mode, fpga is in normal mode\n");
                return -EAGAIN;
        }

        ret = ad7768_spi_reg_read(st, AD7768_INTERFACE_CFG, &regval);
        if (ret < 0) {
                pr_err("Failed to read ad7768 interface confguiration\n");
                return ret;
        }
        pr_info("interface mode: %08x\n", regval);

        return regval;
}

int ad7768_read_register(uint8_t reg)
{
        struct fpga_state *st = fpga_get_data();
        unsigned int regval;
        int ret;

        if (st->cfg_mode == FPGA_CFG_MODE_CFG_NORMAL) {
                pr_err("Failed to read ad7768 reg %02x, fpga is in normal mode\n", reg);
                return -EAGAIN;
        }

        if (reg > 0x59)
                return -EAGAIN;

        ret = ad7768_spi_reg_read(st, reg, &regval);

        if (ret < 0) {
                pr_err("Failed to read ad7768 register %02x\n", reg);
                return ret;
        }
        pr_info("reg: %02x value: %02x\n", reg, regval);

        return regval;
}

int ad7768_get_power_mode(void)
{
        struct fpga_state *st = fpga_get_data();
        unsigned int regval;
        int ret;

        if (st->cfg_mode == FPGA_CFG_MODE_CFG_NORMAL) {
                pr_err("Failed to read ad7768 power mode, fpga is in normal mode\n");
                return -EAGAIN;
        }

        ret = ad7768_spi_reg_read(st, AD7768_POWER_MODE, &regval);
        if (ret < 0) {
                pr_err("Failed to read ad7768 power mode\n");
                return ret;
        }

        st->power_mode = AD7768_POWER_MODE_GET_POWER_MODE(regval);

        return st->power_mode;
}

int ad7768_set_filter_type(unsigned int filter)
{
        struct fpga_state *st = fpga_get_data();
        int ret;

        if (st->cfg_mode == FPGA_CFG_MODE_CFG_NORMAL) {
                pr_err("Failed to set ad7768 filter type, fpga is in normal mode\n");
                return -EAGAIN;
        }

        pr_err("writing filter: addr: %02x, mask %02x, filter type %02x\n",
               AD7768_CH_MODE,
               AD7768_CH_MODE_FILTER_TYPE_MSK,
               AD7768_CH_MODE_FILTER_TYPE_MODE(filter));
        ret = ad7768_spi_write_mask(st, AD7768_CH_MODE,
                                    AD7768_CH_MODE_FILTER_TYPE_MSK,
                                    AD7768_CH_MODE_FILTER_TYPE_MODE(filter));
        if (ret < 0) {
                pr_err("Failed to set ad7768 filter type, spi write\n");
                return ret;
        }

        return ad7768_sync(st);
}

int ad7768_get_filter_type(void)
{
        struct fpga_state *st = fpga_get_data();
        unsigned int filter;
        int ret;

        if (st->cfg_mode == FPGA_CFG_MODE_CFG_NORMAL) {
                pr_err("Failed to get ad7768 filter type, fpga is in normal mode\n");
                return -EAGAIN;
        }

        ret = ad7768_spi_reg_read(st, AD7768_CH_MODE, &filter);
        if (ret < 0) {
                pr_err("Failed to get ad7768 filter type, spi read\n");
                return ret;
        }

        return AD7768_CH_MODE_GET_FILTER_TYPE(filter);
}

int ad7768_set_channel_standby(unsigned char ch_mask)
{
        struct fpga_state *st = fpga_get_data();
        int ret;

        pr_info("%s E\n", __FUNCTION__);
        if (st->cfg_mode == FPGA_CFG_MODE_CFG_NORMAL) {
                pr_err("Failed to set ad7768 filter type, fpga is in normal mode\n");
                return -EAGAIN;
        }

        pr_err("Writing channel standby: addr: %02x, channel mask %02x\n",
               AD7768_CH_STANDBY,
               ch_mask);
        ret = ad7768_spi_reg_write(st, AD7768_CH_STANDBY,
                                    ch_mask);
        if (ret < 0) {
                pr_err("Failed to set ad7768 channel standby, spi write\n");
                return ret;
        }

        return ad7768_sync(st);
}

int ad7768_get_channel_standby(void)
{
        struct fpga_state *st = fpga_get_data();
        unsigned int standby;
        int ret;

        if (st->cfg_mode == FPGA_CFG_MODE_CFG_NORMAL) {
                pr_err("Failed to get ad7768 filter type, fpga is in normal mode\n");
                return -EAGAIN;
        }

        pr_err("Reading channel standby: addr: %02x\n",
               AD7768_CH_STANDBY);
        ret = ad7768_spi_reg_read(st, AD7768_CH_STANDBY, &standby);
        if (ret < 0) {
                pr_err("Failed to get ad7768 channel standby, spi read\n");
                return ret;
        }

        return standby;
}

int ad7768_get_revision(void)
{
        struct fpga_state *st = fpga_get_data();
        unsigned int revision;
        int ret;

        if (st->cfg_mode == FPGA_CFG_MODE_CFG_NORMAL) {
                pr_err("Failed to get ad7768 revision, fpga is in normal mode\n");
                return -EAGAIN;
        }

        ret = ad7768_spi_reg_read(st, AD7768_REVISION, &revision);
        if (ret < 0) {
                pr_err("Failed to get ad7768 revision, spi read\n");
                return ret;
        }

        return revision;
}

static int ad7768_set_sampling_freq(unsigned int freq)
{
        struct fpga_state *st = fpga_get_data();
        int power_mode = -1;
        unsigned int i, j;
        int ret = 0;

        if (!freq)
                return -EINVAL;

        if (st->cfg_mode == FPGA_CFG_MODE_CFG_NORMAL) {
                pr_err("Failed to set ad7768 sampling freq, fpga is in normal mode\n");
                return -EAGAIN;
        }

        mutex_lock(&st->lock);

        for (i = 0; i < AD7768_NUM_CONFIGS; i++) {
                for (j = 0; j < AD7768_CONFIGS_PER_MODE; j++) {
                        if (freq == ad7768_sampling_rates[i][j]) {
                                power_mode = i;
                                break;
                        }
                }
        }

        if (power_mode == -1) {
                ret = -EINVAL;
                pr_err("Power mode -1\n");
                goto freq_err;
        }
        else {
                pr_err("Power mode %d\n", power_mode);
        }

        ret = ad7768_set_clk_divs(st, ad7768_mclk_divs[power_mode], freq);
        if (ret < 0) {
                pr_err("Clock divisor %d\n", power_mode);
                goto freq_err;
        }

        st->sampling_freq = freq;

        ret = ad7768_set_power_mode(power_mode);
        if (ret < 0)
                goto freq_err;

freq_err:
        mutex_unlock(&st->lock);

        return ret;
}

static int ad7768_get_sampling_freq(void)
{
        struct fpga_state *st = fpga_get_data();

        return st->sampling_freq;
}

static int __init fpga_spi_init(void)
{
        int     ret;
        struct  spi_master *cfg_master;

        if((alloc_chrdev_region(&dev, 0, 1, "fpga_driver")) < 0) {
                pr_info("Cannot allocate major number\n");
                return -1;
        }

        cdev_init(&fpga_cdev,&fops);

        if((cdev_add(&fpga_cdev, dev, 1)) < 0){
                pr_info("Cannot add the device to the system\n");
                goto err_class;
        }

        if((dev_class = class_create(THIS_MODULE, "fpga_class")) == NULL) {
                pr_info("Cannot create the struct class\n");
                goto err_class;
        }

        if((device_create(dev_class, NULL, dev, NULL, "fpga_device")) == NULL) {
                pr_info("Cannot create the Device 1\n");
                goto err_device;
        }

        fpga_kobj = kobject_create_and_add("fpga_sysfs", NULL);

        if(sysfs_create_groups(fpga_kobj, attr_groups)) {
                pr_err("Cannot create sysfs file......\n");
                goto err_sysfs;
        }

        /* conf QEd */
        cfg_master = spi_busnum_to_master(fpga_spi_device_cfg_info.bus_num);
        if(cfg_master == NULL) {
                pr_err("SPI cfg_master not found.\n");
                goto err_gpio;
        }

        /* create a new slave device for cfg */
        fpga_spi_cfg = spi_new_device(cfg_master, &fpga_spi_device_cfg_info);

        if(fpga_spi_cfg == NULL) {
                pr_err("Failed to create slave.\n");
                goto err_gpio;
        }

        ret = spi_setup(fpga_spi_cfg);
        if(ret) {
                pr_err("Failed to setup slave.\n");
                spi_unregister_device(fpga_spi_cfg);
                goto err_gpio;
        }

        st.spi_cfg = fpga_spi_cfg;

        mutex_init(&st.lock);

        /* GPIO fpga power */
        if(gpio_is_valid(power_gpio) == false) {
                pr_err("GPIO %d is not valid\n", power_gpio);
                goto err_gpio;
        }
        else {
                if(gpio_request(power_gpio,"FPGA_POWER_GPIO") < 0) {
                        pr_err("ERROR: GPIO %d request\n", power_gpio);
                        goto err_gpio;
                }
                else {
                        gpio_direction_output(power_gpio, 1);
                        pr_info("Set power gpio to %d\n", gpio_get_value(power_gpio));
                        /* let at least 100ms to load firmware */
                        msleep(150);
                        pr_info("Paused for 150 ms power gpio to %d\n", gpio_get_value(power_gpio));
                }
        }

        /* GPIO fpga reset */
        if(gpio_is_valid(reset_gpio) == false) {
                pr_err("GPIO %d is not valid\n", reset_gpio);
                goto err_gpio;
        }
        else {
                if(gpio_request(reset_gpio,"FPGA_RESET_GPIO") < 0) {
                        pr_err("ERROR: GPIO %d request\n", reset_gpio);
                        goto err_gpio;
                }
                else {
                        gpio_direction_output(reset_gpio, 1);
                        pr_info("Set reset gpio to %d\n", gpio_get_value(reset_gpio));
                }
        }

        ret = fpga_get_id();
        if (ret < 0) {
                pr_err("Failed to get fpga id\n");
                spi_unregister_device(fpga_spi_cfg);
                goto err_kzalloc;
        }
        pr_info("FPGA ID %02x", ret);

        ret = fpga_get_window_size();
        if (ret < 0) {
                pr_err("Failed to get fpga windows size\n");
                spi_unregister_device(fpga_spi_cfg);
                goto err_kzalloc;
        }
        pr_info("FPGA windows size %02x", ret);

        ret = fpga_get_stat();
        if (ret < 0) {
                pr_err("Failed to get stat\n");
                spi_unregister_device(fpga_spi_cfg);
                goto err_kzalloc;
        }
        pr_info("FPGA stat %02x", ret);

        pr_info("SPI driver Registered\n");
        return 0;

err_kzalloc:
err_gpio:
err_sysfs:
        kobject_put(fpga_kobj);
err_device:
        class_destroy(dev_class);
err_class:
        unregister_chrdev_region(dev, 1);
        cdev_del(&fpga_cdev);
        return -ENODEV;
}

static void __exit fpga_spi_exit(void)
{
		gpio_free(reset_gpio);
        gpio_free(power_gpio);

        if(fpga_spi_cfg) {
                spi_unregister_device(fpga_spi_cfg);
        }

        kobject_put(fpga_kobj);
        device_destroy(dev_class,dev);
        class_destroy(dev_class);
        cdev_del(&fpga_cdev);
        unregister_chrdev_region(dev, 1);
        pr_info("SPI driver Unregistered\n");
}

module_init(fpga_spi_init);
module_exit(fpga_spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lantronix <lantronix@lantronix.com>");
MODULE_DESCRIPTION("An FPGA QED driver");
MODULE_VERSION("1.01");
