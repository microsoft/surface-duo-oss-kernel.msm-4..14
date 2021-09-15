#ifndef AD7768_H_
#define AD7768_H_

#include <linux/clk.h>

/* #include <linux/fpga/adi-axi-common.h> */
/* #include <linux/iio/iio.h> */
/* AD7768 registers definition */
#define AD7768_CH_STANDBY           0x00
#define AD7768_CH_MODE              0x01
#define AD7768_CH_MODE_B            0x02
#define AD7768_POWER_MODE           0x04
#define AD7768_DATA_CONTROL         0x06
#define AD7768_INTERFACE_CFG        0x07
#define AD7768_STATUS               0x09
#define AD7768_REVISION             0x0a

/* AD7768_CH_MODE */
#define AD7768_CH_MODE_FILTER_TYPE_MSK      BIT(3)
#define AD7768_CH_MODE_FILTER_TYPE_MODE(x)  (((x) & 0x1) << 3)
#define AD7768_CH_MODE_GET_FILTER_TYPE(x)   (((x) >> 3) & 0x1)
#define AD7768_CH_MODE_DEC_RATE_MSK     GENMASK(2, 0)
#define AD7768_CH_MODE_DEC_RATE_MODE(x)     (((x) & 0x7) << 0)

/* AD7768_POWER_MODE */
#define AD7768_POWER_MODE_POWER_MODE_MSK    GENMASK(5, 4)
#define AD7768_POWER_MODE_POWER_MODE(x)     (((x) & 0x3) << 4)
#define AD7768_POWER_MODE_GET_POWER_MODE(x) (((x) >> 4) & 0x3)
#define AD7768_POWER_MODE_MCLK_DIV_MSK      GENMASK(1, 0)
#define AD7768_POWER_MODE_MCLK_DIV_MODE(x)  (((x) & 0x3) << 0)

/* AD7768_DATA_CONTROL */
#define AD7768_DATA_CONTROL_SPI_RESET_MSK   GENMASK(1, 0)
#define AD7768_DATA_CONTROL_SPI_RESET_1     0x03
#define AD7768_DATA_CONTROL_SPI_RESET_2     0x02
#define AD7768_DATA_CONTROL_SPI_SYNC_MSK    BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC        BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC_CLEAR  0

/* AD7768_INTERFACE_CFG */
#define AD7768_INTERFACE_CFG_DCLK_DIV_MSK   GENMASK(1, 0)
#define AD7768_INTERFACE_CFG_DCLK_DIV_MODE(x)   (((x) & 0x3) << 0)

#define AD7768_MAX_SAMP_FREQ    256000
#define AD7768_WR_FLAG_MSK(x)   (0x80 | ((x) & 0x7F))

#define AD7768_OUTPUT_MODE_TWOS_COMPLEMENT  0x01
#define AD7768_CONFIGS_PER_MODE         0x06
#define AD7768_NUM_CONFIGS          0x04
#define AD7768_MAX_RATE             (AD7768_CONFIGS_PER_MODE - 1)
#define AD7768_MIN_RATE             0

/* AD7768_STATUS */

enum ad7768_power_modes {
        AD7768_LOW_POWER_MODE,
        AD7768_MEDIAN_MODE = 2,
        AD7768_FAST_MODE
};

unsigned int ad7768_mclk_divs[] = {
        [AD7768_LOW_POWER_MODE] = 32,
        [AD7768_MEDIAN_MODE] = 8,
        [AD7768_FAST_MODE] = 4
};

int ad7768_sampling_rates[AD7768_NUM_CONFIGS][AD7768_CONFIGS_PER_MODE] = {
        [AD7768_LOW_POWER_MODE] = { 1000, 2000, 4000, 8000, 16000, 32000 },
        [AD7768_MEDIAN_MODE] = { 4000, 8000, 16000, 32000, 64000, 128000 },
        [AD7768_FAST_MODE] = { 8000, 16000, 32000, 64000, 128000, 256000 }
};

static const unsigned int ad7768_sampl_freq_avail[9] = {
        1000, 2000, 4000, 8000, 16000, 32000, 64000, 128000, 256000
};

enum ad7768_device_ids {
        ID_AD7768
};

static const int ad7768_dec_rate[6] = {
        32, 64, 128, 256, 512, 1024
};

static const char * const ad7768_filter_type_enum[] = {
        "WIDEBAND",
        "SINC5"
};

/* ADC COMMON */
#endif /* AD7768_H_ */
