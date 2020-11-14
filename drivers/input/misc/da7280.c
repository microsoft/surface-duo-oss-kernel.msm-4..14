// SPDX-License-Identifier: GPL-2.0+
/*
 * DA7280 Haptic device driver
 *
 * Copyright (c) 2020 Microsoft Corporation
 * Copyright (c) 2019 Dialog Semiconductor.
 * Author: Roy Im <Roy.Im.Opensource@diasemi.com>
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include "da7280.h"

/* DEBUG ONLY */
#define WORKQUEUE_DEBUG	0
#define SUPPORT_ENABLED	0
#define DA7280_MS_PIN_CTRL
#undef DA7280_DEBUG

/* uV unit for voltage rate */
#define DA7280_VOLTAGE_RATE_MAX		6000000
#define DA7280_VOLTAGE_RATE_STEP	23400
#define DA7280_NOMMAX_DFT		0x6B
#define DA7280_ABSMAX_DFT		0x78

#define DA7280_IMPD_MAX			1500000000
#define DA7280_IMPD_DEFAULT		22000000

#define DA7280_IMAX_DEFAULT		0x0E
/* uA unit step and limit for IMAX*/
#define DA7280_IMAX_STEP		7200
#define DA7280_IMAX_LIMIT		252000

#define DA7280_RESONT_FREQH_DFT		0x39
#define DA7280_RESONT_FREQL_DFT		0x32
#define DA7280_MIN_RESONAT_FREQ_HZ	50
#define DA7280_MAX_RESONAT_FREQ_HZ	300
#define DA7280_MIN_PWM_FREQ_KHZ		10
#define DA7280_MAX_PWM_FREQ_KHZ		250

#define DA7280_SEQ_ID_MAX		15
#define DA7280_SEQ_LOOP_MAX		15
#define DA7280_GPI1_SEQ_ID_DEFT	0x0
#define DA7280_SEQ_ID_DUR_MAX_MS	(1000 * USEC_PER_MSEC)

#define DA7280_SNP_MEM_SIZE		100
#define DA7280_SNP_MEM_MAX		DA7280_SNP_MEM_99

#define IRQ_NUM				3

#define DA7280_SKIP_INIT		0x100

#define DA7280_FF_EFFECT_COUNT_MAX	2

#define DA7280_DISABLE_DELAY_USEC	(5200)	/* 5.2 ms */

/* Define this in order to enable the production test */
#define DA7280_ENABLE_ADC_SATUATUION_DETECT

enum da7280_haptic_dev_t {
	DA7280_LRA	= 0,
	DA7280_ERM_BAR	= 1,
	DA7280_ERM_COIN	= 2,
	DA7280_DEV_MAX,
};

enum da7280_op_mode {
	DA7280_INACTIVE		= 0,
	DA7280_DRO_MODE		= 1,
	DA7280_PWM_MODE		= 2,
	DA7280_RTWM_MODE	= 3,
	DA7280_ETWM_MODE	= 4,
	DA7280_OPMODE_MAX,
};

enum da7280_custom_effect_param {
	DA7280_PARAM_SEQ_ID_IDX		= 0,
	DA7280_PARAM_TIMEOUT_SEC_IDX	= 1,
	DA7280_PARAM_TIMEOUT_MSEC_IDX	= 2,
	DA7280_PARAM_LEN		= 3,
};

enum da7280_cmd {
	DA7280_CMD_STOP		= 0,
	DA7280_CMD_START	= 1,
	DA7280_CMD_GAIN		= 2,
	DA7280_CMD_STOP_BYTIMER	= 3,
	DA7280_CMD_EFFECT	= 4,
	DA7280_CMD_MAX,
};

#define DA7280_FF_CONSTANT_MODE	DA7280_DRO_MODE
#define DA7280_FF_PERIODIC_MODE	DA7280_RTWM_MODE

struct da7280_gpi_ctl {
	u8 seq_id;
	u8 mode;
	u8 polarity;
};

struct haptic_cmd_effect {
	uint16_t effect_type;
	uint16_t replay_length;
	int16_t constant_level;
	/* periodic waveform */
	uint16_t p_waveform;
	/* Periodic custom data */
	int16_t p_cust_data[DA7280_PARAM_LEN];
	/* periodic magnitude */
	int16_t p_magntd;
};
struct haptic_cmd_data {
#if WORKQUEUE_DEBUG
	unsigned long idx;	/* Debug only */
#endif
	/* 0: stop, 1: start, 2: setgain, 3: stopByTimer */
	int cmd;
	/* use for setgain command */
	int gain;
	struct haptic_cmd_effect effect;
};

struct da7280_haptic {
	struct regmap *regmap;
	struct input_dev *input_dev;
	struct device *dev;
	struct i2c_client *client;
	struct pwm_device *pwm_dev;

	bool legacy;
	int pwm_id;

	struct workqueue_struct *hpri_wq;
	struct work_struct work_handler;

	struct hrtimer duration_timer;
	struct hrtimer disable_timer;

	/* struct haptic_cmd_data cmd_data; */
	DECLARE_KFIFO(da7280_fifo, struct haptic_cmd_data, 256);

	int val;
	u16 gain;

	s16 level;
	unsigned int magnitude;

	u8 dev_type;
	u8 op_mode;
	u8 dt_op_mode;
	u16 nommax;
	u16 absmax;
	u32 imax;
	u32 impd;
	u32 resonant_freq_h;
	u32 resonant_freq_l;
	bool bemf_sense_en;
	bool freq_track_en;
	bool acc_en;
	bool rapid_stop_en;
	bool amp_pid_en;
	u8 ps_seq_id;
	u8 ps_seq_loop;
	struct da7280_gpi_ctl gpi_ctl[3];
	bool mem_update;
	u8 snp_mem[DA7280_SNP_MEM_SIZE];
	u8 enabled;
	int length_us;
#ifdef DA7280_MS_PIN_CTRL
	struct pinctrl *da7280_pinctrl;
#endif
	spinlock_t bus_lock;
	struct mutex mutex;
	/* seq_id_duration */
	u32 seq_id_dur[DA7280_SEQ_ID_MAX+1];

};

struct da7280_haptic_work {
	struct work_struct haptic_work;
	struct da7280_haptic *haptic;
#if WORKQUEUE_DEBUG
	/* for debug : Ordering check with w_cnt */
	unsigned int work_idx;
#endif
};

#if WORKQUEUE_DEBUG
#define MAX_HANDLED_WQ_IDX	4096
#define MAX_EMPTY_CNT		4096

static unsigned int w_cnt;	/* queued workqueue count */
static unsigned int w_idx;	/* handled workqueue index */
static unsigned int empty_cnt;
static unsigned int empty[MAX_EMPTY_CNT] = {0, };
static unsigned int prev_idx;

static unsigned long idx_cnt;
#endif


static void da7280_haptic_work_handler(struct work_struct *work);
#if SUPPORT_ENABLED
static int da7280_haptic_status(struct da7280_haptic *haptics);

#define IS_HAPTIC_WORKING(X) da7280_haptic_status(X)
#endif

static bool da7280_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DA7280_IRQ_EVENT1:
	case DA7280_IRQ_EVENT_WARNING_DIAG:
	case DA7280_IRQ_EVENT_SEQ_DIAG:
	case DA7280_IRQ_STATUS1:
	case DA7280_TOP_CTL1:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config da7280_haptic_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = DA7280_SNP_MEM_MAX,
	.volatile_reg = da7280_volatile_register,
};

static int da7280_read(struct da7280_haptic *haptics,
		       unsigned int addr, unsigned int *val)
{
	int error = 0;

	error = regmap_read(haptics->regmap, addr, val);
	if (error) {
		dev_err(haptics->dev,
			"Failed(%d): Read addr 0x%x\n",
			error, addr);
	}

	return error;
}

static int da7280_write(struct da7280_haptic *haptics,
			unsigned int addr, unsigned int val)
{
	int error = 0;

	error = regmap_write(haptics->regmap, addr, val);
	if (error) {
		dev_err(haptics->dev,
			"Failed(%d): Write addr 0x%x\n",
			addr, error);
	}

	return error;
}

static int da7280_update_bits(struct da7280_haptic *haptics,
			      unsigned int addr,
			      unsigned int mask,
			      unsigned int val)
{
	int error = 0;

	error = regmap_update_bits(haptics->regmap, addr, mask, val);
	if (error) {
		dev_err(haptics->dev,
			"Failed(%d): Update addr 0x%x, val 0x%x, mask 0x%x\n",
			error, addr, val, mask);
	}

	return error;
}

static int da7280_haptic_mem_update(struct da7280_haptic *haptics)
{
	int error;
	unsigned int val;

	/* It is recommended to update the patterns
	 * during haptic is not working in order to avoid conflict
	 */

	error = da7280_read(haptics, DA7280_IRQ_STATUS1, &val);
	if (error) {
		dev_err(haptics->dev,
			"%s DA7280_IRQ_STATUS1 error=%d\n", __func__, error);
		return error;
	}
	if (val & DA7280_STA_WARNING_MASK) {
		dev_err(haptics->dev,
			"Warning! Please check HAPTIC status.\n");
		return -EBUSY;
	}

	/* Patterns are not updated if the lock bit is enabled */
	val = 0;
	error = da7280_read(haptics, DA7280_MEM_CTL2, &val);
	if (error) {
		dev_err(haptics->dev,
			"%s: DA7280_MEM_CTL2 error=%d\n", __func__, error);
		return error;
	}
	if (~val & DA7280_WAV_MEM_LOCK_MASK) {
		dev_err(haptics->dev, "Please unlock the bit first\n");
		return -EACCES;
	}

	/* Set to Inactive mode to make sure safety */
	error = da7280_update_bits(haptics,
				   DA7280_TOP_CTL1,
				   DA7280_OPERATION_MODE_MASK, 0);
	if (error) {
		dev_err(haptics->dev,
			"%s: DA7280_TOP_CTL1 error=%d\n", __func__, error);
		return error;
	}

	error = da7280_read(haptics, DA7280_MEM_CTL1, &val);
	if (error) {
		dev_err(haptics->dev,
			"%s: DA7280_MEM_CTL1 error=%d\n", __func__, error);
		return error;
	}

	error = regmap_bulk_write(haptics->regmap, val,
				  haptics->snp_mem,
				  DA7280_SNP_MEM_MAX - val + 1);
	if (error) {
		dev_err(haptics->dev,
			"regmap_bulk_write error=%d\n", error);
	}

	return error;

}

static int da7280_haptic_set_pwm(struct da7280_haptic *haptics)
{
	struct pwm_args pargs;
	u64 period_mag_multi;
	unsigned int pwm_duty;
	int error;

	pwm_get_args(haptics->pwm_dev, &pargs);
	period_mag_multi =
		(u64)(pargs.period * haptics->magnitude);
	if (haptics->acc_en)
		pwm_duty =
			(unsigned int)(period_mag_multi >> 16);
	else
		pwm_duty =
			(unsigned int)((period_mag_multi >> 16)
				+ pargs.period) / 2;

	error = pwm_config(haptics->pwm_dev,
			   pwm_duty, pargs.period);
	if (error) {
		dev_err(haptics->dev, "failed to configure pwm : %d\n", error);
		return error;
	}

	error = pwm_enable(haptics->pwm_dev);
	if (error) {
		pwm_disable(haptics->pwm_dev);
		dev_err(haptics->dev,
			"failed to enable haptics pwm device : %d\n", error);
	}

	return error;
}

#if SUPPORT_ENABLED
/* if the return value < 0 then it means i2c error
 * if the return value == 0, it means Inactive mode
 * if the return value == 1, it means Active mode(it is working now)
 */
static int da7280_haptic_status(struct da7280_haptic *haptics)
{
	int error = 0;
	unsigned int val = 0;

	error = da7280_read(haptics,
			    DA7280_TOP_CTL1, &val);
	if (error) {
		dev_err(haptics->dev,
			"%s, i2c err: %d\n", __func__, error);
		return error;
	}

#ifdef OPERATION_MODE_ONLY
	/* It returns with only the operation mode now.*/
	if ((val & DA7280_OPERATION_MODE_MASK))
		return true;
	return false;
#else
	/* It returns with
	 *  the operation mode + additional information:
	 *	SEQ_START in case of waveform modes or
	 *	OVERRIDE_VAL in case of DRO mode.
	 */
	switch ((val & DA7280_OPERATION_MODE_MASK)) {

	case DA7280_INACTIVE:
		return false;

	case DA7280_DRO_MODE:
		val = 0;
		error = da7280_read(haptics,
				    DA7280_TOP_CTL2, &val);
		if (error) {
			dev_err(haptics->dev,
				"%s, i2c err: %d\n", __func__, error);
			return error;
		}
		if (val)
			return true;
		return false;

	case DA7280_PWM_MODE:
		return true;

	case DA7280_RTWM_MODE:
	case DA7280_ETWM_MODE:
		if (val & DA7280_SEQ_START_MASK)
			return true;
		return false;

	default:
		dev_err(haptics->dev,
			"%s: Invalid Mode(%d)\n",
			__func__, haptics->op_mode);
		return -EINVAL;
	}
#endif
}
#endif

static void da7280_haptic_enable(struct da7280_haptic *haptics)
{
	int error = 0;
	s64 secs;
	unsigned long nsecs;

	if (haptics->enabled) {
		dev_warn(haptics->dev,
			 "%s: haptics->enabled already true\n",
			 __func__);
		return;
	}

	switch (haptics->op_mode) {
	case DA7280_DRO_MODE:
		/* the valid range check when acc_en is enabled */
		if (haptics->acc_en && haptics->level > 0x7F)
			haptics->level = 0x7F;
		else if (haptics->level > 0xFF)
			haptics->level = 0xFF;

		/* Set driver level
		 * as a % of ACTUATOR_NOMMAX(nommax)
		 */
		error = da7280_write(haptics,
				     DA7280_TOP_CTL2,
				     haptics->level);
		if (error) {
			dev_err(haptics->dev,
				"%s: i2c err, driving level: %d, level(%d)\n",
				__func__, error, haptics->level);
			return;
		}
		break;
	case DA7280_PWM_MODE:
		if (da7280_haptic_set_pwm(haptics))
			return;
		break;
	case DA7280_RTWM_MODE:
		/* PS_SEQ_ID will be played
		 * as many times as the PS_SEQ_LOOP
		 */
		break;
	case DA7280_ETWM_MODE:
		/* Now users are able to control the GPI(N)
		 * assigned to GPI_0, GPI1 and GPI2 accordingly
		 * please see the datasheet for details.
		 * GPI(N)_SEQUENCE_ID will be played
		 * as many times as the PS_SEQ_LOOP
		 */
		break;
	default:
		dev_err(haptics->dev,
			"%s: Invalid Mode(%d)\n",
			__func__, haptics->op_mode);
		return;
	}

	error = da7280_update_bits(haptics,
				   DA7280_TOP_CTL1,
				   DA7280_OPERATION_MODE_MASK,
				   haptics->op_mode);
	if (error) {
		dev_err(haptics->dev,
			"%s: i2c err for op_mode setting : %d\n",
			__func__, error);
		return;
	}

	if (haptics->op_mode == DA7280_PWM_MODE ||
	    haptics->op_mode == DA7280_RTWM_MODE) {
		error = da7280_update_bits(haptics,
					   DA7280_TOP_CTL1,
					   DA7280_SEQ_START_MASK,
					   DA7280_SEQ_START_MASK);
		if (error) {
			dev_err(haptics->dev,
				"%s: i2c err for sequence triggering : %d\n",
				__func__, error);
			return;
		}
	}

	haptics->enabled = true;
	if (haptics->length_us > 0 && (haptics->op_mode == DA7280_DRO_MODE)) {
		secs = haptics->length_us / USEC_PER_SEC;
		nsecs = (haptics->length_us % USEC_PER_SEC) *
			NSEC_PER_USEC;

		hrtimer_start(&haptics->duration_timer, ktime_set(secs, nsecs),
			      HRTIMER_MODE_REL);
	}
}

static void da7280_haptic_disable(struct da7280_haptic *haptics)
{
	int error;

	if (!haptics->enabled) {
#ifdef DA7280_DEBUG
		dev_info(haptics->dev,
			 "%s: !haptics->enabled\n", __func__);
#endif
		return;
	}

	/* Set to Inactive mode */
	error = da7280_update_bits(haptics,
				   DA7280_TOP_CTL1,
				   DA7280_OPERATION_MODE_MASK, 0);
	if (error) {
		dev_err(haptics->dev,
			"%s: i2c err for op_mode off : %d\n",
			__func__, error);
		return;
	}

	switch (haptics->op_mode) {
	case DA7280_DRO_MODE:
		error = da7280_write(haptics,
				     DA7280_TOP_CTL2, 0);
		if (error) {
			dev_err(haptics->dev,
				"%s: i2c err for DRO mode off : %d\n",
				__func__, error);
			return;
		}
		break;
	case DA7280_PWM_MODE:
		pwm_disable(haptics->pwm_dev);
		break;
	case DA7280_RTWM_MODE:
	case DA7280_ETWM_MODE:
		error = da7280_update_bits(haptics,
					   DA7280_TOP_CTL1,
					   DA7280_SEQ_START_MASK, 0);
		if (error) {
			dev_err(haptics->dev,
				"%s: i2c err for Waveform mode off : %d\n",
				__func__, error);
			return;
		}
		break;
	default:
		dev_err(haptics->dev,
			"%s: Invalid Mode(%d)\n", __func__, haptics->op_mode);
		return;
	}

	haptics->enabled = false;
}


static enum hrtimer_restart
	da7280_haptic_duration_timer(struct hrtimer *timer)
{
	struct da7280_haptic *haptics =
		container_of(timer, struct da7280_haptic, duration_timer);
	struct haptic_cmd_data cmd_data;
	unsigned long flags;
	struct da7280_haptic_work *work;

	haptics->length_us = 0;

	spin_lock_irqsave(&haptics->bus_lock, flags);

	/* only queue the timeout if we are currently enabled */
	if (haptics->enabled) {
#if WORKQUEUE_DEBUG
		cmd_data.idx = idx_cnt++;
#endif
		cmd_data.cmd = DA7280_CMD_STOP_BYTIMER;
		cmd_data.gain = 0;

		kfifo_in(&haptics->da7280_fifo, &cmd_data, 1);

		work = kmalloc(sizeof(*work), GFP_ATOMIC);
		if (work) {
			work->haptic = haptics;
#if WORKQUEUE_DEBUG
			work->work_idx = w_cnt++;
#endif
			INIT_WORK(&work->haptic_work,
				  da7280_haptic_work_handler);
			if (!queue_work(haptics->hpri_wq,
					&work->haptic_work)) {
				kfree(work);
				WARN_ON(1);
			}
		} else {
			dev_err(haptics->dev, "kmalloc failed\n");
		}


	} else {
#ifdef DA7280_DEBUG
		dev_info(haptics->dev,
			 "%s: skipping timer as already disabled\n",
			 __func__);
#endif
	}

	spin_unlock_irqrestore(&haptics->bus_lock, flags);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart
	da7280_haptic_disable_timer(struct hrtimer *timer)
{
	/* Do nothing here */
	return HRTIMER_NORESTART;
}

static void da7280_haptics_set_gain_cmd(struct da7280_haptic *haptics,
					int gain)
{
	int error = 0;

	haptics->gain = gain;

	if (!gain) {
		dev_warn(haptics->dev, "%s: gain==0\n", __func__);
		return;
	}

	if (gain > 0x7fff)
		gain = 0x7fff;

	/* Need to set the operation mode first
	 * for proper gain setting of the op mode
	 */
	switch (haptics->op_mode) {
	/* Only support DRO_MODE */
	case DA7280_DRO_MODE:
		if (haptics->acc_en == 1)
			haptics->level = (u8)(gain * 0x7F / 0x7fff);
		else if (haptics->acc_en == 0) {
			haptics->level = (u8)(gain * 0xFF / 0x7fff);

			if (gain <= 0x3FFF)
				haptics->level = haptics->level + 0x80;
			else if (gain > 0x3FFF)
				haptics->level = haptics->level - 0x80;
		} else {
			dev_err(haptics->dev,
				"%s: Invalid acc_en %d.\n",
				__func__, haptics->acc_en);
		}

		/* Set driver level
		 * as a % of ACTUATOR_NOMMAX(nommax)
		 */
		error = da7280_write(haptics,
				     DA7280_TOP_CTL2,
				     haptics->level);
		if (error) {
			dev_err(haptics->dev,
				"%s: i2c err for driving level set : %d\n",
				__func__, error);
			return;
		}
		break;

	default:
		dev_err(haptics->dev,
			"%s: Invalid Mode(%d)\n",
			__func__, haptics->op_mode);
		break;
	}
}

static void da7280_haptics_upload_effect_cmd(struct da7280_haptic *haptics,
		struct haptic_cmd_effect *effect)
{
	int error;
	s16 level;
	int tmp;

	switch (effect->effect_type) {

	case FF_CONSTANT:
		haptics->op_mode = DA7280_FF_CONSTANT_MODE;
		haptics->length_us = effect->replay_length * USEC_PER_MSEC;

		level = effect->constant_level;
		tmp = level * 254;
		haptics->level = tmp / 0x7FFF;
		break;

	/* RTWM/ETWM modes support this type */
	case FF_PERIODIC:
		if (effect->p_waveform != FF_CUSTOM) {
			dev_err(haptics->dev,
				"%s: ERROR Only accept custom waveforms\n",
				__func__);
			return;
		}

		haptics->ps_seq_id =
			effect->p_cust_data[DA7280_PARAM_SEQ_ID_IDX] <=
			DA7280_SEQ_ID_MAX ?
			effect->p_cust_data[DA7280_PARAM_SEQ_ID_IDX] : 0;

		haptics->op_mode = DA7280_FF_PERIODIC_MODE;

		error = da7280_write(haptics,
			     DA7280_SEQ_CTL2,
			     haptics->ps_seq_id << DA7280_PS_SEQ_ID_SHIFT |
			     haptics->ps_seq_loop << DA7280_PS_SEQ_LOOP_SHIFT);
		if (error) {
			dev_err(haptics->dev,
				"%s: i2c err for driving level set : %d\n",
				__func__, error);
			return;
		}

		haptics->length_us = effect->replay_length * USEC_PER_MSEC;

		level = effect->p_magntd;
		tmp = level * 254;
		haptics->level = tmp / 0x7FFF;
		break;

	default:
		dev_err(haptics->dev, "%s: Unsupported effect type: %d\n",
			__func__, effect->effect_type);
		return;
	}
}

static void da7280_haptic_work_handler(struct work_struct *work)
{
	struct da7280_haptic_work *w = container_of(work,
						    struct da7280_haptic_work,
						     haptic_work);
	struct haptic_cmd_data cmd_data = {0};
	int count;
	unsigned long flags;
	bool done = false;

	struct da7280_haptic *haptics = w->haptic;

	mutex_lock(&haptics->mutex);

#if WORKQUEUE_DEBUG
	if (w_idx < MAX_HANDLED_WQ_IDX) {
		w_idx++;

		/* check dropping/ordering */
		if (prev_idx == 0)
			prev_idx = w->work_idx;
		else if (w->work_idx != 0) {
			if (prev_idx != (w->work_idx - 1)) {
				dev_err(haptics->dev,
					"prev_idx(%d) == w->work_idx(%d) - 1\n",
					prev_idx, w->work_idx);
				WARN_ON(1);
			}
			prev_idx = w->work_idx;
		}
	}
#endif
	do {
		spin_lock_irqsave(&haptics->bus_lock, flags);

		count = kfifo_out(&haptics->da7280_fifo, &cmd_data, 1);

		if (count) {
			spin_unlock_irqrestore(&haptics->bus_lock, flags);

			switch (cmd_data.cmd) {
			case DA7280_CMD_START:
				da7280_haptic_enable(haptics);
				break;
			case DA7280_CMD_STOP:
			case DA7280_CMD_STOP_BYTIMER:
				da7280_haptic_disable(haptics);
				break;
			case DA7280_CMD_GAIN:
				da7280_haptics_set_gain_cmd(haptics,
							    cmd_data.gain);
				break;
			case DA7280_CMD_EFFECT:
				da7280_haptics_upload_effect_cmd(
					haptics, &cmd_data.effect);
			default:
				break;
			}

		} else {
			spin_unlock_irqrestore(&haptics->bus_lock, flags);
			done = true;
		}

	} while (0);

	kfree((void *)work);
	mutex_unlock(&haptics->mutex);
}

static int da7280_haptics_erase(struct input_dev *dev, int effect_id)
{
	struct da7280_haptic *haptics = input_get_drvdata(dev);
	int delay_us;

	/*
	 * 1. ff_erase sequence is: erase_effect()
	 *	a. ff->playback(dev, effect_id, 0);   -> STOP
	 *	b. error = ff->erase(dev, effect_id); -> ERASE EFFECT
	 * 2. After playback stop command,
	 *    system should wait two time-slot ( 2.6 ms * 2)
	 *    before upload_effect.
	 */
	delay_us = DA7280_DISABLE_DELAY_USEC;
	hrtimer_start(&haptics->disable_timer,
			ktime_set(0, delay_us * NSEC_PER_USEC),
			HRTIMER_MODE_REL);
	return 0;
}

static int da7280_haptics_upload_effect(struct input_dev *dev,
		struct ff_effect *effect, struct ff_effect *old)
{
	struct haptic_cmd_data cmd_data;
	struct da7280_haptic *haptics = input_get_drvdata(dev);
	unsigned long flags;
	int err = 0;
	ktime_t rem;
	s64 time_us;
	struct da7280_haptic_work *work;
	bool fifo_empty = false;

	if (hrtimer_active(&haptics->disable_timer)) {
		rem = hrtimer_get_remaining(&haptics->disable_timer);
		time_us = ktime_to_us(rem);
		if (time_us > 0)
			usleep_range(time_us, time_us + 100);
	}

	/* wait for FIFO to be empty */
	while (!fifo_empty) {
		spin_lock_irqsave(&haptics->bus_lock, flags);
		fifo_empty = kfifo_is_empty(&haptics->da7280_fifo);
#if WORKQUEUE_DEBUG
		if (empty_cnt < MAX_EMPTY_CNT)
			empty[empty_cnt++] = kfifo_len(&haptics->da7280_fifo);
#endif
		spin_unlock_irqrestore(&haptics->bus_lock, flags);
	}

	/* need to copy data outside of spin lock */
	if ((effect->type == FF_PERIODIC) &&
	    (effect->u.periodic.waveform == FF_CUSTOM)) {
		if (copy_from_user(cmd_data.effect.p_cust_data,
				    effect->u.periodic.custom_data,
				    sizeof(s16) * DA7280_PARAM_LEN)) {
			dev_err(haptics->dev, "%s: copy_from_user failed\n",
				__func__);
			err = -EFAULT;
		}
	}

	spin_lock_irqsave(&haptics->bus_lock, flags);

#if WORKQUEUE_DEBUG
	cmd_data.idx = idx_cnt++;
#endif
	cmd_data.cmd = DA7280_CMD_EFFECT;
	cmd_data.gain = 0;
	cmd_data.effect.effect_type = effect->type;

	switch (cmd_data.effect.effect_type) {
	case FF_CONSTANT:
		cmd_data.effect.replay_length = effect->replay.length;
		cmd_data.effect.constant_level = effect->u.constant.level;
		break;
	case FF_PERIODIC:
		if (effect->u.periodic.waveform == FF_CUSTOM) {
			if (0 == haptics->seq_id_dur[cmd_data.effect.p_cust_data[0]]) {
				dev_err(haptics->dev,
					"%s: Duration for effect %d is 0, rejecting\n"
					, __func__,cmd_data.effect.p_cust_data[0]);
				err = -EINVAL;
			} else {

				/* copied data above, outside of spin lock */
				cmd_data.effect.p_waveform =
					effect->u.periodic.waveform;
			}
		} else {
			dev_err(haptics->dev,
				"%s: effect->u.periodic.waveform != FF_CUSTOM\n"
				, __func__);
			err = -EINVAL;
		}

		cmd_data.effect.p_magntd =
			effect->u.periodic.magnitude;
		cmd_data.effect.p_cust_data[DA7280_PARAM_TIMEOUT_SEC_IDX] =
			haptics->seq_id_dur[cmd_data.effect.p_cust_data[0]]
			/ USEC_PER_SEC;
		cmd_data.effect.p_cust_data[DA7280_PARAM_TIMEOUT_MSEC_IDX] =
			(haptics->seq_id_dur[cmd_data.effect.p_cust_data[0]]
				% USEC_PER_SEC) / USEC_PER_MSEC;

		spin_unlock_irqrestore(&haptics->bus_lock, flags);

		/*
		 * See HAL code
		 */
		if (copy_to_user(effect->u.periodic.custom_data,
				 cmd_data.effect.p_cust_data,
				 sizeof(s16) * DA7280_PARAM_LEN))
			err = -EFAULT;

		spin_lock_irqsave(&haptics->bus_lock, flags);

		break;
	default:
		dev_err(haptics->dev,
			"%s: cmd_data.effect.effect_type unknown=%d\n",
			__func__, cmd_data.effect.effect_type);
		err = -EINVAL;
		break;
	}

	/* only push onto FIFO if no error */
	if (!err) {
		kfifo_in(&haptics->da7280_fifo, &cmd_data, 1);

		work = kmalloc(sizeof(*work), GFP_ATOMIC);
		if (work) {
			work->haptic = haptics;
#if WORKQUEUE_DEBUG
			work->work_idx = w_cnt++;
#endif
			INIT_WORK(&work->haptic_work,
				  da7280_haptic_work_handler);
			if (!queue_work(haptics->hpri_wq,
					&work->haptic_work)) {
				kfree(work);
				WARN_ON(1);
			}
		} else {
			dev_err(haptics->dev, "kmalloc failed\n");
		}
	}

	spin_unlock_irqrestore(&haptics->bus_lock, flags);

	return err;
}

static int da7280_haptics_playback(struct input_dev *dev,
				   int effect_id, int val)
{
	struct haptic_cmd_data cmd_data;
	struct da7280_haptic *haptics = input_get_drvdata(dev);
	unsigned long flags;
	struct da7280_haptic_work *work;

	/* Cancel the timer if we are disabling,
	 * don't want the extra disable from the timer
	 */
	if (!val)
		hrtimer_cancel(&haptics->duration_timer);

	spin_lock_irqsave(&haptics->bus_lock, flags);
#if WORKQUEUE_DEBUG
	cmd_data.idx = idx_cnt++;
#endif
	cmd_data.cmd = (!!val) ? DA7280_CMD_START : DA7280_CMD_STOP;
	cmd_data.gain = 0;

	kfifo_in(&haptics->da7280_fifo, &cmd_data, 1);

	work = kmalloc(sizeof(*work), GFP_ATOMIC);
	if (work) {
		work->haptic = haptics;
#if WORKQUEUE_DEBUG
		work->work_idx = w_cnt++;
#endif
		INIT_WORK(&work->haptic_work, da7280_haptic_work_handler);
		if (!queue_work(haptics->hpri_wq, &work->haptic_work)) {
			kfree(work);
			WARN_ON(1);
		}
	} else {
		dev_err(haptics->dev, "kmalloc failed\n");
	}

	spin_unlock_irqrestore(&haptics->bus_lock, flags);

	return 0;
}

static void da7280_haptics_set_gain(struct input_dev *dev, u16 gain)
{
	struct da7280_haptic *haptics = input_get_drvdata(dev);
	unsigned long flags;
	struct haptic_cmd_data cmd_data;
	struct da7280_haptic_work *work;

	spin_lock_irqsave(&haptics->bus_lock, flags);
#if WORKQUEUE_DEBUG
	cmd_data.idx = idx_cnt++;
#endif
	cmd_data.cmd = DA7280_CMD_GAIN;
	cmd_data.gain = gain;

	kfifo_in(&haptics->da7280_fifo, &cmd_data, 1);

	work = kmalloc(sizeof(*work), GFP_ATOMIC);
	if (work) {
		work->haptic = haptics;
#if WORKQUEUE_DEBUG
		work->work_idx = w_cnt++;
#endif
		INIT_WORK(&work->haptic_work, da7280_haptic_work_handler);
		if (!queue_work(haptics->hpri_wq, &work->haptic_work)) {
			kfree(work);
			WARN_ON(1);
		}
	} else {
		dev_err(haptics->dev, "kmalloc failed\n");
	}

	spin_unlock_irqrestore(&haptics->bus_lock, flags);
}

static int da7280_haptic_open(struct input_dev *dev)
{
	struct da7280_haptic *haptics = input_get_drvdata(dev);
	int error;

	error = da7280_update_bits(haptics,
				   DA7280_TOP_CTL1,
				   DA7280_STANDBY_EN_MASK,
				   DA7280_STANDBY_EN_MASK);
	if (error) {
		dev_err(haptics->dev,
			"%s: Failed to open haptic, i2c error : %d\n",
			__func__, error);
	}


#if (WORKQUEUE_DEBUG)
	/* Initializes */
	w_cnt = 0;
	w_idx = 0;
	empty_cnt = 0;
	prev_idx = 0;
	idx_cnt = 0;
	memset(empty, 0, ARRAY_SIZE(empty));
#endif

	return error;
}

static void da7280_haptic_close(struct input_dev *dev)
{
	struct da7280_haptic *haptics = input_get_drvdata(dev);
	int error;
	unsigned long flags;

	/* stop the timer */
	hrtimer_cancel(&haptics->duration_timer);
	hrtimer_cancel(&haptics->disable_timer);

	/* flush the work queue (don't want to destroy it) */
	flush_workqueue(haptics->hpri_wq);

	/* flush the FIFO */
	spin_lock_irqsave(&haptics->bus_lock, flags);
	kfifo_reset(&haptics->da7280_fifo);
	spin_unlock_irqrestore(&haptics->bus_lock, flags);

	error = da7280_update_bits(haptics,
				   DA7280_TOP_CTL1,
				   DA7280_OPERATION_MODE_MASK, 0);
	if (error)
		goto error_i2c;

	if (haptics->op_mode == DA7280_DRO_MODE) {
		error = da7280_write(haptics,
				     DA7280_TOP_CTL2, 0);

		if (error)
			goto error_i2c;
	}

	error = da7280_update_bits(haptics,
				   DA7280_TOP_CTL1,
				   DA7280_STANDBY_EN_MASK, 0);
	if (error)
		goto error_i2c;

#if (WORKQUEUE_DEBUG)	//  Dump
{
	int i;

	dev_err(haptics->dev,
		">> Dump workqueue\n");
	dev_err(haptics->dev,
		"1. Dropping Case? w_idx(%d) == w_cnt(%d) should be matched.\n",
		w_idx, w_cnt);

	dev_err(haptics->dev,
		"2. Dump remained kfifo count\n");
	for (i = 0; i < empty_cnt; i++) {
		dev_err(haptics->dev, "%3d ", empty[i]);

		if (i % 10 == 0)
			dev_err(haptics->dev, "\n");
	}
	dev_err(haptics->dev,
		"<< Dump workqueue\n");
}
#endif
	return;

error_i2c:
	dev_err(haptics->dev, "%s: DA7280-haptic i2c error : %d\n",
		__func__, error);
}

static u8 da7280_haptic_of_mode_str(struct device *dev,
				    const char *str)
{
	if (!strcmp(str, "LRA"))
		return DA7280_LRA;
	else if (!strcmp(str, "ERM-bar"))
		return DA7280_ERM_BAR;
	else if (!strcmp(str, "ERM-coin"))
		return DA7280_ERM_COIN;

	dev_warn(dev, "%s: Invalid string - set to default\n", __func__);
	return DA7280_LRA;
}

static u8 da7280_haptic_of_gpi_mode_str(struct device *dev,
					const char *str)
{
	if (!strcmp(str, "Single-pattern"))
		return 0;
	else if (!strcmp(str, "Multi-pattern"))
		return 1;

	dev_warn(dev, "%s: Invalid string - set to default\n", __func__);
	return 0;
}

static u8 da7280_haptic_of_gpi_pol_str(struct device *dev,
				       const char *str)
{
	if (!strcmp(str, "Rising-edge"))
		return 0;
	else if (!strcmp(str, "Falling-edge"))
		return 1;
	else if (!strcmp(str, "Both-edge"))
		return 2;

	dev_warn(dev, "%s: Invalid string - set to default\n", __func__);
	return 0;
}

static u8 da7280_haptic_of_volt_rating_set(u32 val)
{
	u32 voltage;

	voltage = val / DA7280_VOLTAGE_RATE_STEP + 1;

	if (voltage > 0xFF)
		return 0xFF;
	return (u8)voltage;
}

static void da7280_parse_properties(struct device *dev,
				    struct da7280_haptic *haptics)
{
	char gpi_str1[] = "dlg,gpi0-seq-id";
	char gpi_str2[] = "dlg,gpi0-mode";
	char gpi_str3[] = "dlg,gpi0-polarity";
	char duration_us[] = "dlg,seq-id-0-len-us";
	unsigned int i, mem[DA7280_SNP_MEM_SIZE];
	const char *str;
	u32 val;

	if (!device_property_read_string(dev, "dlg,actuator-type", &str))
		haptics->dev_type =
			da7280_haptic_of_mode_str(dev, str);
	else /* if no property, then use the mode inside chip */
		haptics->dev_type = DA7280_DEV_MAX;

	if (device_property_read_u32(dev, "dlg,op-mode", &val) >= 0)
		if (val > 0 && val < DA7280_OPMODE_MAX)
			haptics->dt_op_mode = val;
		else
			haptics->dt_op_mode = DA7280_DRO_MODE;
	else
		haptics->dt_op_mode = DA7280_DRO_MODE;

	if (device_property_read_u32(dev, "dlg,nom-microvolt", &val) >= 0)
		if (val < DA7280_VOLTAGE_RATE_MAX)
			haptics->nommax =
				da7280_haptic_of_volt_rating_set(val);
		else
			haptics->nommax = DA7280_SKIP_INIT;
	else /* if no property, then use the value inside chip */
		haptics->nommax = DA7280_SKIP_INIT;

	if (device_property_read_u32(dev, "dlg,abs-max-microvolt",
				     &val) >= 0)
		if (val < DA7280_VOLTAGE_RATE_MAX)
			haptics->absmax =
				da7280_haptic_of_volt_rating_set(val);
		else
			haptics->absmax = DA7280_SKIP_INIT;
	else
		haptics->absmax = DA7280_SKIP_INIT;

	if (device_property_read_u32(dev, "dlg,imax-microamp", &val) >= 0)
		if (val < DA7280_IMAX_LIMIT)
			haptics->imax = (val - 28600)
					/ DA7280_IMAX_STEP + 1;
		else
			haptics->imax = DA7280_IMAX_DEFAULT;
	else
		haptics->imax = DA7280_IMAX_DEFAULT;

	if (device_property_read_u32(dev, "dlg,impd-micro-ohms", &val) >= 0)
		if (val <= DA7280_IMPD_MAX)
			haptics->impd = val;
		else
			haptics->impd = DA7280_IMPD_DEFAULT;
	else
		haptics->impd = DA7280_IMPD_DEFAULT;

	if (device_property_read_u32(dev, "dlg,resonant-freq-hz",
				     &val) >= 0) {
		if (val < DA7280_MAX_RESONAT_FREQ_HZ &&
		    val > DA7280_MIN_RESONAT_FREQ_HZ) {
			haptics->resonant_freq_h =
				((1000000000 / (val * 133332 / 100)) >> 7)
				& 0xFF;
			haptics->resonant_freq_l =
				((1000000000 / (val * 133332 / 100))
				& 0x7F) - 1;
		} else {
			haptics->resonant_freq_h =
				DA7280_RESONT_FREQH_DFT;
			haptics->resonant_freq_l =
				DA7280_RESONT_FREQL_DFT;
		}
	} else {
		haptics->resonant_freq_h = DA7280_SKIP_INIT;
		haptics->resonant_freq_l = DA7280_SKIP_INIT;
	}

	if (device_property_read_u32(dev, "dlg,ps-seq-id", &val) >= 0)
		if (val <= DA7280_SEQ_ID_MAX)
			haptics->ps_seq_id = val;
		else
			haptics->ps_seq_id = 0;
	else /* if no property, set to zero as a default do nothing */
		haptics->ps_seq_id = 0;

	if (device_property_read_u32(dev, "dlg,ps-seq-loop", &val) >= 0)
		if (val <= DA7280_SEQ_LOOP_MAX)
			haptics->ps_seq_loop = val;
		else
			haptics->ps_seq_loop = 0;
	else /* if no property, then do nothing */
		haptics->ps_seq_loop = 0;

	/* GPI0~2 Control */
	for (i = 0; i < 3; i++) {
		gpi_str1[7] = '0' + i;
		if (device_property_read_u32 (dev, gpi_str1, &val) >= 0)
			if (val <= DA7280_SEQ_ID_MAX)
				haptics->gpi_ctl[i].seq_id = val;
			else
				haptics->gpi_ctl[i].seq_id =
					DA7280_GPI1_SEQ_ID_DEFT + i;
		else /* if no property, then do nothing */
			haptics->gpi_ctl[i].seq_id =
				DA7280_GPI1_SEQ_ID_DEFT + i;

		gpi_str2[7] = '0' + i;
		if (!device_property_read_string(dev, gpi_str2, &str))
			haptics->gpi_ctl[i].mode =
				da7280_haptic_of_gpi_mode_str(dev, str);
		else
			haptics->gpi_ctl[i].mode = 0;

		gpi_str3[7] = '0' + i;
		if (!device_property_read_string(dev, gpi_str3, &str))
			haptics->gpi_ctl[i].polarity =
				da7280_haptic_of_gpi_pol_str(dev, str);
		else
			haptics->gpi_ctl[i].polarity = 0;
	}

	for (i = 0; i <= DA7280_SEQ_ID_MAX; i++) {
		duration_us[11] = (i < 10) ? ('0' + i) : ('a' + (i % 10));
		if (device_property_read_u32 (dev, duration_us, &val) >= 0) {
			if (val <= DA7280_SEQ_ID_DUR_MAX_MS)
				haptics->seq_id_dur[i] = val;
			else
				haptics->seq_id_dur[i] = 0;
		} else {
			/* if no property, then set to zero */
			haptics->seq_id_dur[i] = 0;
		}
	}

	haptics->bemf_sense_en =
		device_property_read_bool(dev, "dlg,bemf-sens-enable");
	haptics->freq_track_en =
		device_property_read_bool(dev, "dlg,freq-track-enable");
	haptics->acc_en =
		device_property_read_bool(dev, "dlg,acc-enable");
	haptics->rapid_stop_en =
		device_property_read_bool(dev, "dlg,rapid-stop-enable");
	haptics->amp_pid_en =
		device_property_read_bool(dev, "dlg,amp-pid-enable");

	if (device_property_read_u32_array(dev, "dlg,mem-array",
					   &mem[0],
					   DA7280_SNP_MEM_SIZE) >= 0) {
		haptics->mem_update = true;
		for (i = 0; i < DA7280_SNP_MEM_SIZE; i++) {
			if (mem[i] > 0xff)
				haptics->snp_mem[i] = 0x0;
			else
				haptics->snp_mem[i] = (u8)mem[i];
		}
	} else {
		haptics->mem_update = false;
	}
}

static irqreturn_t da7280_irq_handler(int irq, void *data)
{
	struct da7280_haptic *haptics = data;
	u8 events[IRQ_NUM];
#ifdef DA7280_ENABLE_ADC_SATUATUION_DETECT
	unsigned int sat_event = 0;
#endif
	int error;
	bool op_mode_clear = false;

	/* Check what events have happened */
	error = regmap_bulk_read(haptics->regmap, DA7280_IRQ_EVENT1,
				 events, IRQ_NUM);
	if (error) {
		dev_err(haptics->dev,
			"%s: ERROR regmap_bulk_read\n", __func__);
		goto error_i2c;
	}

#ifdef DA7280_ENABLE_ADC_SATUATUION_DETECT
	error = da7280_read(haptics,
			    DA7280_IRQ_EVENT_ACTUATOR_FAULT,
			    &sat_event);
	if (error) {
		dev_err(haptics->dev,
			"%s: ACTUATOR_FAULT read error\n", __func__);
		goto error_i2c;
	}
#endif

	/* Clear events */
	if (events[0]) {
		error = da7280_write(haptics,
				     DA7280_IRQ_EVENT1, events[0]);
		if (error) {
			dev_err(haptics->dev,
				"%s: ERROR regmap_write\n", __func__);
			goto error_i2c;
		}
	}

#ifdef DA7280_ENABLE_ADC_SATUATUION_DETECT
	if (sat_event) {
		error = da7280_write(haptics,
				     DA7280_IRQ_EVENT_ACTUATOR_FAULT,
				     sat_event);
		if (error) {
			dev_err(haptics->dev,
				"%s: ERROR regmap_write\n", __func__);
			goto error_i2c;
		}

		if (sat_event & DA7280_ADC_SAT_FAULT_MASK) {
			dev_err(haptics->dev,
				"%s: Check if the actuator is connected\n",
				__func__);
			op_mode_clear = true;
		}
	}
#endif
	if (events[0] & DA7280_E_SEQ_FAULT_MASK) {
		/* Stop first if Haptic is working
		 * Otherwise, the fault may happen continually
		 * even though the bit is cleared.
		 */
		dev_err(haptics->dev, "%s: DA7280_E_SEQ_FAULT_MASK\n",
			__func__);
		op_mode_clear = true;
		if (events[2] & DA7280_E_SEQ_ID_FAULT_MASK)
			dev_info(haptics->dev,
				 "Please reload PS_SEQ_ID & mem data\n");
		if (events[2] & DA7280_E_MEM_FAULT_MASK)
			dev_info(haptics->dev,
				 "Please reload the mem data\n");
		if (events[2] & DA7280_E_PWM_FAULT_MASK)
			dev_info(haptics->dev,
				 "Please restart PWM interface\n");
	}

	if (events[0] & DA7280_E_WARNING_MASK) {
		if (events[1] & DA7280_E_LIM_DRIVE_MASK ||
		    events[1] & DA7280_E_LIM_DRIVE_ACC_MASK)
			dev_warn(haptics->dev,
				 "Please reduce the driver level\n");
		if (events[1] & DA7280_E_LIM_DRIVE_ACC_MASK)
			dev_warn(haptics->dev,
				 "Please Check the mem data format\n");
	}

	if (events[0] & DA7280_E_SEQ_DONE_MASK)
		dev_info(haptics->dev,
			 "%s: DA7280_E_SEQ_DONE_MASK\n", __func__);

	if (op_mode_clear) {
		error = da7280_update_bits(haptics,
					   DA7280_TOP_CTL1,
					   DA7280_OPERATION_MODE_MASK, 0);
		if (error) {
			dev_err(haptics->dev,
				"%s: ERROR regmap_update_bits\n", __func__);
			goto error_i2c;
		}
	}

	return IRQ_HANDLED;

error_i2c:
	dev_err(haptics->dev, "%s: da7280 i2c error : %d\n", __func__, error);
	return IRQ_NONE;
}

static int da7280_init(struct da7280_haptic *haptics)
{
	int error, i;
	unsigned int val = 0;
	u32 v2i_factor;
	u8 mask = 0;

	/* If device type is DA7280_DEV_MAX,
	 * then just use default value inside chip.
	 */
	if (haptics->dev_type == DA7280_DEV_MAX) {
		error = da7280_read(haptics, DA7280_TOP_CFG1, &val);
		if (error)
			goto error_i2c;
		if (val & DA7280_ACTUATOR_TYPE_MASK)
			haptics->dev_type = DA7280_ERM_COIN;
		else
			haptics->dev_type = DA7280_LRA;
	}

	/* Apply user settings */
	if (haptics->dev_type == DA7280_LRA) {
		if (haptics->resonant_freq_l != DA7280_SKIP_INIT) {
			error = da7280_write(haptics,
					     DA7280_FRQ_LRA_PER_H,
					     haptics->resonant_freq_h);
			if (error)
				goto error_i2c;
			error = da7280_write(haptics,
					     DA7280_FRQ_LRA_PER_L,
					     haptics->resonant_freq_l);
			if (error)
				goto error_i2c;
		}
	} else if (haptics->dev_type == DA7280_ERM_COIN) {
		error = da7280_update_bits(haptics,
					   DA7280_TOP_INT_CFG1,
					   DA7280_BEMF_FAULT_LIM_MASK, 0);
		if (error)
			goto error_i2c;

		mask = DA7280_TST_CALIB_IMPEDANCE_DIS_MASK |
			DA7280_V2I_FACTOR_FREEZE_MASK;
		val = DA7280_TST_CALIB_IMPEDANCE_DIS_MASK |
			DA7280_V2I_FACTOR_FREEZE_MASK;
		error = da7280_update_bits(haptics,
					   DA7280_TOP_CFG4,
					   mask, val);
		if (error)
			goto error_i2c;

		haptics->acc_en = false;
		haptics->rapid_stop_en = false;
		haptics->amp_pid_en = false;
	}

	mask = DA7280_ACTUATOR_TYPE_MASK |
			DA7280_BEMF_SENSE_EN_MASK |
			DA7280_FREQ_TRACK_EN_MASK |
			DA7280_ACCELERATION_EN_MASK |
			DA7280_RAPID_STOP_EN_MASK |
			DA7280_AMP_PID_EN_MASK;

	val = (haptics->dev_type ? 1 : 0)
			<< DA7280_ACTUATOR_TYPE_SHIFT |
		(haptics->bemf_sense_en ? 1 : 0)
			<< DA7280_BEMF_SENSE_EN_SHIFT |
		(haptics->freq_track_en ? 1 : 0)
			<< DA7280_FREQ_TRACK_EN_SHIFT |
		(haptics->acc_en ? 1 : 0)
			<< DA7280_ACCELERATION_EN_SHIFT |
		(haptics->rapid_stop_en ? 1 : 0)
			<< DA7280_RAPID_STOP_EN_SHIFT |
		(haptics->amp_pid_en ? 1 : 0)
			<< DA7280_AMP_PID_EN_SHIFT;

	error = da7280_update_bits(haptics,
				   DA7280_TOP_CFG1, mask, val);
	if (error)
		goto error_i2c;

	error = da7280_update_bits(haptics,
				   DA7280_TOP_CFG5,
				   DA7280_V2I_FACTOR_OFFSET_EN_MASK,
				   haptics->acc_en ?
				   DA7280_V2I_FACTOR_OFFSET_EN_MASK : 0);
	if (error)
		goto error_i2c;

	error = da7280_update_bits(haptics,
				   DA7280_TOP_CFG2,
				   DA7280_MEM_DATA_SIGNED_MASK,
				   haptics->acc_en ?
				   0 : DA7280_MEM_DATA_SIGNED_MASK);
	if (error)
		goto error_i2c;

	if (haptics->nommax != DA7280_SKIP_INIT) {
		error = da7280_write(haptics,
				     DA7280_ACTUATOR1,
				     haptics->nommax);
		if (error)
			goto error_i2c;
	}

	if (haptics->absmax != DA7280_SKIP_INIT) {
		error = da7280_write(haptics, DA7280_ACTUATOR2,
				     haptics->absmax);
		if (error)
			goto error_i2c;
	}

	error = da7280_update_bits(haptics,
				   DA7280_ACTUATOR3,
				   DA7280_IMAX_MASK,
				   haptics->imax);
	if (error)
		goto error_i2c;

	v2i_factor =
		haptics->impd * (haptics->imax + 4) / 1610400;
	error = da7280_write(haptics,
			     DA7280_CALIB_V2I_L,
			     v2i_factor & 0xff);
	if (error)
		goto error_i2c;
	error = da7280_write(haptics,
			     DA7280_CALIB_V2I_H,
			     v2i_factor >> 8);
	if (error)
		goto error_i2c;

	error = da7280_update_bits(haptics,
				   DA7280_TOP_CTL1,
				   DA7280_STANDBY_EN_MASK,
				   DA7280_STANDBY_EN_MASK);
	if (error)
		goto error_i2c;

	if (haptics->mem_update) {
		error = da7280_haptic_mem_update(haptics);
		if (error)
			goto error_i2c;
	}

	/* Set  PS_SEQ_ID and PS_SEQ_LOOP */
	val = haptics->ps_seq_id << DA7280_PS_SEQ_ID_SHIFT |
		haptics->ps_seq_loop << DA7280_PS_SEQ_LOOP_SHIFT;
	error = da7280_write(haptics,
			     DA7280_SEQ_CTL2, val);
	if (error)
		goto error_i2c;

	/* GPI(N) CTL */
	for (i = 0; i < 3; i++) {
		val = haptics->gpi_ctl[i].seq_id
				<< DA7280_GPI0_SEQUENCE_ID_SHIFT |
			haptics->gpi_ctl[i].mode
				<< DA7280_GPI0_MODE_SHIFT |
			haptics->gpi_ctl[i].polarity
				<< DA7280_GPI0_POLARITY_SHIFT;
		error = da7280_write(haptics,
				     DA7280_GPI_0_CTL + i, val);
		if (error)
			goto error_i2c;
	}

	/* Clear Interrupts */
	error = da7280_write(haptics, DA7280_IRQ_EVENT1, 0xff);
	if (error)
		goto error_i2c;

	error = da7280_update_bits(haptics,
				   DA7280_IRQ_MASK1,
				   DA7280_SEQ_FAULT_M_MASK
				   | DA7280_SEQ_DONE_M_MASK, 0);
	if (error)
		goto error_i2c;

#ifdef DA7280_ENABLE_ADC_SATUATUION_DETECT
	val = 0;
#else
	val = DA7280_ADC_SAT_M_MASK;
#endif
	error = da7280_update_bits(haptics,
				   DA7280_IRQ_MASK2,
				   DA7280_ADC_SAT_M_MASK, 0);
	if (error)
		goto error_i2c;


	/* Settings for specific LRA, sprinter x */
	error = da7280_write(haptics, DA7280_TOP_INT_CFG1, 0xCC);
	if (error)
		goto error_i2c;
	error = da7280_write(haptics, DA7280_TOP_INT_CFG6_H, 0x05);
	if (error)
		goto error_i2c;
	error = da7280_write(haptics, DA7280_TOP_INT_CFG6_L, 0x14);
	if (error)
		goto error_i2c;
	error = da7280_write(haptics, DA7280_TOP_INT_CFG7_H, 0x02);
	if (error)
		goto error_i2c;
	error = da7280_write(haptics, DA7280_TOP_INT_CFG7_L, 0x94);
	if (error)
		goto error_i2c;
	error = da7280_write(haptics, DA7280_TOP_INT_CFG8, 0x73);
	if (error)
		goto error_i2c;
	error = da7280_write(haptics, DA7280_TRIM4, 0x9C);
	if (error)
		goto error_i2c;
	error = da7280_write(haptics, DA7280_FRQ_CTL, 0x02);
	if (error)
		goto error_i2c;
	error = da7280_write(haptics, DA7280_TRIM3, 0x0E);
	if (error)
		goto error_i2c;
	error = da7280_write(haptics, DA7280_TOP_CFG4, 0x00);
	if (error)
		goto error_i2c;

	haptics->enabled = false;
	return 0;

error_i2c:
	dev_err(haptics->dev, "haptic init - I2C error : %d\n", error);
	return error;
}

/* Valid format for ps_seq_id
 * echo X > ps_seq_id
 * ex) echo 2 > /sys/class/..../ps_seq_id
 * 0 <= X <= 15.
 */
static ssize_t ps_seq_id_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	long val = 0xff;
	int error;

	if (kstrtol(&buf[0], 0, &val) < 0)
		goto err;

	error = da7280_update_bits(haptics,
				   DA7280_SEQ_CTL2,
				   DA7280_PS_SEQ_ID_MASK,
				   (val & 0xf) >> DA7280_PS_SEQ_ID_SHIFT);
	if (error) {
		dev_err(haptics->dev,
			"%s: failed to update register : %d\n",
			__func__, error);
		return error;
	}

	haptics->ps_seq_id = val & 0xf;

	return count;

err:
	dev_err(haptics->dev, "%s: Invalid input\n", __func__);
	return count;
}

static ssize_t ps_seq_id_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	int error;
	unsigned int val;

	error = da7280_read(haptics, DA7280_SEQ_CTL2, &val);
	if (error) {
		dev_err(haptics->dev,
			"%s: failed to read register : %d\n",
			__func__, error);
		return error;
	}
	val = (val & DA7280_PS_SEQ_ID_MASK)
		>> DA7280_PS_SEQ_ID_SHIFT;

	return sprintf(buf, "ps_seq_id is %d\n", val);
}

/* Valid format for ps_seq_loop
 * echo X > ps_seq_loop
 * ex) echo 2 > /sys/class/..../ps_seq_loop
 * 0 <= X <= 15.
 */
static ssize_t ps_seq_loop_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	long val = 0xff;
	int error;

	if (kstrtol(&buf[0], 0, &val) < 0)
		goto err;

	error = da7280_update_bits(haptics,
				   DA7280_SEQ_CTL2,
				   DA7280_PS_SEQ_LOOP_MASK,
				   (val & 0xF) << DA7280_PS_SEQ_LOOP_SHIFT);
	if (error) {
		dev_err(haptics->dev,
			"%s: failed to update register : %d\n",
			__func__, error);
		return error;
	}

	haptics->ps_seq_loop = (val & 0xF);

	return count;
err:
	dev_err(haptics->dev, "%s: Invalid input\n", __func__);
	return count;
}

static ssize_t ps_seq_loop_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	int error;
	unsigned int val;

	error = da7280_read(haptics, DA7280_SEQ_CTL2, &val);
	if (error) {
		dev_err(haptics->dev,
			"%s: failed to read register : %d\n",
			__func__, error);
		return error;
	}
	val = (val & DA7280_PS_SEQ_LOOP_MASK)
				>> DA7280_PS_SEQ_LOOP_SHIFT;

	return sprintf(buf, "ps_seq_loop is %d\n", val);
}

/* Valid format for GPIx_SEQUENCE_ID
 * echo X > ./gpi_seq_id0
 * Range of X: 0 <= X <= 15
 * ex)
 *	echo 1 > /sys/class/..../gpi_seq_id0
 *	echo 2 > /sys/class/..../gpi_seq_id1
 *	echo 3 > /sys/class/..../gpi_seq_id2
 */
static ssize_t gpi_seq_id0_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	long val = 0xff;
	int error;

	if (kstrtol(&buf[0], 0, &val) < 0)
		goto err;

	error = da7280_update_bits(haptics,
				   DA7280_GPI_0_CTL,
				   DA7280_GPI0_SEQUENCE_ID_MASK,
				   (val & 0xf)
				   << DA7280_GPI0_SEQUENCE_ID_SHIFT);
	if (error) {
		dev_err(haptics->dev,
			"%s: failed to update register : %d\n",
			__func__, error);
		return error;
	}

	haptics->gpi_ctl[0].seq_id = val & 0xf;

	return count;

err:
	dev_err(haptics->dev, "%s: Invalid input\n", __func__);
	return count;
}

static ssize_t gpi_seq_id0_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	int error;
	unsigned int val;

	error = da7280_read(haptics, DA7280_GPI_0_CTL, &val);
	if (error) {
		dev_err(haptics->dev,
			"%s: failed to read register : %d\n",
			__func__, error);
		return error;
	}
	val = (val & DA7280_GPI0_SEQUENCE_ID_MASK)
		>> DA7280_GPI0_SEQUENCE_ID_SHIFT;

	return sprintf(buf, "gpi_seq_id0 is %d\n", val);
}

static ssize_t gpi_seq_id1_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	long val = 0xff;
	int error;

	if (kstrtol(&buf[0], 0, &val) < 0)
		goto err;

	error = da7280_update_bits(haptics,
				   DA7280_GPI_1_CTL,
				   DA7280_GPI1_SEQUENCE_ID_MASK,
				   (val & 0xf)
				   << DA7280_GPI1_SEQUENCE_ID_SHIFT);
	if (error) {
		dev_err(haptics->dev,
			"%s: failed to update register : %d\n",
			__func__, error);
		return error;
	}

	haptics->gpi_ctl[1].seq_id = val & 0xf;

	return count;

err:
	dev_err(haptics->dev, "%s: Invalid input\n", __func__);
	return count;
}

static ssize_t gpi_seq_id1_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	int error;
	unsigned int val;

	error = da7280_read(haptics, DA7280_GPI_1_CTL, &val);
	if (error) {
		dev_err(haptics->dev,
			"%s: failed to read register : %d\n",
			__func__, error);
		return error;
	}
	val = (val & DA7280_GPI1_SEQUENCE_ID_MASK)
		>> DA7280_GPI1_SEQUENCE_ID_SHIFT;

	return sprintf(buf, "gpi_seq_id1 is %d\n", val);
}

static ssize_t gpi_seq_id2_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t count)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	long val = 0xff;
	int error;

	if (kstrtol(&buf[0], 0, &val) < 0)
		goto err;

	error = da7280_update_bits(haptics,
				   DA7280_GPI_2_CTL,
				   DA7280_GPI2_SEQUENCE_ID_MASK,
				   (val & 0xf)
				   << DA7280_GPI2_SEQUENCE_ID_SHIFT);
	if (error) {
		dev_err(haptics->dev,
			"%s: failed to update register : %d\n",
			__func__, error);
		return error;
	}

	haptics->gpi_ctl[2].seq_id = val & 0xf;

	return count;

err:
	dev_err(haptics->dev, "%s: Invalid input\n", __func__);
	return count;
}

static ssize_t gpi_seq_id2_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	int error;
	unsigned int val;

	error = da7280_read(haptics, DA7280_GPI_2_CTL, &val);
	if (error) {
		dev_err(haptics->dev,
			"%s: failed to read register : %d\n",
			__func__, error);
		return error;
	}
	val = (val & DA7280_GPI2_SEQUENCE_ID_MASK)
		>> DA7280_GPI2_SEQUENCE_ID_SHIFT;

	return sprintf(buf, "gpi_seq_id2 is %d\n", val);
}

#define MAX_PTN_REGS DA7280_SNP_MEM_SIZE
#define MAX_USER_INPUT_LEN (5 * DA7280_SNP_MEM_SIZE)
struct parse_data_t {
	int len;
	u8 val[MAX_PTN_REGS];
};

static int da7280_parse_args(struct device *dev,
			     char *cmd, struct parse_data_t *ptn)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	char *tok;		/* used to separate tokens */
	const char ct[] = " \t"; /* space or tab delimits the tokens */
	int tok_count = 0;	/* total number of tokens parsed */
	int i = 0, val;

	ptn->len = 0;

	/* parse the input string */
	while ((tok = strsep(&cmd, ct)) != NULL) {
		/* this is a value to be written to the register */
		if (kstrtouint(tok, 0, &val) < 0) {
			dev_err(haptics->dev,
				"%s: failed to read from %s\n",
				__func__, tok);
			break;
		}

		if (i < MAX_PTN_REGS) {
			ptn->val[i] = val;
			i++;
		}
		tok_count++;
	}

	/* decide whether it is a read or write operation based on the
	 * value of tok_count and count_flag.
	 * tok_count = 0: no inputs, invalid case.
	 * tok_count = 1: write one value.
	 * tok_count > 1: write multiple values/patterns.
	 */
	switch (tok_count) {
	case 0:
		dev_err(haptics->dev, "%s: ERROR -EINVAL\n", __func__);
		return -EINVAL;
	case 1:
		ptn->len = 1;
		break;
	default:
		ptn->len = i;
	}

	return 0;
}

static ssize_t
patterns_store(struct device *dev,
	       struct device_attribute *attr,
	       const char *buf,
	       size_t count)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	struct parse_data_t mem;
	char cmd[MAX_USER_INPUT_LEN];
	unsigned int val;
	int error;

	error = da7280_read(haptics, DA7280_MEM_CTL1, &val);
	if (error)
		return error;

	if (count > MAX_USER_INPUT_LEN)
		memcpy(cmd, buf, MAX_USER_INPUT_LEN);
	else
		memcpy(cmd, buf, count);

	/* chop of '\n' introduced by echo at the end of the input */
	if (cmd[count - 1] == '\n')
		cmd[count - 1] = '\0';

	if (da7280_parse_args(dev, cmd, &mem) < 0)
		return -EINVAL;

	memcpy(haptics->snp_mem, mem.val, mem.len);

	error = da7280_haptic_mem_update(haptics);
	if (error)
		return error;

	return count;
}

static DEVICE_ATTR_RW(ps_seq_id);
static DEVICE_ATTR_RW(ps_seq_loop);
static DEVICE_ATTR_RW(gpi_seq_id0);
static DEVICE_ATTR_RW(gpi_seq_id1);
static DEVICE_ATTR_RW(gpi_seq_id2);
static DEVICE_ATTR_WO(patterns);
static struct attribute *da7280_sysfs_attr[] = {
	&dev_attr_ps_seq_id.attr,
	&dev_attr_ps_seq_loop.attr,
	&dev_attr_gpi_seq_id0.attr,
	&dev_attr_gpi_seq_id1.attr,
	&dev_attr_gpi_seq_id2.attr,
	&dev_attr_patterns.attr,
	NULL,
};

static const struct attribute_group da7280_attr_group = {
	.attrs = da7280_sysfs_attr,
};


static const char pinctrl_name[] = "da7280_default";
static int da7280_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct da7280_haptic *haptics;
	struct input_dev *input_dev;
	struct ff_device *ff;
	unsigned int period2freq;
	int error;
#ifdef DA7280_MS_PIN_CTRL
	struct pinctrl_state *state;
#endif
	haptics = devm_kzalloc(dev, sizeof(*haptics), GFP_KERNEL);
	if (!haptics)
		return -ENOMEM;

	/* need to init FIFO */
	INIT_KFIFO(haptics->da7280_fifo);

	haptics->dev = dev;

	if (!client->irq) {
		dev_err(dev, "%s: No IRQ configured\n", __func__);
		return -EINVAL;
	}

#ifdef DA7280_MS_PIN_CTRL
	haptics->da7280_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(haptics->da7280_pinctrl)) {
		if (PTR_ERR(haptics->da7280_pinctrl) == -EPROBE_DEFER) {
			dev_err(dev, "%s: pinctrl not ready\n", __func__);
			return -EPROBE_DEFER;
		}
		dev_err(dev, "%s: Target does not use pinctrl\n", __func__);
		haptics->da7280_pinctrl = NULL;
		return -EINVAL;
	}

	state = pinctrl_lookup_state(haptics->da7280_pinctrl, pinctrl_name);
	if (IS_ERR(state)) {
		dev_err(dev, "%s: cannot find '%s'\n", __func__, pinctrl_name);
		return -EINVAL;
	}

	error = pinctrl_select_state(haptics->da7280_pinctrl, state);
	if (error) {
		dev_err(dev, "%s: pinctrl_select_state error=%d\n",
			__func__, error);
		return error;
	}
#endif
	da7280_parse_properties(&client->dev, haptics);

	spin_lock_init(&haptics->bus_lock);
	mutex_init(&haptics->mutex);

	if (haptics->dt_op_mode == DA7280_PWM_MODE) {
		/* Get pwm and regulatot for haptics device */
		haptics->pwm_dev = devm_pwm_get(&client->dev, NULL);
		if (IS_ERR(haptics->pwm_dev)) {
			dev_err(dev, "%s: failed to get PWM device\n",
				__func__);
			return PTR_ERR(haptics->pwm_dev);
		}

		/*
		 * FIXME: pwm_apply_args() should be removed when switching to
		 * the atomic PWM API.
		 */
		pwm_apply_args(haptics->pwm_dev);

		/* Check PWM Period, it must be in 10k ~ 250kHz */
		period2freq = 1000000 / pwm_get_period(haptics->pwm_dev);
		if (period2freq < DA7280_MIN_PWM_FREQ_KHZ ||
		    period2freq > DA7280_MAX_PWM_FREQ_KHZ) {
			dev_err(dev, "%s: Not supported PWM frequency(%d)\n",
					__func__, period2freq);
			return -EINVAL;
		}
	}


	/* Init work queue */
	haptics->hpri_wq = alloc_workqueue("da7280_wq",  WQ_HIGHPRI |
						WQ_MEM_RECLAIM | WQ_UNBOUND, 1);
	if (!haptics->hpri_wq) {
		dev_err(dev, "%s: Workqueue alloc failed\n", __func__);
		return -ENOMEM;
	}

	hrtimer_init(&haptics->duration_timer,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	haptics->duration_timer.function = da7280_haptic_duration_timer;
	hrtimer_init(&haptics->disable_timer,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	haptics->disable_timer.function = da7280_haptic_disable_timer;

	INIT_WORK(&haptics->work_handler, da7280_haptic_work_handler);

	haptics->client = client;
	i2c_set_clientdata(client, haptics);

	haptics->regmap =
		devm_regmap_init_i2c(client, &da7280_haptic_regmap_config);
	if (IS_ERR(haptics->regmap)) {
		error = PTR_ERR(haptics->regmap);
		dev_err(dev, "%s: Failed to allocate register map : %d\n",
				__func__, error);
		return error;
	}

	error = devm_request_threaded_irq(dev, client->irq, NULL,
					  da7280_irq_handler,
					  IRQF_ONESHOT|IRQF_TRIGGER_LOW,
					  "da7280-haptics", haptics);
	if (error != 0) {
		dev_err(dev, "%s: Failed to request IRQ : %d\n",
				__func__, client->irq);
		return error;
	}

	error = da7280_init(haptics);
	if (error) {
		dev_err(dev, "%s: failed to initialize device\n", __func__);
		return error;
	}

	/* Initialize input device for haptic device */
	input_dev = devm_input_allocate_device(dev);
	if (!input_dev) {
		dev_err(dev, "%s: failed to allocate input device\n", __func__);
		return -ENOMEM;
	}

	input_dev->name = "da7280-haptic";
	input_dev->dev.parent = client->dev.parent;
	input_dev->open = da7280_haptic_open;
	input_dev->close = da7280_haptic_close;
	input_set_drvdata(input_dev, haptics);
	haptics->input_dev = input_dev;

	input_set_capability(haptics->input_dev, EV_FF, FF_PERIODIC);
	input_set_capability(haptics->input_dev, EV_FF, FF_CUSTOM);
	input_set_capability(haptics->input_dev, EV_FF, FF_CONSTANT);
	input_set_capability(haptics->input_dev, EV_FF, FF_GAIN);

	error = input_ff_create(haptics->input_dev, DA7280_FF_EFFECT_COUNT_MAX);
	if (error) {
		dev_err(dev, "%s: create FF input device failed(%d)\n",
			__func__, error);
		return error;
	}

	ff = input_dev->ff;
	ff->upload = da7280_haptics_upload_effect;
	ff->playback = da7280_haptics_playback;
	ff->set_gain = da7280_haptics_set_gain;
	ff->erase = da7280_haptics_erase;

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "%s: failed to register input device\n", __func__);
		return error;
	}

	error = devm_device_add_group(dev, &da7280_attr_group);
	if (error) {
		dev_err(dev, "%s: Failed to create sysfs attributes: %d\n",
			__func__, error);
	}

	return error;
}

static int __maybe_unused da7280_suspend(struct device *dev)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	int error = 0;

	mutex_lock(&haptics->input_dev->mutex);

	da7280_haptic_disable(haptics);

	error = da7280_update_bits(haptics,
				   DA7280_TOP_CTL1,
				   DA7280_STANDBY_EN_MASK, 0);
	if (error) {
		dev_err(haptics->dev,
			"%s: I2C error : %d\n", __func__, error);
	}

	mutex_unlock(&haptics->input_dev->mutex);
	return error;
}

static int __maybe_unused da7280_resume(struct device *dev)
{
	struct da7280_haptic *haptics = dev_get_drvdata(dev);
	int error = 0;

	mutex_lock(&haptics->input_dev->mutex);

	error = da7280_update_bits(haptics,
				   DA7280_TOP_CTL1,
				   DA7280_STANDBY_EN_MASK,
				   DA7280_STANDBY_EN_MASK);
	if (error) {
		dev_err(haptics->dev,
			"%s: i2c error : %d\n", __func__, error);
	}

	mutex_unlock(&haptics->input_dev->mutex);
	return error;
}

static const struct of_device_id da7280_of_match[] = {
	{ .compatible = "dlg,da7280", },
	{ }
};
MODULE_DEVICE_TABLE(of, da7280_of_match);

static const struct i2c_device_id da7280_i2c_id[] = {
	{ "da7280", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, da7280_i2c_id);

static SIMPLE_DEV_PM_OPS(da7280_pm_ops,
		 da7280_suspend, da7280_resume);

static struct i2c_driver da7280_driver = {
	.driver		= {
		.name	= "da7280",
		.of_match_table = of_match_ptr(da7280_of_match),
		.pm	= &da7280_pm_ops,
	},
	.probe	= da7280_probe,
	.id_table	= da7280_i2c_id,
};
module_i2c_driver(da7280_driver);

MODULE_DESCRIPTION("DA7280 haptics driver");
MODULE_AUTHOR("Roy Im <Roy.Im.Opensource@diasemi.com>");
MODULE_LICENSE("GPL");
