/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/qca6234_pwrif.h>

#define GPIO_WLAN_DEEP_SLEEP_N  89
#define GPIO_WLAN_DEEP_SLEEP_N_DRAGON  89
#define WLAN_RESET_OUT          1
#define WLAN_RESET              0

#define GPIO_BT_SLEEP_N  121
#define BT_RESET_OUT          1
#define BT_RESET              0

static const char *id = "WLAN";

/**
 * vos_chip_power_qca6234() - WLAN Power Up Seq for WCN1314 rev 2.0 on qca6234
 * @on - Turn WLAN ON/OFF (1 or 0)
 *
 * Power up/down WLAN by turning on/off various regs and asserting/deasserting
 * Power-on-reset pin. Also, put XO A0 buffer as slave to wlan_clk_pwr_req while
 * turning ON WLAN and vice-versa.
 *
 * This function returns 0 on success or a non-zero value on failure.
 */
int vos_chip_power_qca6234(int on)
{
	static char wlan_on;
	static const char *vregs_qwlan_name[] = {
		"8921_l10",
		"8921_s4",
	};
	
	static const int vregs_qwlan_val_min[] = {
		3300000,
		1800000,
	};
	static const int vregs_qwlan_val_max[] = {
		3300000,
		1800000,
	};
	static const int vregs_qwlan_peek_current[] = {
		150000,
		150000,
	};
		
	static struct regulator *vregs_qwlan[ARRAY_SIZE(vregs_qwlan_name)];
	static struct msm_xo_voter *wlan_clock;
	int ret, i, rc = 0;
	unsigned wlan_gpio_deep_sleep = GPIO_WLAN_DEEP_SLEEP_N;
	/*       */

	rc = gpio_request(128, "GPIO 10");
	if (rc) {
		pr_err("WLAN GPIO 10 request failed\n");
		goto fail;
	}
	rc = gpio_direction_output(128, 1);

	/* WLAN RESET and CLK settings */
	if (on && !wlan_on) {
		/*
		 * Program U12 GPIO expander pin IO1 to de-assert (drive 0)
		 * WLAN_EXT_POR_N to put WLAN in reset
		 */
		rc = gpio_request(wlan_gpio_deep_sleep, "WLAN_DEEP_SLEEP_N");
		if (rc) {
			pr_err("WLAN reset GPIO %d request failed\n",
					wlan_gpio_deep_sleep);
			goto fail;
		}
		rc = gpio_direction_output(wlan_gpio_deep_sleep,
				WLAN_RESET);
		if (rc < 0) {
			pr_err("WLAN reset GPIO %d set output direction failed",
					wlan_gpio_deep_sleep);
			goto fail_gpio_dir_out;
		}
		
		rc = gpio_request(GPIO_BT_SLEEP_N, "BT_DEEP_SLEEP_N");
		if (rc) {
			pr_err("BT reset GPIO %d request failed\n",
					GPIO_BT_SLEEP_N);
			goto fail;
		}
		rc = gpio_direction_output(GPIO_BT_SLEEP_N,
				BT_RESET);
		if (rc < 0) {
			pr_err("BT reset GPIO %d set output direction failed",
					GPIO_BT_SLEEP_N);
			goto fail_gpio_dir_out;
		}


		/* Configure TCXO to be slave to WLAN_CLK_PWR_REQ */
		if (wlan_clock == NULL) {
			wlan_clock = msm_xo_get(MSM_XO_TCXO_A0, id);
			if (IS_ERR(wlan_clock)) {
				pr_err("Failed to get TCXO_A0 voter (%ld)\n",
						PTR_ERR(wlan_clock));
				goto fail_gpio_dir_out;
			}
		}

		rc = msm_xo_mode_vote(wlan_clock, MSM_XO_MODE_PIN_CTRL);
		if (rc < 0) {
			pr_err("Configuring TCXO to Pin controllable failed"
					"(%d)\n", rc);
			goto fail_xo_mode_vote;
		}
	} else if (!on && wlan_on) {
		if (wlan_clock != NULL)
			msm_xo_mode_vote(wlan_clock, MSM_XO_MODE_OFF);
		gpio_set_value_cansleep(wlan_gpio_deep_sleep, WLAN_RESET);
		gpio_free(wlan_gpio_deep_sleep);
		gpio_set_value_cansleep(GPIO_BT_SLEEP_N, BT_RESET);
		gpio_free(GPIO_BT_SLEEP_N);
	}

	/* WLAN VREG settings */
	for (i = 0; i < ARRAY_SIZE(vregs_qwlan_name); i++) {
		if (on && !wlan_on)	{
			vregs_qwlan[i] = regulator_get(NULL,
					vregs_qwlan_name[i]);
			if (IS_ERR(vregs_qwlan[i])) {
				pr_err("regulator get of %s failed (%ld)\n",
						vregs_qwlan_name[i],
						PTR_ERR(vregs_qwlan[i]));
				rc = PTR_ERR(vregs_qwlan[i]);
				goto vreg_get_fail;
			}
			if (vregs_qwlan_val_min[i] || vregs_qwlan_val_max[i]) {
				rc = regulator_set_voltage(vregs_qwlan[i],
						vregs_qwlan_val_min[i],
						vregs_qwlan_val_max[i]);
				if (rc) {
					pr_err("regulator_set_voltage(%s) failed\n",
							vregs_qwlan_name[i]);
					goto vreg_fail;
				}
			}
			
			if (vregs_qwlan_peek_current[i]) {
				rc = regulator_set_optimum_mode(vregs_qwlan[i],
						vregs_qwlan_peek_current[i]);
				if (rc < 0)
					pr_err("vreg %s set optimum mode"
						" failed to %d (%d)\n",
						vregs_qwlan_name[i], rc,
						 vregs_qwlan_peek_current[i]);
			}
			rc = regulator_enable(vregs_qwlan[i]);
			if (rc < 0) {
				pr_err("vreg %s enable failed (%d)\n",
						vregs_qwlan_name[i], rc);
				goto vreg_fail;
			}
			
		} else if (!on && wlan_on) {

			if (vregs_qwlan_peek_current[i]) {
				/* For legacy reasons we pass 1mA current to
				 * put regulator in LPM mode.
				 */
				rc = regulator_set_optimum_mode(vregs_qwlan[i],
									 1000);
				if (rc < 0)
					pr_info("vreg %s set optimum mode"
								"failed (%d)\n",
						vregs_qwlan_name[i], rc);
				rc = regulator_set_voltage(vregs_qwlan[i], 0 ,
							vregs_qwlan_val_max[i]);
				if (rc)
					pr_err("regulator_set_voltage(%s)"
								"failed (%d)\n",
						vregs_qwlan_name[i], rc);

			}

			rc = regulator_disable(vregs_qwlan[i]);
			if (rc < 0) {
				pr_err("vreg %s disable failed (%d)\n",
						vregs_qwlan_name[i], rc);
				goto vreg_fail;
			}
			regulator_put(vregs_qwlan[i]);
		}
	}
	if (on) {
		gpio_set_value_cansleep(wlan_gpio_deep_sleep, WLAN_RESET_OUT);
		wlan_on = true;
		mdelay(6);
		gpio_set_value_cansleep(GPIO_BT_SLEEP_N, BT_RESET_OUT);
		
	}
	else
		wlan_on = false;
	return 0;

vreg_fail:
	regulator_put(vregs_qwlan[i]);
	
vreg_get_fail:
	i--;
	while (i >= 0) {
		ret = !on ? regulator_enable(vregs_qwlan[i]) :
			regulator_disable(vregs_qwlan[i]);
		if (ret < 0) {
			pr_err("vreg %s %s failed (%d) in err path\n",
					vregs_qwlan_name[i],
					!on ? "enable" : "disable", ret);
		}
		
		regulator_put(vregs_qwlan[i]);
		
		i--;
	}
	if (!on)
		goto fail;
fail_xo_mode_vote:
	msm_xo_put(wlan_clock);
fail_gpio_dir_out:
	gpio_free(wlan_gpio_deep_sleep);
fail:
	return rc;
}
EXPORT_SYMBOL(vos_chip_power_qca6234);

int WIFI_RF_status;
EXPORT_SYMBOL(WIFI_RF_status);

int BT_RF_status;
EXPORT_SYMBOL(BT_RF_status);

void qca6234_wifi_gpio(bool on)
{
	gpio_direction_output(GPIO_WLAN_DEEP_SLEEP_N, on);
}
EXPORT_SYMBOL(qca6234_wifi_gpio);

void qca6234_bt_gpio(bool on)
{
	gpio_direction_output(GPIO_BT_SLEEP_N, on);
}
EXPORT_SYMBOL(qca6234_bt_gpio);
