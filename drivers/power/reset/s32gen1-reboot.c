/*
 * NXP S32GEN1 reboot driver
 * Copyright 2018,2021 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <asm/system_misc.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <s32/s32-gen1/mc_me.h>
#include <s32/s32-gen1/rgm.h>

static const struct of_device_id s32gen1_reboot_of_match[] = {
	{ .compatible = "fsl,s32gen1-prstc", .data = (void *) 0 },
	{}
};

struct	s32gen1_reboot_priv {
	struct regmap *mc_me;
	struct regmap *mc_rgm;
};

static struct s32gen1_reboot_priv	s32gen1_reboot_priv = {0};

static int s32gen1_reboot(struct notifier_block *this, unsigned long mode,
			  void *cmd)
{
	unsigned long timeout;

	if (!s32gen1_reboot_priv.mc_rgm || !s32gen1_reboot_priv.mc_me)
		return 0;

	regmap_write(s32gen1_reboot_priv.mc_rgm,
		     MC_RGM_FRET, MC_RGM_FRET_VALUE);

	regmap_write(s32gen1_reboot_priv.mc_me,
		     MC_ME_MODE_CONF, MC_ME_MODE_CONF_FUNC_RST);

	regmap_write(s32gen1_reboot_priv.mc_me,
		     MC_ME_MODE_UPD, MC_ME_MODE_UPD_UPD);

	regmap_write(s32gen1_reboot_priv.mc_me,
		     MC_ME_CTL_KEY, MC_ME_CTL_KEY_KEY);

	regmap_write(s32gen1_reboot_priv.mc_me,
		     MC_ME_CTL_KEY, MC_ME_CTL_KEY_INVERTEDKEY);

	timeout = jiffies + HZ;
	while (time_before(jiffies, timeout))
		cpu_relax();

	return 0;
}

static struct notifier_block s32gen1_reboot_nb = {
	.notifier_call = s32gen1_reboot,
	.priority = 192,
};

static int s32gen1_reboot_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int err;

	s32gen1_reboot_priv.mc_me =
		syscon_regmap_lookup_by_compatible("fsl,s32gen1-mc_me");
	if (IS_ERR(s32gen1_reboot_priv.mc_me)) {
		dev_err(dev, "Cannot map 'MC_ME' resource\n");
		return -ENODEV;
	}

	s32gen1_reboot_priv.mc_rgm =
		syscon_regmap_lookup_by_compatible("fsl,s32gen1-rgm");
	if (IS_ERR(s32gen1_reboot_priv.mc_rgm)) {
		dev_err(dev, "Cannot map 'RGM' resource\n");
		return -ENODEV;
	}

	err = register_restart_handler(&s32gen1_reboot_nb);
	if (err) {
		dev_err(dev, "Failed to register handler\n");
		return err;
	}

	return 0;
}

static struct platform_driver s32gen1_reboot_driver = {
	.probe = s32gen1_reboot_probe,
	.driver = {
		.name = "s32gen1-power-reset",
		.of_match_table = s32gen1_reboot_of_match,
	},
};

static int __init s32gen1_reboot_init(void)
{
	return platform_driver_register(&s32gen1_reboot_driver);
}
device_initcall(s32gen1_reboot_init);
