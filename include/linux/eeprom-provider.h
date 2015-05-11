/*
 * EEPROM framework provider.
 *
 * Copyright (C) 2015 Srinivas Kandagatla <srinivas.kandagatla@linaro.org>
 * Copyright (C) 2013 Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _LINUX_EEPROM_PROVIDER_H
#define _LINUX_EEPROM_PROVIDER_H

struct eeprom_device;

struct eeprom_config {
	struct device		*dev;
	const char		*name;
	int			id;
	struct module		*owner;
};

#if IS_ENABLED(CONFIG_EEPROM)

struct eeprom_device *eeprom_register(struct eeprom_config *cfg);
int eeprom_unregister(struct eeprom_device *eeprom);

#else

static inline struct eeprom_device *eeprom_register(struct eeprom_config *cfg)
{
	return NULL;
}
static inline int eeprom_unregister(struct eeprom_device *eeprom)
{
	return -ENOSYS;
}

#endif /* CONFIG_EEPROM */

#endif  /* ifndef _LINUX_EEPROM_PROVIDER_H */
