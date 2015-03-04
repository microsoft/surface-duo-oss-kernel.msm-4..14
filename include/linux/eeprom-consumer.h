/*
 * EEPROM framework consumer.
 *
 * Copyright (C) 2015 Srinivas Kandagatla <srinivas.kandagatla@linaro.org>
 * Copyright (C) 2013 Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _LINUX_EEPROM_CONSUMER_H
#define _LINUX_EEPROM_CONSUMER_H

struct eeprom_cell;

struct eeprom_block {
	loff_t offset;
	size_t count;
};
#if IS_ENABLED(CONFIG_EEPROM)
struct eeprom_cell *eeprom_cell_get(const char *ename,
				    struct eeprom_block *blocks, int nblocks);
void eeprom_cell_put(struct eeprom_cell *cell);
char *eeprom_cell_read(struct eeprom_cell *cell, ssize_t *len);
int eeprom_cell_write(struct eeprom_cell *cell, const char *buf, ssize_t len);
#else

static inline struct eeprom_cell *eeprom_cell_get(const char *ename,
				    struct eeprom_block *blocks, int nblocks)
{
	return NULL;
}

static inline void eeprom_cell_put(struct eeprom_cell *cell)
{
}

static inline char *eeprom_cell_read(struct eeprom_cell *cell, ssize_t *len)
{
	return NULL;
}

static inline int eeprom_cell_write(struct eeprom_cell *cell,
				    const char *buf, ssize_t len)
{
	return -ENOSYS;
}
#endif /* CONFIG_EEPROM */

#if IS_ENABLED(CONFIG_EEPROM) && IS_ENABLED(CONFIG_OF)
struct eeprom_cell *of_eeprom_cell_get(struct device *dev, int index);
struct eeprom_cell *of_eeprom_cell_get_byname(struct device *dev,
					      const char *name);
#else
static inline struct eeprom_cell *of_eeprom_cell_get(
					struct device *dev, int index)
{
	return NULL;
}
static inline struct eeprom_cell *of_eeprom_cell_get_byname(struct device *dev,
							    const char *name)
{
	return NULL;
}
#endif
#endif  /* ifndef _LINUX_EEPROM_CONSUMER_H */
