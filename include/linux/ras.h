#ifndef __RAS_H__
#define __RAS_H__

#include <asm/errno.h>
#include <linux/uuid.h>

#ifdef CONFIG_DEBUG_FS
int ras_userspace_consumers(void);
void ras_debugfs_init(void);
int ras_add_daemon_trace(void);
#else
static inline int ras_userspace_consumers(void) { return 0; }
static inline void ras_debugfs_init(void) { return; }
static inline int ras_add_daemon_trace(void) { return 0; }
#endif

#ifdef CONFIG_RAS
void log_non_standard_event(const uuid_le *sec_type,
			    const uuid_le *fru_id, const char *fru_text,
			    const u8 sev, const u8 *err, const u32 len);
#else
static void log_non_standard_event(const uuid_le *sec_type,
				   const uuid_le *fru_id, const char *fru_text,
				   const u8 sev, const u8 *err,
				   const u32 len) { return; }
#endif

#endif /* __RAS_H__ */
