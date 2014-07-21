#ifndef __PERF_SVGHELPER_H
#define __PERF_SVGHELPER_H

#include "types.h"

extern void open_svg(const char *filename, int cpus, int rows, u64 start, u64 end);
extern void svg_box(int Yslot, u64 start, u64 end, const char *type);
extern void svg_sample(int Yslot, int cpu, u64 start, u64 end);
extern void svg_waiting(int Yslot, u64 start, u64 end);
extern void svg_cpu_box(int cpu, u64 max_frequency, u64 turbo_frequency);
void svg_gpu_box(int cpu);


extern void svg_process(int cpu, u64 start, u64 end, const char *type, const char *name);
extern void svg_cstate(int cpu, u64 start, u64 end, int type);
extern void svg_pstate(int cpu, u64 start, u64 end, u64 freq);
void svg_flip(int cpu, u64 start, u64 end, int crtc, const void *obj);
void svg_fence(int Yslot, u64 start, u64 end, u32 fence, int ret);


extern void svg_time_grid(void);
extern void svg_legenda(void);
extern void svg_wakeline(u64 start, int row1, int row2);
extern void svg_partial_wakeline(u64 start, int row1, char *desc1, int row2, char *desc2);
extern void svg_interrupt(u64 start, int row);
extern void svg_text(int Yslot, u64 start, const char *text);
extern void svg_close(void);

extern int svg_page_width;

#endif /* __PERF_SVGHELPER_H */
