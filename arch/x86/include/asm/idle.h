#ifndef _ASM_X86_IDLE_H
#define _ASM_X86_IDLE_H

void enter_idle(void);
void exit_idle(void);

void c1e_remove_cpu(int cpu);

#endif /* _ASM_X86_IDLE_H */
