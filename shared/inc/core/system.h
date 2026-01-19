#ifndef SYSTEM_H
#define SYSTEM_H

#include "common-defines.h"

#define SYSTICK_FREQ (1000)
#define CPU_FREQ (48000000)

void system_setup(void);
uint64_t system_get_ticks(void);
void system_delay(uint64_t milis);
void system_teardown(void);

#endif // SYSTEM_H
