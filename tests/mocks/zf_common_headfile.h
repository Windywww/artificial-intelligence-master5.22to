#ifndef ZF_COMMON_HEADFILE_H
#define ZF_COMMON_HEADFILE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t unused;
} seekfree_assistant_oscilloscope_struct;

extern volatile float time_line;
void system_delay_ms(uint32_t ms);

#endif
