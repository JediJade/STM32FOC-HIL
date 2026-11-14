/*
 * A line console on the USB CDC port, in the drive's dialect.
 *
 * Replies are `word name value name value ...`, which is what the drive
 * answers and what `studio/hil.py` and the FOC studio both parse. One
 * terminal reads both boards; one habit covers both.
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

void console_init(void);
void console_poll(void);

/* From main.c, for the state line. */
uint32_t millis(void);
uint32_t hil_model_ticks(void);
uint32_t hil_late_ticks(void);

#endif /* CONSOLE_H */
