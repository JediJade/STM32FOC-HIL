/*
 * The three duties the drive is putting out, measured on this board's pins.
 *
 * Each phase goes into a timer in PWM input mode: the hardware latches the
 * period into CCR1 and the high time into CCR2 on every cycle, with no
 * interrupt and no CPU. Reading a duty is reading two registers.
 *
 * That matters at 20 kHz. Catching every edge with an interrupt would be
 * 120,000 interrupts a second on a chip that also has to step a motor model
 * every 50 us; the hardware does it for free instead.
 *
 * WHAT A DUTY MEANS HERE. The drive's PWM is centre-aligned with ARR 1800
 * at 72 MHz, so one period is 3600 timer ticks either side. Its SVPWM wrote
 * a compare value of `900 + (v * 900 >> 15)`, which makes the fraction of
 * the period the pin spends high exactly `0.5 + v/65536`. Inverting that
 * gives the phase voltage back in the same Q15 the drive used.
 *
 * DEAD TIME. The pin is high for one dead-time less than the compare value
 * asked for - 1.000 us of every 50, measured on the bench. The drive did
 * not intend that 2 per cent; it is the hardware protecting the bridge. The
 * voltage handed to the model adds it back, so that the model sees what the
 * controller asked for rather than what the gate driver allowed. Set
 * `pwmin_deadtime(0)` to see the difference.
 */
#ifndef PWMIN_H
#define PWMIN_H

#include <stdint.h>
#include "foc.h"

/* TIM2 CH1 on PA0, TIM3 CH1 on PA6, TIM4 CH1 on PB6 - phases A, B, C. */
void pwmin_init(void);

/* How many timer ticks to add back for the dead-time, per period.
   72 at 72 MHz is the 1.000 us the drive asks its bridge for. */
void pwmin_deadtime(uint16_t ticks);

/* The three phase voltages, Q15, as the drive's SVPWM meant them: the
   common mode the drive added is removed again here, because a motor's star
   point cannot see it either. */
void pwmin_phase_voltages(int16_t *va, int16_t *vb, int16_t *vc);

/* The same thing in the two-axis frame the model steps in. */
foc_ab pwmin_voltage(void);

/* 1 while all three phases are switching: a period measured on each within
   the last few milliseconds. 0 means the drive is not driving. */
int pwmin_live(void);

/* For the console: raw period and high time of one phase, in timer ticks. */
void pwmin_raw(int phase, uint16_t *period, uint16_t *high);

/* ---------------------------------------------------------- the bridge --
 *
 * The three low sides as well, watched rather than measured.
 *
 *   PB7  <- the drive's PB13   phase A low
 *   PB8  <- the drive's PB14   phase B low
 *   PB9  <- the drive's PB15   phase C low
 *
 * Sampled once per model tick, which is 20,000 times a second at an
 * arbitrary phase of the drive's own period. That is not fast enough to
 * catch an overlap that lasts a microsecond - a DMA sampler would be - but
 * it catches the one that matters most: a bridge told to hold both
 * transistors on, which is what a wrong polarity or a mis-set BDTR does,
 * and which destroys hardware in the time it takes to say so.
 *
 * A moment where both are LOW is not a fault: that is the dead-time, and
 * about one sample in fifty lands in it.
 */

/* From the model tick, once per period. */
void     pwmin_watch(void);

/* How many samples have caught a phase with both transistors on. Anything
   but zero means the drive would have shorted its own bus. */
uint32_t pwmin_overlaps(int phase);
uint32_t pwmin_overlaps_total(void);
void     pwmin_overlaps_clear(void);

/* 1 while the six pins are doing something: the drive is armed and
   switching. 0 once all six have been low for several milliseconds, which
   is what MOE = 0 looks like from outside the chip. */
int      pwmin_outputs_on(void);

#endif /* PWMIN_H */
