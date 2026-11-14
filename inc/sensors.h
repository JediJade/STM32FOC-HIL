/*
 * What the drive reads back: the sensor signals, on real pins.
 *
 * Today that is the three Hall outputs. The throttle voltage and the phase
 * current want a DAC, which this part does not have - an SPI DAC or a PWM
 * and an RC filter goes here when stage two arrives.
 */
#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

/* PB12, PB13, PB14 push-pull, held low until the first update. */
void    sensors_init(void);

/* How much of a turn one Hall sensor stays on for, in angle units: 32768
   is a latch, and a 3144 is nearer 27000. The drive's `hwid` parameter is
   the same number, and the two should be set to match. */
void    sensors_hall_width(uint16_t width);

/* From the control tick: read the model's rotor and drive the three pins. */
void    sensors_update(void);

/* What was driven out last, for the console. */
uint8_t sensors_hall_code(void);

/*
 * Drive a fixed pattern instead of the model's, for checking the wiring:
 * 0..7 holds those three bits, -1 gives the model back.
 *
 * Bringing up a rig, the first question is never "is the angle right" but
 * "is the wire there". A known pattern on this side and a read on the other
 * answers it in one line, and names which of the three is missing.
 */
void    sensors_force(int code);
int     sensors_forced(void);

#endif /* SENSORS_H */
