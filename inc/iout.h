/*
 * The model's two phase currents, as voltages the drive can measure.
 *
 * Stage one sent the drive an angle and nothing else, so the drive went on
 * regulating the current of its own model - a model nothing on this bench
 * could load. Put a brake on the rotor here and the drive's current did not
 * move, because the drive's rotor was not the one being braked.
 *
 * This closes the other half. TIM1 carries two PWM outputs at 300 kHz whose
 * duty is the phase current; an RC filter on each turns that into a voltage,
 * and the drive's ADC reads them where a shunt amplifier would have been.
 * The drive then regulates a current that this board decides.
 *
 *   HIL PA8 ---[ 10k ]---+--- drive PA0      (phase a)
 *                        |
 *                      [10nF]
 *                        |
 *                       GND
 *
 *   HIL PA9 ---[ 10k ]---+--- drive PA1      (phase b)
 *                        |
 *                      [10nF]
 *                        |
 *                       GND
 *
 * Why those values: 300 kHz against a 100 us time constant leaves under 1 %
 * of carrier ripple, and passes everything up to about 1.6 kHz - far above
 * the electrical frequency of anything this bench turns. The capacitor also
 * sits where the drive's sample-and-hold wants one, so the 10k does not cost
 * the conversion any accuracy.
 *
 * Mid-rail is zero current. A full-scale Q15 current is the whole swing, so
 * the drive sees 1.65 V +- 1.65 V and nothing can ask either pin to leave
 * the supply rails.
 */
#ifndef IOUT_H
#define IOUT_H

#include <stdint.h>

/* TIM1 becomes the carrier as well as the model's tick: 240 steps at
   300 kHz, with the update event - and so this board's 20 kHz tick -
   every fifteenth period. 240 x 15 = 3600, which is 20 kHz exactly. */
#define IOUT_STEPS   240u

void iout_init(void);

/* Off parks both pins at mid-rail, which the drive reads as zero current. */
void iout_enable(int on);
int  iout_enabled(void);

/* Q15 phase currents in, duty out. Called once per model tick. */
void iout_write(int16_t ia, int16_t ib);

/* What went out last, in steps, for the console to report. */
uint16_t iout_duty_a(void);
uint16_t iout_duty_b(void);

/* How many steps away from mid-rail a full-scale Q15 current is driven.
   Lower it to make the bench's current look smaller to the drive. */
void     iout_set_span(int32_t steps);
int32_t  iout_span(void);

#endif /* IOUT_H */
