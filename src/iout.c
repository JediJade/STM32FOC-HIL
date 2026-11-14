#include "iout.h"
#include "stm32f103xb.h"

#define MID        (IOUT_STEPS / 2u)
#define SPAN_MAX   ((int32_t)MID - 2)

static uint8_t  on;
static int32_t  span = SPAN_MAX;
static uint16_t last_a = MID;
static uint16_t last_b = MID;

void iout_init(void)
{
  /*
   * PA8 and PA9 as TIM1 CH1 and CH2, alternate function push-pull.
   *
   * Only those two nibbles are touched: PA12 in the same register is the
   * USB pull-up, which main.c drives by hand to force a re-enumeration,
   * and PA13/PA14 are the debug port. Writing the whole register here
   * would take the board off SWD and leave no way back in.
   */
  /*
   * 2 MHz, not 50 MHz. These two pins sit next to PA11 and PA12, which are
   * the USB data lines, and a 300 kHz square wave with 50 MHz edges on them
   * was enough to knock the USB device off the bus within a second of the
   * drive starting to switch. The carrier needs none of that slew rate: at
   * 2 MHz the edge is still a hundredth of the shortest pulse.
   */
  GPIOA->CRH = (GPIOA->CRH & ~0xFFuL) | 0xAAuL;

  /* PWM mode 1 on both channels, with the compare registers preloaded so a
     write lands at the update event rather than part way through a pulse. */
  TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE
              | (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

  /* TIM1 is an advanced timer: its outputs stay off until MOE is set, which
     is a safety interlock on a part meant to drive a bridge. */
  TIM1->CCR1 = MID;
  TIM1->CCR2 = MID;
  TIM1->EGR  = TIM_EGR_UG;

  /*
   * On from the start, like the Hall outputs.
   *
   * This board's job is to be the motor. A motor's current is there
   * whether anyone asked for it, and a bench that has to be told to
   * produce one is a bench that silently does nothing after a reset -
   * which is exactly how an afternoon gets spent looking for a fault in
   * two resistors. Mid-rail is zero current, so a board with nothing
   * wired to these pins drives a steady 1.65 V into nothing.
   */
  iout_enable(1);
}

void iout_enable(int enable)
{
  on = (enable != 0) ? 1u : 0u;
  if (on == 0u) {
    /*
     * Off means the pins stop switching, not merely that they sit at
     * half duty. A 300 kHz carrier still running on two pins next to the
     * USB data lines is not "off" by any measure that matters, and
     * leaving it there makes every experiment that follows ambiguous.
     *
     * Mid-rail first, so that whatever charge the filter holds decays
     * towards the drive's zero rather than towards a rail, then the
     * outputs are released.
     */
    last_a = (uint16_t)MID;
    last_b = (uint16_t)MID;
    TIM1->CCR1 = MID;
    TIM1->CCR2 = MID;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
  } else {
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM1->BDTR |= TIM_BDTR_MOE;
  }
}

int iout_enabled(void)
{
  return (int)on;
}

static uint16_t duty_of(int16_t q15)
{
  int32_t d = (int32_t)MID + (((int32_t)q15 * span) / 16384);

  if (d < 1)                        { d = 1; }
  if (d > (int32_t)IOUT_STEPS - 1)  { d = (int32_t)IOUT_STEPS - 1; }
  return (uint16_t)d;
}

void iout_write(int16_t ia, int16_t ib)
{
  if (on == 0u) {
    return;
  }
  last_a = duty_of(ia);
  last_b = duty_of(ib);
  TIM1->CCR1 = last_a;
  TIM1->CCR2 = last_b;
}

uint16_t iout_duty_a(void)
{
  return last_a;
}

uint16_t iout_duty_b(void)
{
  return last_b;
}

void iout_set_span(int32_t steps)
{
  if (steps < 1)        { steps = 1; }
  if (steps > SPAN_MAX) { steps = SPAN_MAX; }
  span = steps;
}

int32_t iout_span(void)
{
  return span;
}
