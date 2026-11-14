#include "stm32f103xb.h"
#include "pwmin.h"

/*
 * One timer per phase, each in PWM input mode on its channel 1 pin:
 *
 *   TIM2 CH1  PA0   phase A
 *   TIM3 CH1  PA6   phase B
 *   TIM4 CH1  PB6   phase C
 *
 * PWM input mode wires one input to two channels inside the timer: channel
 * 1 captures on the rising edge and resets the counter, channel 2 captures
 * on the falling edge. After an edge pair, CCR1 holds the period and CCR2
 * holds the high time, both in timer ticks at 72 MHz.
 */

#define DEAD_DEFAULT   72u          /* 1.000 us at 72 MHz */
#define STALE_PERIODS  200u         /* ~10 ms of no edges = not driving */

static TIM_TypeDef *const tim[3] = { TIM2, TIM3, TIM4 };
static uint16_t dead = DEAD_DEFAULT;
static uint16_t last_period[3];
static uint16_t last_high[3];
static uint16_t stale[3];

static void pwm_input(TIM_TypeDef *t)
{
  t->PSC = 0u;
  t->ARR = 0xFFFFu;

  /* CC1 on TI1 (rising, resets the counter), CC2 on TI1 (falling). */
  t->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos)     /* CC1S = 01: IC1 on TI1 */
           | (2u << TIM_CCMR1_CC2S_Pos);    /* CC2S = 10: IC2 on TI1 */
  t->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;

  /* Slave mode: reset on TI1FP1, so CCR1 is a whole period every time. */
  t->SMCR = (5u << TIM_SMCR_TS_Pos) | (4u << TIM_SMCR_SMS_Pos);
  t->CR1  = TIM_CR1_CEN;
}

void pwmin_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

  /*
   * All six inputs are pulled DOWN, not left floating.
   *
   * Floating was the obvious choice - the drive drives these pins push-pull
   * and has no trouble overriding a pull-down - and it was wrong. With
   * nothing connected the pins pick up whatever is in the air: the first
   * run of this firmware counted 84,642 overlaps and had the model spinning
   * at a random speed, from six pins reading noise.
   *
   * Pulled down, an unwired bench reads six zeros, which is exactly what a
   * bridge with its outputs off looks like. The rig says "nothing is
   * driving" instead of inventing a fault.
   */
  GPIOA->CRL = (GPIOA->CRL & ~0x0F00000FuL) | 0x08000008uL;   /* PA0, PA6 */
  GPIOA->BRR = (1uL << 0) | (1uL << 6);                       /* pull down */

  GPIOB->CRL = (GPIOB->CRL & ~0xFF000000uL) | 0x88000000uL;   /* PB6, PB7 */
  GPIOB->CRH = (GPIOB->CRH & ~0x000000FFuL) | 0x00000088uL;   /* PB8, PB9 */
  GPIOB->BRR = (1uL << 6) | (1uL << 7) | (1uL << 8) | (1uL << 9);

  for (int i = 0; i < 3; i++) {
    pwm_input(tim[i]);
    last_period[i] = 0u;
    last_high[i]   = 0u;
    stale[i]       = STALE_PERIODS;
  }
}

void pwmin_deadtime(uint16_t ticks)
{
  dead = ticks;
}

/* Read one phase, keeping the last good pair if no new one has arrived. */
static void sample(int i)
{
  TIM_TypeDef *t = tim[i];
  uint16_t period;
  uint16_t high;

  /*
   * A capture register keeps its last value for ever. Reading it says what
   * the pin was doing, not whether it is still doing it - which is how the
   * first version of this came to report a live bridge several seconds
   * after the drive had been disarmed. CC1IF is the flag that means "a new
   * capture has landed since you last looked", and reading CCR1 clears it.
   */
  if ((t->SR & TIM_SR_CC1IF) == 0u) {
    if (stale[i] < STALE_PERIODS) {
      stale[i]++;
    }
    return;
  }
  period = (uint16_t)t->CCR1;
  high   = (uint16_t)t->CCR2;

  /*
   * A plausible period is the only thing worth believing. At 20 kHz it is
   * 3600 ticks; anything outside a wide band around that is a glitch, a
   * first capture after power-on, or a pin that is not switching.
   */
  if ((period > 1800u) && (period < 14400u) && (high <= period)) {
    last_period[i] = period;
    last_high[i]   = high;
    stale[i]       = 0u;
  } else if (stale[i] < STALE_PERIODS) {
    stale[i]++;
  }
}

void pwmin_raw(int phase, uint16_t *period, uint16_t *high)
{
  int i = phase & 3;

  if (i > 2) { i = 2; }
  sample(i);
  *period = last_period[i];
  *high   = last_high[i];
}

/*
 * duty = (high + dead) / period, and the drive's SVPWM put the phase
 * voltage at `duty - 0.5` of full scale. 16 bits of headroom are enough:
 * high and period are both well under 2^16, so (high << 16) fits in 32.
 */
static int16_t phase_voltage(int i)
{
  uint32_t period = last_period[i];
  uint32_t high;
  int32_t v;

  if ((period == 0u) || (stale[i] >= STALE_PERIODS)) {
    return 0;
  }
  high = (uint32_t)last_high[i] + (uint32_t)dead;
  if (high > period) {
    high = period;
  }
  v = (int32_t)((high << 16) / period) - 32768;
  if (v > 32767)  { v = 32767; }
  if (v < -32768) { v = -32768; }
  return (int16_t)v;
}

void pwmin_phase_voltages(int16_t *va, int16_t *vb, int16_t *vc)
{
  int32_t v[3];
  int32_t common;

  for (int i = 0; i < 3; i++) {
    sample(i);
    v[i] = phase_voltage(i);
  }

  /*
   * Take the common mode back out. The drive's SVPWM deliberately added it
   * to win 15 per cent of voltage, and a motor's floating star point cannot
   * see it - so the model must not see it either, or every phase would
   * carry a third harmonic that is not there.
   */
  common = (v[0] + v[1] + v[2]) / 3;
  *va = (int16_t)(v[0] - common);
  *vb = (int16_t)(v[1] - common);
  *vc = (int16_t)(v[2] - common);
}

foc_ab pwmin_voltage(void)
{
  int16_t va, vb, vc;

  pwmin_phase_voltages(&va, &vb, &vc);
  (void)vc;                       /* foc_clarke assumes a + b + c == 0 */
  return foc_clarke(va, vb);
}

/* ---------------------------------------------------------- the bridge -- */

#define IDLE_TICKS   100u          /* 5 ms of six low pins = outputs off */

static uint32_t ovl[3];
static uint32_t idle;
static uint8_t  outputs;

void pwmin_watch(void)
{
  uint32_t a = GPIOA->IDR;
  uint32_t b = GPIOB->IDR;
  uint32_t hi;
  uint32_t lo;
  uint32_t both;

  /* PA0, PA6, PB6 are the high sides; PB7, PB8, PB9 the low ones. Two
     reads, one of each port, and the six bits line up. */
  hi = ((a >> 0) & 1u) | (((a >> 6) & 1u) << 1) | (((b >> 6) & 1u) << 2);
  lo = ((b >> 7) & 1u) | (((b >> 8) & 1u) << 1) | (((b >> 9) & 1u) << 2);

  both = hi & lo;
  if (both != 0u) {
    for (int i = 0; i < 3; i++) {
      if ((both & (1u << i)) != 0u) {
        ovl[i]++;
      }
    }
  }

  /*
   * All six low is either the dead-time - which lasts a microsecond and
   * never lines up on all three phases for long - or the outputs being off.
   * Several milliseconds of it is the second one.
   */
  if ((hi | lo) == 0u) {
    if (idle < IDLE_TICKS) {
      idle++;
    }
  } else {
    idle = 0u;
  }
  outputs = (uint8_t)((idle < IDLE_TICKS) ? 1u : 0u);
}

uint32_t pwmin_overlaps(int phase)
{
  int i = phase & 3;

  return (i < 3) ? ovl[i] : 0u;
}

uint32_t pwmin_overlaps_total(void)
{
  return ovl[0] + ovl[1] + ovl[2];
}

void pwmin_overlaps_clear(void)
{
  ovl[0] = 0u;
  ovl[1] = 0u;
  ovl[2] = 0u;
}

int pwmin_outputs_on(void)
{
  return (int)outputs;
}

int pwmin_live(void)
{
  for (int i = 0; i < 3; i++) {
    if (stale[i] >= STALE_PERIODS) {
      return 0;
    }
  }
  return 1;
}
