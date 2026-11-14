/*
 * The HIL board: a motor that is not there, wired to a drive that cannot
 * tell the difference.
 *
 * A second Blue Pill, the same part as the drive, at the same 72 MHz. Once
 * every 50 us - the drive's own control period - it:
 *
 *   1. reads the three PWM duties the drive is putting out, latched by
 *      hardware on its own pins (pwmin.c);
 *   2. turns them back into the two-axis voltage the drive's SVPWM meant
 *      and steps the motor model one period (vmotor.c, the drive's file,
 *      copied without a change);
 *   3. drives the Hall pins the drive reads (sensors.c).
 *
 * The drive is then set to `hsrc 1` and `sens 1`: it stops using its own
 * internal model's sensors and reads the pins instead. Its angle now comes
 * off a wire, through a GPIO, from another chip - which is the whole point.
 * Everything in its list of "never connected" is on the far side of a pin.
 *
 * WHY THE SAME PART. The model steps in a period of exactly the same
 * length, with the same arithmetic, on the same clock. Anything the two
 * boards disagree about is then a real disagreement and not a difference
 * in the hardware they run on.
 *
 * PINS
 *
 *   PA0   <- the drive's PA8    phase A     TIM2 CH1, PWM input
 *   PA6   <- the drive's PA9    phase B     TIM3 CH1, PWM input
 *   PB6   <- the drive's PA10   phase C     TIM4 CH1, PWM input
 *   PB12  -> the drive's PB5    Hall A
 *   PB13  -> the drive's PB6    Hall B
 *   PB14  -> the drive's PB7    Hall C
 *   GND   -- the drive's GND    the two boards must share it
 *   PC13      the LED: on while the drive is switching
 *   PA11/PA12 USB: the console, the same CDC device the drive presents
 *
 * NOT WIRED YET: the current back into the drive's PA0, and the throttle
 * into its PA2. Both want a DAC, which this part has not got. An MCP4921
 * on SPI1 settles in 8 us, which is well inside a control period; a PWM
 * and an RC filter would take about a hundred.
 */

#include "stm32f103xb.h"
#include "usb_cdc.h"
#include "iout.h"
#include "pwmin.h"
#include "sensors.h"
#include "vmotor.h"
#include "foc.h"
#include "console.h"

#define LED_PIN   13U     /* PC13, active low */

static volatile uint32_t tick_ms;
static volatile uint32_t model_ticks;
static volatile uint32_t late_ticks;

uint32_t millis(void)
{
  return tick_ms;
}

uint32_t hil_model_ticks(void)
{
  return model_ticks;
}

uint32_t hil_late_ticks(void)
{
  return late_ticks;
}

void SysTick_Handler(void)
{
  tick_ms++;
}

/* usb_cdc.c waits in milliseconds while the host enumerates. The drive's
   main.c owns this function there; it owns it here too. */
void delay_ms(uint32_t ms)
{
  uint32_t start = tick_ms;

  while ((tick_ms - start) < ms) {
  }
}

/* HSE 8 MHz -> PLL x9 -> 72 MHz, APB1 36 MHz, APB2 72 MHz.
   Identical to the drive's, because it is the drive's. */
static void clock_init(void)
{
  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

  RCC->CR |= RCC_CR_HSEON;
  while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
  }

  RCC->CFGR = RCC_CFGR_PLLSRC
            | RCC_CFGR_PLLMULL9
            | RCC_CFGR_HPRE_DIV1
            | RCC_CFGR_PPRE1_DIV2
            | RCC_CFGR_PPRE2_DIV1;

  RCC->CR |= RCC_CR_PLLON;
  while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {
  }

  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
  }
}

static void led_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
  GPIOC->CRH = (GPIOC->CRH & ~(0xFUL << (4U * (LED_PIN - 8U))))
             | (0x2UL << (4U * (LED_PIN - 8U)));
  GPIOC->BSRR = (1UL << LED_PIN);                   /* off */
}

static void led(int on)
{
  GPIOC->BSRR = on ? (1UL << (LED_PIN + 16U)) : (1UL << LED_PIN);
}

/*
 * Pull D+ low briefly so the host notices a disconnect and enumerates
 * again. The drive firmware carries the same function and the same comment,
 * because the same thing happens to both boards: one just flashed over SWD
 * keeps the host's old session and never comes back as a new device.
 */
static void usb_force_reenumerate(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

  GPIOA->CRH &= ~(0xFUL << (4U * (12U - 8U)));
  GPIOA->CRH |=  (0x2UL << (4U * (12U - 8U)));   /* output push-pull 2 MHz */
  GPIOA->BSRR = (1UL << (12U + 16U));            /* drive PA12 low */
  delay_ms(100U);

  GPIOA->CRH &= ~(0xFUL << (4U * (12U - 8U)));
  GPIOA->CRH |=  (0x4UL << (4U * (12U - 8U)));   /* back to input floating */
  delay_ms(10U);
}

/*
 * TIM1 keeps the model's period and carries the current outputs.
 *
 * It used to count to 3600 once per tick. It now counts to 240 fifteen
 * times, which is the same 20 kHz tick - the update event only fires when
 * the repetition counter runs out - and gives a 300 kHz carrier on CH1
 * and CH2 for the two currents. Whole periods either way: 240 x 15 = 3600,
 * so the tick is not a rounded-off 20 kHz, it is the same 20 kHz.
 */
static void tick_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  TIM1->PSC = 0U;
  TIM1->ARR = IOUT_STEPS - 1U;
  TIM1->RCR = 15U - 1U;
  iout_init();
  TIM1->DIER = TIM_DIER_UIE;
  TIM1->CR1 = TIM_CR1_CEN;
  NVIC_SetPriority(TIM1_UP_IRQn, 0);       /* above USB */
  NVIC_EnableIRQ(TIM1_UP_IRQn);
}

/*
 * The whole bench, once every 50 us.
 *
 * It must finish inside the period or the model falls behind the drive, so
 * the overrun is counted rather than hoped about: `late` in the state line
 * is the number of ticks that arrived while the last one was still running.
 */
void TIM1_UP_IRQHandler(void)
{
  foc_ab v;

  if ((TIM1->SR & TIM_SR_UIF) == 0U) {
    return;
  }
  TIM1->SR = (uint16_t)~TIM_SR_UIF;

  static volatile uint8_t busy;
  if (busy != 0U) {
    late_ticks++;
    return;
  }
  busy = 1U;

  pwmin_watch();                  /* the six pins, before anything else */
  v = pwmin_voltage();
  vmotor_step(v.alpha, v.beta);
  sensors_update();

  {
    /* The currents go out as they are measured, in the same tick that
       produced them: a current the drive reads a period late is a current
       measured at an angle the rotor has already left. */
    int16_t ia, ib;

    vmotor_phase_currents(&ia, &ib);
    iout_write(ia, ib);
  }
  model_ticks++;

  busy = 0U;
}

int main(void)
{
  uint32_t next_blink = 0U;
  int lit = 0;

  clock_init();
  SysTick_Config(72000000U / 1000U);
  led_init();

  vmotor_reset();
  pwmin_init();
  sensors_init();
  console_init();
  usb_force_reenumerate();
  usb_cdc_init();

  tick_init();

  while (1) {
    console_poll();

    /*
     * The LED says what the board is doing without a terminal: solid while
     * the drive is switching, a slow blink while it is not. A board that
     * has stopped stepping its model shows neither, because the blink is
     * driven from the model's own tick count.
     */
    if (pwmin_live() != 0) {
      led(1);
    } else if (model_ticks > next_blink) {
      next_blink = model_ticks + 10000U;        /* 500 ms of model time */
      lit = !lit;
      led(lit);
    }
  }
}
