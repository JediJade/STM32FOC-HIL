#include "stm32f103xb.h"
#include "sensors.h"
#include "vmotor.h"

/*
 * The signals a motor's sensors would have produced, driven onto pins.
 *
 *   PB12 -> the drive's PB5   Hall A
 *   PB13 -> the drive's PB6   Hall B
 *   PB14 -> the drive's PB7   Hall C
 *
 * The drive pulls those three up and reads them as open-collector switches,
 * so push-pull outputs here drive them cleanly in both directions. A real
 * 3144 pulls down and lets go; this does the same thing, faster.
 *
 * The code comes from `vmotor_hall()` - the drive firmware's own function,
 * in the file copied across unchanged. When the drive is set to read its
 * pins (`hsrc 1`) instead of its internal model, it gets the same three
 * bits it would have computed for itself. That is the whole trick: the
 * answer does not change, only the path it takes to arrive.
 */

#define HALL_SHIFT   12u          /* PB12, PB13, PB14 */
#define HALL_MASK    (7u << HALL_SHIFT)

static uint16_t hall_width = 32768u;    /* half a turn: a latch-like sensor */
static uint8_t  last_code;

void sensors_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

  /*
   * PB12..PB14 output push-pull, 2 MHz - the edges only have to be clean at
   * the drive's 20 kHz sampling, not fast.
   *
   * CRH holds PB8..PB15, four bits each, so PB12 starts at bit 16 and PB14
   * ends at bit 27: the mask is 0x0FFF0000. The first version of this line
   * used 0x00FFF000, which is PB11..PB13 - one pin to the right. PB12 and
   * PB13 came out as outputs by luck, PB14 stayed an input, and the drive
   * read its pull-up as a Hall sensor stuck high. The bench found it in
   * five seconds with `set hall`; reading the line did not.
   */
  GPIOB->CRH = (GPIOB->CRH & ~0x0FFF0000uL) | 0x02220000uL;
  GPIOB->BSRR = HALL_MASK << 16;          /* all low until the model says */
  last_code = 0u;
}

void sensors_hall_width(uint16_t width)
{
  hall_width = width;
}

static int forced = -1;

void sensors_force(int code)
{
  forced = (code < 0) ? -1 : (code & 7);
}

int sensors_forced(void)
{
  return forced;
}

void sensors_update(void)
{
  uint8_t code = (forced >= 0) ? (uint8_t)forced : vmotor_hall(hall_width);
  uint32_t set = ((uint32_t)code & 7u) << HALL_SHIFT;
  uint32_t clear = (~(uint32_t)code & 7u) << (HALL_SHIFT + 16u);

  /* One write, so the three pins change together. A drive that caught them
     mid-change would see a state its calibration never recorded and count
     it as an error - which is a real fault mode, but not one this board
     should be inventing. */
  GPIOB->BSRR = set | clear;
  last_code = code;
}

uint8_t sensors_hall_code(void)
{
  return last_code;
}
