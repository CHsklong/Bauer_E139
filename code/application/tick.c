
#include <stdint.h>
#include "board.h"
#include "tick.h"
#include "mcdrv.h"
uint32_t ticks = 0;

uint32_t getTicks()
{
  // ticks = ((0xffffffff-BOARD_TIMER->CHANNEL[kPIT_Chnl_0].CVAL)>>14);
  ticks = board_get_ms_tick();
  return ticks;
}
uint32_t delayTicks;
uint8_t expireTicks(uint32_t oldTicks, uint32_t period) // period ms
{
  uint32_t curTicks = getTicks();
  delayTicks = ((curTicks >= oldTicks) ? (curTicks - oldTicks) : ((0xffffffff) - oldTicks + curTicks));
  if (delayTicks >= period)
    return true;

  return false;
}
