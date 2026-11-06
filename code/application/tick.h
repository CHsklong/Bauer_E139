#ifndef _TICK_H_
#define _TICK_H_


uint32_t getTicks(void);
uint8_t expireTicks(uint32_t oldTicks, uint32_t period);

#endif
