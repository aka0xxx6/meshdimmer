#ifndef DELAY_WRAPPER_H_
#define DELAY_WRAPPER_H_

#include <inttypes.h>


#ifdef AVR
/**
 * Waits for ms milliseconds. BUSY WAITING!
 * @param ms milliseconds
 */
void delay(uint16_t ms);
#endif

/**
 * Waits for us microseconds. BUSY WAITING!
 * @param us microseconds
 */
void delay_us(uint16_t us);

#endif
