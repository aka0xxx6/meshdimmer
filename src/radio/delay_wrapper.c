#ifdef AVR

#include <avr/io.h>
#include <util/delay.h>

#include "delay_wrapper.h"

void delay(uint16_t ms) {
	while (ms > 0) {
		_delay_ms(1);
		--ms;
	}
}

void delay_us(uint16_t us) {
	while (us > 0) {
		_delay_us(1);
		--us;
	}
}

#endif
