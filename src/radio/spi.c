#ifdef AVR

#include <avr/interrupt.h>
#include "spi.h"



void spi_init() {
	// input: miso -> set pullup
	PORTB |= (1 << PIN_MISO);

	// output: mosi, sck
	DDRB |= (1 << PIN_MOSI) | (1 << PIN_SCK)
#ifdef __AVR_ATmega1284P__
		| (1 << PB4) // SS as output
#elif  __AVR_ATmega328__ | __AVR_ATmega328P__
		| (1 << PB2) // SS as output
#endif
	;

	// Enable SPI, Master, set clock rate fck/4
	SPCR = (1 << SPE) | (1 << MSTR);
}


uint8_t spi_read_write(uint8_t value) {
	SPDR = value;
	while (!(SPSR & (1 << SPIF))) {}
	return SPDR;
}

#endif
