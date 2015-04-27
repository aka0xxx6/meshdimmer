#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>

#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define PIN_MOSI	PB3
#define PIN_MISO 	PB4
#define PIN_SCK		PB5

#elif __AVR_ATmega1284P__
#define PIN_MOSI	PB5
#define PIN_MISO 	PB6
#define PIN_SCK		PB7

#else
#error PLATFORM NOT SUPPORTED
#endif



/**
 * Initialize SPI: Set port direction and enable SPI module.
 */
void spi_init(void);

/**
 * Reads and writes one byte from/to SPI port.
 *
 * @param value value to write over SPI
 * @return read value
 */
uint8_t spi_read_write(uint8_t value);



#endif
