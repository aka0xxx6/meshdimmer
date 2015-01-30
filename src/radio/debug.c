#ifdef AVR

#include <avr/io.h>
#include "debug.h"

#define DEBUG_BAUDRATE 19200UL
#define DEBUG_BAUDREGISTER ((F_CPU + DEBUG_BAUDRATE * 8 ) / (DEBUG_BAUDRATE * 16) - 1)


#ifdef __AVR_ATmega8__
#define WHILE_NOT_SENT		while (!( UCSRA & (1 << UDRE))) { }
#define SEND_REGISTER		UDR
#elif __AVR_ATmega328P__ || __AVR_ATmega1284P__
#define WHILE_NOT_SENT		while (!( UCSR0A & (1 << UDRE0))) { }
#define SEND_REGISTER		UDR0
#else
#error PLATFORM NOT SUPPORTED
#endif


#ifdef __AVR_ATmega8__
void DEBUG_init() {
	UBRRH = DEBUG_BAUDREGISTER >> 8;
	UBRRL = DEBUG_BAUDREGISTER & 0xFF;

	// Enable transmitter 
	UCSRB = (1 << TXEN);

	// 8data, 1stop bit
	UCSRC = (1 << URSEL) | ( 1 << UCSZ1) | (1 << UCSZ0);
}
#elif __AVR_ATmega328P__ || __AVR_ATmega1284P__
void DEBUG_init() {
	UBRR0H = DEBUG_BAUDREGISTER >> 8;
	UBRR0L = DEBUG_BAUDREGISTER & 0xFF;

	// Enable transmitter
	UCSR0B = (1 << TXEN0);

	// 8data, 1stop bit
	UCSR0C = ( 1 << UCSZ01) | (1 << UCSZ00);
}
#endif




void DEBUG_byte(uint8_t data) {
	WHILE_NOT_SENT;
	SEND_REGISTER = data;
}


void DEBUG_newline() {
	DEBUG_byte('\n');
}

void DEBUG_message(uint8_t * msg, uint8_t len) {
	while (len) {
		DEBUG_byte(*msg);
		++msg;
		--len;
	}
}

void DEBUG_number(uint16_t u) {
	uint8_t tmp = 0;
	WHILE_NOT_SENT;
	tmp = u / 10000;
	if (tmp != 0)
		SEND_REGISTER = tmp + '0';

	WHILE_NOT_SENT;
	tmp = ((u / 1000) % 10);
	if (tmp != 0)
		SEND_REGISTER = tmp + '0';

	WHILE_NOT_SENT;
	SEND_REGISTER = ((u / 100) % 10)  + '0';

	WHILE_NOT_SENT;
	SEND_REGISTER = ((u / 10) % 10)  + '0';

	WHILE_NOT_SENT;
	SEND_REGISTER = (u % 10)  + '0';
}

void DEBUG_number_hex(uint8_t u) {
	WHILE_NOT_SENT;
	SEND_REGISTER = '0';
	WHILE_NOT_SENT;
	SEND_REGISTER = 'x';
	
	uint8_t hex = u / 16;
	if (hex < 10) {
		WHILE_NOT_SENT;
		SEND_REGISTER = hex + '0';
	}
	else {
		WHILE_NOT_SENT;
		SEND_REGISTER = hex - 10 + 'A';
	}
	hex = u % 16;
	if (hex < 10) {
		WHILE_NOT_SENT;
		SEND_REGISTER = hex + '0';
	}
	else {
		WHILE_NOT_SENT;
		SEND_REGISTER = hex - 10 + 'A';
	}
}

#endif
