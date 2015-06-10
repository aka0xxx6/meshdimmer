// Triac output = PD4
// Zero-cross interrupt PD2
// Button GND --> PB0
// Potentimeter--> ADC0

// Defined in makefile!
//#define F_CPU 1000000UL  // 1MHz internal clock
//#define F_CPU 8000000UL  // 8MHz internal clock
//#define fullOn 10
//#define fullOff 246

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

#include "radio/spi.h"
#include "radio/debug.h"
#include "radio/dll.h"
#include "radio/delay_wrapper.h"

//#include <avr/cpufunc.h>

#define DLL_ADDRESS 0x30

#include "table.h"

static inline void initADC0(void) {
	ADCSRA |= (1 << ADEN); /* enable ADC */
	ADMUX |= (1 << REFS0); /* reference voltage on AVCC */
	ADCSRA |= (1 << ADPS1) | (1 << ADPS0); /* ADC clock prescaler /8 */

	DIDR0 |= (1 << ADC0D); //0x01; //Data Input Disable Register, Disconnects DI
}

static uint16_t dimtime; //static

ISR (INT0_vect) {
	TCNT1 = 0;
	OCR1A = dimtime;
	TIMSK1 |= (1 << OCIE1A); // Enable timer interrupt
}

ISR(TIMER1_COMPA_vect) {
	TIFR1 |= (1 << ICF1); // Clear interrupts
	TIMSK1 &= ~(1 << OCIE1A); // Disable timer interrupt

	PORTD |= (1 << PD4); // Fire triac
	// Wait 10 us, busy wait
	// 1 MHz
	//_delay_loop_2(1); //four cpu cycles
	//_delay_loop_1(2), //six cpu cycles

	// 8 MHz
	_delay_loop_2(20); //80 cpu cycles

	PORTD &= ~(1 << PD4); // Stop triac
}

int main(void) {
	uint16_t value;
	uint8_t dll_ret;
	uint8_t buffer[DLL_MAX_PACKET_LEN];
	uint8_t length;

	DEBUG_init();
	DEBUG_message((uint8_t *) "debug init done\n", 16);

	spi_init();
	DEBUG_message((uint8_t *) "spi init done\n", 14);

	dll_ret = DLL_init(DLL_ADDRESS, 15, 21, 0);
	if (dll_ret != 0) {
		DEBUG_message((uint8_t *) "dll init failed:", 16);
		DEBUG_number_hex(dll_ret);
		DEBUG_newline();
	} else {
		DEBUG_message((uint8_t *) "DLL init done\n", 14);
	}

	DDRD |= (1 << PD4);      //PD4 = Output(Triac), the rest is input //0x10
	initADC0();

	EICRA |= ((1 << ISC01) | (1 << ISC00)); // set INT0 to trigger on RISING edge
	EIMSK |= (1 << INT0);     // Turns on INT0

	DDRB &= ~(0b00000001); // makes double-sure we're in input mode on PB0
	PORTB |= (0b00000001); // enables pull-up resistor on PB0
						   // Write to PORTx for setting pull-ups
						   // Read from PINx to know if pressed

	//Timer interrupt
	//TCCR1B |= (1 << CS10); // Use system clock without prescaling // 1 MHZ
	TCCR1B |= (1 << CS11); // 8 Mhz
	TIFR1 |= (1 << ICF1); // Clear interrupts

	// turn on interrupts
	sei();


	value = 220;
	while (1)                         // infinite main loop
	{
		if ((PINB & (1 << 0)) == 0) { // Button pressed

			ADCSRA |= (1 << ADSC); // start ADC conversion
			loop_until_bit_is_clear(ADCSRA, ADSC); // wait until done
			value = (ADC / 4); // read ADC in
			DEBUG_byte('b');
			DEBUG_number(value);
			DEBUG_newline();
		}

		if (DLL_receive(buffer, &length)) {
			if (length == 2) {
				value = *((uint16_t*) buffer) / 4;
			}

			DEBUG_message(buffer, length);
			DEBUG_newline();
			DEBUG_byte('r');
			DEBUG_number(value);
			DEBUG_newline();
		}


		_delay_ms(50);        // wait 50ms between cycles
		dimtime = lookup[255-value]; // TODO: Fix the order we send stuff in!

	}

}
