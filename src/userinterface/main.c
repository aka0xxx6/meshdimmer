/*
 * main.c
 *
 *  Created on: 11.02.2015
 *      Author: robin
 */

#include "radio/spi.h"
#include "radio/debug.h"
#include "radio/dll.h"
#include "radio/delay_wrapper.h"

#define DLL_ADDRESS 0x20
//#define DLL_ADDRESS 0x30



void ADC_init() {
	// use vcc as reference voltage
	ADMUX = (1 << REFS0);

	// enable ADC
	ADCSRA = (1 << ADEN);
}


uint16_t ADC_read_value(uint8_t channel) {

	// select channel
	 ADMUX = (ADMUX & 0xF8) | channel;

	// start conversion
	ADCSRA |= (1 << ADSC);

	// busy wait for conversion end
	while (ADCSRA & (1<<ADSC) ) { }

	return ADC;
}



int main() {
	uint16_t adc_value;
	uint8_t dll_ret;

	DEBUG_init();
	DEBUG_message((uint8_t *)"debug init done\n", 16);

	spi_init();
	DEBUG_message((uint8_t *)"spi init done\n", 14);

	ADC_init();
	DEBUG_message((uint8_t *)"ADC init done\n", 14);

	sei();
	dll_ret = DLL_init(DLL_ADDRESS, 15, 21, 0);
	if (dll_ret != 0) {
		DEBUG_message((uint8_t *)"dll init failed:", 16);
		DEBUG_number_hex(dll_ret);
		DEBUG_newline();
	} else {
		DEBUG_message((uint8_t *)"DLL init done\n", 14);
	}

	if (DLL_ADDRESS == 0x20) {
		while(1) {
			adc_value = ADC_read_value(0x00);
			uint8_t ret = DLL_send(DLL_SEND_TYPE_ACK, 0x30, (uint8_t*)&adc_value, 2);
			DEBUG_number(ret);
			DEBUG_newline();
			delay(20);
		}
	}

	if (DLL_ADDRESS == 0x30) {
		uint8_t buffer[DLL_MAX_PACKET_LEN];
		uint8_t length = 0;

		while (1) {
			DLL_wait_on_rx(1000);
			if (DLL_receive(buffer, &length)) {
				if (length == 2) {
					DEBUG_number(*((uint16_t*)buffer));
					DEBUG_newline();
				}
			}
		}
	}



	return 0;
}

