/*
 * main.c
 *
 *  Created on: 11.02.2015
 *      Author: robin
 */

#include <stdlib.h>
#include "radio/spi.h"
#include "radio/debug.h"
#include "radio/dll.h"
#include "radio/delay_wrapper.h"

#define DLL_ADDRESS 0x20



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
	int16_t adc_value;
	int16_t adc_value_old = 0;
	uint8_t dll_ret;
	uint16_t adc_values[8];
	uint8_t adc_index = 0;
	uint32_t adc_avarage = 0;
	uint8_t i = 0;

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



	while(1) {
		adc_values[adc_index] = ADC_read_value(0x00);
		adc_index = (adc_index + 1) % 8;

		adc_avarage = 0;
		for (i = 0; i < 8; ++i) {
			adc_avarage += adc_values[i];
		}

		adc_value = adc_avarage >> 3;

		if (abs(adc_value - adc_value_old) > 5 ) {
			uint8_t ret1 = DLL_send(DLL_SEND_TYPE_ACK, 0x30, (uint8_t*)&adc_value, 2);
			uint8_t ret2 = DLL_send(DLL_SEND_TYPE_ACK, 0x31, (uint8_t*)&adc_value, 2);
			DEBUG_number(adc_value);
			DEBUG_byte(' ');
			DEBUG_number(ret1);
			DEBUG_byte(' ');
			DEBUG_number(ret2);
			DEBUG_newline();
			adc_value_old = adc_value;
		}

	}



	return 0;
}

