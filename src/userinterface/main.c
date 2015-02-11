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

int main() {
	uint8_t dll_ret;
	uint8_t dimmerValue = 0;

	DEBUG_init();
	DEBUG_message((uint8_t *)"debug init done\n", 16);

	sei();
	spi_init();
	DEBUG_message((uint8_t *)"spi init done\n", 14);

	dll_ret = DLL_init(DLL_ADDRESS, 15, 21, 0);
	if (dll_ret != 0) {
		DEBUG_message((uint8_t *)"dll init failed:", 16);
		DEBUG_number_hex(dll_ret);
		DEBUG_newline();
	}

	while(1) {

		DLL_send(DLL_SEND_TYPE_ACK, 0x30, &dimmerValue, 1);
		delay(50);

		++dimmerValue;
	}


	return 0;
}

