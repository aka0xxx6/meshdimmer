/*
 * dll_test.c
 *
 *  Created on: 28.01.2015
 *      Author: robin
 */

#include "../radio/spi.h"
#include "../radio/debug.h"
#include "../radio/dll.h"

#define DLL_ADDRESS 0x20
//#define DLL_ADDRESS 0x21

int main() {
	uint8_t buffer[DLL_MAX_PACKET_LEN];
	uint8_t length;
	uint8_t dll_ret;

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

		if (DLL_receive(buffer, &length)) {
			DEBUG_message(buffer, length);
			DEBUG_newline();
		}


		if (0x21 == DLL_ADDRESS) {
			DLL_send(DLL_SEND_TYPE_ACK, 0x20, (uint8_t*)"0123456789", 10);
			DEBUG_byte('.');
		}
	}


	return 0;
}

