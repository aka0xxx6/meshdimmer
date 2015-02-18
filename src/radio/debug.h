#ifndef DEBUG_H_
#define DEBUG_H_

#include <inttypes.h>
#include <string.h>

void DEBUG_init();
void DEBUG_newline();
void DEBUG_byte(uint8_t data);
void DEBUG_message(uint8_t * msg, uint8_t len);
void DEBUG_number(uint16_t u);
void DEBUG_number_hex(uint8_t u);

#define DEBUG_puts(x) do { DEBUG_message((uint8_t *)(x), strlen(x));} while (0)

#endif
