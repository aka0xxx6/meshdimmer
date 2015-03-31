#ifndef DEBUG_H_
#define DEBUG_H_

#include <inttypes.h>
#include <string.h>

/* Supress false warning from splint */
/*@ -exportlocal @*/
void DEBUG_init(void);
void DEBUG_newline(void);
void DEBUG_byte(uint8_t data);
void DEBUG_message(uint8_t * msg, uint8_t len);
void DEBUG_number(uint16_t u);
void DEBUG_number_hex(uint8_t u);

#define DEBUG_puts(x) do { DEBUG_message((uint8_t *)(x), (uint8_t)strlen(x));} while (0==1)

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define HERE __FILE__ ":" TOSTRING(__LINE__)
/* Use: DEBUG_puts(HERE) to output file and line number */

#endif
