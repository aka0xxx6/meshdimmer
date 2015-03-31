#include <avr/io.h>
#include "i2c.h"
#include "debug.h"

/** Initializes the hardware */
static void initHardware(void ) {
   i2c_init();
   DEBUG_init();
   /* TODO: init radio */
}

int main(void)
{
   /*
      Toggle pin PB1 once at reset, to trigger logic analyzer
      TODO: REMOVE once i2c communication works as intended.
    */
   {
      DDRB = (1U<<1);
      PORTB = 0;
      PORTB = (1U<<1);
      PORTB = 0;
   }

   initHardware();

   DEBUG_puts("Init done!\n");

   i2c_test();

   for(;;) {
      DEBUG_puts("IDLE");
   }
}
