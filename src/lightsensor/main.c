#include <avr/io.h>
#include "i2c.h"
#include "spi.h"
#include "debug.h"
#include "dll.h"
#include "delay_wrapper.h"


#define PID_P_GAIN 100
#define PID_I_GAIN 1
#define PID_D_GAIN 10


#define DLL_ADDRESS ((uint8_t) 0x25)


/** Initializes the hardware */
static void initHardware(void ) {
   uint8_t dll_ret;

   DEBUG_init();
   i2c_init();
   spi_init();
   sei();
   dll_ret = DLL_init(DLL_ADDRESS, (uint8_t) 15, (uint8_t)21, 0);
   if (dll_ret != 0) {
      DEBUG_puts("DLL init failed:");
      DEBUG_number_hex(dll_ret);
      DEBUG_newline();
   }
}


int main(void)
{
   /* radio module */
   uint8_t radio_buf[DLL_MAX_PACKET_LEN];
   uint8_t radio_packet_len;
   uint16_t sensor_data = (uint16_t) 10000;
   uint16_t radio_data = (uint16_t) 10000;
   uint16_t control_output = 0;

   /* PID controller */

   int16_t P_error = 0;
#if 0
   int16_t P_error_prev = 0;
   int16_t I_error = 0;
   int16_t D_error = 0;
#endif

   /*
      Toggle pin PB1 once at reset, to trigger logic analyzer
TODO: REMOVE once i2c communication works as intended.
    */
   {
      DDRB = (uint8_t) (1U<<1);
      PORTB = 0;
      PORTB = (uint8_t) (1U<<1);
      PORTB = 0;
   }

   initHardware();

   DEBUG_puts("Init done!\n");
#if 0
   for(;;) {
      if(i2c_isDone()) {
         DEBUG_number(i2c_getValue());
         DEBUG_newline();
         DEBUG_byte((uint8_t) '\r');
      }
   }

   /* Not reached, i2c_test does not return */

#else
   for(;;) {

      /* Try to get new input data from radio or sensor */
      bool new_radio_data = (bool) DLL_receive(radio_buf, &radio_packet_len);
      bool new_sensor_data = i2c_isDone();


      /* do control loop only, if input data is updated */
      if (new_radio_data || new_sensor_data) {

         if (new_radio_data) {
            if (radio_packet_len == (uint8_t) 2U) {
               radio_data = *((uint16_t*) radio_buf);
            }
         }
         if(new_sensor_data) {
            sensor_data = i2c_getValue();
         }

         /* TODO: transform sensor_data and radio_data to same value range */

         if (sensor_data > radio_data) {
            /* Too bright */
            DEBUG_byte((uint8_t) '+');
            if(control_output < (uint16_t) 1013) {
               control_output += (uint16_t) 10;
            }
         }
         else {
            /* Too dark */
            DEBUG_byte((uint8_t) '-');
            if(control_output > 0) {
               control_output -= (uint16_t) 10;
            }
         }
#if 0
         P_error_prev = P_error;
         P_error = (int16_t)radio_data - (int16_t)sensor_data;
         I_error += P_error;
         D_error = P_error - P_error_prev;

         control_output = (uint16_t) (PID_P_GAIN * P_error +
               PID_I_GAIN * I_error + PID_D_GAIN * D_error);
#endif

         /* TODO: transform output data to right value range */
         DEBUG_number(sensor_data);
         DEBUG_byte((uint8_t) ' ');
         DEBUG_number(control_output);
         DEBUG_newline();
         DEBUG_number((uint16_t) P_error);
         DEBUG_newline();
         DEBUG_byte((uint8_t) '\r');

         /* send control output to dimmer */
         (void) DLL_send(DLL_SEND_TYPE_ACK, (uint8_t) 0x30,
               (uint8_t*)&control_output, sizeof(control_output));
      }
   }
#endif
}
