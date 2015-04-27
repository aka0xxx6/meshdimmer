#include <avr/io.h>
#include "i2c.h"
#include "radio/spi.h"
#include "radio/debug.h"
#include "radio/dll.h"
#include "radio/delay_wrapper.h"


#define PID_P_GAIN 100
#define PID_I_GAIN 1
#define PID_D_GAIN 10


#define DLL_ADDRESS 0x25


/** Initializes the hardware */
static void initHardware(void ) {
   uint8_t dll_ret;

   i2c_init();
   spi_init();
   DEBUG_init();
   sei();
   dll_ret = DLL_init(DLL_ADDRESS, 15, 21, 0);
   if (dll_ret != 0) {
      DEBUG_message((uint8_t *)"DLL init failed:", 16);
      DEBUG_number_hex(dll_ret);
      DEBUG_newline();
   }
}


int i2c_getValue(uint16_t * value) {
	*value = 0x0123;
}

int main(void)
{
	/* radio module */
	uint8_t radio_buffer[DLL_MAX_PACKET_LEN];
	uint8_t radio_packet_length;

	/* PID controller */
	int16_t P_error = 0;
	int16_t P_error_prev = 0;
	int16_t I_error = 0;
	int16_t D_error = 0;




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

   i2c_test();



   for(;;) {
	   uint16_t sensor_data;
	   uint16_t radio_data;
	   uint16_t control_output;

	   // try to get new input data from radio (target value) or sensor (actual value)
	   uint8_t new_radio_data = DLL_receive(radio_buffer, &radio_packet_length);
	   uint8_t new_sensor_data = i2c_getValue(&sensor_data);

	   // do control loop only, if input data is updated
	   if (new_radio_data || new_sensor_data) {

		   if (new_radio_data) {
			   if (radio_packet_length == 2) {
				   radio_data = *((uint16_t*) radio_buffer);
				}
		   }

		   // TODO: transform sensor_data and radio_data to same value range

		   P_error_prev = P_error;
	   	   P_error = radio_data - sensor_data;
	   	   I_error += P_error;
	   	   D_error = P_error - P_error_prev;

	   	   control_output = PID_P_GAIN * P_error + PID_I_GAIN * I_error + PID_D_GAIN * D_error;

	   	   // TODO: transform output data to right value range

	   	   // send control output to dimmer
		   uint8_t ret1 = DLL_send(DLL_SEND_TYPE_ACK, 0x30, (uint8_t*)&control_output, 2);
	   }
   }
}
