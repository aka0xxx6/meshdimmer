// Triac output = PD4
// Zero-cross interrupt PD2
// Button GND --> PB0
// Potentimeter--> ADC0

#define F_CPU 1000000UL  // 1MHz internal clock
//#define fullOn 10
//#define fullOff 246


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
//#include "lcd.h"
#include <avr/cpufunc.h>

uint16_t value;
uint16_t adcValue;

static inline void initADC0(void) {
	ADCSRA |= (1 << ADEN); /* enable ADC */
	ADMUX |= (1 << REFS0); /* reference voltage on AVCC */
	ADCSRA |= (1 << ADPS1) | (1 << ADPS0); /* ADC clock prescaler /8 */

	DIDR0 = 0x01; //Data Input Disable Register, Disconnects DI
}

static int dimtime;

ISR (INT0_vect)
{
	TCNT1 = 0;
	OCR1A = dimtime;
	TIMSK1 = (1 << OCIE1A); // Enable timer interrupt
}

ISR(TIMER1_COMPA_vect){
	PORTD |= (1 << PD4); // Fire triac
	TIFR1 = (1 << ICF1); // Clear interrupts
	TIMSK1 &= ~(1 << OCIE1A); // Disable timer interrupt

	// Wait 10 us, busy wait
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	_NOP();

	PORTD &= ~(1 << PD4); // Stop triac
}

int main (void)
{
	value = 10;
	DDRD = 0x10;      //PD4 = Output(Triac), the rest is input
	initADC0();
	//lcd_init(LCD_DISP_ON); // initialize LCD

#ifdef EICRA 	// Hack to make Eclipse happy...
	EICRA |= ( (1<<ISC01) | (1<<ISC00) ); // set INT0 to trigger on RISING edge
	EIMSK |= (1<<INT0);     // Turns on INT0
#else
#error EICRA not defined!
#endif // End of hack



	DDRB &= ~(0b00000001); // makes double-sure we're in input mode on PB0
	PORTB |= (0b00000001); // enables pull-up resistor on PB0
						   // Write to PORTx for setting pull-ups
						   // Read from PINx to know if pressed


	//Timer interrupt
	TCCR1B = (1 << CS10); // Use system clock without prescaling
	TIFR1 = (1 << ICF1); // Clear interrupts

	sei();                    // turn on interrupts


  while (1)                         // infinite main loop
  {
	  //value = fullOff;
	  value = 246;

	if((PINB & (1 << 0)) == 0){ //Button pressed


    	 ADCSRA |= (1 << ADSC); /* start ADC conversion */
    	 loop_until_bit_is_clear(ADCSRA, ADSC); /* wait until done */
	     adcValue = (ADC/4.0); /* read ADC in */
	     	 if(adcValue > 226){value = 236;}
	     	 else
	     	 	 {
	     		 	 value = 10 + adcValue;
	     	 	 }
	     	//char print[15];
	     	//sprintf(print, "%d", value); //int --> char[]
	     	//lcd_puts(print);

	    }




	    //_delay_ms(50);                // wait 1000ms between cycles
	    dimtime = 39 * value; //100000 - 10 us / 256 = 39
	    //lcd_clrscr();
  }

}


