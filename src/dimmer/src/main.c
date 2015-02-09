// Triac output = PD3
// Zero-cross interrupt PD2

#define F_CPU 1000000UL  // 1MHz internal clock
//#define fullOn 10
//#define fullOff 246

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include "lcd.h"
#include <avr/cpufunc.h>

int value;

//dimtime = (39 * value); //100000 - 10 us / 256 = 39
static int dimtime;

ISR (INT0_vect)
{
	TCNT1 = 0;
	OCR1A = dimtime;
	TIMSK1 = (1 << OCIE1A); // Enable timer interrupt
}

ISR(TIMER1_COMPA_vect){
	PORTD |= (1 << PD3); // Fire triac
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

	PORTD &= ~(1 << PD3); // Stop triac
}

int main (void)
{
	value = 246;
	DDRD = 0x08;      //PD3 = Output(Triac), the rest is input

#ifdef EICRA 	// Hack to make Eclipse happy...
	EICRA |= ( (1<<ISC01) | (1<<ISC00) ); // set INT0 to trigger on RISING edge
	EIMSK |= (1<<INT0);     // Turns on INT0
#else
#error EICRA not defined!
#endif // End of hack

	lcd_init(LCD_DISP_ON); // initialize LCD
	//lcd_puts("WELCOME!!");
	//_delay_ms(1000);

	DDRD &= ~(0b11100000); // makes double-sure we're in input mode on PD5-7
	PORTD |= (0b11100000); // enables pull-up resistor on PD5-7
						   // Write to PORTx for setting pull-ups
						   // Read from PINx to know if pressed

	DDRB &= ~(1 << PB1); //Input mode
	PORTB |= (1<<PB1); //Activate pull-up

	DDRB |= 0b00000001; //Led PB0 Output

	//Timer interrupt
	TCCR1B = (1 << CS10); // Use system clock without prescaling
	TIFR1 = (1 << ICF1); // Clear interrupts

	sei();                    // turn on interrupts


  while (1)                         // infinite main loop
  {
	  value = 246;

		if((PINB & (1 << 1)) == 0){ //Button 1 pressed

				value = 10;
	    }


		else if((PIND & (1 << 5)) == 0){ //Button 2 pressed

	    	value = 78;
	    }


		else if((PIND & (1 << 6))==0){ //Button 3 pressed

	    	value = 130;
	    }



		else if((PIND & (1 << 7))==0){ //Button 4 pressed

	    	value = 236;
	    }



	    char print[15];
	    sprintf(print, "%d", value); //int --> char[]
	    lcd_puts(print);

	    _delay_ms(50);                // wait 1000ms between cycles
	    lcd_clrscr();
	    dimtime = 39 * value;
  }

}


