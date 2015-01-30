/*
 ATmega8, 48, 88, 168, 328
 
 /Reset PC6|1   28|PC5
 PD0|2   27|PC4
 PD1|3   26|PC3
 PD2|4   25|PC2
 PD3|5   24|PC1
 PD4|6   23|PC0
 Vcc|7   22|Gnd
 Gnd|8   21|Aref
 PB6|9   20|AVcc
 PB7|10  19|PB5 SCK
 PD5|11  18|PB4 MISO
 PD6|12  17|PB3 MOSI
 PD7|13  16|PB2
 LED    PB0|14  15|PB1
 */

#include <avr/io.h>      // defines all macros and symbols
#include <util/delay.h>  // defines time delay functions

int main (void)
{
    DDRB = 0x01;         // pin 0 of PORTB as output
    DDRC = 0x01;         // pin 1 of PORTC as output
    DDRD = 0x80;         // pin 7 of PORTD as output


    
    while (1)            // infinite main loop
    {
        PORTB = 0x01;      // switch PB0 to 1
        _delay_ms(500);    // wait 1/2 second
        PORTB = 0x00;      // switch PB0 to 0
        _delay_ms(500);    // wait 1/2 second
        
        PORTC = 0x01;      // switch PB0 to 1
        _delay_ms(500);    // wait 1/2 second
        PORTC = 0x00;      // switch PB0 to 0
        _delay_ms(500);    // wait 1/2 second
        
        PORTD = 0x80;      // switch PB0 to 1
        _delay_ms(500);    // wait 1/2 second
        PORTD = 0x00;      // switch PB0 to 0
        _delay_ms(500);    // wait 1/2 second
    }
}
