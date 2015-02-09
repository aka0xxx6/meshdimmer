/*
 * dll.h
 *
 *  Created on: 10.02.2014
 *      Author: robin
 */

#ifndef DLL_H_
#define DLL_H_

#include <stdint.h>

//************************** DLL CONFIG ****************************//
// options
#define DLL_BROADCAST_ADDRESS 	0xFF
#define DLL_SEND_TIMEOUT_MS		10 /**< Maximal timeout on send for receiving an ACK */

#define DLL_HARDWARE_RFM70      1
#define DLL_HARDWARE_NRF24L01P  2

// use either DLL_HARDWARE_RFM70 or DLL_HARDWARE_NRF24L01P
//#define DLL_HARDWARE            DLL_HARDWARE_RFM70
#define DLL_HARDWARE            DLL_HARDWARE_NRF24L01P

#ifndef DLL_HARDWARE
#error no DLL hardware specified
#endif


// controller specific stuff
#ifdef __ARM_ARCH_6__
#include <wiringPi.h>
#define IRQ_INPUT 					pinMode(5, INPUT)
#define IRQ_PULLUP  				pullUpDnControl(5, PUD_UP)
#define CE_OUTPUT 					pinMode(6, OUTPUT)
#define CE_SET 						digitalWrite(6, HIGH)
#define CE_CLR 						digitalWrite(6, LOW)
#define DLL_ENABLE_INTERRUPT 		wiringPiISR(5, INT_EDGE_FALLING, DLL_interrupt)

// SPI
#define DLL_SPI_CHANNEL			0
#define CSN_OUTPUT						// dummy
#define CSN_SET							// dummy

#elif AVR
#include <avr/io.h>
#include <avr/interrupt.h>

#ifdef __AVR_ATmega1284P__

#define PIN_CSN		PB0
#define PIN_CE		PB1

#define IRQ_INPUT
#define CE_OUTPUT					DDRB |= (1 << PIN_CE)
#define CE_SET 						PORTB |= (1 << PIN_CE)
#define CE_CLR 						PORTB &= ~(1 << PIN_CE)

#define DLL_ENABLE_INTERRUPT		EICRA |= (1 << ISC11); EIMSK |= (1 << INT1)
#define DLL_INTERRUPT_VECT 		    INT1_vect


#define CSN_OUTPUT					DDRB |= (1 << PIN_CSN)
#define CSN_SET 					PORTB |= (1 << PIN_CSN)
#define CSN_CLR 					PORTB &= ~(1 << PIN_CSN)

#elif __AVR_ATmega8__

#define PIN_CSN		PB1
#define PIN_CE 		PB2

#define IRQ_INPUT
#define CE_OUTPUT					DDRB |= (1 << PIN_CE)
#define CE_SET 						PORTB |= (1 << PIN_CE)
#define CE_CLR 						PORTB &= ~(1 << PIN_CE)


#define DLL_ENABLE_INTERRUPT		MCUCR |= (1 << ISC01); GICR |= (1 << INT0)
#define DLL_INTERRUPT_VECT 		    INT0_vect

// SPI
#define CSN_OUTPUT					DDRB |= (1 << PIN_CSN)
#define CSN_SET 					PORTB |= (1 << PIN_CSN)
#define CSN_CLR 					PORTB &= ~(1 << PIN_CSN)


#elif __AVR_ATmega328P__

#define PIN_CSN		PB1
#define PIN_CE 		PB2

#define IRQ_INPUT
#define CE_OUTPUT					DDRB |= (1 << PIN_CE)
#define CE_SET 						PORTB |= (1 << PIN_CE)
#define CE_CLR 						PORTB &= ~(1 << PIN_CE)

#define DLL_INIT_INTERRUPT			EICRA |= (1 << ISC01); EIMSK |= (1 << INT0)
#define DLL_ENABLE_INTERRUPT		EIMSK |= (1 << INT0)
#define DLL_DISABLE_INTERRUPT		EIMSK &= ~(1 << INT0)
#define DLL_INTERRUPT_VECT 		    INT0_vect

// SPI
#define CSN_OUTPUT					DDRB |= (1 << PIN_CSN)
#define CSN_SET 					PORTB |= (1 << PIN_CSN)
#define CSN_CLR 					PORTB &= ~(1 << PIN_CSN)


#else
#error NOT DEFINED PLATFORM
#endif

#else
#error NOT DEFINED PLATFORM
#endif


//************************** DLL DEFINITIONS ************************//
#define DLL_MAX_PACKET_LEN 			0x20 /**< max packet length is 32 */

typedef enum {
	/** send data and request an ACK */
	DLL_SEND_TYPE_ACK 			= 0xA0,
	/** send data without requesting an ACK */
	DLL_SEND_TYPE_NO_ACK 		= 0xB0
} dll_send_type;


/** indicator how many packets are left on the module */
extern volatile uint8_t DLL_irq_rx_counter;


//************************ DLL FUNCTIONS **************************//

/**
 * Initial the Date Link Layer hardware. SPI have to be initialized before calling this function.
 *
 * @param address address of this module
 * @param auto_retransmission_count number of retries on failure: 0..15
 * @param rf_channel channel for communication: 0..127
 * @param air_data_rate 0: 1Mbps; 1: 2Mbps; 2: 250kbps (250kpbs not supported by RFM70)
 * @return 0: successful; otherwise error code
 */
uint8_t DLL_init(uint8_t address, uint8_t auto_retransmission_count, uint8_t rf_channel, uint8_t air_data_rate);

/**
 * Sends packet of the given length to receiver_address.
 *
 * @param type DLL_SEND_TYPE_ACK or DLL_SEND_TYPE_NO_ACK
 * @param receiver_address address of the receiver
 * @param data data to be send
 * @param length number of bytes to be send, maximum DLL_MAX_PACKET_LEN
 * @return 1: successfully sent; 0 otherwise. For DLL_SEND_TYPE_NO_ACK you will get
 * every time 1.
 */
uint8_t DLL_send(dll_send_type type, uint8_t receiver_address, uint8_t * data, uint8_t length);

/**
 * Try to receive data. It is required to use a buffer of length DLL_MAX_PACKET_LEN.
 *
 * @param buffer packet content
 * @param length number of bytes received
 * @return return: 0: nothing received; otherwise received a packet
 */
uint8_t DLL_receive(uint8_t * buffer, uint8_t * length);

/**
 * Store data in the data pipes of the radio module. When the module sends an ACK
 * the content of the data pipe will also transmitted as part of the ACK packet.
 * This only works if the receiving address is equal with the address, which the
 * DLL_init function was called with.
 * The data will only transmitted once.
 *
 * @param buffer data to be transmitted with the next ACK
 * @param length length of the buffer
 */
void DLL_ACK_payload(uint8_t * buffer, uint8_t length);

/**
 * Waiting on an incoming packet, with the timeout value timeout_ms in milliseconds.
 * WARNING: NOT RELIABLE FUNCTION, use DLL_receive to be sure that no packet is available.
 * Hint: Use this function due to the reliability with low timeout_ms values like 100-500, otherwise
 * use DLL_receive more often.
 *
 * @param timeout_ms timeout in milliseconds
 */
void DLL_wait_on_rx(uint16_t timeout_ms);

/**
 * Drive the DLL into or out of the standby mode to save energy.
 * @param activate 0: go to standby mode; otherwise go in active receiving mode.
 */
void DLL_standby(uint8_t activate);

/**
 * Print all registers via DEBUG interface for debug purpose.
 */
void DLL_print();


#endif /* DLL_H_ */
