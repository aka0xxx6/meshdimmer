/*
 * dll.c
 *
 *  Created on: 10.02.2014
 *      Author: robin
 */
#ifdef __ARM_ARCH_6__
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdio.h>
#include <string.h>
#elif AVR
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "delay_wrapper.h"
#include "spi.h"
#include "debug.h"
#else
#error PLATFORM NOT DEFINED
#endif


#include "dll.h"


//************************** DLL COMMANDS **************************//
#define DLL_CMD_R_REGISTER			0x00 // read register
#define DLL_CMD_W_REGISTER			0x20 // write register
#define DLL_CMD_R_RX_PAYLOAD 			0x61 // read RX payload
#define DLL_CMD_W_TX_PAYLOAD			0xA0 // write TX payload
#define DLL_CMD_FLUSH_TX 				0xE1 // flush TX FIFO
#define DLL_CMD_FLUSH_RX 				0xE2 // flush RX FIFO
#define DLL_CMD_REUSE_TX_PL 			0xE3 // reuse last transmitted payload, packets are repeatedly retransmitted as long as CE is high
#if DLL_HARDWARE == DLL_HARDWARE_RFM70
#define DLL_CMD_ACTIVATE				0x50 // activate features (R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK)
#endif
#define DLL_CMD_R_RX_PL_WID			0x60 // read RX-payload width for RX FIFO
#define DLL_CMD_W_ACK_PAYLOAD			0xA8 // write payload to be transmitted together with ACK packet
#define DLL_CMD_TX_PAYLOAD_NOACK		0xB0 // disable AUTOACK on this specific packet
#define DLL_CMD_NOP 					0xFF // no operation, used to read the STATUS register
//*********************** DLL ADDRESSES ***************************//
#define DLL_REG0_CONFIG				0x00 // configuration register
#define DLL_REG0_EN_AA 				0x01 // enable AUTOACK function
#define DLL_REG0_EN_RXADDR 			0x02 // enable rx addresses
#define DLL_REG0_SETUP_AW 			0x03 // setup of address widths
#define DLL_REG0_SETUP_RETR 			0x04 // setup of automatic retransmission
#define DLL_REG0_RF_CH 				0x05 // RF channel
#define DLL_REG0_RF_SETUP 			0x06 // RF setup
#define DLL_REG0_STATUS 				0x07 // status register
#define DLL_REG0_OBSERVE_TX 			0x08 // transmit observe register
#define DLL_REG0_CD 					0x09 // carrier detect
#define DLL_REG0_RX_ADDR_P0 			0x0A // RX data pipe 0
#define DLL_REG0_RX_ADDR_P1 			0x0B // RX data pipe 1
#define DLL_REG0_RX_ADDR_P2 			0x0C // RX data pipe 2
#define DLL_REG0_RX_ADDR_P3 			0x0D // RX data pipe 3
#define DLL_REG0_RX_ADDR_P4 			0x0E // RX data pipe 4
#define DLL_REG0_RX_ADDR_P5 			0x0F // RX data pipe 5
#define DLL_REG0_TX_ADDR 				0x10 // TX address
#define DLL_REG0_RX_PW_P0 			0x11 // number of bytes in RX payload in data pipe 0
#define DLL_REG0_RX_PW_P1 			0x12 // number of bytes in RX payload in data pipe 1
#define DLL_REG0_RX_PW_P2 			0x13 // number of bytes in RX payload in data pipe 2
#define DLL_REG0_RX_PW_P3 			0x14 // number of bytes in RX payload in data pipe 3
#define DLL_REG0_RX_PW_P4 			0x15 // number of bytes in RX payload in data pipe 4
#define DLL_REG0_RX_PW_P5 			0x16 // number of bytes in RX payload in data pipe 5
#define DLL_REG0_FIFO_STATUS 			0x17 // FIFO status register
#define DLL_REG0_DYNPD 				0x1C // enable dynamic payload length
#define DLL_REG0_FEATURE 				0x1D // feature register
//*************************** DLL STATES **************************//
#define DLL_STATUS_RBANK				0x80 // register bank selection state
#define DLL_STATUS_RX_DR				0x40 // data ready RX FIFO interrupt
#define DLL_STATUS_TX_DS				0x20 // data sent TX FIFO interrupt
#define DLL_STATUS_MAX_RT 			0x10 // maximum number of retransmits interrupt
#define DLL_STATUS_RX_P_NO			0x0E // data pipe number for the payload available for trading from RX FIFO
#define DLL_STATUS_TX_FULL 			0x01 // TX FIFO full flag

#define DLL_FIFO_STATUS_TX_REUSE 		0x40 // reuse last transmitted data packet if set high
#define DLL_FIFO_STATUS_TX_FULL 		0x20 // TX FIFP full flag
#define DLL_FIFO_STATUS_TX_EMPTY 		0x10 // TX FIFO empty flag
#define DLL_FIFO_STATUS_RX_FULL 		0x02 // RX FIFO full flag
#define DLL_FIFO_STATUS_RX_EMPTY 		0x01 // RX FIFO empty flag



#if DLL_HARDWARE == DLL_HARDWARE_RFM70
// Bank1 register initialization value
const uint8_t Bank1_reg0_5[][4]
#ifdef AVR
                             PROGMEM
#endif
                             = {
	{0x40, 0x4B, 0x01, 0xE2},  // from 0 to 8 inversed => MSByte first; LSByte last
	{0xC0, 0x4B, 0x00, 0x00},
	{0xD0, 0xFC, 0x8C, 0x02},
	{0x99, 0x00, 0x39, 0x41},
	{0xF9, 0x9E, 0x86, 0x0B},
	{0x24, 0x06, 0x7F, 0xA6}
};

const uint8_t Bank1_reg12_13[][4]
#ifdef AVR
                               PROGMEM
#endif
                               = {
	{0x00, 0x12, 0x73, 0x00},
	{0x36, 0xB4, 0x80, 0x00}
};

const uint8_t Bank1_Reg14[]
#ifdef AVR
                          PROGMEM
#endif
                          = {
	0x41,0x20,0x08,0x04,0x81,0x20,0xCF,0xF7,0xFE,0xFF,0xFF
};

#endif

// Bank0 address suffix
const uint8_t Address_Suffix[]
#ifdef AVR
                             PROGMEM
#endif
                             = {0x42, 0x42, 0x42, 0x42};






uint8_t DLL_address;
uint8_t DLL_status;
volatile uint8_t DLL_irq_tx_ds_flag;
volatile uint8_t DLL_irq_max_rt_flag;
volatile uint8_t DLL_irq_rx_counter;





uint8_t DLL_read_reg(uint8_t reg) {
#ifdef __ARM_ARCH_6__
	uint8_t data[2] = { reg, 0 };
	wiringPiSPIDataRW(DLL_SPI_CHANNEL, data, 2);
	DLL_status = data[0];
	return data[1];
#elif AVR
	uint8_t value;
	DLL_DISABLE_INTERRUPT;
	CSN_CLR;
	DLL_status = spi_read_write(reg);	// select register
	value = spi_read_write(0x00);		// read value (send dummy)
	CSN_SET;
	DLL_ENABLE_INTERRUPT;
	return value;
#endif
}

#ifdef AVR
__attribute__((always_inline)) uint8_t DLL_read_reg_interrupt(uint8_t reg) {
	uint8_t value;
	CSN_CLR;
	spi_read_write(reg);	// select register
	value = spi_read_write(0x00);		// read value (send dummy)
	CSN_SET;
	return value;
}

#endif


void DLL_wait_on_rx(uint16_t timeout_ms) {
	uint16_t counter = 0;

	while (DLL_irq_rx_counter == 0 && counter < timeout_ms) {
		++counter;
        delay(1);
	}
}


/**
 * Writes one byte to a register.
 *
 * @param reg address of register
 * @param value data to be written
 */
void DLL_write_reg(uint8_t reg, uint8_t value) {

#ifdef __ARM_ARCH_6__
	uint8_t data[2] = { reg, value };
	wiringPiSPIDataRW(DLL_SPI_CHANNEL, data, 2);
	DLL_status = data[0];
#elif AVR
	cli();
	CSN_CLR;
	DLL_status = spi_read_write(reg); // select register
	spi_read_write(value);  			// write value
	CSN_SET;
	sei();
#endif
}


#ifdef __ARM_ARCH_6__
void DLL_interrupt() {
	uint8_t status = DLL_read_reg(DLL_CMD_R_REGISTER | DLL_REG0_STATUS);
#elif AVR
ISR(DLL_INTERRUPT_VECT) {
	uint8_t status = DLL_read_reg_interrupt(DLL_CMD_R_REGISTER | DLL_REG0_STATUS);
#endif
	if ((status & DLL_STATUS_RX_DR) == DLL_STATUS_RX_DR) {
		++DLL_irq_rx_counter;
	} else if ((status & DLL_STATUS_TX_DS) == DLL_STATUS_TX_DS) {
		DLL_irq_tx_ds_flag = 1;
	} else if ((status & DLL_STATUS_MAX_RT) == DLL_STATUS_MAX_RT) {
		DLL_irq_max_rt_flag = 1;
	}
}


/**
 * Reads data with the given length from address reg into the buffer.
 *
 * @param reg address of the register
 * @param buffer data
 * @param length number of bytes to be read
 */
void DLL_read_buf(uint8_t reg, uint8_t * buffer, uint8_t length) {
#ifdef __ARM_ARCH_6__
	uint8_t buf[DLL_MAX_PACKET_LEN + 1];
	buf[0] = reg;
	wiringPiSPIDataRW(DLL_SPI_CHANNEL, buf, length + 1);
	memcpy(buffer, buf + 1, length);
	DLL_status = buf[0];
#elif AVR
	uint8_t i = 0;
	cli();
	CSN_CLR;
	DLL_status = spi_read_write(reg);		// select register
	for (i = 0; i < length; ++i) {			// read buffer
		buffer[i] = spi_read_write(0x00);
	}
	CSN_SET;
	sei();
#endif

}

/**
 * Writes the given buffer over SPI to the module in the register reg.
 *
 * @param reg address of the register
 * @param buffer data to be written
 * @param length length of the data
 */
void DLL_write_buf(uint8_t reg, uint8_t * buffer, uint8_t length) {
#ifdef __ARM_ARCH_6__
	uint8_t buf[DLL_MAX_PACKET_LEN + 1];
	buf[0] = reg;
	memcpy(buf + 1, buffer, length);
	wiringPiSPIDataRW(DLL_SPI_CHANNEL, buf, length + 1);
	DLL_status = buf[0];
#elif AVR
	uint8_t i;
	cli();
	CSN_CLR;
	DLL_status = spi_read_write(reg);		// select register
	for (i = 0; i < length; ++i) {			// read buffer
		spi_read_write(buffer[i]);
	}
	CSN_SET;
	sei();
#endif
}

void DLL_print() {
#ifdef AVR
	uint8_t i = 0;
	uint8_t buffer[5];
	for(i = 0; i <= 0x09; ++i) {
		DEBUG_number_hex(i);
		DEBUG_byte(':');
		DEBUG_number_hex(DLL_read_reg(i));
		DEBUG_newline();
	}
	DEBUG_number_hex(0x0A);
	DEBUG_byte(':');
	DLL_read_buf(0x0A, buffer, 5);
	DEBUG_message(buffer, 5);

	DEBUG_number_hex(0x0B);
	DEBUG_byte(':');
	DLL_read_buf(0x0B, buffer, 5);
	DEBUG_message(buffer, 5);

	for(i = 0x0C; i <= 0x0F; ++i) {
		DEBUG_number_hex(i);
		DEBUG_byte(':');
		DEBUG_number_hex(DLL_read_reg(i));
		DEBUG_newline();
	}

	DEBUG_number_hex(0x10);
	DEBUG_byte(':');
	DLL_read_buf(0x10, buffer, 5);
	DEBUG_message(buffer, 5);

	for(i = 0x11; i <= 0x17; ++i) {
		DEBUG_number_hex(i);
		DEBUG_byte(':');
		DEBUG_number_hex(DLL_read_reg(i));
		DEBUG_newline();
	}
	for(i = 0x1C; i <= 0x1D; ++i) {
		DEBUG_number_hex(i);
		DEBUG_byte(':');
		DEBUG_number_hex(DLL_read_reg(i));
		DEBUG_newline();
	}


#else
	uint8_t i = 0;
	    uint8_t buffer[5];
	    for(i = 0; i <= 0x09; ++i) {
	        printf("%02x:%02x\n", i, DLL_read_reg(i));
	    }

	    DLL_read_buf(0x0A, buffer, 5);
	    printf("0x1A: ");
        for (i = 0; i < 5; ++i)
            printf("%02x ", buffer[i]);
        printf("\n");

	    DLL_read_buf(0x0B, buffer, 5);
	    printf("0x1B: ");
        for (i = 0; i < 5; ++i)
            printf("%02x ", buffer[i]);
        printf("\n");


	    for(i = 0x0C; i <= 0x0F; ++i) {
	        printf("%02x:%02x\n", i, DLL_read_reg(i));
	    }

        DLL_read_buf(0x10, buffer, 5);
        printf("0x10: ");
        for (i = 0; i < 5; ++i)
            printf("%02x ", buffer[i]);
        printf("\n");


	    for(i = 0x11; i <= 0x17; ++i) {
	        printf("%02x:%02x\n", i, DLL_read_reg(i));
	    }
	    for(i = 0x1C; i <= 0x1D; ++i) {
	        printf("%02x:%02x\n", i, DLL_read_reg(i));
	    }
#endif
}
void DLL_rx_mode() {
	uint8_t value = DLL_read_reg(DLL_CMD_R_REGISTER | DLL_REG0_STATUS);
	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_STATUS, value); // clear RX_DR or TX_DS or MAX_RT interrupt flag

	CE_CLR;

	// set PRX mode
	value = DLL_read_reg(DLL_CMD_R_REGISTER | DLL_REG0_CONFIG);
	value |= 0x01;
	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_CONFIG, value);
	CE_SET;
}

void DLL_tx_mode() {
	uint8_t value = DLL_read_reg(DLL_CMD_R_REGISTER | DLL_REG0_STATUS);
	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_STATUS, value); // clear RX_DR or TX_DS or MAX_RT interrupt flag

	CE_CLR;

	// set PTX mode
	value = DLL_read_reg(DLL_REG0_CONFIG);
	value &= 0xFE;
	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_CONFIG, value);

	CE_SET;
}

#if DLL_HARDWARE == DLL_HARDWARE_RFM70
void DLL_set_regbank(uint8_t bank) {
	uint8_t current_bank = (DLL_read_reg(DLL_CMD_R_REGISTER | DLL_REG0_STATUS) & DLL_STATUS_RBANK) ? 1 : 0;

	if (bank != current_bank) {
		DLL_write_reg(DLL_CMD_ACTIVATE, 0x53); // toggle register bank
	}
}
#endif


void DLL_set_tx_address(uint8_t address) {
	uint8_t WriteArr[5];
	uint8_t j = 0;

	WriteArr[0] = address;
	for (j = 1; j < 5; j++) {
#ifdef __ARM_ARCH_6__
		WriteArr[j] = Address_Suffix[j-1];
#elif AVR
		WriteArr[j] = pgm_read_byte(&Address_Suffix[j-1]);
#endif

	}
	DLL_write_buf(DLL_CMD_W_REGISTER | DLL_REG0_TX_ADDR, WriteArr, 5);
}

void DLL_set_rx_address(uint8_t address) {
	uint8_t WriteArr[5];
	uint8_t j = 0;

	WriteArr[0] = address;
	for (j = 1; j < 5; j++) {
#ifdef __ARM_ARCH_6__
		WriteArr[j] = Address_Suffix[j-1];
#elif AVR
		WriteArr[j] = pgm_read_byte(&Address_Suffix[j-1]);
#endif
	}

	DLL_write_buf(DLL_CMD_W_REGISTER | DLL_REG0_RX_ADDR_P0, WriteArr, 5);
}


uint8_t DLL_send(dll_send_type type, uint8_t receiver_address, uint8_t * data, uint8_t length) {
	uint8_t status = 0;
	uint16_t timeout = DLL_SEND_TIMEOUT_MS * 10;

	if (length > DLL_MAX_PACKET_LEN)
		return 0;

	//if (receiver_address == DLL_BROADCAST_ADDRESS)
	//    return 0x10;

	DLL_tx_mode();

	DLL_set_tx_address(receiver_address);
    DLL_set_rx_address(receiver_address);

	DLL_irq_tx_ds_flag = 0;
	DLL_irq_max_rt_flag = 0;
	DLL_write_buf(type, data, length); // Writes data to buffer

	while (DLL_irq_tx_ds_flag == 0 && DLL_irq_max_rt_flag == 0 && timeout > 0) {
#ifdef __ARM_ARCH_6__
		delayMicroseconds(100);
#elif AVR
		delay_us(100);
#endif
		--timeout;
	}

	status = DLL_read_reg(DLL_REG0_STATUS); // get status: TX_DS, MAX_RT
	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_STATUS, DLL_STATUS_TX_DS | DLL_STATUS_MAX_RT); // reset
	DLL_write_reg(DLL_CMD_FLUSH_TX, 0);

	DLL_set_rx_address(DLL_address);
	DLL_set_tx_address(DLL_address);
	DLL_rx_mode();


#ifdef AVR
        DEBUG_message((uint8_t *)"dll s ", 6);
        DEBUG_message(data, length);
        DEBUG_newline();
#else
        int i = 0;
        printf("dll s ");
        for (i = 0; i < length; ++i) {
            printf("%02x ", data[i]);
        }
        printf("\n");

#endif

	return !!(status & DLL_STATUS_TX_DS);
}


void DLL_ACK_payload(uint8_t * buffer, uint8_t length) {
	DLL_write_buf(DLL_CMD_W_ACK_PAYLOAD, buffer, length);
}


uint8_t DLL_receive(uint8_t * buffer, uint8_t * length) {

	uint8_t status = DLL_read_reg(DLL_CMD_R_REGISTER | DLL_REG0_STATUS);

	if ((status & DLL_STATUS_RX_P_NO) != DLL_STATUS_RX_P_NO) {

		*length = DLL_read_reg(DLL_CMD_R_RX_PL_WID);
		if (*length > DLL_MAX_PACKET_LEN) {
#ifdef AVR
			DEBUG_message((uint8_t *)"DLL_receive, length too long\n", 31);
#endif
			*length = 0;
		} else {
			DLL_read_buf(DLL_CMD_R_RX_PAYLOAD, buffer, *length);
		}
		DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_STATUS, DLL_STATUS_RX_DR);	// clear rx interrupt flag
		if (DLL_irq_rx_counter > 0)
			--DLL_irq_rx_counter;

#ifdef AVR
		DEBUG_message((uint8_t *)"dll r ", 6);
		DEBUG_message(buffer, *length);
		DEBUG_newline();
#else
		int i = 0;
		printf("dll r ");
		for (i = 0; i < *length; ++i) {
		    printf("%02x ", buffer[i]);
		}
		printf("\n");

#endif

		return 1;
	}

	return 0;
}


void DLL_standby(uint8_t activate) {
	if (activate) {
		CE_CLR;
	}
	else {
		CE_SET;
	}
}


uint8_t DLL_init(uint8_t address, uint8_t auto_retransmission_count, uint8_t rf_channel, uint8_t air_data_rate) {
#if DLL_HARDWARE == DLL_HARDWARE_RFM70
    uint8_t i = 0;
#endif
	uint8_t j = 0;
	uint8_t WriteArr[12];

	DLL_irq_rx_counter = 0;

	DLL_address = address;

	CSN_OUTPUT;
	CE_OUTPUT;
	CE_SET;
	CSN_SET;
	IRQ_INPUT;

	delay(250);

	// ******************** write configuration register bank ********************
#if DLL_HARDWARE == DLL_HARDWARE_RFM70
	DLL_set_regbank(0);
#endif


	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_CONFIG, 0x0F); // reflect RX_DR/TX_DS/MAX_RT interrupts, enable CRC, 2 byte CRC, POWER UP, PRX

	if ((auto_retransmission_count & 0xF0) != 0)
		return 2;
	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_SETUP_RETR, 0x00 | auto_retransmission_count); // auto retransmission delay (250us) with auto_retransmission_count retries

	if ((rf_channel & 0x80) != 0)
		return 3;
	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_RF_CH, rf_channel); // channel

#if DLL_HARDWARE == DLL_HARDWARE_RFM70
	if ((air_data_rate & 0xFE) != 0)
		return 4;
	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_RF_SETUP, 0x37 | (air_data_rate << 3)); // air data rate (1M/2M), out power 5dbm, setup LNA gain
#endif
#if DLL_HARDWARE == DLL_HARDWARE_NRF24L01P
	if ((air_data_rate & 0xFC) != 0)
        return 4;
    DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_RF_SETUP, 0x06 | (air_data_rate & 0x02 << 5) | (air_data_rate & 0x01 << 3)); // air data rate (250k/1M/2M), out power 0dbm
#endif



	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_RX_PW_P0, 0x20); // Number of bytes in RX payload in data pipe0(32 byte)
	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_RX_PW_P1, 0x20); // Number of bytes in RX payload in data pipe1(32 byte)

	// prepare address prefix in temporarily array
	for(j = 1; j < 5; j++) {
#ifdef __ARM_ARCH_6__
		WriteArr[j] = Address_Suffix[j-1];
#elif AVR
		WriteArr[j] = pgm_read_byte(&Address_Suffix[j-1]);
#endif
	}

	// set unicast TX and RX address and broadcast receiving address
	WriteArr[0] = DLL_address;
	DLL_write_buf(DLL_CMD_W_REGISTER | DLL_REG0_RX_ADDR_P0, WriteArr, 5);
	WriteArr[0] = DLL_BROADCAST_ADDRESS;
	DLL_write_buf(DLL_CMD_W_REGISTER | DLL_REG0_RX_ADDR_P1, WriteArr, 5);
	WriteArr[0] = DLL_address;
	DLL_write_buf(DLL_CMD_W_REGISTER | DLL_REG0_TX_ADDR, WriteArr, 5);

#if DLL_HARDWARE == DLL_HARDWARE_RFM70
	i = DLL_read_reg(29); // read Feature Register
	if (i==0) {// i!=0 showed that chip has been actived. so do not active again.
		DLL_write_reg(DLL_CMD_ACTIVATE, 0x73); // Active
	}
#endif

	// this two registers should be initialized after
	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_DYNPD, 0x03); // enable dynamic payload length data pipe0/1
	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_FEATURE, 0x07); // enable dynmaic payload length, enable payload with ACK, enables W_TX_PAYLOAD_NOACK command


#if DLL_HARDWARE == DLL_HARDWARE_RFM70
	// ******************** write register bank 1 registers ********************
	DLL_set_regbank(1);



	// Write register 0 - 5
	for (i = 0; i <= 5; ++i) {
		for (j = 0; j < 4; ++j) {
#ifdef __ARM_ARCH_6__
			WriteArr[j] = Bank1_reg0_5[i][j];
#elif AVR
			WriteArr[j] = pgm_read_byte(&Bank1_reg0_5[i][j]);
#endif
		}
		DLL_write_buf(DLL_CMD_W_REGISTER | i, WriteArr, 4);
	}


	// write register 12
	for (i = 0; i < 4; ++i) {
#ifdef __ARM_ARCH_6__
			WriteArr[i] = Bank1_reg12_13[0][i];
#elif AVR
			WriteArr[i] = pgm_read_byte(&Bank1_reg12_13[0][i]);
#endif
	}
	DLL_write_buf(DLL_CMD_W_REGISTER | DLL_REG0_RX_ADDR_P2, WriteArr, 4);


	// write register 13
	for (i = 0; i < 4; ++i) {
#ifdef __ARM_ARCH_6__
		WriteArr[i] = Bank1_reg12_13[1][i];
#elif AVR
			WriteArr[i] = pgm_read_byte(&Bank1_reg12_13[1][i]);
#endif

	}
	DLL_write_buf(DLL_CMD_W_REGISTER | DLL_REG0_RX_ADDR_P3, WriteArr, 4);

	// write register 14
	for(j=0;j<11;j++) {
#ifdef __ARM_ARCH_6__
		WriteArr[j] = Bank1_Reg14[j];
#elif AVR
		WriteArr[j] = pgm_read_byte(&Bank1_Reg14[j]);
#endif

	}
	DLL_write_buf(DLL_CMD_W_REGISTER | DLL_REG0_RX_ADDR_P4, WriteArr, 11);


	// init done, create default settings
	delay(50);
	DLL_set_regbank(0);

#endif

	// enable interrupt
	DLL_INIT_INTERRUPT;

	// clear irq flags
	DLL_write_reg(DLL_CMD_W_REGISTER | DLL_REG0_STATUS, DLL_STATUS_TX_DS | DLL_STATUS_MAX_RT | DLL_STATUS_RX_DR);

	// flush pipes
	DLL_tx_mode();
	DLL_write_reg(DLL_CMD_FLUSH_TX, 0);
	DLL_rx_mode();
	DLL_write_reg(DLL_CMD_FLUSH_RX, 0);


#ifdef DEBUG_MODE_DLL
	DEBUG_message((uint8_t *)"DLL init done!", 16);
	DEBUG_newline();
#endif


	DLL_print();

	return 0;
}
