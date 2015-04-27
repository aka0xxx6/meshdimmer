#include "i2c.h"
#include "twi.h"
#include "debug.h"

#define I2C_ADDRESS ((uint8_t) 0x41)

enum i2c_command {
   I2C_START = 0,
   I2C_WRITE = 1,
   I2C_READ  = 2,
   I2C_STOP  = 3
};

static uint8_t i2c_transmit(enum i2c_command cmd) {
   switch(cmd) {
      case I2C_START:
         /* Send Start Condition */
         TWCR = (uint8_t) ((1 << TWINT) | (1 << TWSTA) | (1 << TWEN));
         break;
      case I2C_WRITE:
         /* Send data, without ACK */
         TWCR = (uint8_t) ((1 << TWINT) | (1 << TWEN));
         break;
      case I2C_READ:
         /* Read Data and ACK afterwards */
         TWCR = (uint8_t) ((1 << TWINT) | (1 << TWEN) | (1<<TWEA));
         break;
      case I2C_STOP:
         /* Send Stop Condition */
         TWCR = (uint8_t) ((1 << TWINT) | (1 << TWEN) | (1 << TWSTO));
         return 0;
   }

   PORTB = (uint8_t) (1U<<1); /* for timing */
   /* Wait until TWINT in TWCR is set */
   while((TWCR & (1<<TWINT)) == 0) {
      /* Spin wait */
   }
   PORTB = 0; /* end timing */

   /* Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0) */
   return (TWSR & TW_STATUS_MASK);
}


static bool i2c_start(uint8_t addr) {
   /*
      Send I2C start condition and request I2C address addr
      Returns true on success, false otherwise.

      If the low bit of addr is set, the request is a read request, otherwise it
      is a write request.
    */

   uint8_t tmp;
   uint8_t nak, ack; /* Used in return code check, the codes are different for
                        read/transmit mode. */

   if((addr & 1) != 0) {
      /* Master read */
      nak = TW_MR_SLA_NACK;
      ack = TW_MR_SLA_ACK;
   }
   else {
      /* Master transmit */
      nak = TW_MT_SLA_NACK;
      ack = TW_MT_SLA_ACK;
   }

   /* Send and check I2C start condition */
   tmp = i2c_transmit(I2C_START);
   if( (tmp != TW_START) && (tmp != TW_REP_START) ) {
      DEBUG_puts("Start condition send Failed!");
      DEBUG_number_hex(tmp);
      DEBUG_puts(HERE);
      for(;;); /* Panic on fail */
   }

   /* Address is transmitted here */
   TWDR = addr;
   tmp = i2c_transmit(I2C_WRITE);

   if(tmp == nak) {
      /* Got nack, address not reachable */
      return false;
   }
   else if (tmp != ack) {
      DEBUG_number_hex(addr);
      DEBUG_number_hex(ack);
      DEBUG_number_hex(tmp);
      DEBUG_puts(HERE);
      for(;;); /* Panic on fail */
   }

   /* Got ack sucessfully. */
   return true;
}

static uint8_t i2c_write(uint8_t len, uint8_t *data) {
   /*
      Returns the number of successfully written bytes
    */

   uint8_t counter, tmp;

   for(counter=0; counter<len; counter++) {
      TWDR =  data[counter];
      tmp = i2c_transmit(I2C_WRITE);
      if (tmp != TW_MT_SLA_ACK && tmp != TW_MT_DATA_ACK) {
         DEBUG_number_hex(tmp);
         DEBUG_puts(HERE);
         break;
      }
   }
   return counter;
}

static uint8_t i2c_read(uint8_t len, /*@out@*/ uint8_t data[]) {
   /*
      Returns the number of bytes read
    */
   uint8_t counter, tmp;
   data[0] = 0U; /* Make lint happy */

   for(counter=0; counter < len; counter++) {
      tmp = i2c_transmit(I2C_READ);
      if(tmp != TW_MR_DATA_ACK) {
         DEBUG_number_hex(tmp);
         DEBUG_puts(HERE);
         break;
      }
      data[counter] = TWDR;
   }
   return counter;
}

static bool i2c_writeWithAddress(uint8_t addr, uint8_t reg,
      uint8_t len, uint8_t data[])
{
   /*
      Returns true on success, false otherwise.
    */
   if(!i2c_start(((addr)<<1) | 0)) {
      DEBUG_puts(HERE);
      return false;
   }
   if(i2c_write((uint8_t)1U, &reg) != (uint8_t)1U) {
      DEBUG_puts(HERE);
      return false;
   }
   if(i2c_write(len, data) != len) {
      DEBUG_puts(HERE);
      return false;
   }
   return true;
}


static bool i2c_readWithAddress(uint8_t addr, uint8_t reg,
      uint8_t len, /*@out@*/ uint8_t data[] )
{
   /*
      Returns true on success, false otherwise.
    */
   data[0] = 0U; /* Make lint happy */

   if(!i2c_start(((addr)<<1) | 0)) {
      DEBUG_puts(HERE);
      return false;
   }
   if(i2c_write((uint8_t)1U, &reg) != (uint8_t)1U) {
      DEBUG_puts(HERE);
      return false;
   }
   if(!i2c_start(((addr)<<1) | 1)) {
      DEBUG_puts(HERE);
      return false;
   }
   if(i2c_read(len, data) != len) {
      DEBUG_puts(HERE);
      return false;
   }
   return true;
}

static void protocolSetup( uint8_t addr) {

   uint8_t buf[8];

   /* Read Interrupt Status register (register 0x00) */
   if(!i2c_readWithAddress(addr, (uint8_t)0x00U, (uint8_t)1U, buf)) {
      DEBUG_puts(HERE);
      return;
   }

   /* Set thresholds */
   buf[0] = 0;
   buf[1] = 0;
   buf[2] = 0;
   buf[3] = 0;
   buf[4] = 0;
   /* Write Interrupt thresholds, registers 0x14 - 0x18 */
   if(!i2c_writeWithAddress(addr, (uint8_t)0x14U, (uint8_t)5U, buf)) {
      DEBUG_puts(HERE);
      return;
   }

   /* Write Ambient Configuration register (register 0x02) */
   buf[0] = (uint8_t) 0x02U;
   if(!i2c_writeWithAddress(addr, (uint8_t)0x02U, (uint8_t)1U, buf)) {
      DEBUG_puts(HERE);
      return;
   }

   /* Write 0x21 to the Main Configuration register (register 0x01) */
   buf[0] = (uint8_t) 0x21U;
   if(!i2c_writeWithAddress(addr, (uint8_t)0x01U, (uint8_t)1U, buf)) {
      DEBUG_puts(HERE);
      return;
   }

   /* Data bytes reading ends here */
   (void) i2c_transmit(I2C_STOP);
   DEBUG_puts("All ok!");
}

uint16_t i2c_getValue(void) {
   uint16_t val;
   uint8_t buf[12];

   /* Clear interrupt */
   if(!i2c_readWithAddress(I2C_ADDRESS, (uint8_t)0x00U, (uint8_t)1U, buf)) {
      DEBUG_puts(HERE);
      return 0;
   }
   /* Read ambient light value */
   if(!i2c_readWithAddress(I2C_ADDRESS, (uint8_t)0x04U, (uint8_t)12U, buf)) {
      DEBUG_puts(HERE);
      return 0;
   }

   val = (((uint16_t) buf[0]) << 8) | ((uint16_t) buf[1]);

  /* Write 0x03 to Ambient Configuration register (register 0x02) */
   buf[0] = (uint8_t) 0x02U;
   if(!i2c_writeWithAddress(I2C_ADDRESS, (uint8_t)0x02U, (uint8_t)1U, buf)) {
      DEBUG_puts(HERE);
      return 0;
   }


   (void) i2c_transmit(I2C_STOP);

   return val;
}

void i2c_init(void) {
   TWCR |= (1<<TWEN); /* Enable TWI module */
   PORTC |= ((1<<3) | (1<<4) | (1<<5)); /* Enable pullup */

   /* initialize TWI clock */
   TWSR = (uint8_t) 0U;
   TWBR = (uint8_t) 0x10U;       /* smallest TWBR value, see note [5] */

   protocolSetup(I2C_ADDRESS);
}

bool i2c_isDone(void) {
   return (PINC & (1 << 3)) == 0;
}
