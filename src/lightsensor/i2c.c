#include "i2c.h"
#include "twi.h"
#include "debug.h"

void i2c_init(void) {
   TWCR |= (1<<TWEN); /* Enable TWI module */
   PORTC |= ((1<<4) | (1<<5)); /* Enable pullup */

   /* TODO: set up I2C IRQ */

   /* initialize TWI clock */
   TWSR = 0U;
   TWBR = 0x30U;       /* smallest TWBR value, see note [5] */
}

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

   PORTB = (1U<<1); /* for timing */
   /* Wait until TWINT in TWCR is set */
   while((TWCR & (1<<TWINT)) == 0) {
      /* Spin wait */
   }
   PORTB = 0; /* end timing */

   /* Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0) */
   return (TWSR & TW_STATUS_MASK);
}



static void readOnce( uint8_t addr) {

   uint8_t tmp;
   uint8_t counter;

   tmp = i2c_transmit(I2C_START);

   if( (tmp != TW_START) && (tmp != TW_REP_START) ) {
      DEBUG_puts("Start condition send Failed!");
      DEBUG_number_hex(tmp);
      DEBUG_puts(HERE);
      for(;;); /* Bail on fail */
   }

   /* Write slave address + W to TWDR */
   TWDR = ((addr)<<1) | 0;

   tmp = i2c_transmit(I2C_WRITE);
   if(tmp == TW_MT_SLA_NACK) {
      DEBUG_puts("N");
      return;
   }
   else if (tmp != TW_MT_SLA_ACK) {
      DEBUG_number_hex(tmp);
      DEBUG_puts(HERE);
      for(;;); /* Bail on fail */
   }

   /* Only reached on ACK */

   /* Send register address */
   TWDR = 0x00U;

   tmp = i2c_transmit(I2C_WRITE);
   if(tmp == TW_MT_SLA_NACK) {
      DEBUG_puts("N");
      return;
   }
   else if (tmp != TW_MT_DATA_ACK) {
      DEBUG_number_hex(tmp);
      DEBUG_puts(HERE);
      for(;;); /* Bail on fail */
   }

   /* Time to get data! */
   tmp = i2c_transmit(I2C_START);
   if(tmp != TW_REP_START) {
      DEBUG_puts("Repeated start condition send Failed!");
      DEBUG_puts(HERE);
      DEBUG_number_hex(tmp);
      for(;;); /* Bail on fail */
   }

   /* Write slave address + R to TWDR */
   TWDR = ((addr)<<1) | 1;

   tmp = i2c_transmit(I2C_WRITE);
   if(tmp == TW_MR_SLA_NACK) {
      DEBUG_puts("N");
      return;
   }
   else if (tmp != TW_MR_SLA_ACK) {
      DEBUG_number_hex(tmp);
      DEBUG_puts("? (TWSR on slave address)\n");
      for(;;); /* Bail on fail */
   }

   /* Only reached on ACK */

   /* Data bytes reading starts here */
   for(counter=0; counter<0x22U; counter++) {
      tmp = i2c_transmit(I2C_READ);
      if(tmp == TW_MR_DATA_ACK) {
#if 0
         msg = "Got ACK!\n"; */
#endif
      }
      else if(tmp == TW_MR_DATA_NACK) {
#if 0
         msg = "N"; // "Got an nak\n";
#endif
         break;
      }
      else {
         DEBUG_number_hex(tmp);
         DEBUG_puts("? (TWSR on data read)\n");
         for(;;); /* Bail on fail */
      }
   }
   /* Data bytes reading ends here */
   tmp = i2c_transmit(I2C_STOP);
}

void i2c_test(void) {
   for(;;) {
      readOnce(0x41U);
   }
}

