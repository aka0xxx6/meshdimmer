#ifndef I2C_H
#define I2C_H

#include <inttypes.h>

/*@ignore@*/
typedef uint8_t bool;
#define true ((bool) 1);
#define false ((bool) 0);
/*@end@*/


void i2c_init(void);
bool i2c_isDone(void);
uint16_t i2c_getValue(void);

#endif /* I2C_H */
