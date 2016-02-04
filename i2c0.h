#ifndef _I2C0_H_
#define _I2C0_H_
 
/* includes */
#include <stdlib.h>
#include <stdint.h>
#include "LPC17xx.h"
 
/* defines */
#define MODE_100kbps 100000
#define MODE_400kbps 400000
#define MODE_1Mbps 1000000
 
/* typedefs */
 
/* functions */
 
// Initialize the I2C hardware.
// see 'readme'
void i2c0_init(uint32_t i2c_freq, uint8_t int_pri);
 
// Performs a I2C master send function.
// Returns the number of bytes sent successfully.
// Returns 0xFFFFFFFF if slave did not response on bus.
// This is NOT thread safe.
uint32_t i2c0_send(uint8_t address, uint8_t* buffer, uint32_t length);
 
// Performs a I2C master receive function.
// Returns the number of bytes received successfully.
// Returns 0xFFFFFFFF if slave did not response on bus.
// This is NOT thread safe.
uint32_t i2c0_recv(uint8_t address, uint8_t* buffer, uint32_t length);
 
/*** DEBUG ***/uint8_t* i2c_buf(void);
/*** DEBUG ***/uint32_t i2c_pos(void);
 
#endif /* _I2C0_H_ */
