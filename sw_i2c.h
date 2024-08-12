#ifndef XC_I2C_H
#define	XC_I2C_H

#include<xc.h>
#define _XTAL_FREQ   4000000UL 
#define ICLK PORTCbits.RC3
#define IDAT PORTCbits.RC4
#define TIDAT TRISCbits.TRISC4

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_wb(unsigned char val);
unsigned char i2c_rb(unsigned char ack);
void i2c_acktst(unsigned char val);
#endif
