#include "sw_i2c.h"

void delay(void)
{
      __delaywdt_us(20); 
 /* #asm
    CLRWDT ;
     NOP;
     NOP;
     NOP;
     NOP;
     NOP;
     NOP;
     NOP;
     NOP;
     NOP;
     NOP;
     NOP;
     NOP;
  #endasm
  * */
}

void i2c_init(void)
{
  TIDAT=0;
  ICLK=1;
  IDAT=1;
 //  delay();//raed
}

void i2c_start(void)
{
  ICLK=1;
  IDAT=1;
  delay();
  IDAT=0;
  delay();
}

void i2c_stop(void)
{
  ICLK=1;
  IDAT=0;
  delay();
  IDAT=1;
  delay();
}

void i2c_wb(unsigned char val)
{
  unsigned char i;
  ICLK=0;
  for(i=0;i<8;i++)
  {
    IDAT=((val>>(7-i))& 0x01);
   // delay();//added
    ICLK=1;
    delay();
    ICLK=0;
     // delay();//raed
  }	
  IDAT=1;
  delay();
  ICLK=1;
  delay();
  ICLK=0;
  // delay();//raed
}

unsigned char i2c_rb(unsigned char ack)
{
  char i;
  unsigned char ret=0;

  ICLK=0;
  IDAT=1;
  TIDAT=1;
 // IDAT=1;raed
   // delay();//raed
  for(i=0;i<8;i++)
  {
    ICLK=1;
    delay();
    ret|=(IDAT<<(7-i));
    ICLK=0;
  // delay();//raed
  }
  TIDAT=0;
  CLRWDT();//delay on cycle
  if(ack)
    IDAT=0;
  else
	IDAT=1;
  delay();
  ICLK=1;
  delay();
  ICLK=0;
//  delay();//raed
  return ret;
}




