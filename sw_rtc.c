
#include"sw_i2c.h"
#include"sw_rtc.h"

volatile char date[10];
volatile char time[10];


unsigned char getd(unsigned char nn)
{
 return ((nn & 0xF0)>>4)+0x30;
}

unsigned char getu(unsigned char nn)
{
  return (nn  & 0x0F)+0x30;
}

//--------------------- Reads time and date information from RTC (PCF8563)
void rtc_r(void) 
{
  unsigned char tmp;

  i2c_start();
  i2c_wb(0xD0);
  i2c_wb(0);

  i2c_start();
  i2c_wb(0xD1);
  tmp= 0x7F & i2c_rb(1); //segundos
  time[5]=':';
  time[6]=getd(tmp);
  time[7]=getu(tmp);
  time[8]=0;

  tmp= 0x7F & i2c_rb(1); //minutos
  time[2]=':';
  time[3]=getd(tmp);
  time[4]=getu(tmp);

  tmp= 0x3F & i2c_rb(1); //horas
  time[0]=getd(tmp);
  time[1]=getu(tmp);

  tmp = i2c_rb(1); //dia semana
  date[9] = tmp & 0x07;

  tmp= 0x3F & i2c_rb(1); //dia
  date[0]=getd(tmp);
  date[1]=getu(tmp);


  tmp= 0x1F & i2c_rb(1); //mes
  date[2]='/'; 
  date[3]=getd(tmp);
  date[4]=getu(tmp);

  tmp=  i2c_rb(0); //ano
  date[5]='/';
  date[6]=getd(tmp);
  date[7]=getu(tmp);
  date[8]=0;

  i2c_stop();

}

void rtc_w(void)
{
   i2c_start();
   i2c_wb(0xD0);
   i2c_wb(0x30);//start at seconds
   i2c_wb(0x44);//11seconds
   i2c_wb(0x44);//22 minutes
   i2c_wb(0x12);//15:3 PM
   i2c_wb(0x05);//wed Tuesday
   i2c_wb(0x13);//2/5/2018 day of month 
   i2c_wb(0x12);//2/5/2018 month 5 may of .. could be an issue, datasheet says 1--12 mont, simulator 0 --11
   i2c_wb(0x23);//2/5/2018 month may of
   i2c_stop();  
   
   
   
    /*
   i2c_start();          // issue start signal
   i2c_wb(0xA2);         // address PCF8563
   i2c_wb(0);            // start from word at address 0 (configuration word)
   i2c_wb(0x20);         // write $20 to config. (pause counter...)
   i2c_wb(0);            // write 0 to cents word
   i2c_wb(0x20);         // write $20 to seconds word
   i2c_wb(0x34);         // write $30 to minutes word
   i2c_wb(0x11);         // write $11 to hours word
   i2c_wb(0x24);         // write $24 to day word
   i2c_wb(0x04);         // write $04 to weekday
   i2c_wb(0x02);         // write $08 to month
   i2c_wb(0x14);         // write $08 to year
   i2c_stop();           // issue stop signal
   i2c_start();          // issue start signal
   i2c_wb(0xA2);         // address PCF8530
   i2c_wb(0);            // start from word at address 0
   i2c_wb(0);            // write 0 to config word (enable counting)
   i2c_stop();           // issue stop signal
     * */

}



