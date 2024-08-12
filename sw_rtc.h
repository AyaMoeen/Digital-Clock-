#ifndef XC_RTC_H
#define	XC_RTC_H

extern volatile char date[10];  /*dd/mm/aa*/
extern volatile char time[10];  /*hh:mm:ss*/

unsigned char getd(unsigned char nn);
unsigned char getu(unsigned char nn);
void rtc_w(void);

void rtc_r(void);
#endif

