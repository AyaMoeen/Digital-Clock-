/*
 * File:   timers.c
 * Author: 
 *
 * Created on February 26, 2019, 8:29 AM
 */
#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#include <xc.h>
#include <stdio.h>
#include "lcd_x8.h"
#include "sw_rtc.h"
#include "sw_i2c.h"
#include "my_adc.h"
#include "my_ser.h"
#include "my_pwm.h"
#define STARTVALUE  3036
#define MAX_COMMAND_LENGTH 5
#define TIME_INCREMENT_SECONDS 1
char receivedCommand[MAX_COMMAND_LENGTH];  // Assuming the maximum command length is 5 characters
char *wkday[8] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Err"};
// Function prototypes
void readRTCandUpdateLCD(void);
void readAnalogInputs(void);
void setFanSpeed(int percentage);
void executeCommand(char command);
void setupPorts(void);
void delay_ms(unsigned int n);
unsigned int RPS_count = 0;
unsigned char seconds = 0;
unsigned char minutes = 0;
unsigned char hours = 0;
unsigned char weekday = 0;
unsigned char dayOfMonth = 0;
unsigned char month = 0;
unsigned char year = 0;
unsigned int raw_val1 =0;
unsigned int percentage1=0;
///////////////////////////////////////////////////////////////////////////////
void initADC() {
    // Initialize ADC module
    ADCON2=0;
    ADCON0bits.ADON = 1;  // Enable ADC module
    ADCON2bits.ADFM = 1;  // Right justify the result
    ADCON1bits.VCFG0 = 0; // Use VDD as reference voltage
    ADCON1bits.VCFG1 = 0; // Use VSS as reference voltage
}


void initTimers01(void) {
    T0CON = 0;
    //T0CONbits.T0CS = 0;
    //T0CONbits.PSA = 0;
    //T0CONbits.T08BIT = 1;
    INTCONbits.T0IF = 0;
    T0CONbits.T0PS0 = 1; // 16 prescalar
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS2 = 1;
    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);

    T1CONbits.TMR1CS = 1; //external clock ,emasuring the speed of the fan in RPS
    T1CONbits.T1CKPS1 = 0;
    T1CONbits.T1CKPS0 = 0;


    TMR1H = 0;
    TMR1L = 0;
    INTCONbits.GIE = 1; //enable only timer 0 interrupt
    INTCONbits.T0IE = 1;
    T1CONbits.TMR1ON = 1;
    T0CONbits.TMR0ON = 1;

}
void lcdPrint(void){
    char Buffer[32]; // for sprintf
    float AN[3];     // To store the voltages of AN0, AN1, AN2
    int raw_val;
    unsigned char channel;
    float voltage;
      int RPS;
    
      rtc_r();
        lcd_gotoxy(1, 1);
        lcd_puts((char *) date);
        lcd_gotoxy(4, 2);
        lcd_puts((char *) time);
        
        PORTCbits.RC5 = !PORTCbits.RC5;
        delay_ms(1000); //read ADC AN0,AN1, AN2 every 2 seconds
        for (channel = 0; channel < 3; channel++) {
            // read the adc voltage
            voltage = read_adc_voltage((unsigned char) channel);
            AN[channel] = voltage; // store in array AN0--AN2      
        }
        lcd_gotoxy(1, 3);
        sprintf(Buffer, "%3.2f, %3.2f, %3.2f", AN[0], AN[1],AN[2]);
        lcd_puts(Buffer);
        lcd_gotoxy(10, 1);
        RPS = RPS_count;
        sprintf(Buffer, "S=%4.2f", RPS/7.0); // Display Speed
        lcd_puts(Buffer);       // speed = Revolution per second
        lcd_gotoxy(1, 4);
        sprintf(Buffer, "D=%4d %6.2f", raw_val1, (raw_val1 * 100.0) / 1023.0);
        lcd_puts(Buffer); // Above displays duty 0--1023, and also as percentage
        lcd_gotoxy(14, 4);
        lcd_putc('%');
    
}
void __interrupt(high_priority) highIsr(void)//new syntax
{
    RPS_count = ((unsigned int) TMR1H << 8) | (TMR1L); //

    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);
    TMR1H = 0;
    TMR1L = 0;
    INTCONbits.T0IF = 0;

}
void configure(void){
      ADCON1 = 0x0F;
    TRISC = 0x80;
    TRISD = 0;
    TRISE = 0;
    delay_ms(100);
    PORTD = 0;
    i2c_init();
    setupSerial();
    rtc_w();
    setupPorts();
    lcd_init();
    init_adc_no_lib();
    init_pwm1();
    
}
 
 void delay_ms(unsigned int n)
{
    int i;
    for (i=0; i < n; i++){
         __delaywdt_ms(1) ; 
    }
}

void updateRTC(void) {
    i2c_start();
    i2c_wb(0xD0);
    i2c_wb(0);
    i2c_wb((seconds / 10 << 4) + seconds % 10);  // Update seconds
    i2c_wb((minutes / 10 << 4) + minutes % 10);  // Update minutes
    i2c_wb((hours / 10 << 4) + hours % 10);      // Update hours
    i2c_wb(weekday);                              // Update weekday
    i2c_wb((dayOfMonth / 10 << 4) + dayOfMonth % 10);  // Update day of month
    i2c_wb((month / 10 << 4) + month % 10);      // Update month
    i2c_wb((year / 10 << 4) + year % 10);        // Update year
    i2c_stop();
    
}
///////////////////////////////////////////////////////////////////////////////
void executeWriteCommand(char commandType, unsigned char value) {
    switch (commandType) {
        case 's':
            // Write Seconds
            if (value <= 59) {
                seconds = value;
            }
            break;
        case 'm':
            // Write Minutes
            if (value <= 59) {
                minutes = value;
            }
            break;
        case 'h':
            // Write Hours
            if (value <= 23) {
                hours = value;
            }
            break;
        case 'w':
            // Write Weekday
            if (value >= 1 && value <= 7) {
                weekday = value;
            }
            break;
        case 'd':
            // Write Day of Month
            if (value >= 1 && value <= 31) {
                dayOfMonth = value;
            }
            break;
        case 'M':
            // Write Month
            if (value >= 1 && value <= 12) {
                month = value;
               
            }
            break;
        case 'y':
            // Write Year
            if (value <= 99) {
                year = value;
            }
            break;
        default:
            lcd_putc('\f'); //clears the display
             char Buffer[32];
            sprintf(Buffer,"INVALID COMMAND\n");
            lcd_putc(Buffer);
            delay_ms(1000);
            break;
    }
}

// Function to execute PWM Write Command
void executePWMCommand(unsigned int percentage) {
    // Write PWM Percentage
    if (percentage <= 99) {
         float tmp = percentage*1023.0/100.0;//scale 0--100 to 0--1023
    raw_val1 = (int)(tmp +0.5); // for rounding
    if ( raw_val1> 1023) raw_val1 = 1023;// Do not exceed max value
    set_pwm1_raw(raw_val1);
        // Implement your PWM control logic here
    }
}
void main(void) {
    unsigned char v = 0;
   unsigned char k = 0;
   unsigned char RecvedChar = 0;
   unsigned char Command=0;
   unsigned char Value1,Value2=0;
   char Buffer[128]; 
   int raw_val;
   float AN[3];     // To store the voltages of AN0, AN1, AN2
   unsigned char channel;
   int RPS;
   float voltage;
   configure();
   lcd_putc('\f');
   initTimers01();    // These will be used to measure the speed
   TRISCbits.RC0 = 1; //Timer1 clock

   while (1) {
        // while(!(is_byte_available()));
          if (is_byte_available()) { // Read serial, if receive S
                RecvedChar = read_byte_no_lib(); // Then start sending to serial
                if (RecvedChar == '<') 
                {
                     while(!(is_byte_available()));
                   //  if (is_byte_available()) { 
                     RecvedChar = read_byte_no_lib(); 
                     if (RecvedChar == 'R' || RecvedChar =='r') 
                  {
                         while(!(is_byte_available()));
                         RecvedChar=read_byte_no_lib();
                                            if(RecvedChar=='t'){
                                                
                                                while(!(is_byte_available()));
                         RecvedChar=read_byte_no_lib();
                                            if(RecvedChar=='>'){
                         send_string_no_lib("\r\n");
                     // rtc_r();
                      sprintf(Buffer, time);
                      send_string_no_lib((unsigned char *)Buffer);
                      send_string_no_lib("\r\n");
                      continue;
                      
                     }
                                            }
                         
                     else if (RecvedChar == 'D') 
                  {
                          while(!(is_byte_available()));
                         RecvedChar=read_byte_no_lib();
                                            if(RecvedChar=='>'){
                      send_string_no_lib("\r\n");
                      //rtc_r();
                      sprintf(Buffer, date);
                      send_string_no_lib((unsigned char *)Buffer);
                      send_string_no_lib("\r\n");
                       continue;
                                            } }
                     else if (RecvedChar == 'A') 
                  {
                          while(!(is_byte_available()));
                         RecvedChar=read_byte_no_lib();
                                            if(RecvedChar=='>'){
                      send_string_no_lib("\r\n");
                      for (channel = 0; channel < 3; channel++) {
                      voltage = read_adc_voltage((unsigned char) channel);
                      AN[channel] = voltage; // store in array AN0--AN2
                      }
                      sprintf(Buffer, "AN0= %3.2f volt, AN1 = %3.2f volt, AN2 = %3.2f volt,\r",AN[0],AN[1],AN[2]);
                      send_string_no_lib((unsigned char *)Buffer);
                      send_string_no_lib("\r\n");
                       continue;
                                            }}
                     else if (RecvedChar == 'P') 
                  {
                          while(!(is_byte_available()));
                         RecvedChar=read_byte_no_lib();
                                            if(RecvedChar=='>'){
                      send_string_no_lib("\r\n");
                      raw_val = read_adc_raw_no_lib(0); // read raw value for POT1 
                      set_pwm1_raw(raw_val);
                      sprintf(Buffer, "D=%5d ,%6.2f", raw_val1, (raw_val1 * 100.0) / 1023.0);
                      send_string_no_lib((unsigned char *)Buffer);// Above displays duty 0--1023, and also as percentage 
                      send_string_no_lib( "%");
                      send_string_no_lib("\r\n");
                       continue;
                                            }}
                     else if (RecvedChar == 'F') 
                  {
                          while(!(is_byte_available()));
                         RecvedChar=read_byte_no_lib();
                                            if(RecvedChar=='>'){
                      send_string_no_lib("\r\n");
                       RPS = RPS_count;
                      sprintf(Buffer, "Speed=%6.2f RPS\n", RPS/7.0); // Display Speed
                      send_string_no_lib((unsigned char *)Buffer);
                      send_string_no_lib("\r\n");
                       continue;
                                            }}
                
                //}  
                }
//                else if (RecvedChar == '<') {/////// for write 
//                 if (is_byte_available()) { 
//                     RecvedChar = read_byte_no_lib(); 
                else if (RecvedChar == 'W') {
                         while(!(is_byte_available()));
                      //   if(is_byte_available){
                             Command=read_byte_no_lib();
                             switch (Command) {                             
                        case 's':
                        case 'm':
                        case 'h':
                        case 'w':
                        case 'd':
                        case 'M':
                        case 'y':
                            while(!(is_byte_available()));
                         //   if(is_byte_available){
                             Value1=read_byte_no_lib();
                             while(!(is_byte_available()));
                        //     if(is_byte_available){
                             Value2=read_byte_no_lib();
                              while(!(is_byte_available()));
                         RecvedChar=read_byte_no_lib();
                                            if(RecvedChar=='>'){
                             if((!is_byte_available()));
                             send_string_no_lib("\r\n");
                            executeWriteCommand(Command, (Value1 - '0') * 10 + (Value2 - '0'));
                                            }
                            break;
                                 //   }
                        //    }
                            // Valid Write command type
                            
                        case 'P':
                            // PWM Write command
                            while(!(is_byte_available()));
                        //    if(is_byte_available){
                             Value1=read_byte_no_lib();
                             while(!(is_byte_available()));
                        //     if(is_byte_available){
                             Value2=read_byte_no_lib();
                              if((!is_byte_available()));
                               while(!(is_byte_available()));
                         RecvedChar=read_byte_no_lib();
                                            if(RecvedChar=='>'){
                            send_string_no_lib("\r\n");
                            percentage1=(unsigned int)(Value1 - '0') * 10 + (unsigned int)(Value2 - '0');
                            executePWMCommand(percentage1);}
                            break;
                             //       }
                        //    }
                        default:
                            // Invalid command type
                            // Handle error or ignore
                            break;
                    } continue;
                         //   }
                        } //write
                }
                
            }
          seconds += TIME_INCREMENT_SECONDS;
        if (seconds >= 60) {
            seconds = 0;
            minutes++;
            if (minutes >= 60) {
                minutes = 0;
                hours++;
                if (hours >= 24) {
                    hours = 0;
                    dayOfMonth++;
                    if (dayOfMonth > 31) {
                        dayOfMonth = 1;
                        month++;
                        if (month > 12) {
                            month = 1;
                            year++;
                            if (year > 99) {
                                year = 0; // Reset to 0 for simplicity, adjust as needed
                            }
                        }
                    }
                }
            }
        }
           updateRTC();
        lcdPrint();
       // delay_ms(2000);
        CLRWDT(); 
    }

    return;
}
void setupPorts(void)
{
    ADCON0 =0; //Disable ADC
    ADCON1 = 0x0C;  // 3 analog input 
    TRISB = 0xFF;   // all pushbuttons are inputs
    TRISC = 0x80;   // RX input , others output
    TRISA = 0xFF;   // All inputs
    TRISD = 0x00;   // All outputs
    TRISE= 0x00;    // All outputs
}