/*
 * File:   main.c
 * Author: X220
 *
 * Created on 2017/12/31, 下午 11:57
 */
#pragma config OSC = INTIO67      // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 1        // Watchdog Timer Postscale Select bits (1:1)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)


#include <xc.h>
#include <pic18f4520.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
//#include "lcd.h"
#include "gps.h"
#include "xlcd.h"

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 1000000UL
#endif
#define NUM_OF_MODES 5
void Delay_us(int us);
void Data(char Value);           
void Cmd(int Value);            
void Send2Lcd(const char Adr, const char *Lcd);
void LCD_String_xy(char row,char pos,const char *msg);
void MSdelay(unsigned int val);
void LCD_Command(char cmd );
void LCD_Char(char dat);
void LCD_String(const char *msg);
const char header_GPRMC[] = "GPRMC";
const char header_GPGGA[] = "GPGGA";

volatile uint8_t mode = 0;
char buf[2][120];
unsigned char oled_buf[40];
volatile bool both_sentence_ready = 0;
volatile bool test = 0;
char *token, *speedy;
float distance = 0.0;
extern char lat, lon, mag;

void __interrupt(low_priority) externInterrupt0(void)
{
    if (INTCON3bits.INT1IE && INTCON3bits.INT1IF) {
        INTCON3bits.INT1IE = 0;
        INTCON3bits.INT1IF = 0;
        TMR1 = 54597; // around 0.35 sec
        PIE1bits.TMR1IE = 1;
        PIR1bits.TMR1IF = 0;
        T1CONbits.TMR1ON = 1;
        mode = (mode + 1) % NUM_OF_MODES;
    } else if (PIE1bits.TMR1IE && PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0;
        PIE1bits.TMR1IE = 0;
        T1CONbits.TMR1ON = 0;
        INTCON3bits.INT1IE = 1;
        INTCON3bits.INT1IF = 0;
    }
}

void nmea_read(void)
{
    both_sentence_ready = true;
    char c = RCREG;
    static bool sentence_start = 0;
    static unsigned char line_idx = 0; // to select which line of buf to write
    static unsigned char idx = 0;
//    while(1){
//        Send2Lcd(0xc0,c);
//    }
    
    if (c == '$') { 
        sentence_start = 1;
        idx = 0;
//        buf[line_idx][idx++] = c;
    } 

    if (!sentence_start) return;
    if (c == '\n' && sentence_start) {
        sentence_start = 0;
        buf[line_idx][idx++] = c;
        buf[line_idx][idx] = '\0';
 
        if (buf[line_idx][1] == 'G' && buf[line_idx][2] == 'P' && buf[line_idx][3] == 'R' && buf[line_idx][4] == 'M' && buf[line_idx][5] == 'C'){
            line_idx++;
        }
        else if (buf[line_idx][1] == 'G' && buf[line_idx][2] == 'P' && buf[line_idx][3] == 'G' && buf[line_idx][4] == 'G' && buf[line_idx][5] == 'A'){
            line_idx++;
        }

        if (line_idx == 2) {
//            INTCONbits.GIEH = 0;
//            INTCONbits.GIEL = 0;
//            PIE1bits.RCIE = 0;
            RCSTAbits.CREN = 0;
            line_idx = 0;
            both_sentence_ready = 1;
            while(1){
                token =  strtok(buf[0], ",");
                strcat(token,"\0");
                Send2Lcd(0x80,"SPEED");
                if(strcmp(token,"$GPGGA") == 0){

                token =  strtok(NULL, ",");
                token =  strtok(NULL, ",");
                token =  strtok(NULL, ",");
                token =  strtok(NULL, ",");
                token =  strtok(NULL, ",");
                token =  strtok(NULL, ",");
                speedy =  strtok(NULL, ",");
                sprintf(speedy,"%3.2f km/hr",atof(speedy)*1.850f);
                Send2Lcd(0xc0, speedy);
                    //speedy++;
                }
                else if(strcmp(token, "$GPRMC") == 0){
                    //token =  strtok(buf[1], ",");
                    token =  strtok(NULL, ",");
                    token =  strtok(NULL, ",");
                    token =  strtok(NULL, ",");
                    token =  strtok(NULL, ",");
                    token =  strtok(NULL, ",");
                    token =  strtok(NULL, ",");
                    speedy =  strtok(NULL, ",");
                    sprintf(speedy,"%3.2f km/hr",atof(speedy)*1.850f);
                    Send2Lcd(0xc0, speedy);
                    //speedy++;
                }
            }
        }
        return;
    }
 //Send2Lcd(0xc0,token);
    buf[line_idx][idx++] = c;
}

void __interrupt(high_priority) uartInterrupt(void)
{
    if (PIE1bits.RCIE && PIR1bits.RCIF) {
        if (RCSTAbits.OERR == 1) {
            RCSTAbits.OERR = 0;
        }
        nmea_read();
        PIR1bits.RCIF = 0;
    }
    return;
}

char *itoa(char *buffer, int i) {
    unsigned int n;
    unsigned int negate = 0;
    int c = 6;

    if (i < 0) {
    negate=1;
    n = -i;
    } else if (i == 0) {
    buffer[0] = '0';
    buffer[1] = 0;
    return buffer;
    } else {
    n = i;
    }
    buffer[c--] = 0;
    do {
    buffer[c--] = (n % 10) + '0';
    n = n / 10;
    } while (n);
    if (negate) {
    buffer[c--] = '-';
    }
    return &buffer[c+1];
}

void uart_init(void)
{
    TRISCbits.TRISC6 = 1;  // Setting by data sheet
    TRISCbits.TRISC7 = 1;
    OSCCONbits.IRCF = 4;   // 1MHz
    BAUDCONbits.BRG16 = 1; // Read Baud rate table
    TXSTAbits.BRGH = 1;
    SPBRG = 25;

    // Serial enable
    TXSTAbits.SYNC = 0;    // choose the async moode
    RCSTAbits.SPEN = 1;    // open serial port

    //setting TX/RX
    PIR1bits.TXIF = 0;
    PIR1bits.RCIF = 0;
    TXSTAbits.TXEN = 0;    // Disable Tx
    RCSTAbits.CREN = 1;    // Enable Rx
    //setting TX/RX interrupt
    PIE1bits.TXIE = 0;     // Tx interrupt
    IPR1bits.TXIP = 0;     // Setting Tx as high/low priority interrupt
    PIE1bits.RCIE = 1;     // Rx interrupt
    IPR1bits.RCIP = 1;     // Setting Rc as high/low priority interrupt
}

void timer1_extInt1_init(void)
{
    IPR1bits.TMR1IP = 0;
    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 0;

    T1CONbits.RD16 = 1; // 16 bit timer1
    T1CONbits.T1CKPS = 3; // 1:8 prescale

    INTCON3bits.INT1IP = 0;
    INTCON3bits.INT1IF = 0;
    INTCON2bits.INTEDG1 = 1; // rising edge
    INTCON3bits.INT1IE = 1;
}
void lcd_initial()
{
    TRISC=0X00;                  /* PORTC(control lines) configured as o/p  */
    TRISD=0X00;                  /* PORTD(data lines) configured as o/p     */
    TRISCbits.RC4 = 1;
    TRISCbits.RC5 = 1;
    OSCCONbits.IRCF0 = 0;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;//4M
    T2CONbits.TMR2ON = 1;
    T2CONbits.T2OUTPS3 = 1;
    T2CONbits.T2OUTPS2 = 1;
    T2CONbits.T2OUTPS1 = 1;
    T2CONbits.T2OUTPS0 = 1;//16
    T2CONbits.T2CKPS1 = 1;//16
    PIR1bits.TMR2IF = 0;
    PIE1bits.TMR2IE = 1;
    IPR1bits.TMR2IP = 1;
    INTCONbits.GIE = 1;
    PR2 = 39;
    Delay_us(25);              
    Cmd(0X38);                   /* Double Line Display Command             */
    Cmd(0X0C);                   /* Display ON Command                      */
    Cmd(0X01);                   /* Clear Display Command                   */
    Cmd(0X06);                   /* Auto Increment Location Address Command */
    
    //char str[]="21.89" ;
    
}
/*************************************************************************
* Function    : Cmd                                                      *
*                                                                        *
* Description : Function to send a command to LCD                        *
*                                                                        *
* Parameters  : Value - command to be sent                               *
**************************************************************************/
void Cmd(int Value)
{
    PORTD = Value;               /* Write the command to data lines         */
    RC0   = 0;                   /* RS-0(command register)                  */
    RC1   = 1;                   /* E-1(enable)                             */
    Delay_us(25);                
    RC1   = 0;                   /* E-0(enable)                             */
}
 
/*************************************************************************
* Function    : Data                                                     *
*                                                                        *
* Description : Function to display single character on LCD              *
*                                                                        *
* Parameters  : Value - character to be displayed                        *
**************************************************************************/
void Data(char Value)
{
    volatile char v = Value;
    PORTD = Value;               /* Write the character to data lines       */
    RC0   = 1;                   /* RS-1(data register)                     */
    RC1   = 1;                   /* E-1(enable)                             */
    Delay_us(25);                
    RC1   = 0;                   /* E-0(enable)                             */
}
 
/*************************************************************************
* Function    : Send2LCD                                                 *
*                                                                        *
* Description : Function to display string on LCD                        *
*                                                                        *
* Parameters  : Adr - location                                           *
*               String to be displayed                                   *
**************************************************************************/
void Send2Lcd(const char Adr, const char *Lcd)
{
    Cmd(Adr);                    /* Address of location to display string   */
    while(*Lcd!='\0')            /* Check for termination character         */
    {    
        Data(*Lcd);                 /* Display the character on LCD            */    
        Lcd++;                      /* Increment the pointer                   */
    }
}
 
/*************************************************************************
* Function    : Delay_us                                                 *
*                                                                        *
* Description : Function for 1 microsecond delay                         *
*                                                                        *
* Parameter   : us - delay in microseconds                               *
**************************************************************************/
void Delay_us(int us)
{
    us=us>>1;
    while(us!=1)
    us--;
}

void main(void)
{

    __delay_ms(400);
    lcd_initial();
    
    GPS_common_init();

    timer1_extInt1_init();
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;
    RCONbits.IPEN = 1;
    uart_init();

    for(;;){
 
        }
}