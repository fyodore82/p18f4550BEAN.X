/* 
 * File:   p18f4550main.c
 * Author: fedor
 *
 * Created on 12 ?????? 2014 ?., 10:51
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "../XC8USB/src/BEAN.h"
#include "../i2c/i2c/i2c_my.h"
//#include "i2c_p18f4550.h"
//#include <delays.h>


/** CONFIGURATION **************************************************/

// CONFIG1L
#pragma config PLLDIV = 2       // PLL Prescaler Selection bits (Divide by 2 (8 MHz oscillator input))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 2       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
//#pragma config FOSC = INTOSCIO_EC  // Oscillator Selection bits (HS oscillator, PLL enabled (HSPLL))
#pragma config FOSC = HSPLL_HS  // Oscillator Selection bits (HS oscillator, PLL enabled (HSPLL))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting)
#pragma config VREGEN = ON      // USB Voltage Regulator Enable bit (USB voltage regulator enabled)

// CONFIG2H
#pragma config WDT = OFF         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

unsigned char BEANFirstByteInSnd, BEANNumToSend, BEANDelayBtwCmd, Tmr1Repeat;

void OutRet (char addr, char *ret, char Read)
{
    char gie = INTCONbits.GIE;
    INTCONbits.GIE = 0x0;               // Disable ALL interrupts
    EECON1 = 0b00000000;     // Access EEPROM
    EEADR = addr;               // Address EEPROM
    if (Read)
    {
        EECON1bits.RD = 1;
        *ret = EEDATA;
    }
    else
    {
        EEDATA = *ret;
        EECON1bits.WREN = 0x1;      // Write enabled
        EECON2 = 0x55;              // Magic sequence
        EECON2 = 0xAA;
        EECON1bits.WR = 0x1;

        // EECON1bits.WREN = 0x0;
        while(EECON1bits.WR);
        Delay10KTCYx(10);     // Safe Delay
    }
    EECON1bits.WREN = 0b0;      // Write enabled
    INTCONbits.GIE = gie;            // Enable Interrupts back
}


//#define _XTAL_FREQ 48000000UL

// To transfer to PIC16 we should wait some time between bytes, sent to PIC16.
// If we don't wait, nACK not sent and we cannot complete transfer

void Timer1ONOFF(unsigned char ON, unsigned char cTMR1H, unsigned char cTMR1L)
{

    if (ON)
    {
//        T0CON = 0;
        T1CONbits.TMR1ON = 0;
        T1CON = 0b01110000;
        TMR1H = cTMR1H;
        TMR1L = cTMR1L;
 //       OutRet(0x34)
        PIR1bits.TMR1IF = 0;
        PIE1bits.TMR1IE = 1;
#if defined (__18F4550)
        IPR1bits.TMR1IP = 1;
#endif
        T1CONbits.TMR1ON = 1;
    }
    else
        T1CONbits.TMR1ON = 0;
}
/*
 * 
 */
void main(void)
{
    char x;
    char onceRC7 = 0, onceRC6 = 0;

    PORTD = 0;
    LATD = 0;
    PORTB = 0;

    TRISA = 0xFF;
    TRISB = 0b11011111;     // RB0, RB1 - inputs for I2C module
                            // RB4, RB5 - input for IOC
    TRISC = 0xFF; // All inputs
    TRISD = 0b11101111;     // RD2 = BEAN, input
                            // RD4 = BEAN output
    TRISE = 0;
    ADCON1 = 0b00001111;

    RCONbits.IPEN = 1;
//    INTCONbits.TMR0IE = 1;
//    INTCON2bits.RBIP = 1;
//    INTCON2bits.T0IP = 1;
//    INTCONbits.RBIE = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;

    //__delay_ms(10000);
//    OutRet(0x55, &once, 0);
    i2c_init(0);
    Timer0ONOFF(0, TMR0_NOTRESET);
    IOCONOFF (0);
    InitBEANVars(0);

    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    __delay_ms(10);
    
    Timer0ONOFF(1, TMR0_NOTRESET);
    IOCONOFF(1); // With IOC

/*    BEANSndSt = BEAN_TR_SOF;
    BEANSndBitPos = 7;
    BEANSndByteCount = 0;
    while (BEANSndSt != BEAN_NO_TR)
    {
        BEANNextBitCount = 0;
        BEANGetNextBitToSend();
        BEANRecBuff1[0]++;
        BEANRecBuff1[BEANRecBuff1[0]] = (BEANNextBit ? 0x80 : 0) | BEANNextBitCount;

        BEAN_OUT = BEANNextBit;
    }
    TXNbytes(0, 0, BEANRecBuff1[0] + 1, BEANRecBuff1);*/
  //  x = BEANStartSend();
//    x = 0;
    //OutRet (0x23, &x, 0);


    while (1)
    {
        __delay_ms(1);

        if (BEANCmdToSend[0] == 0 && BEANNumToSend > 0 && T1CONbits.TMR1ON == 0)    // We've not set it yet
        {
            if (BEANDelayBtwCmd > 3)
                Tmr1Repeat = (BEANDelayBtwCmd / 4);

            switch (BEANDelayBtwCmd - (Tmr1Repeat * 4))
            {
                case 0:
                    Timer1ONOFF (1, 0x15, 0x9F);    // 0xFFFF - 60000
                    break;
                case 1:
                    Timer1ONOFF (1, 0xc5, 0x67);    // 15000 = 10 ms * (48000000 / (4 * 8 * 1000)) - 15000 ticks = 10 ms
                    break;
                case 2:
                    Timer1ONOFF (1, 0x8a, 0xcf);    // 30000
                    break;
                case 3:
                    Timer1ONOFF (1, 0x50, 0x37);    // 45000
                    break;
            }
        }
/*        if (PORTAbits.RA0 && BEANSndSt == BEAN_NO_TR)
        {
            BEANCmdToSend[0] = 0b10110011;
            BEANCmdToSend[1] = 0b11111110;
            BEANCmdToSend[2] = 0b01110001;
            BEANCmdToSend[3] = 0b00100010;
            BEANCmdToSend[4] = 0b01010101;
            __delay_ms(10);

            BEANStartSend();
        }
*/
        if (PORTCbits.RC6)
        {
            if (onceRC6 < 2 && BEANCmdToSend[0] == 0)
            {
                if (onceRC6 == 0)
                {

                    BEANFirstByteInSnd = BEANCmdToSend[0];
                    BEANNumToSend = 2 - 1;
                    BEANDelayBtwCmd = 2;   // Delay in 10th milliseconds
                    Tmr1Repeat = 0;

                    BEANCmdToSend[0] = 0xb4;
                    BEANCmdToSend[1] = 0x18;
                    BEANCmdToSend[2] = 0x36;
                    BEANCmdToSend[3] = 0x00;
                    BEANCmdToSend[4] = 0x1e;
                    Timer0ONOFF(1, TMR0_NOTRESET);

                }
    /*            if (onceRC6 == 1)
                {
                    BEANFirstByteInSnd = BEANCmdToSend[0];
                    BEANNumToSend = 2 - 1;
                    BEANDelayBtwCmd = 2;   // Delay in 10th milliseconds
                    Tmr1Repeat = 0;

                    BEANCmdToSend[0] = 0xb4;
                    BEANCmdToSend[1] = 0x18;
                    BEANCmdToSend[2] = 0x36;
                    BEANCmdToSend[3] = 0x07;
                    BEANCmdToSend[4] = 0x1e;
                }*/
                onceRC6=2;
            }
        }
        else
        {
            onceRC6 = 0;
        }

        if (PORTCbits.RC7)
        {
            if (!onceRC7)
            {
  //              Timer0ONOFF(1, TMR0_1TICK * TRM0_NOTR_CONDITION);
                onceRC7 = 1;
    //            InitBEANVars(0);
 //               IOCONOFF(1); // With IOC
                if (BEANRecBuff1[(BEANRecBuff1[1] & 0x0F) + 2] != Crc8(&BEANRecBuff1[1], (BEANRecBuff1[1] & 0x0F) + 1))
                    BEANRecBuff1[0] |= 0x80;    // CRC incorrect
                TXNbytes(0, 0, (BEANRecBuff1[0] & 0x7F) + 1, BEANRecBuff1);
                __delay_ms(1);

                if (BEANRecBuff2[(BEANRecBuff2[1] & 0x0F) + 2] != Crc8(&BEANRecBuff2[1], (BEANRecBuff2[1] & 0x0F) + 1))
                    BEANRecBuff2[0] |= 0x80;    // CRC incorrect
                TXNbytes(1, 0, (BEANRecBuff2[0] & 0x7F) + 1, BEANRecBuff2);
                __delay_ms(1);

                if (BEANRecBuff3[(BEANRecBuff3[1] & 0x0F) + 2] != Crc8(&BEANRecBuff3[1], (BEANRecBuff3[1] & 0x0F) + 1))
                    BEANRecBuff3[0] |= 0x80;    // CRC incorrect
                TXNbytes(2, 0, (BEANRecBuff3[0] & 0x7F) + 1, BEANRecBuff3);
                __delay_ms(1);

                if (BEANRecBuff4[(BEANRecBuff4[1] & 0x0F) + 2] != Crc8(&BEANRecBuff4[1], (BEANRecBuff4[1] & 0x0F) + 1))
                    BEANRecBuff4[0] |= 0x80;    // CRC incorrect
                TXNbytes(3, 0, (BEANRecBuff4[0] & 0x7F) + 1, BEANRecBuff4);
            }
        }
        else
        {
            if (onceRC7)
            {
        //        Timer0ONOFF(0, 0);
          //      IOCBEANIN (0);
                //InitBEANVars(0);
                onceRC7 = 0;
                //TXNbytes(0, 0, 5, BEANCmdToSend);
                //OutRet (0x33, &BEANRecBuff1[0], 0);
                //OutRet (0x34, &BEANRecBuff1[1], 0);
                ///OutRet (0x35, &BEANRecBuff1[2], 0);
                //OutRet (0x36, &BEANRecBuff1[3], 0);

            }
        }
    }
}

void Tmr1Int()
{
//        OutRet (Tmr1Repeat + 0x10, &Tmr1Repeat, 0);
    if (Tmr1Repeat == 0)    // Send
    {
//        char ret = 0xAA;
        if (BEANNumToSend != 0)
        {
            Timer1ONOFF(0, 0, 0);
            BEANCmdToSend[0] = BEANFirstByteInSnd;
            if (BEANNumToSend != 0xFF)  // FF means send indefinitely
                BEANNumToSend--;

            if (BEANTrSt == BEAN_NO_TR)
            {
                IOCONOFF(1); // With IOC
                Timer0ONOFF(1, TMR0_NOTRESET);
            }
        }
    }
    else
    {
        Timer1ONOFF (1, 0x15, 0x9F);    // 0xFFFF - 60000
        Tmr1Repeat--;
    }
//    char ret = Tmr1Repeat;

}

void interrupt SYS_InterruptHigh(void)
{
    if (INTCONbits.TMR0IF && INTCONbits.TMR0IE)
    {
        char bean;
        bean = BEAN_IN_DIG;
        INTCONbits.TMR0IF = 0;
        TmrInt(bean);
        return;
    }
    if (PIR1bits.TMR1IF && PIE1bits.TMR1IE)
    {
        PIR1bits.TMR1IF = 0;
        Tmr1Int();
        return;
    }
    if (PIR1bits.TMR2IF && PIE1bits.TMR2IE)
    {
        PIR1bits.TMR2IF = 0;
        Tmr2Int();
        //Tmr2Sync = 1;
        return;
    }
    if (INTCONbits.RBIE && INTCONbits.RBIF)
    {
        char bean = PORTBbits.RB4;
        bean = BEAN_IN_DIG;
        asm ("NOP");
        asm ("NOP");
        INTCONbits.RBIF = 0;

        IOCInt (bean);
        return;
    }
//    #if defined(USB_INTERRUPT)
//        USBDeviceTasks();
//    #endif
}

