/* 
 * File:   i2c_p18f4550.h
 * Author: fedor
 *
 * Created on 12 ?????? 2014 ?., 20:42
 */

#ifndef I2C_P18F4550_H
#define	I2C_P18F4550_H

#include <p18cxxx.h>
#include <delays.h>

#ifdef	__cplusplus
extern "C" {
#endif


#define SCL TRISBbits.TRISB1
#define SDA TRISBbits.TRISB0
#define SCL_IN PORTBbits.RB1
#define SDA_IN PORTBbits.RB0
//void __delay_us (unsigned char us);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_P18F4550_H */

