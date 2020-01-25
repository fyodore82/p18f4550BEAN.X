#include "i2c_p18f4550.h"

void __delay_us (unsigned char us)
{

    while(us--)
    {
        Delay1TCY();    // 11 Delay1TCY function calls as while itself has 1 instruction cycle
        Delay1TCY();
        Delay1TCY();
        Delay1TCY();
        Delay1TCY();
        Delay1TCY();
        Delay1TCY();
        Delay1TCY();
        Delay1TCY();
        Delay1TCY();
        Delay1TCY();
    }
}