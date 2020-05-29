#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

volatile uint32_t pwmOut = 4766;

// 0.025ms dead-band = 156 counts out of 125000
// Forward actual starting point = 9375 + 156 = 9531
// Reverse actual starting point = 9375 - 156 = 9219
//4688+78=4766
//4688-78=4610
// 5%=3125
// 7.5% = 4688
// 6250-(62500*0.025*0.85) =4922
// 3125+(62500*0.025*0.85) =4453

int upFlag = 1;
volatile int flag = 0;

void triangle(void)
{
    uint32_t status=0;
    status = TimerIntStatus(TIMER0_BASE,true);
    TimerIntClear(TIMER0_BASE,status);
    if (flag==1)
    {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwmOut-60);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, pwmOut-60);
    }
    UARTprintf("pwm: %d     ",pwmOut);
    if (pwmOut == 4922)
    {
        upFlag = 0;
    }
    if (pwmOut == 4453)
    {
        upFlag = 1;
    }

    if (pwmOut > 4766 && pwmOut <= 4922 || pwmOut == 4766)
    {
        if (upFlag == 1)
        {
            pwmOut++;
        }
        else if (pwmOut == 4766 && upFlag == 0)
        {
            pwmOut = 4610;
        }
        else
        {
            pwmOut = pwmOut - 1;
        }

    }
    else if (pwmOut >= 4453 && pwmOut < 4610 || pwmOut == 4610)
    {
        if (upFlag == 0)
        {
            pwmOut = pwmOut - 1;
        }
        else if (pwmOut == 4610 && upFlag == 1)
        {
            pwmOut = 4766;
        }
        else
        {
            pwmOut++;
        }
    }

}
