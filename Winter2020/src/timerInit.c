//Configures a timer
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
//#include <boost/preprocessor.hpp>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"


//void ConfigureTimerAndInterrupt(void (*f)(void),void (*ff)(void))

void ConfigureTimerAndInterrupt(void (*f)(void))
{
    uint32_t period;
    uint32_t period2;
    period = 500000; //1s
    period2 = 50000;

    // 5 regular 16/32 bit timer block, 5 wide 32/64 bit timer block
    // Each timer block has timer output A and B
    // See Section 11 of the data sheet for more Information

    // Enable 16/32 bit Timer 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    }

    // TIMER_CFG_PERIODIC means full-width periodic
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Timer load set
    // !! WARNGING !!
    // DO NOT USE SysCtlClockGet() TO SET THE PERIOD
    // IT WILL NOT WORK AS INTENDED, USE THE period VARIABLE INSTEAD

    TimerLoadSet(TIMER0_BASE, TIMER_A, period-1);

    //Enable timer 0A
    TimerEnable(TIMER0_BASE, TIMER_A);

    //Configuration for timer based interrupt if needed

    //Link the timer0,A and the ISR together
    // ISR is the your routine function
    TimerIntRegister(TIMER0_BASE, TIMER_A, *f);

    //Enable interrupt on timer 0A
    IntEnable(INT_TIMER0A);

    //Enable timer interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


//    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER5))
//    {
//    }
//    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
//    TimerLoadSet(TIMER5_BASE, TIMER_A, period2 -1);
//
//    TimerIntRegister(TIMER5_BASE, TIMER_A, *ff);
//
//    TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
//
//    TimerEnable(TIMER5_BASE, TIMER_A);

}
