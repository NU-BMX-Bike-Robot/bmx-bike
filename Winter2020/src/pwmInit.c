//Configures a PWM signal
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"

void ConfigurePWM(void)
{
    // Divide the system frequency by 8.
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);

    // Tiva has 2 PWM modules
    // Each module has 4 PWM generators
    // Each PWM generator has two outputs that shares the same timer and frequency.
    // I think you can set individual duty cycle to these two outputs

    // This enables PWM module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
    {
    }

    // Use PWM2 and PWM3 of module 0, which would be the first output of generator 1 of module 0
    // See data sheet for PWM pin-out
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);

    // Configure the PWM generator for count down mode with immediate updates to the parameters.
    // SYNC parameter has something to do with the change of the period parameter N
    // NO_SYNC means the parameter can be changed without a synchronize event trigger
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    // N = (1 / f) * SysClk.  Where N is the PWMGenPeriodSet
    // function parameter, f is the desired frequency, and SysClk is the
    // PWM clock frequency after this function.
    // DIV = 8, N = 125000
    // DIV = 4, N = 250000 and so on
    // 5% duty cycle of 125000 is 6250 (FULL REVERSE)
    // 7.5% duty cycle of 125000 is 9375 (NEUTRAL)
    // 10% duty cycle  of 125000 is 12500 (FULL FORWARD)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 62500);

    // Set the pulse width of PWM2 and PWM3 for a 7.5% duty cycle.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 4688);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 4688);

    // Start the timers in generator 1.
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    // Enable the outputs.
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT|PWM_OUT_3_BIT, true);



}
