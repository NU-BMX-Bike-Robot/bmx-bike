#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/qei.h"

#include "utils/uartstdio.h"

#include "bmx_init.h"

void InitializeTiva()
{
  // Enable lazy stacking for interrupt handlers.  This allows floating-point
  // instructions to be used within interrupt handlers, but at the expense of
  // extra stack usage.

  // Set the clocking to run directly from the crystal.
  SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);


  // Enable the GPIO port that is used for the on-board LED.
   //
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  // Enable the GPIO pins for the LED (PF2 & PF3).
   //
   GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
   GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

  ConfigureUART();
  ConfigureI2C();
  ConfigureBluetoothUART();
  ConfigureQEI();
  ConfigureQEIVel();
}

void ConfigureUART()
{

    // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
}


void ConfigureI2C()
{
    /*
      See section 15 and 17 of the TivaWare™ Peripheral Driver Library for more usage info.
    */
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {}

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

void ConfigureQEI(void){

  // Enable peripherals.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

  //
  // Wait for the QEI0 module to be ready.
  //
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0)) {}

  //
  // Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work.
  //
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; // In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
  HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

  //
  // Set what pins are PhA0, PhB0, and IDX0.
  //
  GPIOPinConfigure(GPIO_PD6_PHA0);
  GPIOPinConfigure(GPIO_PD7_PHB0);
  GPIOPinConfigure(GPIO_PD3_IDX0);

  //
  // Set GPIO pins for QEI. This sets them pull up and makes them inputs.
  //
  GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7 | GPIO_PIN_3);

  //
  // Disable peripheral and int before configuration.
  //
  QEIDisable(QEI0_BASE);
  QEIIntDisable(QEI0_BASE, QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

  //
  // Configure quadrature encoder.
  //
  QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_RESET_IDX | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 8192);

  //
  // Enable quadrature encoder.
  //
  QEIEnable(QEI0_BASE);

  //
  // Set position to a middle value so we can see if things are working.
  //
  QEIPositionSet(QEI0_BASE, 512);

}

void ConfigureQEIVel(void){

  // Configure quadrature encoder.
  //
  QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet() / 3); //Accumulates encoder value every sec

  //
  // Enable quadrature velocity module.
  //
  QEIVelocityEnable(QEI0_BASE);


}

void
//Using uart1 with interrupt enabled
ConfigureBluetoothUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Enable UART1
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);


    //configure divisor and format
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    // UARTStdioConfig(1, 9600, 16000000);

    // UARTEchoSet(true);

    /*IntMasterEnable(); //enable processor interrupts
    IntEnable(INT_UART1); //enable the UART interrupt
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT); *///only enable RX and TX interrupt

}
