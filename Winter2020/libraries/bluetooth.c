#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "bluetooth.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"


/*void UART1IntHandler(void) {

  uint32_t ui32Status;
  uint32_t  intChar;
  int8_t cChar;
  ui32Status = UARTIntStatus(UART1_BASE, true); //get interrupt status
  UARTIntClear(UART1_BASE, ui32Status); //clear the asserted interrupts
  while(UARTCharsAvail(UART1_BASE))
  {
    //intChar = UARTCharGetNonBlocking(UART1_BASE);
    //cChar = (unsigned char)(intChar & 0xFF);
    //UARTCharPutNonBlocking(UART1_BASE, intChar); //echo character

    //if(cChar == 'pose'){

    //  UARTCharPutNonBlocking(UART1_BASE, pos);
    //}
    //else if(cChar == '0'){

    //  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    //}

  }
} */

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

/*void
ConfigureUART(void) //Using UART1 without interrupts
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

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(1, 115200, 16000000);
} */

void printString(char *string, int len){

  //char string[]="This is a test string.";
  //int len = sizeof(string);

  int buffer[len];

  int i=0,j;
  while(string[i]!='\0') {
    buffer[i]=string[i];
    i++;
  }

  for(i=0;i<len-1;i++){
    UARTCharPut(UART1_BASE,buffer[i]);
  }

  //Add newline (CR and LF)
  UARTCharPut(UART1_BASE,10); //LF
  UARTCharPut(UART1_BASE,13); //CR

}

void printInt(int n){

  int* buffer[10];
  int len = intToASCII(n,buffer);

  int i;
  //Print each digit
  for(i = len-1; -1<i;i--){

    UARTCharPut(UART1_BASE,buffer[i]);
  }

  //Add newline (CR and LF)
  UARTCharPut(UART1_BASE,10); //LF
  UARTCharPut(UART1_BASE,13); //CR



}

int intToASCII(int n, int * arr){

      // Iterates from the least significant digit to the most significant
      int len = 0;
      while (n != 0) {
       arr[len] = (n % 10) + 48;
       n /= 10;
       len++;
     }

     return len;
}
