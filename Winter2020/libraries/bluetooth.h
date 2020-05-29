#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include "driverlib/uart.h"
#include "utils/uartstdio.h"

void UART1IntHandler(void);
void ConfigureUART(void);
void printString(char *string, int len);
void printInt(int n);
int intToASCII(int n, int * arr);

#endif // ____BLUETOOTH_H____
