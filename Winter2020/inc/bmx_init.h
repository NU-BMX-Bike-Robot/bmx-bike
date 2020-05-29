/// \file bmx_init.h
/// \brief Initialization file. These functions are used to initialize the various
/// features of the tiva.

#ifndef BMX_INIT_HG
#define BMX_INIT_HG

/// \brief main initialization routine. Call this first in main function.
///
void InitializeTiva();

/// \brief Configure UART0. Sets the proper pins and other tiva settings to
/// initialize serial communication.
void ConfigureUART();

/// \brief Configure I2C0. Sets the proper pins and other tiva settings to
/// initialize i2c communication.
void ConfigureI2C();

/// \brief Configure UART1 to communicate with HC-05 (Bluetooth module)
// using PB0 (RX) and PB1 (TX)
void ConfigureBluetoothUART();

/// \brief Configure QEI0 to interpret quadruture signals from the encoder and get encoder counts
// using PD6 (A+) and PD7 (B+) and PD3 (inddex)
void ConfigureQEI();

/// \brief Configure QEI to get encoder count change every second 
void ConfigureQEIVel();



#endif
