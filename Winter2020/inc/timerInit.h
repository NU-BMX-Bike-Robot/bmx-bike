#ifndef TIMERINIT_H
#define TIMERINIT_H


//Configures a timer
//void ConfigureTimerAndInterrupt(void (*f)(void),void (*ff)(void));
extern volatile int flag;
void ConfigureTimerAndInterrupt(void (*f)(void));

#endif /* TIMERINIT_H */
