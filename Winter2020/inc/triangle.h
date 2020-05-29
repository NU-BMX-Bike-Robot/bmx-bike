#ifndef TRIANGLE_H
#define TRIANGLE_H


//Configures a triangle wave
extern volatile uint32_t pwmOut;

extern int upFlag;

void triangle(void);

#endif /* TRIANGLE_H */
