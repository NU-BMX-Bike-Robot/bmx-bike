#include "bmx_encoder.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "driverlib/qei.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"


int pulsesToDegrees(float pulses, float ppr){

    float deg = pulses/ppr;

    return (int) deg;
}
