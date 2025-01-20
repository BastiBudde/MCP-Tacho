#ifndef TOUCH_H
#define TOUCH_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "font.h"

#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"

#define TOUCH_X_MAX 4094
#define TOUCH_Y_MAX 4094


// Function Prototypes
void touch_write(unsigned char value);

unsigned int touch_read();

#endif // TOUCH_H
