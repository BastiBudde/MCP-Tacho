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

// Defines
#define X_MAIN_BUTTON 650
#define Y_MAIN_BUTTON 400
#define X_BUTTON_LENGTH 100
#define Y_BUTTON_LENGTH 60
#define X_SIDE_BUTTON 50
#define Y_SIDE_BUTTON 400
#define X_COLOR_BUTTON 650
#define Y_COLOR_BUTTON 300

#define X_RIGHT 0
#define X_LEFT 3000
#define Y_RIGHT 3000
#define Y_LEFT 3000

// Global Variable
extern uint8_t button_changed;
extern uint8_t color_changed;
extern int xpos;
extern int ypos;

// Function Prototypes
void touch_write(unsigned char value);

unsigned int touch_read();

void GPIOPortDIntHandler();

#endif // TOUCH_H
