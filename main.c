/*
 * Name: MCP-Tacho
 * Author: Kim Luu, Johanna, Sebastian Budde
 * Version: 1.0
 *
 * */
#include "font.h"
#include "LCDDisplay.h"
#include "Touch.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

//#define FUN

#define TIMER0_FREQ 10
#define SPEED_EVAL_FREQ 2


#define TIRE_CIRCUM_CM 171.0f // cm
#define CM2M_f(cm)  (((float)cm) /   100.0f)
#define M2KM_f(m)   (((float)m)  /   100.0f)
#define CM2KM_f(cm) (((float)cm) / 10000.0f)

#define PI 3.14159265358979323846

#define CX 400
#define CY 250
#define RAD 200

#define VEL_XPOS 150
#define VEL_YPOS 300
#define VEL_FONT_COLOR WHITE

#define KM_XPOS 400
#define KM_YPOS 300
#define KM_FONT_COLOR WHITE

#define DIR_XPOS 650
#define DIR_YPOS 200
#define DIR_FONT_COLOR(dir) ((dir)==FORWARD ? GREEN : RED)

// BUTTONS
#define BUTTON_COLOR GREY
#define BUTTON_BORDER_COLOR WHITE
#define BUTTON_TEXT_COLOR WHITE

#define BUTTON_WIDTH (DISPLAY_X_MAX/3)
#define BUTTON_BORDER_WIDTH 2

#define BUTTON_PADDING 10
#define BUTTON_HEIGHT (FONT_SIZE_Y + 2*BUTTON_PADDING)


#define X0_BUTTON_BL 0
#define Y0_BUTTON_BL (Y1_BUTTON_BL - BUTTON_HEIGHT)
#define X1_BUTTON_BL (X0_BUTTON_BL + BUTTON_WIDTH)
#define Y1_BUTTON_BL (DISPLAY_Y_MAX-1)

#define X0_BUTTON_BC (X1_BUTTON_BL + 1)
#define Y0_BUTTON_BC Y0_BUTTON_BL
#define X1_BUTTON_BC (X0_BUTTON_BC + BUTTON_WIDTH)
#define Y1_BUTTON_BC Y1_BUTTON_BL

#define X0_BUTTON_BR (X1_BUTTON_BC + 1)
#define Y0_BUTTON_BR Y0_BUTTON_BL
#define X1_BUTTON_BR (X0_BUTTON_BR + BUTTON_WIDTH)
#define Y1_BUTTON_BR Y1_BUTTON_BL

#define DPX_2_TPX(X) (((X) * DISPLAY_X_MAX) / TOUCH_X_MAX)
#define DPY_2_TPY(Y) (((Y) * DISPLAY_Y_MAX) / TOUCH_Y_MAX)

#define CHECK_BUTTON_BL(x, y) ( DPX_2_TPX((x)) > X0_BUTTON_BL && DPX_2_TPX((x)) < X1_BUTTON_BL && DPY_2_TPY((y)) > Y0_BUTTON_BL && DPY_2_TPY((y)) < Y1_BUTTON_BL )
#define CHECK_BUTTON_BC(x, y) ( DPX_2_TPX((x)) > X0_BUTTON_BC && DPX_2_TPX((x)) < X1_BUTTON_BC && DPY_2_TPY((y)) > Y0_BUTTON_BC && DPY_2_TPY((y)) < Y1_BUTTON_BC )
#define CHECK_BUTTON_BR(x, y) ( DPX_2_TPX((x)) > X0_BUTTON_BR && DPX_2_TPX((x)) < X1_BUTTON_BR && DPY_2_TPY((y)) > Y0_BUTTON_BR && DPY_2_TPY((y)) < Y1_BUTTON_BR )



enum Direction {
    FORWARD  = 'F',
    BACKWARD = 'B'
};

enum Pages {
    PAGE1,
    PAGE2
};


enum Direction    direction;                 // for remembering direction of rotation
enum Pages        page = PAGE1;              // for remembering page to be displayed
volatile uint32_t g_ui32SysClock;            // System Clk Frequency
volatile uint32_t g_ui32EdgeCntS1S2  = 0;    // Edge Counter for S1 and S2
volatile uint32_t g_ui32DailyCM      = 0;    // Daily CM counter
volatile float    g_fSpeedKMH        = 0.0f; // Speed calculated from edge count
volatile float    g_fSpeedKMHPrev    = 0.0f; // Speed calculated from edge count
volatile float    g_fSpeedKMHSmooth  = 0.0f; // Speed calculated from edge count
volatile float    g_fRevsPerSec      = 0.0f; // RPS calculated from edge count
uint16_t          g_ui16NeedelTipX   = 0;    // Last x position of needle tip
uint16_t          g_ui16NeedelTipY   = 0;    // Last y position of needle tip
volatile uint16_t g_ui16SpeedCounter = 0;

// Global Variable
colors backroundColor = BLACK;
colors fontColor= WHITE;
colors carColor= RED;

volatile bool buttonBL = false;
volatile bool buttonBC = false;
volatile bool buttonBR = false;
uint32_t xpos;
uint32_t ypos;

#ifdef FUN
// fun
uint8_t colidx = 1;
#endif
/********************************************************************************
     Page Functions
*********************************************************************************/

void initFirstPage(){
    draw_filled_rectangle(0, 0, DISPLAY_X_MAX-1, DISPLAY_Y_MAX-1, backroundColor);
    drawString(VEL_XPOS+3*FONT_SPACING, VEL_YPOS, "km/h", font, fontColor, backroundColor);
    drawString(KM_XPOS+6*FONT_SPACING, KM_YPOS, "km", font, fontColor, backroundColor);
    draw_Button(X0_BUTTON_BL, Y0_BUTTON_BL, X1_BUTTON_BL, Y1_BUTTON_BL, "Theme", BUTTON_COLOR, BUTTON_BORDER_COLOR, BUTTON_BORDER_WIDTH, BUTTON_TEXT_COLOR);
    draw_Button(X0_BUTTON_BC, Y0_BUTTON_BC, X1_BUTTON_BC, Y1_BUTTON_BC, "Reset", BUTTON_COLOR, BUTTON_BORDER_COLOR, BUTTON_BORDER_WIDTH, BUTTON_TEXT_COLOR);
    draw_Button(X0_BUTTON_BR, Y0_BUTTON_BR, X1_BUTTON_BR, Y1_BUTTON_BR, "Next", BUTTON_COLOR, BUTTON_BORDER_COLOR, BUTTON_BORDER_WIDTH, BUTTON_TEXT_COLOR);
}

void initSecondPage(){
    draw_filled_rectangle(0, 0, DISPLAY_X_MAX-1, DISPLAY_Y_MAX-BUTTON_HEIGHT-1, backroundColor);
    draw_Button(X0_BUTTON_BL, Y0_BUTTON_BL, X1_BUTTON_BL, Y1_BUTTON_BL, "Back", BUTTON_COLOR, BUTTON_BORDER_COLOR, BUTTON_BORDER_WIDTH, BUTTON_TEXT_COLOR);
    draw_Button(X0_BUTTON_BC, Y0_BUTTON_BC, X1_BUTTON_BC, Y1_BUTTON_BC, "Color", BUTTON_COLOR, BUTTON_BORDER_COLOR, BUTTON_BORDER_WIDTH, BUTTON_TEXT_COLOR);
    draw_Button(X0_BUTTON_BR, Y0_BUTTON_BR, X1_BUTTON_BR, Y1_BUTTON_BR, "IDK", BUTTON_COLOR, BUTTON_BORDER_COLOR, BUTTON_BORDER_WIDTH, BUTTON_TEXT_COLOR);
}

void drawFirstPage(){
    // delete old needle
    draw_line_bresenham(CX, CY, g_ui16NeedelTipX, g_ui16NeedelTipY, backroundColor);
    rasterHalfCircle(CX, CY, RAD, false, fontColor); // Oberer Halbkreis
    rasterHalfCircle(CX, CY, (RAD-30), false, GREY);
    rasterHalfCircle(CX, CY, (50), false, ORANGE);


    // Winkel in Bogenma�
    float theta = PI - ((g_fSpeedKMHSmooth / 400.0f) * PI);
    if (theta > PI){
        theta = PI;
    }else if (theta < 0){
        theta = 0.0f;
    }

    // Koordinaten berechnen
    g_ui16NeedelTipX = (int)(CX + RAD * cos(theta));
    g_ui16NeedelTipY = (int)(CY - RAD * sin(theta)); // Minus f�r "Pixelkoordinaten"

    draw_line_bresenham(CX, CY, g_ui16NeedelTipX, g_ui16NeedelTipY, ROT);

    drawInteger(VEL_XPOS, VEL_YPOS, font, (uint16_t)g_fSpeedKMH, fontColor, backroundColor);
    drawKM(KM_XPOS, KM_YPOS, font, g_ui32DailyCM, fontColor, backroundColor);
    drawSymbol(DIR_XPOS, DIR_YPOS, font[direction], DIR_FONT_COLOR(direction), backroundColor);
    return;
}

void drawSecondPage(){

}

/********************************************************************************
     ISRs
*********************************************************************************/
// Interrupt for incrementing edge counter when edge in Signal S1 (connected to Port P0)
void P0edgeISR(void){
    //GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);  // to see interrupt duration for debug

    GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_0);              // clear interrupt flag
    g_ui32EdgeCntS1S2++;                                    // increment edge counter

    //GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0);           // to see interrupt duration for debug
}

/********************************************************************************/
// Interrupt for incrementing edge counter when edge in Signal S1 (connected to Port P0)
// and determining direction of rotation
void P1edgeISR(void){
    //GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);          // to see interrupt duration for debug

    GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_1);                      // clear interrupt flag
    g_ui32EdgeCntS1S2++;                                            // increment edge counter

    // determine direction of rotation
    if(GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_1) & GPIO_PIN_1)       // if P1 (S2) high
    {
        if( GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_0) & GPIO_PIN_0){ // if P0 (S1) already high
            direction = FORWARD;
        }else{
            direction = BACKWARD;
        }
    }

    //GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0);                   // to see interrupt duration for debug
}

/********************************************************************************/
// Interrupt for evaluating and resetting edge count, updating display and evaluating touch inputs
// Interrupt will be triggered with frequency set in define TIMER0_FREQ
void timer0AISR(void){
    //TimerDisable(TIMER0_BASE, TIMER_A); // for debug

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // reset interrupt flag
    g_ui16SpeedCounter++;

    if(g_ui16SpeedCounter == (TIMER0_FREQ/SPEED_EVAL_FREQ)){
        g_ui16SpeedCounter = 0;

        g_fSpeedKMHSmooth = g_fSpeedKMH;
        g_fSpeedKMHPrev = g_fSpeedKMH;
        // evaluate counted edges in S1 and S2 signals to calculate Speed and increment Daily CM counter
        g_fRevsPerSec  = ( ((float)g_ui32EdgeCntS1S2) * ((float)SPEED_EVAL_FREQ) ) / 16.0f;
        g_ui32DailyCM += (uint32_t)( ( ((float)g_ui32EdgeCntS1S2)/16.0f ) * TIRE_CIRCUM_CM ); // calculate in floating point precision, then cast to uint
        g_fSpeedKMH    = ( g_fRevsPerSec * TIRE_CIRCUM_CM * 36.0f )/1000.0f;
        g_ui32EdgeCntS1S2 = 0;
    }else{
        g_fSpeedKMHSmooth += ((g_fSpeedKMH - g_fSpeedKMHPrev)*g_ui16SpeedCounter)/(TIMER0_FREQ);
    }

    switch(page){ // Check which page is displayed (buttons have different functionality on each page)
        case PAGE1: // Tacho
            if( buttonBL ){
               if (backroundColor == BLACK){
                   backroundColor = WHITE;
                   fontColor = BLACK;
               }else{
                   backroundColor = BLACK;
                   fontColor = WHITE;
               }
               initFirstPage();

               buttonBL = false;
            }
            else if( buttonBC ){
                g_ui32DailyCM = 0.0f;
                buttonBC = false;
            }
            else if( buttonBR ){
                page = PAGE2;
                initSecondPage();
                buttonBR = false;
            }

            break;

        case PAGE2: //Renstrecke
            if( buttonBL ){
                page = PAGE1;
                initFirstPage();
                buttonBL = false;
            }
            else if( buttonBC ){
                buttonBC = false;
            }
            else if( buttonBR ){
                buttonBR = false;
            }


            break;

        default: break;
    }

    switch(page){
        case PAGE1:
            drawFirstPage();
            break;

        case PAGE2:
            drawSecondPage();
            break;

        default: break;
    }


    // fun
#ifdef FUN
    colidx = (colidx+1) % 10;
    drawString(0, 0, "69", font, colorarray[colidx], backroundColor);
    drawString(DISPLAY_X_MAX-3*FONT_SPACING, 0, "420", font, colorarray[colidx], backroundColor);
#endif

    //TimerEnable(TIMER0_BASE, TIMER_A);  // for debug
}


void sysTickISR(){
    IntDisable(INT_GPIOP0); // Disable interrupts for timing critical section
    IntDisable(INT_GPIOP1);

    int x;
    touch_write(0xD0);                  //Touch Command XPos read
    for (x = 0; x < 10; x++);           //Busy wait
    xpos = touch_read();                //xpos value read ( 0......4095 )
    touch_write(0x90);                  //Touch Command YPos read
    for (x = 0; x < 10; x++);           //Busy wait
    ypos = touch_read();                //ypos value read ( 0.....4095 )

    xpos = TOUCH_X_MAX - xpos;

    IntEnable(INT_GPIOP0); // Re-enable interrupts after timing critical section
    IntEnable(INT_GPIOP1);

    // Check if any button is pressed
    if( CHECK_BUTTON_BL(xpos, ypos) ){
        buttonBL = true;
    }
    else if( CHECK_BUTTON_BC(xpos, ypos) ){
        buttonBC = true;
    }
    else if( CHECK_BUTTON_BR(xpos, ypos) ){
        buttonBR = true;
    }
    else{

    }



    //Site 1
    //Reset, Light/Dark-Mode,Next Site

    //Site 2
    //Back Site, Carosserie Farbe, Reifen Farbe
    //Karosserie*

}



/********************************************************************************
     Hardware init
*********************************************************************************/
void init(void){

    IntMasterDisable(); // Disable all interrupts for duration of initialization

    /////////////////////////
    // System Clock Config //
    /////////////////////////
        // Set the clocking to run from the crystal at 120MHz using PLL.
        g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_MAIN  |
                                             SYSCTL_USE_PLL   |
                                             SYSCTL_XTAL_25MHZ|
                                             SYSCTL_CFG_VCO_480),
                                             120000000);
        // check actual clock speed and stop if config failed
        if(g_ui32SysClock == 0){
            printf("Clock configuration failed. Halting...\n");
            while(1) {};
        }else {
            printf(" Clock Frequency in Hz %d\n", g_ui32SysClock);
        }


    //////////////////////
    // Configure Port M //
    //////////////////////
        // Set Port M Pins 0-7: used as Output of LCD Data

        // enable clock-gate Port M
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM));     // wait until clock ready
        GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, 0xFF); // Ports P0-P7 Outputs


    //////////////////////
    // Configure Port L //
    //////////////////////
        // Set Port L Pins 0-4: used as Output of LCD Control signals:

        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);  // Clock Port L
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL));
        GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3| GPIO_PIN_4);


    //////////////////////
    // Configure Port P //
    //////////////////////
        // Ports P0 and P1 will be used to count all edges in S1 and S2 signals from angle sensor
        // RPS and thereby speed of vehicle can be calculated from edge count and given time interval

        // CLK and power for Port P
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
        while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP)) {}

        // Port P0, P1 Input
        GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

        // Pins P0 and P1 are push-pull, 4mA strength
        GPIOPadConfigSet(GPIO_PORTP_BASE,   GPIO_PIN_0 | GPIO_PIN_1,
                         GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD       );

        // both edges on P0 and P1 generate interrupts
        GPIOIntTypeSet(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_BOTH_EDGES);

        // registering functions as ISRs
        IntRegister(INT_GPIOP1, P1edgeISR);
        IntRegister(INT_GPIOP0, P0edgeISR);

        // Clear possible interrupt flags
        GPIOIntClear(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

        // Set high interrupt priority
        IntPrioritySet(INT_GPIOP1,0x20);
        IntPrioritySet(INT_GPIOP0,0x20);

        // Allow request output from Port unit
        GPIOIntEnable(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

        // Allow request input to NVIC
        IntEnable(INT_GPIOP1);
        IntEnable(INT_GPIOP0);


    //////////////////////
    // Configure Port D //
    //////////////////////
        // Port D will be used for the touch-function of the display
        // Port D Pin 4 gives the interrupt signal
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //Enable clock Port D
        while ((SYSCTL_PRGPIO_R & 0x08) == 0);  //GPIO Clock ready?

        GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_1);
        GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0); //Clk=0


    ///////////////////////
    // Configure Timer0A //
    ///////////////////////
        // activate Timer0
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {}

        // configure Timer0 as 32-Bit-Periodic-Timer
        TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

        // activate Interrupt for Timer0A
        TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock/TIMER0_FREQ) - 1);

        // register ISR for Timer0A
        TimerIntRegister(TIMER0_BASE, TIMER_A, timer0AISR);

        // lower interrupt prio than P1
        IntPrioritySet(INT_TIMER0A,0x40);

        // Switch interrupt source on
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    /////////////////////////////
    // Configure SysTick Timer //
    /////////////////////////////

        // SysTick deaktivieren
        SysTickDisable();

        // SysTick Interrupt aktivieren
        SysTickIntEnable();

        // Lade die berechnete Zeit
        SysTickPeriodSet( (g_ui32SysClock/10) - 1);

        // SysTick-Handler registrieren
        SysTickIntRegister(sysTickISR);

        // set second highest prio
        IntPrioritySet(FAULT_SYSTICK, 0x30);

        // SysTick starten
        SysTickEnable();

    //////////////////////////////
    // Configure & init Display //
    //////////////////////////////
        configure_display_controller_large();
        window_set(0, 0, DISPLAY_X_MAX - 1, DISPLAY_Y_MAX - 1);
        write_command(0x2C);
        int x = 0; int y = 0;
        for (x = 0; x < DISPLAY_X_MAX; x++) {
            for (y = 0; y < DISPLAY_Y_MAX - BUTTON_HEIGHT; y++) {
               write_data((backroundColor>>16)&0xff); // Schwarz
               write_data((backroundColor>>8)&0xff);
               write_data((backroundColor)&0xff);
            }
        }


        rasterHalfCircle(CX, CY, RAD, false, fontColor); // Oberer Halbkreis
        rasterHalfCircle(CX, CY, (RAD-30), false, GREY);
        rasterHalfCircle(CX, CY, (50), false, ORANGE);


        // draw units for speed and daily km counter
        drawString(VEL_XPOS+3*FONT_SPACING, VEL_YPOS, "km/h", font, fontColor, backroundColor);

        drawString(KM_XPOS+6*FONT_SPACING, KM_YPOS, "km", font, fontColor, backroundColor);

        // draw first direction letter
        drawSymbol(DIR_XPOS, DIR_YPOS, font[FORWARD-'!'], DIR_FONT_COLOR(FORWARD), backroundColor);

        // draw buttons
        draw_Button(X0_BUTTON_BL, Y0_BUTTON_BL, X1_BUTTON_BL, Y1_BUTTON_BL, "Theme", BUTTON_COLOR, BUTTON_BORDER_COLOR, BUTTON_BORDER_WIDTH, BUTTON_TEXT_COLOR);
        draw_Button(X0_BUTTON_BC, Y0_BUTTON_BC, X1_BUTTON_BC, Y1_BUTTON_BC, "Reset", BUTTON_COLOR, BUTTON_BORDER_COLOR, BUTTON_BORDER_WIDTH, BUTTON_TEXT_COLOR);
        draw_Button(X0_BUTTON_BR, Y0_BUTTON_BR, X1_BUTTON_BR, Y1_BUTTON_BR, "Next", BUTTON_COLOR, BUTTON_BORDER_COLOR, BUTTON_BORDER_WIDTH, BUTTON_TEXT_COLOR);


        // fun
#ifdef FUN
        drawString(0, 0, "69", font, colorarray[colidx], backroundColor);
        drawString(DISPLAY_X_MAX-3*FONT_SPACING, 0, "420", font, colorarray[colidx], backroundColor);
#endif

    IntMasterEnable(); // Re-enable interrupts
    TimerEnable(TIMER0_BASE, TIMER_A); // start timer

    printf("Init Complete\n");
}



/********************************************************************************
     Empty main()
*********************************************************************************/
int main(void)
{
    init();
    while(1){}
}
