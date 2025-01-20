#include "Touch.h"

uint8_t button_changed;
uint8_t color_changed;
int xpos;
int ypos;

void touch_write(unsigned char value)
{
    unsigned char i = 0x08; // 8 bit command
    unsigned char x, DI;
    GPIO_PORTD_AHB_DATA_R &= 0xFB; //CS=0
    while (i > 0) {
        DI = (value >> 7);
        if (DI == 0) {GPIO_PORTD_AHB_DATA_R &= 0xfe;} //out bit=0
        else {GPIO_PORTD_AHB_DATA_R |= 0x01;} //out bit=1
        value <<= 1; //next value
        GPIO_PORTD_AHB_DATA_R |= 0x08; //Clk=1
        for (x = 0; x < 10; x++);
        GPIO_PORTD_AHB_DATA_R &= 0xf7; //Clk=0
        for (x = 0; x < 10; x++);
        i--;
    }
}


unsigned int touch_read()
{
    unsigned char i = 12; // 12 Bit ADC
    unsigned int x, value = 0x00;
    while (i > 0)
    {
        value <<= 1;
        GPIO_PORTD_AHB_DATA_R |= 0x08; //Clk=1
        for (x = 0; x < 10; x++);
        GPIO_PORTD_AHB_DATA_R &= 0xf7; //Clk=0
        for (x = 0; x < 10; x++);
        value |= ((GPIO_PORTD_AHB_DATA_R >> 1) & 0x01); // read value
        i--;
    }
    GPIO_PORTD_AHB_DATA_R |= 0x04; //CS=1
    return value;
}


void GPIOPortDIntHandler(){
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_4);
    int x;
    for (x = 0; x < 10; x++);           //Busy wait
    xpos = touch_read();                //xpos value read ( 0......4095 )
    touch_write(0x90);                  //Touch Command YPos read
    for (x = 0; x < 10; x++);           //Busy wait
    ypos = touch_read();                //ypos value read ( 0.....4095 )

    if((ypos >= 3000) && (ypos<= 4095)){
        if ((xpos >= 3000) && (xpos<= 4095)){
            if (button_changed){
                color_changed = !color_changed;
            }else{
                button_changed = 0x00;
            }

        }else if((xpos >= 0) && (xpos<= 1000)){
            button_changed = 0x01;
        }
    }
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_4);
}
