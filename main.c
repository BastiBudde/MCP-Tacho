/*
 * Name: MCP-Tacho
 * Author: Kim.Luu
 * Version: 1.0
 *
 * */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c1294ncpdt.h"
#include "bits.h"
#include "driverlib/interrupt.h"

#define CIRCUM 10 // cm
#define CM2M 100

uint32_t cnt0 = 0;
uint32_t cnt1 = 0;

void P0edgeISR(){
    SET(GPIO_PORTP_ICR_R, BIT0);
    cnt0++;
}

void P1edgeISR(){
    SET(GPIO_PORTP_ICR_R, BIT1);
    cnt1++;
}

void init(){


    SET(    SYSCTL_RCGCGPIO_R,  BIT13);         // Enable Port P
    while ((SYSCTL_RCGCGPIO_R & BIT13) == 0);   // GPIO Clock ready?
    CLEAR(  GPIO_PORTP_DIR_R,   BIT0 | BIT1);   // Port P0, P1 Input
    SET(    GPIO_PORTP_DEN_R,   BIT0 | BIT1);   // Port P digital enable P0, P1

    CLEAR(  GPIO_PORTP_IS_R,    BIT0 | BIT1);   // edge-sensitive interrupts on P0, P1
    SET(    GPIO_PORTP_IBE_R,   BIT0 | BIT1);   // both edges generate interrupt P0, P1
    SET(    GPIO_PORTP_ICR_R,   BIT0);          // Clear interrupt flag for P0
    SET(    GPIO_PORTP_ICR_R,   BIT1);          // Clear interrupt flag for P1
    IntRegister(INT_GPIOP0, P0edgeISR);         // registering functions as ISRs
    IntRegister(INT_GPIOP1, P1edgeISR);
    IntEnable(INT_GPIOP0);
    IntEnable(INT_GPIOP1);
    SET(    GPIO_PORTP_IM_R,    BIT0 | BIT1);   // un-maks interrupts from P0, P1




    printf("Init Complete\n");
}

void calcTagKm(int counter, float* value){
    *value = ((((float)CIRCUM)/((float)CM2M)))*counter;
}

int main(void)
{
    init();
    for(;;){}
}
