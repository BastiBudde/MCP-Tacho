/*
 * Name: MCP-Tacho
 * Author: Kim.Luu
 * Version: 1.0
 *
 * */

#include <stdio.h>
#include <stdint.h>
#include "inc/tm4c1294ncpdt.h"

#define CIRCUM 10 // cm
#define CM2M 100

void init(){
    SYSCTL_RCGCGPIO_R = 0x2000;  // Enable Port P
    while ((SYSCTL_RCGCGPIO_R & 0x2000) == 0);  //GPIO Clock ready?
    GPIO_PORTP_DEN_R = 0x03;            //PortD digital enable P0, P1
    GPIO_PORTP_DIR_R = 0x00;            //PortD Input
    printf("Init Complete\n");
}

void calcTagKm(int counter, float* value){
    *value = (((float)CIRCUM/(float)CM2M))*counter;
}

int main(void)
{
    init();
    int counter = 0;
    float tagKm = 0.0;
    while (1) {
        if((GPIO_PORTP_DATA_R & 0x02) == 0x02){
            while((GPIO_PORTP_DATA_R & 0x02) != 0x00);
            calcTagKm(counter++, &tagKm);
            printf("m= %3.3f\n", tagKm);

        }
    }
}
