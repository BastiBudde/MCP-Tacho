#include "LCDDisplay.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "font.h"

#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"

/* same values as array for indexed colors */
int colorarray[]={
    0x00000000,
    0x00FFFFFF,
    0x00AAAAAA,
    0x00FF0000,
    0x0000FF00,
    0x000000FF,
    0x00FFFF00,
    0x00DF00FF,
    0x00E30022,
    0x00E9692C};


///////////////////////////////////////////////////////////////////////////////////
//       Elementary Display output functions  => speed optimized as inline       //
///////////////////////////////////////////////////////////////////////////////////
/*********************************************************************************/
inline void write_command(unsigned char command)
{   GPIO_PORTM_DATA_R = command;        // Write command byte
    GPIO_PORTL_DATA_R = 0x11;           // Chip select = 0, Command mode select = 0, Write state = 0
    GPIO_PORTL_DATA_R = 0x1F;           // Initial state
}
/*********************************************************************************/
inline void write_data(unsigned char data)
{   GPIO_PORTM_DATA_R = data;           // Write data byte
    GPIO_PORTL_DATA_R = 0x15;           // Chip select = 0, Write state = 0
    GPIO_PORTL_DATA_R = 0x1F;           // Initial state
}
/*********************************************************************************/
inline void window_set(uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y)
{
    write_command(0x2A);           // Set row address x-axis
    write_data(min_x >> 8);        // Set start  address           (high byte)
    write_data(min_x);             // as above                     (low byte)
    write_data(max_x >> 8);        // Set stop address             (high byte)
    write_data(max_x);             // as above                     (low byte)
    write_command(0x2B);           // Set column address (y-axis)
    write_data(min_y >> 8);        // Set start column address     (high byte)
    write_data(min_y);             // as above                     (low byte)
    write_data(max_y >> 8);        // Set stop column address      (high byte)
    write_data(max_y);             // as above                     (low byte)
}
/*********************************************************************************/


///////////////////////////////////////////////////////////////////////////////////
//       Elementary Display output functions  => speed optimized as inline       //
///////////////////////////////////////////////////////////////////////////////////
/*********************************************************************************/
void configure_display_controller_large(void) // 800 x 480 pixel ???
{
    GPIO_PORTL_DATA_R = INITIAL_STATE;      // Initial state
    GPIO_PORTL_DATA_R &= ~RST;              // Hardware reset
    SysCtlDelay(10000);                     // wait >1 ms
    GPIO_PORTL_DATA_R |= RST;               //
    SysCtlDelay(12000);                     // wait >1 ms

    write_command(SOFTWARE_RESET);          // Software reset
    SysCtlDelay(120000);                    // wait >10 ms

    write_command(SET_PLL_MN);               // Set PLL Freq to 120 MHz
    write_data(0x24);                        //
    write_data(0x02);                        //
    write_data(0x04);                        //

    write_command(START_PLL);                // Start PLL
    write_data(0x01);                        //
    SysCtlDelay(10000);                      // wait 1 ms

    write_command(START_PLL);                // Lock PLL
    write_data(0x03);                        //
    SysCtlDelay(10000);                      // wait 1 ms

    write_command(SOFTWARE_RESET);           // Software reset
    SysCtlDelay(100000);                     // wait 10 ms

    write_command(0xe6);                    // Set pixel clock frequency
    write_data(0x01);                       // KRR Set LCD Pixel Clock 9MHz
    write_data(0x70);                       // KRR
    write_data(0xA3);                       // KRR

    write_command(SET_LCD_MODE);          // SET LCD MODE SIZE, manual p. 44
    write_data(0x20);                     // ..TFT panel 24bit
    write_data(0x00);                     // ..TFT mode
    write_data(0x03);                     // SET horizontal size = 800-1 (high byte)
    write_data(0x1F);                     // SET horizontal size = 800-1 (low byte)
    write_data(0x01);                     // Set vertical size = 480-1 (high byte)
    write_data(0xDF);                     // Set vertical size = 480-1 (low byte)
    write_data(0x00);                     // Even line RGB sequence / Odd line RGB sequence RGB

    write_command(SET_HORI_PERIOD);       // Set Horizontal Period
    write_data(0x03);                     // Horizontal total period (display + non-displayed)  (high byte)
    write_data(0x5E);                     // Horizontal total period (display + non-display) (low byte)
    write_data(0x00);                     // Non-displayed period between the start of the horizontal sync (LLINE) signal and the first displayed data.
    write_data(0x46);                     // Low byte of the non-display period between the start of the horizontal sync (LLINE) signal and the first display data
    write_data(0x09);                     // Set the vertical sync width
    write_data(0x00);                     // Set horiz.Sync pulse start    (high byte)
    write_data(0x08);                     // Set horiz.Sync pulse start    (low byte)
    write_data(0x00);                     //

    write_command(SET_VERT_PERIOD);         // Set vertical periods, manual  p. 49
    write_data(0x01);                       // Vertical total period (display + non-displayed) in lines (high byte)
    write_data(0xFE);                       // as above (low byte) = total 510  lines
    write_data(0x00);                       //
    write_data(0x0C);                       // The non-displayed period in lines between the start of the frame and the first
                                            // display data = 12 line.s
    write_data(0x0A);                       // Set the vertiacla sync width = 10 pixels
    write_data(0x00);                       // Set vertical sync pulse start position (high byte)
    write_data(0x04);                       // as above (low byte) = total sync pulse start position is 4 lines

    write_command(SET_ADRESS_MODE);         // Pixel address counting = flip display , manual p. 36
    write_data(0x03);                       // necessary to match with touch screen addressing

//  write_command(0x0A);                    // Power control mode not tested in detail
//  write_data(0x1C);

    write_command(SET_PIXEL_DATA_FORMAT);    // set pixel data format 8bit manual p. 78
    write_data(0x00);

    write_command(SET_DISPLAY_ON);           // Set display on  manual p. 78
}


void draw_line_bresenham(int x0, int y0, int x1, int y1, uint32_t color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2, e2;

    while (1) {
        // Setze einen Pixel auf die aktuelle Position
        window_set(x0, y0, x0, y0);  // Pixelposition setzen
        write_command(0x2C);        // Schreibe Pixel-Befehl
        write_data((color >> 16) & 0xff); // Rotanteil
        write_data((color >> 8) & 0xff);  // Gr�nanteil
        write_data(color & 0xff);         // Blauanteil

        if (x0 == x1 && y0 == y1) break; // Linie fertig
        e2 = err;
        if (e2 > -dx) { err -= dy; x0 += sx; }
        if (e2 <  dy) { err += dx; y0 += sy; }
    }
}

void draw_rectangle(int x0, int y0, int x1, int y1, int border_width, uint32_t color) {
    // Zeichne die vier Linien des Rechtecks mit Bresenham
    int i;

    for(i=0; i<border_width; i++){
        draw_line_bresenham(x0+i, y0+i, x1-i, y0+i, color); // obere Linie
        draw_line_bresenham(x1-i, y0+i, x1-i, y1-i, color); // rechte Linie
        draw_line_bresenham(x1-i, y1-i, x0+i, y1-i, color); // untere Linie
        draw_line_bresenham(x0+i, y1-i, x0+i, y0+i, color); // linke Linie
    }
}

void draw_filled_rectangle(int x0, int y0, int x1, int y1, colors color){
    window_set(x0, y0, x1, y1);

    int npixel = (x1 - x0) * (y1 - y0);
    int n;
    write_command(0x2C);
    for (n = 0; n < npixel; n++) {
        write_data((color >> 16) & 0xff); // Rotanteil
        write_data((color >> 8) & 0xff);  // Gr�nanteil
        write_data(color & 0xff);         // Blauanteil
    }

}


void setPixel(int x, int y, uint32_t color) {
    window_set(x, y, x, y);  // Pixelposition setzen
    write_command(0x2C);    // Schreibe Pixel-Befehl
    write_data((color >> 16) & 0xff); // Rotanteil
    write_data((color >> 8) & 0xff);  // Grünanteil
    write_data(color & 0xff);         // Blauanteil
}

void rasterCircle(int cx0, int cy0, int radius, uint32_t color)
{
    int f = 1 - radius;
    int ddF_x = 0;
    int ddF_y = -2 * radius;
    int xx = 0;
    int yy = radius;

    setPixel(cx0, cy0 + radius, color);
    setPixel(cx0, cy0 - radius, color);
    setPixel(cx0 + radius, cy0, color);
    setPixel(cx0 - radius, cy0, color);

    while(xx < yy)
    {
        if (f >= 0)
        {
            yy -= 1;
            ddF_y += 2;
            f += ddF_y;
        }
        xx += 1;
        ddF_x += 2;
        f += ddF_x + 1;

        setPixel(cx0 + xx, cy0 + yy, color);
        setPixel(cx0 - xx, cy0 + yy, color);
        setPixel(cx0 + xx, cy0 - yy, color);
        setPixel(cx0 - xx, cy0 - yy, color);
        setPixel(cx0 + yy, cy0 + xx, color);
        setPixel(cx0 - yy, cy0 + xx, color);
        setPixel(cx0 + yy, cy0 - xx, color);
        setPixel(cx0 - yy, cy0 - xx, color);
    }
}

void rasterHalfCircle(int x0, int y0, int radius, bool upper, uint32_t color) {
    int f = 1 - radius;
    int ddF_x = 0;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;

    // Set only pixels in the upper half or lower half
    if (upper) {
        setPixel(x0, y0 + radius, color); // Top point
    } else {
        setPixel(x0, y0 - radius, color); // Bottom point
    }

    while (x < y) {
        if (f >= 0) {
            y -= 1;
            ddF_y += 2;
            f += ddF_y;
        }
        x += 1;
        ddF_x += 2;
        f += ddF_x + 1;

        // Only draw pixels based on the selected half
        if (upper) {
            setPixel(x0 + x, y0 + y, color); // Top-right
            setPixel(x0 - x, y0 + y, color); // Top-left
            setPixel(x0 + y, y0 + x, color); // Top-right (rotated)
            setPixel(x0 - y, y0 + x, color); // Top-left (rotated)
        } else {
            setPixel(x0 + x, y0 - y, color); // Bottom-right
            setPixel(x0 - x, y0 - y, color); // Bottom-left
            setPixel(x0 + y, y0 - x, color); // Bottom-right (rotated)
            setPixel(x0 - y, y0 - x, color); // Bottom-left (rotated)
        }
    }
}

void drawV(int startpoint_x, int startpoint_y, int height){
    // eine Linie von startpoint zu (startpoint - height/3, height)
    draw_line_bresenham(startpoint_x, startpoint_y, (startpoint_x-(height/2)), (startpoint_y-height),  LILA); // minus, da Nullpunkt oben links
    // eine Linie von startpoint zu (startpoint + height/3, height)
    draw_line_bresenham(startpoint_x, startpoint_y, (startpoint_x+(height/2)), (startpoint_y-height),  LILA);
}

void drawSymbol(int x, int y, const char symbol[212], uint32_t col, uint32_t bgcol) {
    int i, j;
    uint32_t row = 0;
    for (i = 0; i < 53; i++) { //geht die Zeilen durch
      row = (symbol[i*4+3] << 24) | (symbol[i*4+2] << 16) | (symbol[i*4+1] << 8) | symbol[i*4];
      for (j = 0; j < 32; j++) { //geht die Spalten durch
          if ((row >> (32 - j)) & 1) { //shiftet die Zeile i um
              setPixel(x + (32-j), y + i, col);
          }else{
              setPixel(x + (32-j), y + i, bgcol);
          }
      }
    }

    return;
}

void drawString(int x, int y, char* str, const char font[][212], uint32_t col, uint32_t bgcol) {
    uint16_t xx = x;
    while (*str) {
        drawSymbol(xx, y, font[(unsigned char)*str], col, bgcol);
        xx += FONT_SPACING;
        str++;
    }

    return;
}

void drawInteger(int x, int y, const char font[][212], uint16_t number, uint32_t col, uint32_t bgcol) {

    short hunderter = number/100;
    short zehner = (number-100*hunderter)/10;
    short einer = number - 100*hunderter - 10*zehner;

    if(hunderter == 0){
        drawSymbol(x, y, font[' '], col, bgcol);
    }else{
        drawSymbol(x, y, font[hunderter + '0'], col, bgcol);
    }

    if(hunderter == 0 && zehner == 0){
        drawSymbol(x+FONT_SPACING, y, font[' '], col, bgcol);
    }else{
        drawSymbol(x+FONT_SPACING, y, font[zehner + '0'], col, bgcol);
    }

    drawSymbol(x+2*FONT_SPACING, y, font[einer + '0'], col, bgcol);

    return;
}

void drawKM(int x, int y, const char font[][212], uint32_t dailyCM, uint32_t col, uint32_t bgcol) {

    uint32_t num = dailyCM / 1000;

    short hunderter = num/10000;
    short zehner = (num-10000*hunderter)/1000;
    short einer =  (num - 10000*hunderter - 1000*zehner)/100;
    short zehntel = (num - 10000*hunderter - 1000*zehner - 100*einer)/10;
    short hunderstel = num - 10000*hunderter - 1000*zehner - 100*einer - 10*zehntel;


    if(hunderter == 0){
        drawSymbol(x, y, font[' '], col, bgcol);
    }else{
        drawSymbol(x, y, font[hunderter + '0'], col, bgcol);
    }

    if(hunderter == 0 && zehner == 0){
        drawSymbol(x+FONT_SPACING, y, font[' '], col, bgcol);
    }else{
        drawSymbol(x+FONT_SPACING, y, font[zehner + '0'], col, bgcol);
    }

    drawSymbol(x+2*FONT_SPACING, y, font[einer + '0'], col, bgcol);
    drawSymbol(x+3*FONT_SPACING, y, font[','], col, bgcol);
    drawSymbol(x+4*FONT_SPACING, y, font[zehntel + '0'], col, bgcol);
    drawSymbol(x+5*FONT_SPACING, y, font[hunderstel + '0'], col, bgcol);

    return;
}
