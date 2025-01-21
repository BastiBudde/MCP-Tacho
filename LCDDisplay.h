#ifndef LCDDISPLAY_H
#define LCDDISPLAY_H

#include <stdint.h>
#include <stdbool.h>

// constants for display initialization
#define RST 0x10
#define INITIAL_STATE (0x1F)
#define SOFTWARE_RESET (0x01)
#define SET_PLL_MN (0xE2)
#define START_PLL (0xE0)
#define LOCK_PLL (0xE0)  // same as START_PLL
#define SET_LSHIFT (0xE6)
#define SET_LCD_MODE (0xB0)
#define SET_HORI_PERIOD (0xB4)
#define SET_VERT_PERIOD (0xB6)
#define SET_ADRESS_MODE (0x36)
#define SET_PIXEL_DATA_FORMAT (0xF0)
#define SET_DISPLAY_ON (0x29)
#define SET_DISPLAY_OFF (0x29) // not tested ??

#define DISPLAY_X_MAX 800
#define DISPLAY_Y_MAX 480

/* some predefined basic colors to use with names */
typedef enum {
    BLACK  = 0x00000000,
    WHITE  = 0x00FFFFFF,
    GREY   = 0x00AAAAAA,
    RED    = 0x00FF0000,
    GREEN  = 0x0000FF00,
    BLUE   = 0x000000FF,
    YELLOW = 0x00FFFF00,
    LILA   = 0x00DF00FF,
    ROT    = 0x00E30022,
    ORANGE = 0x00E9692C
}colors;

extern int colorarray[];


inline void write_command(unsigned char command);
inline void write_data(unsigned char data);
inline void window_set(uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y);
void configure_display_controller_large(void);
void draw_line_bresenham(int x0, int y0, int x1, int y1, uint32_t color);
void draw_rectangle(int x0, int y0, int x1, int y1, int border_width, uint32_t color);
void draw_filled_rectangle(int x0, int y0, int x1, int y1, colors color);
void setPixel(int x, int y, uint32_t color);
void rasterCircle(int cx0, int cy0, int radius, uint32_t color);
void rasterHalfCircle(int x0, int y0, int radius, bool upper, uint32_t color);
void drawV(int startpoint_x, int startpoint_y, int height);
void drawSymbol(int x, int y, const char digit[212], uint32_t col, uint32_t bgcol);
void drawString(int x, int y, char* str, const char font[][212], uint32_t col, uint32_t bgcol);
void draw_Button(int x0, int y0, int x1, int y1, const char* str, uint32_t bg_color, uint32_t border_color, uint16_t border_width, uint32_t text_color);
void drawInteger(int x, int y, const char font[][212], uint16_t number, uint32_t col, uint32_t bgcol);
void drawKM(int x, int y, const char font[][212], uint32_t dailyCM, uint32_t col, uint32_t bgcol);


#endif // LCDDISPLAY_H
