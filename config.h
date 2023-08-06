#ifndef _RLCD_CONFIG_H_
#define _RLCD_CONFIG_H_

// 
// This file contains only definitions and structures
// to avoid circular dependence.
// 

// 
// Colour format:
//  7 6 5 4 3 2 1 0 <- bit no.
//  _______________
// |-|-|R|R|G|G|B|B|
// 
typedef union {
    uint8_t bits;
} lcd_colour_t;

// Basic colours
#define COLOUR_WHITE   0xff
#define COLOUR_BLACK   0x00
#define COLOUR_RED     0b00110000
#define COLOUR_GREEN   0b00001100
#define COLOUR_BLUE    0b00000011

#define COLOUR_CYAN    (COLOUR_GREEN | COLOUR_BLUE)
#define COLOUR_MAGENTA (COLOUR_RED   | COLOUR_BLUE)
#define COLOUR_YELLOW  (COLOUR_RED   | COLOUR_GREEN)

// Display width
#define RLCD_DISP_W 240
// Display height
#define RLCD_DISP_H 240

// Display colour depth (in bits per channel).
// Add a possibility to make it 1 or monochrome?
#define RLCD_COLOUR_DEPTH 2

// Display frame buffer size (in bytes).
// Add correction for a round display or leave dummy bits as they are.
#define RLCD_BUF_SIZE (RLCD_DISP_W * RLCD_DISP_H) //* RLCD_COLOUR_DEPTH / 8
// #define LINE_WIDTH RLCD_DISP_W+4
// #define RLCD_BUF_SIZE 4*LINE_WIDTH
//(RLCD_DISP_W * RLCD_DISP_H) * sizeof(lcd_colour_t)

// GPIO line position in "config->gpio_bus" array
#define GPIO_BUS_R0_BIT 0
#define GPIO_BUS_R1_BIT 1
#define GPIO_BUS_G0_BIT 2
#define GPIO_BUS_G1_BIT 3
#define GPIO_BUS_B0_BIT 4
#define GPIO_BUS_B1_BIT 5
#define GPIO_BUS_BSP_BIT 6
#define GPIO_BUS_GEN_BIT 7

// #define RLCD_BCK_FREQ 5000000

// thwGEN defined by the number of pixels
// that would be equivalently transmitted in that time.
// 
// thwGEN = 10us
// thwGEN / ( tsRGB + thRGB ) = 10us / 400ns = 25
#define THWGEN_LEN 25

// thhGCK defined by the number of pixels
// that would be equivalently transmitted in that time.
// 
// thhGCK = 4.8us
// thhGCK / ( tsRGB + thRGB ) = 4.8us / 400ns = 12
#define THHGCK_LEN 12

#define RGB_CLK_LEADING_DUMMY_PERIOD_CNT 2
// 35 periods are for 14us of GEN signaling
#define RGB_CLK_TRAILING_DUMMY_PERIOD_CNT ( THWGEN_LEN + THHGCK_LEN + 1)//( RGB_CLK_LEADING_DUMMY_PERIOD_CNT + 33 )


#endif // _RLCD_CONFIG_H_