#ifndef _LS012B7DD06_LIB_H_
#define _LS012B7DD06_LIB_H_

#include "config.h"

#include "graphics.h"

#include "esp_log.h"

// #include "freertos/semphr.h"	// for mutex

// static SemaphoreHandle_t mutex_handle_lcd;

// extern void rlcd_init( void );
extern void rlcd_testGPIOs( uint8_t pins_state );

extern void testTransmit( void );

extern void rlcd_init( void );

extern void togglePWM( void );

extern void rlcd_fillImageWhite( void );
extern void rlcd_fillImageColour( uint8_t colour );
extern void rlcd_putPixel( int16_t x, int16_t y, uint8_t colour );
extern void rlcd_updateImageBuf( void );

extern void rlcd_resume( void );
extern void rlcd_suspend( void );

// int rlcd_vprintf_func( const char *szFormat, va_list args );

#endif // _LS012B7DD06_LIB_H_