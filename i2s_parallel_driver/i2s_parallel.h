#ifndef _I2S_PARALLEL_DRIVER_H_
#define _I2S_PARALLEL_DRIVER_H_

#include <stdint.h>
#include "soc/i2s_struct.h"

#include "config.h"

typedef enum {
    I2S_PARALLEL_BITS_8 	= 8,
    I2S_PARALLEL_BITS_16	= 16,
    I2S_PARALLEL_BITS_32	= 32,
} i2s_parallel_cfg_bits_t;

typedef struct {
    lcd_colour_t* memory;
    size_t size;
} i2s_parallel_buffer_desc_t;

typedef struct {
    int gpio_bus[24];
    // I2S clock output pin
    int gpio_clk;

    uint32_t clock_speed_hz;

    // When set, makes WS active low
    // (adds a leading and trailing edge of the clock line).
    uint8_t tx_right_first;
    uint8_t rx_right_first;

    // Doesn't do anything
    // uint8_t tx_msb_right;
    // Doesn't do anything
    // uint8_t rx_msb_right;

    // uint8_t tx_chan_mod;
    // uint8_t rx_chan_mod;
    // uint16_t tx_msb_right : 1;
    // uint16_t rx_msb_right : 1;
    // uint16_t tx_chan_mod : 3;
    // uint16_t rx_chan_mod : 3;

    i2s_parallel_cfg_bits_t bits;
    i2s_parallel_buffer_desc_t* buf;
} i2s_parallel_config_t;

void i2s_stop( i2s_dev_t* dev );

void i2s_parallel_setup(i2s_dev_t* dev, const i2s_parallel_config_t* cfg);

void outDataBuf_clearImage( void );
void outDataBuf_update( void );
// void i2s_updateOutputBuf( i2s_dev_t *dev, bool all_black );

void i2s_prepareTx( i2s_dev_t *dev );
void i2s_startTx( i2s_dev_t *dev );
// void i2s_send_buf( i2s_dev_t* dev );

// void i2s_send_buf( i2s_dev_t *dev, i2s_parallel_buffer_desc_t* data_buf );
// void i2s_setStopSignal( bool state );
void i2s_setStopSignal( void );
bool getInvalidFlag( i2s_dev_t* dev, int* current_dma_desc );

#endif // _I2S_PARALLEL_DRIVER_H_