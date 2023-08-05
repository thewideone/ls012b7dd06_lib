#include "ls012b7dd06_hal.h"
#include "ls012b7dd06.h"

#include "rom/ets_sys.h"    // for delay

#include "freertos/FreeRTOS.h"  // for task delay
#include "freertos/task.h"      // for task delay

#include "esp_log.h"

#include "i2s_parallel_driver/i2s_parallel.h"

uint8_t rlcd_buf[RLCD_BUF_SIZE] = { //[RLCD_BUF_SIZE] = {
    0xff
    //0x00, 0x00, // 2 dummy bits (clock edges) in front
    // 0x00, 0x00, 0xff,
    // 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
    // 0xff, 0x00, 0x00
    //0x00, 0x00  // 2 dummy bits (clock edges) at the end
};

static const char *TAG = "rlcd_lib";

void rlcd_setupPins( void ){
    // Setup GPIO pins

    // Additional for testing
    gpio_reset_pin(GPIO_NUM_20);
    gpio_set_direction(GPIO_NUM_20, GPIO_MODE_OUTPUT);

    // The following 4 are used for JTAG debugging by default
    gpio_reset_pin(RLCD_BCK);
    gpio_reset_pin(RLCD_R0);
    gpio_reset_pin(RLCD_R1);
    gpio_reset_pin(RLCD_G0);

    gpio_pullup_dis(RLCD_BCK);

    gpio_set_direction(RLCD_GSP, GPIO_MODE_OUTPUT);
    gpio_set_direction(RLCD_GCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(RLCD_GEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RLCD_INTB, GPIO_MODE_OUTPUT);

    gpio_set_direction(RLCD_VB_VCOM, GPIO_MODE_OUTPUT);
    gpio_set_direction(RLCD_VA, GPIO_MODE_OUTPUT);

    gpio_set_direction(RLCD_BSP, GPIO_MODE_OUTPUT);
    gpio_set_direction(RLCD_BCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(RLCD_R0, GPIO_MODE_OUTPUT);
    gpio_set_direction(RLCD_R1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RLCD_G0, GPIO_MODE_OUTPUT);
    gpio_set_direction(RLCD_G1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RLCD_B0, GPIO_MODE_OUTPUT);
    gpio_set_direction(RLCD_B1, GPIO_MODE_OUTPUT);
}

void rlcd_setAllGPIOs( uint8_t pins_state ){
    gpio_set_level( RLCD_GSP, pins_state );
    gpio_set_level( RLCD_GCK, pins_state );
    gpio_set_level( RLCD_GEN, pins_state );
    gpio_set_level( RLCD_INTB, pins_state );

    gpio_set_level( RLCD_VB_VCOM, pins_state );
    gpio_set_level( RLCD_VA, pins_state );

    gpio_set_level( RLCD_BSP, pins_state );
    gpio_set_level( RLCD_BCK, pins_state );
    gpio_set_level( RLCD_R0, pins_state );
    gpio_set_level( RLCD_R1, pins_state );
    gpio_set_level( RLCD_G0, pins_state );
    gpio_set_level( RLCD_G1, pins_state );
    gpio_set_level( RLCD_B0, pins_state );
    gpio_set_level( RLCD_B1, pins_state );
}

void rlcd_testGPIOs( uint8_t pins_state ){
    rlcd_setAllGPIOs( pins_state );

    // Toggle state of all pins
    pins_state = !pins_state;
}

/* void rlcd_fillBlack( void ){
    // Begin frame
    gpio_set_level( RLCD_INTB, 1 );
    ets_delay_us( thsINTB );
    gpio_set_level( RLCD_GSP, 1 );
    ets_delay_us( thsGSP );

    // First dummy GCK high level
    gpio_set_level( RLCD_GCK, 1 );
    ets_delay_us( thwGCK );

    for( uint8_t line_num = 0; line_num < 240; line_num++ ){

        // Line feed starts with GCK low
        gpio_set_level( RLCD_GCK, 0 );

        // 
        // MSB data of a single line:
        // 

        gpio_set_level( RLCD_BSP, 1 );
        // ets_delay_ns( thsBSP );

        // First dummy high BCK level
        gpio_set_level( RLCD_BCK, 1 );
        // ets_delay_us( thwBCK );
        gpio_set_level( RLCD_BCK, 0 );
        // ets_delay_us( tlwBCK );

        gpio_set_level( RLCD_BSP, 0 );

        // Transmit 240 pixels in pairs, that is edges of BCK,
        // so 120 pairs (states of BCK),
        // so 60 periods of BCK
        for( uint8_t i = 0; i < 60 ; i++ ){

            // Pull GSP low in the middle of MSB of the 1st line.
            // GSP is pulled low in the middle of the first low GCK pulse
            // that is around transmission of the 120th pixel in MSB of
            // the 1st line (30th period of BCK).
            if( line_num == 0 && i == 30 )
                gpio_set_level( RLCD_GSP, 0 );

            // Here would be some code for pulling data from
            // the display buffer and setting RGB pins accordingly
            // on rising edges.

            gpio_set_level( RLCD_BCK, 1 );
            // ets_delay_us( thwBCK );

            // And here for data on falling edges.

            gpio_set_level( RLCD_BCK, 0 );
            // ets_delay_us( tlwBCK );
            
        }

        gpio_set_level( RLCD_BCK, 1 );
        // ets_delay_us( tlsGEN );

        // Toggle GEN for the last BCK cycle
        gpio_set_level( RLCD_GEN, 1 );

        // ets_delay_us( thwBCK );
        gpio_set_level( RLCD_BCK, 0 );

        ets_delay_us( thwGEN ); // actually should be (thwGEN - thwBCK)

        gpio_set_level( RLCD_GEN, 0 );
        ets_delay_us( thhGCK );


        // 
        // LSB data of a single line:
        // 

        gpio_set_level( RLCD_GCK, 1 );

        gpio_set_level( RLCD_BSP, 1 );
        // ets_delay_ns( thsBSP );

        gpio_set_level( RLCD_BCK, 1 );
        // ets_delay_us( thwBCK );
        gpio_set_level( RLCD_BCK, 0 );
        // ets_delay_us( tlwBCK );

        gpio_set_level( RLCD_BSP, 0 );

        // Transmit 240 pixels in pairs, that is edges of BCK,
        // so 120 pairs (states of BCK),
        // so 60 periods of BCK
        for( uint8_t i = 0; i < 60 ; i++ ){
            // Here would be some code for pulling data from
            // the display buffer and setting RGB pins accordingly.
            gpio_set_level( RLCD_BCK, 1 );
            // ets_delay_us( thwBCK );
            gpio_set_level( RLCD_BCK, 0 );
            // ets_delay_us( tlwBCK );
            
        }

        gpio_set_level( RLCD_BCK, 1 );
        // ets_delay_us( tlsGEN );

        // Toggle GEN for the last BCK cycle
        gpio_set_level( RLCD_GEN, 1 );

        // ets_delay_us( thwBCK );
        gpio_set_level( RLCD_BCK, 0 );

        ets_delay_us( thwGEN ); // actually should be (thwGEN - thwBCK)

        gpio_set_level( RLCD_GEN, 0 );
        ets_delay_us( thhGCK );
    }

    // Trailing dummy GCK period
    gpio_set_level( RLCD_GCK, 0 );
    ets_delay_us( tlwGCK );
    gpio_set_level( RLCD_GCK, 1 );
    ets_delay_us( thwGCK );
    gpio_set_level( RLCD_GCK, 0 );

    gpio_set_level( RLCD_INTB, 0 );
}
*/

/* All bits are '1's except of the first and the last one.
// This should give total of 62 periods of BCK (that is binary '1's),
// that is 2 dummy periods and 60 for RGB data.
// The first and the last bit can be used for time adjustment of
// the whole BCK signal.
// const rlcd_bck_scan_code_t bck_payload = {
//     .data = 0x7FFFFFFFFFFFFFFE
//     // .data = 0x7FFFFE
// };
*/

i2s_parallel_buffer_desc_t bufdesc;

void rlcd_init( void ){
// void rlcd_init( i2s_parallel_config_t* cfg ){
    rlcd_setupPins();
    rlcd_setAllGPIOs( 0 );

    /* // For uint32_t buffer:

    // 1st transmitted byte octet
    rlcd_buf[0]  = 0xf0f2f4f8;         // 4th 3rd   \ 1st
    rlcd_buf[1]  = 0x01020408;         // 2nd 1st   /
    rlcd_buf[2]  = 0x10121418;         // 4th 3rd   \ 2nd
    rlcd_buf[3]  = 0x20222428;         // 2nd 1st   /
    
    // 2nd transmitted byte octet
    rlcd_buf[4]  = 0x40414248;                  // 3rd  
    rlcd_buf[5]  = 0x80818288;                  // 4th  
    rlcd_buf[6]  = 0x03060C18;  // 0b00000011   // 1st  
                                // 0b00000110
                                // 0b00001100
                                // 0b00011000
    rlcd_buf[7]  = 0x070E1C38;  // 0b00000111   // 2nd  
                                // 0b00001110
                                // 0b00011100
                                // 0b00111000
    */

    /* // For uint16_t buffer:

    // 1st transmitted byte octet
    //              MSB LSB <- MSB goes first from the FIFO
    rlcd_buf[0]  = 0xf0f1;         // 4th 3rd   \ 1st
    rlcd_buf[1]  = 0xf2f4;         // 2nd 1st   /
    rlcd_buf[2]  = 0xf801;         // 4th 3rd   \ 2nd
    rlcd_buf[3]  = 0x0204;         // 2nd 1st   /
    
    // 2nd transmitted byte octet
    rlcd_buf[4]  = 0x0810;         // 3rd  
    rlcd_buf[5]  = 0x1214;   // 4th  
    rlcd_buf[6]  = 0x1820;   // 1st  
    rlcd_buf[7]  = 0x2224;   // 2nd  

    // // 1st transmitted byte octet
    // rlcd_buf[0]  = 0x00f0;         // 4th 3rd   \ 1st
    // rlcd_buf[1]  = 0x00f1;         // 2nd 1st   /
    // rlcd_buf[2]  = 0x00f2;         // 4th 3rd   \ 2nd
    // rlcd_buf[3]  = 0x00f4;         // 2nd 1st   /
    
    // // 2nd transmitted byte octet
    // rlcd_buf[4]  = 0x00f8;         // 3rd  
    // rlcd_buf[5]  = 0x0001;   // 4th  
    // rlcd_buf[6]  = 0x0002;   // 1st  
    // rlcd_buf[7]  = 0x0004;   // 2nd  

    // // 1st transmitted byte octet
    // rlcd_buf[0]  = 0xf0f1;         // 4th 3rd   \ 1st
    // rlcd_buf[1]  = 0xf2f4;         // 2nd 1st   /
    // rlcd_buf[2]  = 0xf801;         // 4th 3rd   \ 2nd
    // rlcd_buf[3]  = 0x0204;         // 2nd 1st   /
    
    // // 2nd transmitted byte octet
    // rlcd_buf[4]  = 0x0810;         // 3rd  
    // rlcd_buf[5]  = 0x2040;   // 4th  
    // rlcd_buf[6]  = 0x8011;   // 1st  
    // rlcd_buf[7]  = 0x1214;   // 2nd  

    */

    /* // For uint8_t buffer:

    // Fill the LCD buffer
    // for( int i=0; i < (BUF_S-2)/8; i++ ){
    //     rlcd_buf[i*8+1] = 0x01;
    //     rlcd_buf[i*8+2] = 0x02;
    //     rlcd_buf[i*8+3] = 0x04;
    //     rlcd_buf[i*8+4] = 0x08;
    //     rlcd_buf[i*8+5] = 0x10;
    //     rlcd_buf[i*8+6] = 0x20;
    //     rlcd_buf[i*8+7] = 0x40;
    //     rlcd_buf[i*8+8] = 0x80;
    //     // rlcd_buf[i+1] = 1 << (i);
    // }

    // 1st transmitted byte quartet
    rlcd_buf[0]  = 0xf0;         // 3rd
    rlcd_buf[1]  = 0xf1;         // 4th
    rlcd_buf[2]  = 0xf2;         // 1st
    rlcd_buf[3]  = 0xf4;         // 2nd
                 //  TX chan mod     1    |   3   |
    // 2nd transmitted byte quartet
    rlcd_buf[4]  = 0xf8;         // 3rd  
    rlcd_buf[5]  = 0b00000001;   // 4th  
    rlcd_buf[6]  = 0b00000010;   // 1st  
    rlcd_buf[7]  = 0b00000100;   // 2nd  
    // 3rd transmitted byte quartet
    rlcd_buf[8]  = 0b00001000;   // 3rd  \ LSB
    rlcd_buf[9]  = 0b00010000;   // 4th  /    of FIFO which is 32-bit (4-byte) long
    rlcd_buf[10] = 0b00100000;   // 1st  \ MSB
    rlcd_buf[11] = 0b01000000;   // 2nd  /    of FIFO
    // 4th transmitted byte quartet etc.
    rlcd_buf[12] = 0b10000000;
    rlcd_buf[13] = 0b00010001;
    rlcd_buf[14] = 0b00010010;
    rlcd_buf[15] = 0b00010100;

    rlcd_buf[16] = 0b00011000;
    rlcd_buf[17] = 0b00010010;
    rlcd_buf[18] = 0b00100010;
    rlcd_buf[19] = 0b01000010;

    rlcd_buf[20] = 0b10000010;
    rlcd_buf[21] = 0b00100001;
    rlcd_buf[22] = 0b00100010;
    rlcd_buf[23] = 0b00100100;

    rlcd_buf[24] = 0b00101000;
    rlcd_buf[25] = 0b00110000;
    rlcd_buf[26] = 0b01100000;
    // rlcd_buf[27] = 0b01000000;
    // rlcd_buf[28] = 0b10000000;
    rlcd_buf[BUF_S-5] = 0xf0;

    rlcd_buf[BUF_S-4] = 0xf1;
    rlcd_buf[BUF_S-3] = 0x00;   // the last byte sent
    rlcd_buf[BUF_S-2] = 0xf4;
    rlcd_buf[BUF_S-1] = 0xf8;

    */

    for( uint16_t i=0; i < RLCD_DISP_W; i++ ){
        rlcd_buf[i] = (1<<2);
    }
    for( uint16_t i=0; i < RLCD_DISP_W; i++ ){
        rlcd_buf[2*i] = (1<<3);
    }



    // for( uint16_t i=0; i < LINE_WIDTH; i++ ){
    //     rlcd_buf[i] = (1<<0);
    // }

    // for( uint16_t i=0; i < LINE_WIDTH; i++ ){
    //     rlcd_buf[LINE_WIDTH + i] = (1<<1);
    // }

    // for( uint16_t i=0; i < LINE_WIDTH; i++ ){
    //     rlcd_buf[LINE_WIDTH*2 + i] = (1<<2);
    // }

    // for( uint16_t i=0; i < LINE_WIDTH; i++ ){
    //     rlcd_buf[LINE_WIDTH*3 + i] = (1<<3);
    // }

    


    // rlcd_buf[1] = 0x01;
    // rlcd_buf[2] = 0x02;
    // rlcd_buf[3] = 0x04;
    // rlcd_buf[4] = 0x08;
    // rlcd_buf[5] = 0x01;
    // rlcd_buf[6] = 0x02;
    // rlcd_buf[7] = 0x04;
    // rlcd_buf[8] = 0x08;

    // 
    // I2S setup
    // 

    // i2s_parallel_buffer_desc_t bufdesc;

    // i2s_parallel_config_t cfg = {
    //     .gpio_bus = {
    //         RLCD_R0, 	// 0 : d0 
    //         RLCD_R1, 	// 1 : d1
    //         RLCD_G0, 	// 2 : d2
    //         RLCD_G1, 	// 3 : d3
    //         RLCD_B0, 	// 4 : d4
    //         RLCD_B1, 	// 5 : d5
    //         RLCD_BSP,	// 6 : d6 (HS?)
    //         RLCD_GEN	// 7 : d7 (VS?)
    //     },
    //     .gpio_clk = RLCD_BCK,

    //     .bits = I2S_PARALLEL_BITS_8,    // 8-bit mode (8 parallel output lines)
    //     .clock_speed_hz = 2*1000*1000,  // pixel clock frequency
    //     .buf = &bufdesc                 // image buffer
    // };

    i2s_parallel_config_t cfg = {
        .gpio_bus = {
            RLCD_R0, 	// 0 : d0 
            RLCD_R1, 	// 1 : d1
            RLCD_G0, 	// 2 : d2
            RLCD_G1, 	// 3 : d3
            RLCD_B0, 	// 4 : d4
            RLCD_B1, 	// 5 : d5
            RLCD_BSP,	// 6 : d6 (HS?)
            RLCD_GEN	// 7 : d7 (VS?)
        },
        .gpio_clk = RLCD_BCK,

		// .tx_right_first = 1,
		// .rx_right_first = 1,
		// .tx_msb_right = 0,
		// .rx_msb_right = 0,
		// .tx_chan_mod = 1,
		// .rx_chan_mod = 1,

        .bits = I2S_PARALLEL_BITS_8,    // 8-bit mode (8 parallel output lines)
        .clock_speed_hz = 2*2*1250000,  // 5MHz pixel clock frequency
        .buf = &bufdesc                 // image buffer
    };

    // cfg->buf = &bufdesc;

    bufdesc.memory = rlcd_buf;
    bufdesc.size = RLCD_BUF_SIZE;

    vTaskDelay(50 / portTICK_PERIOD_MS);    // wait for voltage to stabilize (what voltage??)

    i2s_parallel_setup( &I2S1, &cfg );
    // i2s_parallel_setup( &I2S1, cfg );

    vTaskDelay(50 / portTICK_PERIOD_MS);

    ESP_LOGI( TAG, "I2S init done with flags:\n tx_right_first=%d,\n rx_right_first=%d,\n tx_msb_right=%d,\n rx_msb_right=%d,\n tx_chan_mod=%d,\n rx_chan_mod=%d",
                    cfg.tx_right_first, cfg.rx_right_first, cfg.tx_msb_right, cfg.rx_msb_right, cfg.tx_chan_mod, cfg.rx_chan_mod );

    // ESP_LOGI( TAG, "I2S init done with flags:\n tx_right_first=%d,\n rx_right_first=%d,\n tx_msb_right=%d,\n rx_msb_right=%d,\n tx_chan_mod=%d,\n rx_chan_mod=%d",
    //                 cfg->tx_right_first, cfg->rx_right_first, cfg->tx_msb_right, cfg->rx_msb_right, cfg->tx_chan_mod, cfg->rx_chan_mod );

    // gpio_set_level( RLCD_INTB, 1 );

    // gpio_set_level( RLCD_INTB, 0 );

    /*

    // Power-on sequence:

    // Turn on voltage regulators
    // and wait for them to settle.

    // Wait for 2 GCK cycles
    ets_delay_us( tlwGCK*2 + thwGCK*2 );

    // Pixel memory init: write all screen black.
    rlcd_fillBlack();


    // Wait >=30us for VCOM, VA and VB
    ets_delay_us( 30 );

    // Start VCOM, VA and VB



    // Normal operation:

    // Send a frame
    // ets_delay_ms



    // Power off sequence:

    // Pixel memory init: write all screen black.

    // Wait >=30us for VCOM, VA and VB
    ets_delay_us( 30 );

    // Turn off voltage regulators

    */

}

void testTransmit( void ){
    // i2s_setStopSignal();    // has to be before i2s_send_buf(), because I2S's ISR gets called earlier than any function called here I guess
    // i2s_send_buf( &I2S1, &bufdesc );
    i2s_send_buf( &I2S1 );
    // i2s_setStopSignal();
    
    // ets_delay_us( 30 );
    // i2s_stop( &I2S1 );
}