#ifndef _LS012B7DD06_HAL_H_
#define _LS012B7DD06_HAL_H_

#include "driver/gpio.h"
#include "esp_log.h"

#include "config.h"

// 
// Notes
// 
// Make a general lib files like rlcd.h & *.c for abstract functions.
// Use a timer for VA, VB and VCOM.

// Pin definitions
// To use some of them, gpio_reset(pin_number) has to be called first.

// Start signal for Gate-Driver
#define RLCD_GSP    GPIO_NUM_0  //GPIO_NUM_20
// Clock signal for Gate-Driver
#define RLCD_GCK    GPIO_NUM_2  //GPIO_NUM_21
// Gate enable signal
#define RLCD_GEN    GPIO_NUM_26 //GPIO_NUM_25
// Initial signal for Binary/Gate-Driver
#define RLCD_INTB   GPIO_NUM_25 //GPIO_NUM_26

// Power pins (square wave signal)
// VB is the same as VCOM so they can be shorted (I hope)
// Black signal voltage of LCD
#define RLCD_VB_VCOM    GPIO_NUM_27 //GPIO_NUM_4  //GPIO_NUM_32
// White signal voltage of LCD
#define RLCD_VA         GPIO_NUM_20 //GPIO_NUM_0  //GPIO_NUM_33

// Start signal for the Binary-Driver
#define RLCD_BSP    GPIO_NUM_4  //GPIO_NUM_27
// Clock signal driving of Binary-Driver
#define RLCD_BCK    GPIO_NUM_21 //GPIO_NUM_2  //GPIO_NUM_14
// Red signal for odd Pixels
#define RLCD_R0     GPIO_NUM_12
// Red signal for even Pixels
#define RLCD_R1     GPIO_NUM_13
// Green signal for odd Pixels
#define RLCD_G0     GPIO_NUM_14 //GPIO_NUM_15
// Green signal for even Pixels
#define RLCD_G1     GPIO_NUM_15 //GPIO_NUM_2
// Blue signal for odd Pixels
#define RLCD_B0     GPIO_NUM_32 //GPIO_NUM_4
// Blue signal for even Pixels
#define RLCD_B1     GPIO_NUM_33 //GPIO_NUM_0

// 35, 34, 37, 38 input only

// Time definitions
// GPIO level change with no additional delay takes roughly 330ns.

// BSP setup time high level [ns]
#define thsBSP 200
// BSP setup time low level [ns]
#define tlsBSP 200

// BCK width high level [ns]
#define thwBCK 400
// BCK width low level [ns]
#define tlwBCK 400

// RGB data setup time [ns]
#define tsRGB 200
// RGB data hold time [ns]
#define thRGB 200

// GSP setup time high level [us]
#define thsGSP 10
// GSP setup time low level [us]
#define tlsGSP 10

// GCK width high level [us]
#define thwGCK 64
// GCK width low level [us]
#define tlwGCK 64
// GCK hold time high level [us]
#define thhGCK 4
// GCK setup time low level [us]
#define thsGCK 0

// GEN pulse width high level [us]
#define thwGEN 10
// GEN setup time (low level) [us]
#define tlwGEN 0    // minimum value

// Setup time (high level) [us]
#define thsINTB 10
// Hold time (high level) [us]
#define thhINTB 63

#define RLCD_BCK_FREQ ( 1000000000/thwBCK )   // 2.5MHz
#define RLCD_RGB_FREQ ( 1000000000/thRGB  )   // 5MHz

// It seems that transmission of one MSB or LSB of a single video line
// via I2S takes 63.625us.
// Resolution of GCK should be then 0.125us, that is 8MHz.
#define RLCD_GCK_FREQ 8000000   //( 1000000/thwGCK * 32 )   // 15.625kHz

#define RLCD_GSP_FREQ RLCD_GCK_FREQ

#define RLCD_INTB_FREQ 400000   // 0.4MHz, 2.5us resolution

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (RLCD_VA) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (29) // Frequency in Hertz. Set frequency at 5 kHz

#include "i2s_parallel_driver/i2s_parallel.h"
// // Extern here used for temporary config tests
// extern i2s_parallel_buffer_desc_t bufdesc;

// void rlcd_setupPins( void );
void rlcd_init( void );
// void rlcd_init( i2s_parallel_config_t* cfg );
void rlcd_testGPIOs( uint8_t pins_state );

void togglePWM( void );

void testTransmit( void );

void rlcd_fillImage( void );
void rlcd_updateImageBuf( void );   //bool all_black );

void rlcd_resume( void );
void rlcd_suspend( void );

#endif // _LS012B7DD06_HAL_H_