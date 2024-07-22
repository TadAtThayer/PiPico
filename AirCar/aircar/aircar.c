/*
 * AirCar MarkII
 *
 * Tad Truex, F23
 * Mike Kokko edited for 24X to output time in us and raw quadrature counts
 *
 * Goals:
 * 	At a minimum, replicate the existing 8051 firmware.  Ideally,
 * 	modernize a bit, potentially adding additional sensors (accelerometer),
 * 	possibly a small display, an RTC, and (hopefully) eliminate the need for
 * 	9V battery.
 *
 * 	At a minimum
 *
 * 	1) Count the pulses from the rotary encoder.
 * 	2) Accurately record the value every 28.2uS (CHECK)
 * 	3) Appear as a USB-Drive when connected to a host.
 *
 *	Extras
 *	4) Add an oled display
 *	5) ??
 *
 *
 * 	Status: (Main status at https://github.com/users/tadtruex/projects/14)
 *
 * 	9/21: Pulse counting works fine.  The encoder is 360 PPR.  2.5"-ish
 * 	      diameter on a 16' track = 32 revolutions * 360 * 4 (quadrature) =
 * 	      46080 counts WC.  Signed int32_t is plenty.
 *
 * 	      Question:  The lab handout says 40 PPR - did Doug divide down to
 * 	      fit into an 8 bit number (it looks like the old data format is
 * 	      delta rather than absolute)
 *
 *      10/5: (V0.2) USB mass storage works fine.  It's a bit hokey because I only 
 *            store data for 1 file in memory, and it will re-appear even if
 *            it is deleted on the filesystem...  
 *
 *      1/4/24: Testing first prototypes back from manufacture.
 *            Reduce turn on delay to better catch initial transients.
 *            Reduce data collection by ~10%
 *
 *
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/flash.h"
#include <stdint.h>
#include <stdio.h>
#include <strings.h>

#include "bsp/board.h"
#include "ff.h"
#include <diskio.h>
#include <disk.h>
#include <ramdisk.h>

#include "version.h"


// sampling time for reading encoder
// if we sample too slow (large sample time) we won't get a smooth   
// if we sample too fast (small sample time) velocity signal will be very quantized
#define SAMPLE_TIME_US 20000


// This system will start recording data after the encoder has
// inidicated the wheels have rotate this many degrees.
// BUT NOTE: can also start data collection with USER button to catch full transient 
// #define DEGREES_TO_START 1


// define encoder count at which to stop data collection
// this generally corresponds to near the end of the AirCar track
// 25920 = 18 rev * 1440 counts/rev (approx dist to end of track)
#define MAX_ENC_COUNTS 25920 


#define BUTTON_DEBOUNCE_US 5000

typedef struct _encoder {
    uint leadPinNum;
    uint lagPinNum;
} encoderT;

const encoderT encoder = { 17, 16 };


// Gray coded transitions on the encoder.
#define Quadrant0 ((uint8_t)(0))
#define Quadrant1 ((uint8_t)(2))
#define Quadrant2 ((uint8_t)(3))
#define Quadrant3 ((uint8_t)(1))


// Track the wheel rotations in a 30.2 format (quarter rotations)
volatile int32_t pulseCount30p2 = 0;


// This is a bit of a magic number situation...
//
// We save the data in flash so it will be there in the event of an
//  "accident".  The erasing size is 4KB, so this is
// 1024 samples of uint32_t.  All a bit convoluted, but hopefully
// easy to tweak later if we want.
// M. Kokko changed to 4x1024 = 4096 samples of uint32_t
typedef uint32_t sample_t;
#define maxSamples 4*(FLASH_SECTOR_SIZE / sizeof(sample_t))

volatile sample_t SampleBuffer[ maxSamples ];
#define DATA_OFFSET ((2 * 1<<20) - sizeof(SampleBuffer))
// We have 2MB (magic #) --+                 |
// We store our data at the end -------------+

#define DATA_START (uint8_t *)(XIP_BASE + DATA_OFFSET)
//                                |
// This is where flash starts ----+

volatile uint16_t SampleCount = 2;  // use index 0 for length, skip index 1 to align with 2 value chunks (alternate time and encoder counts)

// Track the number of 28.2us pulses.
volatile int32_t counterTicks = 0;

// Track encoder errors
volatile uint32_t errorCount = 0;

// Interrupt handler for the encoder pin changes
void stateUpdate( uint gpio, uint32_t eventMask );

const uint led = PICO_DEFAULT_LED_PIN;

int do_test(void);

uint8_t pdisk[1<<15];


void pwmInterrupt(void);


BYTE buf[FF_MAX_SS];

void error( const char *buf ){
    printf( "%s\n", buf );
    while(1);
}

void initFS(void);
void writeDataFile( void );
FATFS fs;

int main() {

    uint32_t timecount = 0;    
    bool is_running = false;
    uint32_t start_time = time_us_32();
    uint32_t prev_loop_time = start_time;
    bool led_state = false;
    bool user_button_pressed = false;
    uint16_t user_button_count = 0;

    pulseCount30p2 = 0;
    counterTicks = 0;
    errorCount = 0;

    board_init();

    gpio_init(led);
    gpio_set_dir(led, GPIO_OUT);
    gpio_put(led,1);

    // Prepare for some cave man debugging
    stdio_uart_init();

    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);

    initRamDisk();
    initFS();

    // recall sample buffer and write data file
    // first thing when program starts
    memcpy( (void *)SampleBuffer, DATA_START, sizeof( SampleBuffer ) );
    if(SampleBuffer[0] > maxSamples)  
        SampleBuffer[0] = 0;
    writeDataFile();

    gpio_init_mask(1<< encoder.lagPinNum | 1 << encoder.leadPinNum);

    // Remove this in production
    gpio_set_pulls(16, true, true);
    gpio_set_pulls(17, true, true);

    gpio_set_irq_enabled_with_callback(encoder.lagPinNum, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &stateUpdate);
    gpio_set_irq_enabled_with_callback(encoder.leadPinNum, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &stateUpdate);

    // This is not cool.  Using -28 here causes the chip to hang.
    //  Updating to -28 in the handler is fine.
    //add_repeating_timer_us( -100, timerFunc, NULL, &rt);

    // Use GPIO 4 (for now) as an indicator of interrupt entry.
    gpio_init(4);
    gpio_set_function(4, GPIO_FUNC_SIO);
    gpio_set_dir(4, GPIO_OUT);
    gpio_put(4, 0);

    uint slice_num = pwm_gpio_to_slice_num(4);

    // 125MHz cycle time for PWM clock.  3525 ticks gets us ~28.2us
    pwm_set_wrap(slice_num, 3524);

    // Set channel A output high for one cycle before dropping
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 2000);

    // We're going to need an interrupt for this
    pwm_set_irq_enabled(slice_num, true);

    irq_set_exclusive_handler( PWM_IRQ_WRAP, pwmInterrupt );
    irq_set_priority( PWM_IRQ_WRAP, 0 );


    for( int i = 0; i < 26; i++ ) {
        if ( irq_is_enabled(i) ) {
            printf( "IRQ %d: %d\n", i, irq_get_priority(i) );
        }
    }


    // set up user button for starting data collection
    gpio_init(20);
    gpio_set_function(20, GPIO_FUNC_NULL);
    gpio_set_dir(20, GPIO_IN);
    gpio_set_pulls(20,true,false);

    printf("Main loop start\n");
    printf("maxSamples: %d\r\n",maxSamples);

    while(1){
    
        // get current time in microseconds
        timecount = time_us_32();

        // enforce timing for logging encoder count
        if( is_running && ((timecount - prev_loop_time) >= SAMPLE_TIME_US) && (SampleCount < (maxSamples-1))){
          /*  time_vec[array_idx] = timecount;
            count_vec[array_idx] = pulseCount30p2;
            ++array_idx;
            */
            //printf("%d,%d\r\n",timecount,pulseCount30p2);
           
            // add time and pulse count to sample buffer 
            SampleBuffer[SampleCount++] = timecount;
            SampleBuffer[SampleCount++] = pulseCount30p2;
            prev_loop_time = timecount;
        
        }

        // wait for (and debounce) user button press
        if( !is_running && !user_button_pressed && ((timecount - prev_loop_time) >= BUTTON_DEBOUNCE_US) ){
            if( !gpio_get(20) ){
                if(++user_button_count > 20){
                    user_button_pressed = true;
                }
            } else if(user_button_count > 0){
                --user_button_count;
            }
            prev_loop_time = timecount;
        }

        // Start data collection only after USER button is pushed
        // [REMOVED] Start data collection after wheel rotates by some number of degrees
        // Physical/hardware encoder is natively 360 counts/rev = 1 count/deg
        // So use deg<<2 to multiply by 4 (i.e. convert degrees to quadrature counts) 
        //if ( (!is_running && pulseCount30p2 > DEGREES_TO_START<<2) || user_button_pressed ) {
        if(user_button_pressed){
            is_running = true;
            gpio_put(led,0);
            
            // Set the PWM running
            counterTicks = 0;
            pwm_set_enabled(slice_num, true);
            pwm_clear_irq(slice_num);
            irq_set_enabled(PWM_IRQ_WRAP, true);

            printf( "Sampling enabled\n" );
        }
        
        // copy sample buffer to flash and hang
        if((SampleCount >= maxSamples) || (pulseCount30p2 >= MAX_ENC_COUNTS) ){
            uint32_t currentInterrupts;
            
            gpio_put(led,1);
            printf("Trying to write to file and copy buffer to flash\r\n");
            
            SampleBuffer[0] = SampleCount; // store count in first element of buffer
            writeDataFile(); // don't think this is really needed, it writes out again on restart anyway

            // Copy the buffer to flash
            currentInterrupts = save_and_disable_interrupts();
            flash_range_erase( DATA_OFFSET, sizeof(SampleBuffer) );
            flash_range_program( DATA_OFFSET, (uint8_t *)SampleBuffer, sizeof(SampleBuffer) );
            //restore_interrupts(currentInterrupts);
            printf("Done! %d errors.\n",errorCount);
            
            // hang and flash LED
            led_state = true;
            while(1){

                // flash LED 
                timecount = time_us_32();
                if( (timecount - prev_loop_time) > 100000){
                    led_state = !led_state;
                    gpio_put(led,led_state);
                    prev_loop_time = timecount;
                }

                // try to connect to PC as USB disk
                // doesn't really matter how long this takes after data collection has completed
                // NOTE: THIS ALSO DOESN'T SEEM TO WORK HERE!
                tud_task();
            }
        }

        // try to connect to PC host as USB disk?
        // TinuUSB functionality
        // but only do this when not collecting data to avoid overrunning sample time
        // (althouth this seems to generally execute very quickly)
        if(!is_running){
            tud_task();
        }
    }
}

void writeDataFile( void ) {
    FIL f;
    int n;
    int res;

    if (f_mount( &fs, "", 0 ) ) error("MOUNT");
    if ( (res=f_open( &f, "data.csv", FA_CREATE_ALWAYS | FA_WRITE ))) error( "File Open" );       
    
    n = sprintf( buf, "time_us,enc_count\n");
    f_write( &f, buf, n, &n );

    for ( int i = 2; i < SampleBuffer[0]; i+=2 ) {
        n = sprintf( buf, "%d,", SampleBuffer[i] );
        f_write( &f, buf, n, &n );
        n = sprintf( buf, "%d\n", SampleBuffer[i+1] );
        f_write( &f, buf, n, &n );
    }
    f_sync(&f);
    f_close(&f);
    printf("Closed and synced (%d errors)\n", errorCount);

}

void initFS(void) {

    FIL fil;

    int n;

    // Initialize the ram disk
    //if (f_fdisk(0, (LBA_t[]){100,0}, buf) ) error("FDISK");

    // Create a filesystem
    if (f_mkfs( "", 0, buf, FF_MAX_SS ) ) error("MKFS");

    if (f_mount( &fs, "", 0 ) ) error("MOUNT");

    if (f_open( &fil, "hello.txt", FA_CREATE_NEW | FA_WRITE )) error("FOPEN");

    /* Write a message */
    n = sprintf( buf, "%s", _version_str );
    f_write( &fil, buf, n, &n );

    /* Close the file */
    f_close(&fil);
    f_mount( 0, "", 0 );
}

// handle quadrature counting
void stateUpdate( uint gpio, uint32_t eventMask ){

    static uint8_t currentState = 0;
    static bool done = false;

    if ( !done ) {
        uint8_t nextState = 0;
        uint32_t pinState = gpio_get_all();

        nextState |= (pinState & (1<<encoder.lagPinNum))  ? 1 : 0 ;
        nextState |= (pinState & (1<<encoder.leadPinNum)) ? 2 : 0 ;

        switch ( currentState ) {
            case Quadrant0 :
                if (nextState == Quadrant1) { pulseCount30p2++; }
                if (nextState == Quadrant2) { errorCount++; }
                if (nextState == Quadrant3) { pulseCount30p2--; }
                break;
            case Quadrant1 :
                if (nextState == Quadrant2) { pulseCount30p2++; }
                if (nextState == Quadrant3) { errorCount++; }
                if (nextState == Quadrant0) { pulseCount30p2--; }
                break;
            case Quadrant2 :
                if (nextState == Quadrant3) { pulseCount30p2++; }
                if (nextState == Quadrant0) { errorCount++; }
                if (nextState == Quadrant1) { pulseCount30p2--; }
                break;
            case Quadrant3 :
                if (nextState == Quadrant0) { pulseCount30p2++; }
                if (nextState == Quadrant1) { errorCount++; }
                if (nextState == Quadrant2) { pulseCount30p2--; }
                break;
        }

        currentState = nextState;
    }
}

void pwmInterrupt( void ) {
    counterTicks++;
    gpio_put( 4, !gpio_get(4) );
    pwm_clear_irq(pwm_gpio_to_slice_num(4));
}
