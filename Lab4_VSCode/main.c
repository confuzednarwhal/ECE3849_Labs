/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"

#include "Crystalfontz128x128_ST7735.h"

#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"

#include "buttons.h"
#include "sampling.h"

#include "kiss_fft.h"
#include "_kiss_fft_guts.h"


#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
#define ADC_OFFSET 2047.0

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz


uint32_t gSystemClock = 120000000; // [Hz] system clock frequency

volatile uint32_t gCopiedBuffer[1024];

volatile uint32_t triggerIndex;
uint32_t buttonPressed;

const char * const gVoltageScaleStr[] = {"100 mV", "200 mV", "500 mV", " 1 V", "2 V"};
volatile int currVoltageScaleInt = 3;
volatile int ADC_scaled_values[1024];

volatile int triggerValue = 0;

bool triggerFound = false;

tContext sContext;
tRectangle rectFullScreen;

// CPU load counters
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0.0;

float out_db[128];


uint32_t cpu_load_count(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}

void signal_init(void){
        // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
        GPIOPinConfigure(GPIO_PF2_M0PWM2);
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
        // configure the PWM0 peripheral, gen 1, outputs 2 and 3
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
        // use system clock without division
        PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
        PWMGenConfigure(PWM0_BASE, PWM_GEN_1,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,roundf((float)gSystemClock/PWM_FREQUENCY));
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
        PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();

    // hardware initialization goes here

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    //PWM signal generator
    signal_init();
    ButtonInit();
    ADCInit();

    // initialize timer 3 in one-shot mode for polled timing
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClock * 0.001); // 1 sec interval

    count_unloaded = cpu_load_count();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

//periodic buttons scanning and posts to button_sem3
void clock_func(){
        Semaphore_post(button_sem3); //signal button task
}

//high priority: scans buttons and places ID in to mailbox
void scan_buttons(){
    while(true){
        Semaphore_pend(button_sem3, BIOS_WAIT_FOREVER);
        ButtonISR();
        while(fifo_get(&buttonPressed)){
            Mailbox_post(buttonMailbox, &buttonPressed, BIOS_WAIT_FOREVER);
        }
    }

}

void updateVoltageScale(){
    if(currVoltageScaleInt == 4){
        //creates a wrap
        currVoltageScaleInt = 0;
    }else{
        currVoltageScaleInt++;
    }
}

//mid priority: processes user input from button mailbox
//              if not button pressed, task remains blocked, waiting on mailbox
//              signals display task
void modify_settings(){
    while(true){
        Mailbox_pend(buttonMailbox, &buttonPressed,  BIOS_WAIT_FOREVER);
        //modifies oscilloscope settings

        if(buttonPressed & 4){
            //rising trigger
           triggerValue = 1;
           //Semaphore_post(waveform_sem0);
        }
        else if(buttonPressed & 8){
            //falling trigger
            triggerValue = 2;
            //Semaphore_post(waveform_sem0);
        }
        //changes the voltage scale
        else if(buttonPressed & 2){
            updateVoltageScale();
            //Semaphore_post(processing_sem1);
        }
        Semaphore_post(display_sem2); //signal display task
    }
}

int RisingTrigger(void) // search for rising edge trigger
{
    // Step 1
    int x = getADCBufferIndex() - (Lcd_ScreenWidth/2);  //gets half way point of data on screen
    // Step 2
    triggerFound = true;
    int x_stop = x - ADC_BUFFER_SIZE/2;
    for (; x > x_stop; x--) {
        if ( gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET && gADCBuffer[ADC_BUFFER_WRAP(x) - 1] < ADC_OFFSET)

       break;
    }
    // Step 3
    if (x == x_stop){ // for loop ran to the end
        triggerFound = false; //no trigger was found
        x = getADCBufferIndex() - (Lcd_ScreenWidth/2); // reset x back to how it was initialized
    }
    return x;
}

int FallingTrigger(void) // search for rising edge trigger
{
    // Step 1
    int x = getADCBufferIndex() - (Lcd_ScreenWidth/2);  //gets half way point of data on screen
    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
    triggerFound = true;
    for (; x > x_stop; x--) {
        if ( gADCBuffer[ADC_BUFFER_WRAP(x)] <= ADC_OFFSET && gADCBuffer[ADC_BUFFER_WRAP(x) - 1] > ADC_OFFSET)
        break;
    }
    // Step 3
    if (x == x_stop){ // for loop ran to the end
        triggerFound = false; //no trigger was found
        x = getADCBufferIndex() - (Lcd_ScreenWidth/2); // reset x back to how it was initialized
    }
    return x;
}

//highest priority: searches for trigger and copies into a buffer
//                  signals processing task
void triggerSearch(void){
    IntMasterEnable();
    //int triggerValue = 0;
    while(true){
        Semaphore_pend(waveform_sem0, BIOS_WAIT_FOREVER);
        int a;
        for(a = 0; a <1024; a++){
            gCopiedBuffer[a] = gADCBuffer[ADC_BUFFER_WRAP(a)];
        }
        Semaphore_post(processing_sem1);
    }
}

//lowest priority: scales waveform
//                 signals display task then waveform task
void processing(void){
    // Kiss FFT config memory
    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE];
    size_t buffer_size = KISS_FFT_CFG_SIZE;
    kiss_fft_cfg cfg; // Kiss FFT config
    // complex waveform and spectrum buffers
    static kiss_fft_cpx in[NFFT], out[NFFT];
   // int i;
    // init Kiss FFT
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size);
    while(true){
        Semaphore_pend(processing_sem1, BIOS_WAIT_FOREVER);

        int i=0;
        for (i = 0; i < NFFT; i++) { // generate an input waveform
            in[i].r = gCopiedBuffer[i]; // real part of waveform
            in[i].i = 0; // imaginary part of waveform
        }
        kiss_fft(cfg, in, out); // compute FFT

        // convert first 128 bins of out[] to dB for display
        int z = 0;
        for(z = 0; z<128; z++){
            out_db[z] = 250 + (-10 * log10f(out[z].r * out[z].r + out[z].i * out[z].i));

        }

//        float Vmultiplyer;
//        if(currVoltageScaleInt < 3){
//            Vmultiplyer = 0.001;
//        }else{
//            Vmultiplyer = 1.0;
//        }
//
//        float fScale = (3.3 * 20)/((1 << 12) * (atof(gVoltageScaleStr[currVoltageScaleInt])*Vmultiplyer));
//        int a;
//        for(a = 0; a < 128; a++){
//            ADC_scaled_values[a] = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)gCopiedBuffer[a] - 2090));
//        }
        Semaphore_post(display_sem2);
        Semaphore_post(waveform_sem0);

    }
}

//low priority: draws one complete frame to LCD display
void display(void){

    while(true){
        Semaphore_pend(display_sem2, BIOS_WAIT_FOREVER);

        tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};
        //draws grid on screen
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black

        GrContextForegroundSet(&sContext, ClrBlue);
        uint8_t offset = 4;
        uint8_t pixels_per_div = 20;
        GrLineDrawH(&sContext, 0, 128, offset);
        GrLineDrawH(&sContext, 0, 128, offset+pixels_per_div);
        GrLineDrawH(&sContext, 0, 128, offset+pixels_per_div*2);
        GrLineDrawH(&sContext, 0, 128, offset+pixels_per_div*3);
        GrLineDrawH(&sContext, 0, 128, offset-1+pixels_per_div*3);
        GrLineDrawH(&sContext, 0, 128, offset+1+pixels_per_div*3);
        GrLineDrawH(&sContext, 0, 128, offset+pixels_per_div*4);
        GrLineDrawH(&sContext, 0, 128, offset+pixels_per_div*5);
        GrLineDrawH(&sContext, 0, 128, offset+pixels_per_div*6);
        GrLineDrawV(&sContext, offset, 0, 128);
        GrLineDrawV(&sContext, offset+pixels_per_div, 0, 128);
        GrLineDrawV(&sContext, offset+pixels_per_div*2, 0, 128);
        GrLineDrawV(&sContext, offset+pixels_per_div*3, 0, 128);
        GrLineDrawV(&sContext, offset-1+pixels_per_div*3, 0, 128);
        GrLineDrawV(&sContext, offset+1+pixels_per_div*3, 0, 128);
        GrLineDrawV(&sContext, offset+pixels_per_div*4, 0, 128);
        GrLineDrawV(&sContext, offset+pixels_per_div*5, 0, 128);
        GrLineDrawV(&sContext, offset+pixels_per_div*6, 0, 128);

        GrContextForegroundSet(&sContext, ClrYellow);

        int y = 0;
        for(y = 0; y <128; y++){
            if(y+1 < 128){
                //GrLineDrawV(&sContext, y,  ADC_scaled_values[y],  ADC_scaled_values[y+1]);
                GrLineDrawV(&sContext, y,  out_db[y],  out_db[y+1]);
            }
        }


        //prints voltage scale
//        GrContextForegroundSet(&sContext, ClrWhite);  //white text
//        GrStringDrawCentered(&sContext, gVoltageScaleStr[currVoltageScaleInt], 6, 30, 10, false);

        //calculates CPU load
        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float)count_loaded/count_unloaded; // compute CPU load
        cpu_load = cpu_load *100;
//
        char str[16];
//
//        //prints CPU load
        snprintf(str, sizeof(str), "CPU load = %03f %", cpu_load);
        GrStringDrawCentered(&sContext, str, -1, 60, 120, false);

        GrFlush(&sContext);

    }

}
