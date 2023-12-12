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

#include "inc/tm4c1294ncpdt.h"
#include "audio_waveform.h"


#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
#define ADC_OFFSET 2047.0

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz

uint32_t gPWMSample = 0; // PWM sample counter
uint32_t gSamplingRateDivider = 20; // sampling rate divider

float gSystemClock = 120000000.0; // [Hz] system clock frequency

volatile uint32_t gCopiedBuffer[1024];

volatile uint32_t triggerIndex;
uint32_t buttonPressed;

const char * const gVoltageScaleStr[] = {"100 mV", "200 mV", "500 mV", " 1 V", "2 V"};
volatile int currVoltageScaleInt = 3;
volatile int ADC_scaled_values[1024];

volatile int triggerValue = 0;

bool displayFFT = false;
bool triggerFound = false;

tContext sContext;
tRectangle rectFullScreen;

// CPU load counters
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0.0;

float out_db[128];
bool displayFF = false;

volatile uint32_t period = 0, last_count = 0, factor = 1;
float frequency;
uint32_t audio_period = 258;
uint32_t PWM_AUDIO_FREQ = 495000;  //[Hz]


uint32_t cpu_load_count(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}

void signal_init(){
        // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
        GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
        GPIOPinConfigure(GPIO_PF2_M0PWM2);
        GPIOPinConfigure(GPIO_PF3_M0PWM3);
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
        // configure the PWM0 peripheral, gen 1, outputs 2 and 3
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
        // use system clock without division
        PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
        PWMGenConfigure(PWM0_BASE, PWM_GEN_1,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,roundf((float)gSystemClock/PWM_FREQUENCY));
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
        PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
        PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

void PWM_audio_init(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    // use system clock without division
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, audio_period);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5,roundf((float)gSystemClock/AUDIO_SAMPLING_RATE*0.5f));
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO);
    //calculated gSamplingRateDivider
    gSamplingRateDivider = period/AUDIO_SAMPLING_RATE;
}

void PWM_ISR(void)
{
    PWMGenIntClear(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO); // clear PWM interrupt flag
    // waveform sample index
    int i = (gPWMSample++) / gSamplingRateDivider;
    // write directly to the PWM compare B register
    PWM0_2_CMPB_R = 1 + gWaveform[i];

    if (i >= gWaveformSize) { // if at the end of the waveform array
        // disable these interrupts
        PWMIntDisable(PWM0_BASE, PWM_INT_GEN_2);
        // reset sample index so the waveform starts from the beginning
        gPWMSample = 0;
    }
}

void freq_timer_init(void){
    // config GPIO PD0 as timer input T0CCP0 at BoosterPack Connector #1 pin 14
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_T0CCP0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    // use maximum load value
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffff);
    // use maximum prescale value
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff);
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}



void TimerIntISR(void){
    //clear Timer0A Capture interrupt flag
    TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT);

    //use TimerValueGet to read full 24 bit captured timer count
    uint32_t count = TimerValueGet(TIMER0_BASE, TIMER_A);
    period = (count - last_count) & 0xffffff;
    last_count = count;
}

int freq_calc(void){
    return gSystemClock/period;
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
    freq_timer_init();
    PWM_audio_init();

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
            //increases period
            factor = factor + 1;
            displayFF = false;
            PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,roundf((float)gSystemClock*factor/PWM_FREQUENCY));
        }
        else if(buttonPressed & 8){
            //decreases period
            factor = factor - 1;
            displayFF = false;
            PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,roundf((float)gSystemClock*factor/PWM_FREQUENCY));
        }
        //changes the voltage scale
        else if(buttonPressed & 2){
            updateVoltageScale();
            //Semaphore_post(processing_sem1);
        }
        else if(buttonPressed & 1){
            displayFFT = true;
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
        if(triggerValue == 1 && !displayFFT){
            triggerIndex = RisingTrigger();
            //if no trigger value is found
            //if(triggerFound == false) triggerValue = 0;
                int index = 0;
                index = triggerIndex - 64;

                int a;
                for(a = 0; a <128; a++){
                    gCopiedBuffer[a] = gADCBuffer[ADC_BUFFER_WRAP(index+a)];
                }
        }


        //find falling trigger
        if(triggerValue == 2  && !displayFFT){
            triggerIndex = FallingTrigger();
            //if no trigger value is found
            //if(triggerFound == false) triggerValue = 0;
        }

        //copy into buffer
        //if there is a trigger, copy values to gCopiedBuffer starting from the triggerIndex - 64 (left most pixel of the LCD)
        if(triggerFound && !displayFFT){

            int index = 0;
            index = triggerIndex - 64;
            int a;
            for(a = 0; a <128; a++){
                gCopiedBuffer[a] = gADCBuffer[ADC_BUFFER_WRAP(index+a)];
            }
        }
        //if there is no trigger found, copy values from ADC to gCopiedBuffer
        else{
            triggerFound == false;
            int a;
            for(a = 0; a <128; a++){
                gCopiedBuffer[a] = gADCBuffer[ADC_BUFFER_WRAP(a)];
            }
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

        if(displayFFT){
            int y = 0;
            for(y = 0; y <128; y++){
                if(y+1 < 128){
                    GrLineDrawV(&sContext, y,  out_db[y],  out_db[y+1]);
                }
            }

        }else{
            int y = 0;
            for(y = 0; y <128; y++){
                if(y+1 < 128){
                    GrLineDrawV(&sContext, y,  ADC_scaled_values[y],  ADC_scaled_values[y+1]);
                }
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


        //print frequency
        frequency = freq_calc();
        char freqStr[12];
        snprintf(freqStr, sizeof(freqStr), "f = %03f Hz", frequency);
        GrStringDrawCentered(&sContext, freqStr, -1, 60, 110, false);

        char periodStr[10];
        snprintf(periodStr, sizeof(periodStr), "T = %03u", period);
        GrStringDrawCentered(&sContext, periodStr, -1, 60, 100, false);


        GrFlush(&sContext);

    }

}
