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

#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
#define ADC_OFFSET 2047.0

#include "buttons.h"
#include "sampling.h"

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz

volatile uint32_t gCopiedBuffer[128];

volatile uint32_t triggerIndex;
uint32_t buttonPressed;

const char * const gVoltageScaleStr[] = {"100 mV", "200 mV", "500 mV", " 1 V", "2 V"};
volatile int currVoltageScaleInt = 3;
volatile int ADC_scaled_values[128];

volatile int triggerValue = 0;

bool triggerFound = false;

tContext sContext;

// CPU load counters
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0.0;


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

//    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};


    //PWM signal generator
    signal_init();
    ADCInit();

    /* Start BIOS */
    BIOS_start();

    while(true){
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black

        //draws grid on screen
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

    }

    return (0);
}

//void enable_func(UArg arg1, UArg arg2)
//{
//    IntMasterEnable();
//
//    while (true) {
//        // do nothing
//    }
//}

int RisingTrigger(void) // search for rising edge trigger
{
    // Step 1
    int x = gADCBufferIndex - (Lcd_ScreenWidth/2);  //gets half way point of data on screen
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
        x = gADCBufferIndex - (Lcd_ScreenWidth/2); // reset x back to how it was initialized
    }
    return x;
}

int FallingTrigger(void) // search for rising edge trigger
{
    // Step 1
    int x = gADCBufferIndex - (Lcd_ScreenWidth/2);  //gets half way point of data on screen
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
        x = gADCBufferIndex - (Lcd_ScreenWidth/2); // reset x back to how it was initialized
    }
    return x;
}

void triggerSearch(void){
    IntMasterEnable();
    int triggerValue = 3;
    while(true){
        Semaphore_pend(waveform_sem0, BIOS_WAIT_FOREVER);
        if(triggerValue == 1){
            triggerIndex = RisingTrigger();
            //if no trigger value is found
            if(triggerFound == false) triggerValue = 0;
        }
        //find falling trigger
        else if(triggerValue == 2){
            triggerIndex = FallingTrigger();
            //if no trigger value is found
            if(triggerFound == false) triggerValue = 0;
        }

        //copy into buffer
        //if there is a trigger, copy values to gCopiedBuffer starting from the triggerIndex - 64 (left most pixel of the LCD)
        else if(triggerValue != 0){
            int index = 0;
            index = triggerIndex - 64;
            int a;
            for(a = 0; a <128; a++){
                gCopiedBuffer[a] = gADCBuffer[ADC_BUFFER_WRAP(index+a)];
            }
        }
        //if there is no trigger found, copy values from ADC to gCopiedBuffer
        else{
            int a;
            for(a = 0; a <128; a++){
                gCopiedBuffer[a] = gADCBuffer[ADC_BUFFER_WRAP(a)];
            }
        }

        Semaphore_post(processing_sem1);
//        Semaphore_post(waveform_sem0);
    }
}

void processing(void){
//    IntMasterEnable();
    while(true){
        Semaphore_pend(processing_sem1, BIOS_WAIT_FOREVER);
        float Vmultiplyer;
        if(currVoltageScaleInt < 3){
            Vmultiplyer = 0.001;
        }else{
            Vmultiplyer = 1.0;
        }

        float fScale = (3.3 * 20)/((1 << 12) * (atof(gVoltageScaleStr[currVoltageScaleInt])*Vmultiplyer));
        int a;
        for(a = 0; a < 128; a++){
            ADC_scaled_values[a] = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)gCopiedBuffer[a] - 2090));
        }
        Semaphore_post(display_sem2);
//        Semaphore_post(waveform_sem0);

    }
}

void display(void){
    while(true){
        Semaphore_pend(display_sem2, BIOS_WAIT_FOREVER);

//        tContext sContext;
        GrContextForegroundSet(&sContext, ClrYellow);
        int y = 0;
        for(y = 0; y <128; y++){
            if(y+1 < 128){
                GrLineDrawV(&sContext, y,  ADC_scaled_values[y],  ADC_scaled_values[y+1]);
            }
        }
        Semaphore_post(waveform_sem0);
//        Semaphore_post(processing_sem1);
    }

}
