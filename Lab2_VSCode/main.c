/**
 * main.c
 *
 * ECE 3849 Lab 0 Starter Project
 * Gene Bogdanov    10/18/2017
 *
 * This version is using the new hardware for B2017: the EK-TM4C1294XL LaunchPad with BOOSTXL-EDUMKII BoosterPack.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#define ADC_BUFFER_SIZE 2048
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
#define ADC_OFFSET 2047.0

#include "buttons.h"
#include "sampling.h"

uint32_t gSystemClock; // [Hz] system clock frequency

bool gS1 = false;

volatile uint32_t gTime = 8345; // time in hundredths of a second
// volatile int32_t gADCBufferIndex;
volatile uint16_t gADCBuffer[];
volatile uint32_t gCopiedBuffer[128];

volatile uint32_t triggerIndex;
uint32_t buttonPressed;

const char * const gVoltageScaleStr[] = {"100.0", "200.0", "500.0", " 1.0", "2.0"};
volatile int currVoltageScaleInt = 3;
volatile int ADC_scaled_values[128];

// CPU load counters
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0.0;

//#pragma FUNC_CANNOT_INLINE(cpu_load_count)
//uint32_t cpu_load_count(void);

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

int RisingTrigger(void) // search for rising edge trigger
{
    // Step 1
    int x = gADCBufferIndex - (Lcd_ScreenWidth/2);  //gets half way point of data on screen
    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
    for (; x > x_stop; x--) {
        if ( gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET && gADCBuffer[ADC_BUFFER_WRAP(x) - 1] < ADC_OFFSET)
        break;
    }
    // Step 3
    if (x == x_stop){ // for loop ran to the end
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
    for (; x > x_stop; x--) {
        if ( gADCBuffer[ADC_BUFFER_WRAP(x)] <= ADC_OFFSET && gADCBuffer[ADC_BUFFER_WRAP(x) - 1] > ADC_OFFSET)
        break;
    }
    // Step 3
    if (x == x_stop){ // for loop ran to the end
        x = gADCBufferIndex - (Lcd_ScreenWidth/2); // reset x back to how it was initialized
    }
    return x;
}

float getfScaleValue(){
    float Vmultiplyer;
    if(currVoltageScaleInt < 3){
        Vmultiplyer = 0.001;
    }else{
        Vmultiplyer = 1.0;
    }

    float fScale = (3.3 * 18)/((1 << 12) * (atof(gVoltageScaleStr[currVoltageScaleInt])*Vmultiplyer));
    //float fScale = (3.3 * 20.0)/((1 << 12) * (atof(gVoltageScaleStr[currVoltageScaleInt])*Vmultiplyer));
    //float trial = atof(gVoltageScaleStr[currVoltageScaleInt])*0.001;
    //float val = (3.3 * 18)/((1 << 12) * (atof(gVoltageScaleStr[currVoltageScaleInt])));

    return fScale;
}

void scaleVoltageValues(){
//    float Vmultiplyer;
//    if(currVoltageScaleInt < 3){
//        Vmultiplyer = 0.001;
//    }else{
//        Vmultiplyer = 1.0;
//    }

    float scaleValue = getfScaleValue();
    int a;
    for(a = 0; a < 128; a++){
        ADC_scaled_values[a] = LCD_VERTICAL_MAX/2 - (int)roundf(scaleValue * ((int)gCopiedBuffer[a] - 2090));
    }
}

uint16_t getHighValue(){
    float scaleValue = getfScaleValue();
    uint16_t newHValue = scaleValue * 4086;
    return newHValue;
}

uint8_t getLowValue(){
    float scaleValue = getfScaleValue();
    uint8_t newLValue = scaleValue *10;
    return newLValue;
}

void updateVoltageScale(){
    if(currVoltageScaleInt == 4){
        currVoltageScaleInt = 0;
    }else{
        currVoltageScaleInt++;
    }
    scaleVoltageValues();
}

//uint32_t cpu_load_count(void)
//{
//    uint32_t i = 0;
//    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
//    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
//    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
//        i++;
//    return i;
//}

int main(void)
{
    IntMasterDisable();

    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font


    uint32_t time;  // local copy of gTime
    char str[50];   // string buffer
    // full-screen rectangle
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    signal_init();
    ButtonInit();
    ADCInit();

//    // initialize timer 3 in one-shot mode for polled timing
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
//    TimerDisable(TIMER3_BASE, TIMER_BOTH);
//    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
//    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClock - 1); // 1 sec interval

//    count_unloaded = cpu_load_count();

    IntMasterEnable();

    while (true) {

        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black

        //draws grid on screen
        GrContextForegroundSet(&sContext, ClrBlue);
        uint8_t offset = 9;
        uint8_t pixels_per_div = 18;
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

        while(!fifo_get(&buttonPressed));

        //get the first 128 values from the ADC buffer when S1 is pressed
        if(buttonPressed & 4)
        {
            triggerIndex = RisingTrigger();

            int a;
            for(a = 64; a>0; a--){
                //last 64 samples
                gCopiedBuffer[63+a] = gADCBuffer[triggerIndex+a];
            }
            int b;
            for(b = 0; b<64; b++){
                //first 64 samples
                gCopiedBuffer[b] = gADCBuffer[triggerIndex+b];
            }

            scaleVoltageValues();
        }

        if(buttonPressed & 8){
            triggerIndex = FallingTrigger();

            int a;
            for(a = 64; a>0; a--){
                //last 64 samples
                gCopiedBuffer[63+a] = gADCBuffer[triggerIndex+a];
            }
            int b;
            for(b = 0; b<64; b++){
                //first 64 samples
                gCopiedBuffer[b] = gADCBuffer[triggerIndex+b];
            }

            scaleVoltageValues();
        }

        if(buttonPressed & 2) updateVoltageScale();

        GrContextForegroundSet(&sContext, ClrYellow);

        //for 128 samples and 128 pixels across so each sample is one pixel
        uint16_t high = getHighValue();
        uint8_t low = getLowValue();
        int y = 0;
        for(y = 0; y < 128; y++){
            if(ADC_scaled_values[y] < low){
                // draw to bottom of display
                GrLineDrawH(&sContext, offset+y, offset+1+y, offset + pixels_per_div*6);

            }
            if(ADC_scaled_values[y] <= low & ADC_scaled_values[y+1] >= high | ADC_scaled_values[y+1] <= low & ADC_scaled_values[y] >= high){
                //draw vert line
                GrLineDraw(&sContext, offset+y+1, offset+pixels_per_div*6, offset+y+1, offset);
            }
            if(ADC_scaled_values[y] > high){
                //draw to top of display
                GrLineDrawH(&sContext, offset+y, offset+1+y, offset);
            }
        }

        GrFlush(&sContext); // flush the frame buffer to the LCD


//        count_loaded = cpu_load_count();
//        cpu_load = 1.0f - (float)count_loaded/count_unloaded; // compute CPU load
    }
}
