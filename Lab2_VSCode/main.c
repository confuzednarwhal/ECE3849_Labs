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

#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#define ADC_BUFFER_SIZE 2048
#define ADC_OFFSET 2048

#include "buttons.h"
#include "sampling.h"

uint32_t gSystemClock; // [Hz] system clock frequency

bool gS1 = false;

volatile uint32_t gTime = 8345; // time in hundredths of a second
// volatile int32_t gADCBufferIndex;
volatile uint16_t gADCBuffer[];
volatile uint16_t gCopiedBuffer[128];

uint32_t buttonPressed;

int signal_init(void){
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
    int x = gADCBufferIndex - (Lcd_ScreenWidth/2);
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
            int i = 0;
            for(i = 0; i < 128; i++){
                gCopiedBuffer[i] = gADCBuffer[i];
            }
            //gS1 = false;
        }

        GrContextForegroundSet(&sContext, ClrYellow);

        //for 128 samples and 128 pixels across so each sample is one pixel
        uint16_t high = 4086;
        uint8_t low = 10;
        int y = 0;
        for(y = 0; y < 128; y++){
            if(gCopiedBuffer[y] < low){
                // draw to bottom of display
                GrLineDrawH(&sContext, offset+y, offset+1+y, offset + pixels_per_div*6);

            }
            if(gCopiedBuffer[y] < low & gCopiedBuffer[y+1] > high | gCopiedBuffer[y+1] < low & gCopiedBuffer[y] > high){
                //draw vert line
                GrLineDraw(&sContext, offset+y+1, offset+pixels_per_div*6, offset+y+1, offset);
            }
            if(gCopiedBuffer[y] > high){
                //draw to top of display
                GrLineDrawH(&sContext, offset+y, offset+1+y, offset);
            }
        }

//previous timer code
//        time = gTime; // read shared global only once
////        snprintf(str, sizeof(str), "Time = %06u", time); // convert time to string
//        uint32_t time_m = (time / 100) / 60 % 60;
//        uint32_t time_s = time/100 % 60;
//        uint32_t time_ff = time - ((time_m*60 + time_s)*100);
//
//        snprintf(str, sizeof(str), "Time = %02u:%02u:%02u", time_m, time_s, time_ff);
//
//        GrContextForegroundSet(&sContext, ClrYellow); // yellow text
//        GrStringDraw(&sContext, str, /*length*/ -1, /*x*/ 0, /*y*/ 0, /*opaque*/ false);


        GrFlush(&sContext); // flush the frame buffer to the LCD
    }
}
