#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
//#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"

#include "sampling.h"
#include "inc/tm4c1294ncpdt.h"

#include "driverlib/udma.h"
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/gates/GateHwi.h>

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

#pragma DATA_ALIGN(gDMAControlTable, 1024) // address alignment required
tDMAControlTable gDMAControlTable[64]; // uDMA control table (global)

// public globals
uint32_t gADCSamplingRate;      // [Hz] actual ADC sampling rate



// imported globals
extern uint32_t gSystemClock;   // [Hz] system clock frequency
extern volatile uint32_t gTime; // time in hundredths of a second

//or ADC
//#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
// index wrapping macro
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))

volatile uint32_t gADCErrors = 0; // number of missed ADC deadlines

volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
volatile int32_t gADCBufferIndex;



// initialize ADC
void ADCInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO setup for analog input AIN3
    // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
//    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; // round up
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
//
    // choose ADC1 sequence 0; disable before configuring
    ADCSequenceDisable(ADC1_BASE, 0);
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0); // specify the "Always" trigger
    // in the 0th step, sample channel 3 (AIN3)
    // enable interrupt, and make it the end of sequence
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    // enable the sequence. it is now sampling

//    // enable sequence 0 interrupt in the ADC1 peripheral
//    ADCIntEnable(ADC1_BASE, 0);
//    IntPrioritySet(INT_ADC1SS0, 0); // set ADC1 sequence 0 interrupt priority
//    // enable ADC1 sequence 0 interrupt in int. controller
//    IntEnable(INT_ADC1SS0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(gDMAControlTable);
    // assign DMA channel 24 to ADC1 sequence 0
    uDMAChannelAssign(UDMA_CH24_ADC1_0);
    uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_ALL);
    // primary DMA channel = first half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R, (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);
    // alternate DMA channel = second half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R, (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);

    ADCSequenceDMAEnable(ADC1_BASE, 0); // enable DMA for ADC1 sequence 0
    ADCIntEnableEx(ADC1_BASE, ADC_INT_DMA_SS0); // enable ADC1 sequence 0 DMA interrupt

    ADCSequenceEnable(ADC1_BASE, 0);
}


// is DMA occurring in the primary channel?
volatile bool gDMAPrimary = true;
void ADC_ISR(void) // DMA
{
    ADCIntClearEx(ADC1_BASE, ADC_INT_DMA_SS0); // clear the ADC1 sequence 0 DMA interrupt flag
    // Check the primary DMA channel for end of transfer, and
    // restart if needed.

    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT) == UDMA_MODE_STOP) {
        // restart the primary channel (same as setup)
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R, (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);
        // DMA is currently occurring in the alternate buffer
        gDMAPrimary = false;
    }
    // Check the alternate DMA channel for end of transfer, and
    // restart if needed.
    if(uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT) == UDMA_MODE_STOP){
        //restart alt channel
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R, (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
        gDMAPrimary = true;
    }

    // The DMA channel may be disabled if the CPU is paused by the debugger
    if (!uDMAChannelIsEnabled(UDMA_SEC_CHANNEL_ADC10)) {
        // re-enable the DMA channel
        uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);
    }
}

int32_t getADCBufferIndex(void)
{
    int32_t index;

    IArg keyGateHwi0;
    keyGateHwi0 = GateHwi_enter(gateHwi0);

    if (gDMAPrimary) { // DMA is currently in the primary channel
        index = ADC_BUFFER_SIZE/2 - 1 - uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT);
    }
    else { // DMA is currently in the alternate channel
        index = ADC_BUFFER_SIZE - 1 - uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT);
    }
    GateHwi_leave(gateHwi0, keyGateHwi0);
    return index;
}
