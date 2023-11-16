#ifndef SAMPLING_H
#define SAMPLING_H

#include <stdint.h>

#define ADC_SAMPLING_RATE 1000000   // [samples/sec] desired ADC sampling rate
#define CRYSTAL_FREQUENCY 25000000  // [Hz] crystal oscillator frequency used to calculate clock rates

#define ADC_BUFFER_SIZE 2048
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))

extern volatile uint32_t gButtons;  // debounced button state, one per bit in the lowest bits
extern uint32_t gADCSamplingRate;   // [Hz] actual ADC sampling rate

volatile uint32_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
volatile int32_t gADCBufferIndex;

void ADCInit(void);

void ADC_ISR(void);

#endif /* SAMPLING_H */
