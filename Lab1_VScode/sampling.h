#ifndef SAMPLING_H
#define SAMPLING_H

#include <stdint.h>

#define ADC_SAMPLING_RATE 1000000   // [samples/sec] desired ADC sampling rate
#define CRYSTAL_FREQUENCY 25000000  // [Hz] crystal oscillator frequency used to calculate clock rates

extern volatile uint32_t gButtons;  // debounced button state, one per bit in the lowest bits
extern uint32_t gADCSamplingRate;   // [Hz] actual ADC sampling rate

volatile uint16_t gADCBuffer; // circular buffer

void ADCInit(void);

void ADC_ISR(void);

#endif /* SAMPLING_H */
