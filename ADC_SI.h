/*
 * ADC_SI.h
 *
 * Created: 24.07.2017 23:07:23
 *  Author: USER
 */ 


#ifndef ADC_SI_H_
#define ADC_SI_H_

#include <stdint.h>

#define ADC_CHANNELS_QTY 3			 // number of ADC channels that we are measuring 

// volatile uint8_t cur_ADC_channel;
// volatile uint16_t voltages[ADC_CHANNELS_QTY];

#define ADC_enable()	   ADCSRA |= _BV(ADEN)								// ADC on/off macro
#define ADC_disable() 	   ADCSRA &= ~_BV(ADEN)

//void ADC_auto_trigger_init_8Mhz();

//void adc_get_str(uint8_t adc_ch, uint16_t frc10, char* buf);

void ADC_single_conversion_mode_init_8Mhz();
void ADC_single_conversion_mode_init_1Mhz();
uint16_t ADC_measure(uint8_t ch);
void ADC_get_str(uint8_t adc_ch, uint16_t frc10, char* buf);

#endif /* ADC_SI_H_ */