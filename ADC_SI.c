/*
 * ADC_SI.c
 *
 * Created: 24.07.2017 23:07:43
 *  Author: USER
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "ADC_SI.h"

#define ADMUX_MASK (_BV(REFS1) | _BV(REFS0)) // 10 bit resolution, internal ref and right shift of results
/*
void inline ADC_auto_trigger_init_8Mhz() {
	// turning analog comparator off
	ACSR &= ~_BV(ACIE);																	// first, turn off interrupt
	ACSR |= _BV(ACD);																	// disabling comparator

	//timer0 setup
 	TIMSK0 = _BV(TOIE0);																// enable timer0 overflow interrupt
 	TCCR0B |= _BV(CS02) | _BV(CS00);													// start timer @ 1024 prescalar

	//configuring ADC
	ADMUX = ADMUX_MASK; //(REFS1) | _BV(REFS0) | _BV(ADLAR);							// internal ref and left shift of results
	ADCSRA =_BV(ADEN) |_BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);	// setting ADC enable, enabling ADC interrupt, Auto trigger and prescaler = 128 to get 62.5 kHz @ 8 MHz
	ADCSRB = _BV(ADTS2);																// starting conversion on Timer0 Overflow
}

ISR(TIMER0_OVF_vect) {
}

ISR(ADC_vect) {
	voltages[cur_ADC_channel] = ADCW;
	(cur_ADC_channel == (ADC_CHANNELS_QTY - 1)) ? cur_ADC_channel = 0: ++cur_ADC_channel;
	ADMUX = ADMUX_MASK | cur_ADC_channel;												// setting new ADC channel
}
*/

//--------------------------------------------------------------------------------------------------------------------
//		frc10 is full range coefficient with 10 multiplier: frc = 1023 *10 / (full range = 5)
//		buf has to be size of 6: 2 for int part, dot, 2 for frac part and 1 for 0 terminator
//		depending on voltage value could be three outputs:
//				1)  3.61[0]
//				2)  10.25[0]
//				3)  200[0]
//--------------------------------------------------------------------------------------------------------------------
/*
void adc_get_str(uint8_t adc_ch, uint16_t frc10, char* buf) {

	uint32_t v = (uint32_t)voltages[adc_ch] *1000 / frc10;
	
	uint8_t p = v/100;
	itoa(p, buf, 10);
	if(p < 10) {
		++buf;
	} else if(p < 100) {
		buf += 2;
	} else {
		return;
	}
	*buf++ = '.';
	
	p = v%100;
	if (p < 10) {
		*buf++ = '0';
	}
	itoa(p, buf, 10);
	
}
*/

//-------------------------------------------------------------------------------------------
// ADC_single_conversion_mode_init_8Mhz()
//-------------------------------------------------------------------------------------------
void inline ADC_single_conversion_mode_init_8Mhz() {
	ACSR &= ~_BV(ACIE);							// disable ADC interrupt
	ACSR |= _BV(ACD);							// disable the comparator
	DIDR0 = _BV(ADC0D);							// block digital input buffer on channel 0 (vcc probe) to save power
	ADMUX =  ADMUX_MASK;						// set internal ref
	ADCSRA = _BV(ADPS2) | _BV(ADPS1);			// set prescaler to 64
}
//-------------------------------------------------------------------------------------------
// ADC_single_conversion_mode_init_1Mhz()
//-------------------------------------------------------------------------------------------
void inline ADC_single_conversion_mode_init_1Mhz() {
	ACSR &= ~_BV(ACIE);							// disable ADC interrupt
	ACSR |= _BV(ACD);							// disable the comparator
	DIDR0 = _BV(ADC0D);							// block digital input buffer on channel 0 (vcc probe) to save power
	ADMUX =  ADMUX_MASK;						// set internal ref
	ADCSRA = _BV(ADPS1) | _BV(ADPS0);			// set prescaler to 8
}



//-------------------------------------------------------------------------------------------
// ADC 10 bit measurement on channel ch
//-------------------------------------------------------------------------------------------
uint16_t ADC_measure_10bit(uint8_t ch) {

	ADMUX  = ADMUX_MASK | ch;					// set ADC channel

	ADCSRA |= _BV(ADSC);						// start measurement
	while(!(ADCSRA & _BV(ADIF))) {}				// wait till it's done
	ADCSRA |= _BV(ADIF);						// clear flag

	return ADCW;								// there is no ADCW reg in AVR. Compiler substitute it by reading first from ADCL and then from ADCH
}

//-------------------------------------------------------------------------------------------
// ADC 8 bit measurement on channel ch
//-------------------------------------------------------------------------------------------
uint8_t ADC_measure_8bit(uint8_t ch) {
	
	ADMUX  = ADMUX_MASK |_BV(ADLAR) | ch;       // set channel and left result adjustment
	
	ADCSRA |= _BV(ADSC);						// start measurement
	while(!(ADCSRA & _BV(ADIF))) {}
	ADCSRA |= _BV(ADIF);
	
	return ADCH;								// return only high byte
}


//-------------------------------------------------------------------------------------------
// gets voltage by measure it 8 times in a row.
// returns average value by shifting measurement sum by 3 bits to the right
//-------------------------------------------------------------------------------------------
uint16_t ADC_measure(uint8_t ch) {
	uint16_t vlt = 0;
	for (uint8_t i = 0; i < 8; i++) {
		vlt += ADC_measure_10bit(ch);
	}
	return vlt >>= 3;
}

//--------------------------------------------------------------------------------------------------------------------
//		frc10 is full range coefficient with 10 multiplier: frc = 1023 *10 / (full range = 5)
//		buf has to be size of 6: 2 for int part, dot, 2 for frac part and 1 for 0 terminator
//		depending on voltage value could be three outputs:
//				1)  3.61[0]
//				2)  10.25[0]
//				3)  200[0]
//--------------------------------------------------------------------------------------------------------------------
void ADC_get_str(uint8_t adc_ch, uint16_t frc10, char* buf) {
	
	uint32_t v = (uint32_t)ADC_measure(adc_ch)*1000 / frc10;
	
// 	UARTPrintValDec("vlt=", (uint8_t)v / 10);
// 	UARTPrintln("");
	
	uint8_t p = v/100;
	itoa(p, buf, 10);
	if(p < 10) {
		++buf;
		} else if(p < 100) {
		buf += 2;
		} else {
		return;
	}
	*buf++ = '.';
	
	p = v%100;
	if (p < 10) {
		*buf++ = '0';
	}
	itoa(p, buf, 10);
	
}
