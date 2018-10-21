/*
 * rotenc.c
 *
 * Created: 03.06.2017 17:26:30
 *  Author: Sergey Shelepin
 */ 
#include "rotenc.h"
#include <avr/interrupt.h>

#define FALSE 0
#define TRUE 1

uint8_t encoder_code;
uint8_t encoder_val;

volatile uint8_t re_button_counter;
uint8_t re_button_state;

#define BUT_IS_UP          0
#define BUT_PRESSED        1
#define BUT_SIGNAL_SENT    2

void re_init_at_mcu8Mhz() {

	RE_DDR &= ~0b11;
	RE_BUT_DDR &= ~_BV(RE_BUT_PIN);
	
	TCCR2A |= _BV(WGM21);                         // set CTC mode
	OCR2A = 20;//31;                              // put val into ORCA to get compare match interrupt in wvwry 4 ms
	TIMSK2 = _BV(OCIE2A);						  // enable compare match A interrupt
	TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);   // run timer with 1024 prescalar
	
	re_button_state = BUT_IS_UP;
}

void re_init_at_mcu1Mhz() {

	RE_DDR &= ~0b11;
	RE_BUT_DDR &= ~_BV(RE_BUT_PIN);
	
	TCCR2A |= _BV(WGM21);                         // set CTC mode
	OCR2A = 20;//31;                              // put val into ORCA to get compare match interrupt in wvwry 4 ms
	TIMSK2 = _BV(OCIE2A);						  // enable compare match A interrupt
	TCCR2B = _BV(CS22)  | _BV(CS20);			  // run timer with 128 prescalar
	
	re_button_state = BUT_IS_UP;
}



inline void re_proc() {
		encoder_val |= (RE_PIN_REG & 0x03);
		register uint8_t rf = FALSE;
		switch (encoder_val) {
			case 0b0001:
			case 0b0111:
			case 0b1110:
			case 0b1000:
			case 0b0010:
			case 0b0100:
			case 0b1101:
			case 0b1011:
				rf = TRUE;
			break;
			
			default:
			break;
		}
		
		if (rf) {                                                     // check if rotated 
			encoder_code <<= 2;                                       // 4 steps per 1 click 
			encoder_code |= (encoder_val & 3);                        // add one step to code var
			if(encoder_code == 0x4B) {                                // ccw gray code
				re_rotation_signal = RE_COUNTER_CLOCKWISE_STEP;
			} else if(encoder_code == 0x87) {				          // cw gray code
				re_rotation_signal = RE_CLOCKWISE_STEP;				  
			}
		}
		encoder_val <<=2;                                 
		encoder_val &= 0xF;	
}

void re_but_proc() {
	
	switch(re_button_state) {
		
		case BUT_IS_UP:												 // if button was not pressed
			if(!(RE_BUT_PIN_REG & _BV(RE_BUT_PIN))) {                // check if it's pressed now		
				re_button_state = BUT_PRESSED;	                     // if yes, change the state to button pressed
				re_button_counter = 0;								 // start counter from 0				
			}
		break;
		
		case BUT_PRESSED:											 // if button was pressed
			if(re_button_counter > 10) {                             // wait for some time
				if(!(RE_BUT_PIN_REG & _BV(RE_BUT_PIN))) {            // check if it's still pressed
					re_button_signal = RE_BUTTON_CLICK;              // if yes, send the signal
					re_button_state = BUT_SIGNAL_SENT;
				}				
			}		
		break;
		
		case BUT_SIGNAL_SENT:
			if(RE_BUT_PIN_REG & _BV(RE_BUT_PIN)) {                   // wait until unpressed
					re_button_state = BUT_IS_UP;				     // change state to unpressed
			}		
		break;
	}
}


ISR(TIMER2_COMPA_vect) {
	re_proc();
	++re_button_counter;	
	if(!(re_button_counter & 0b111)) {                               // every 8th time of interrupt call
		re_but_proc();
	}

}