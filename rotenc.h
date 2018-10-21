/*
 * rotenc.h
 *
 * Created: 03.06.2017 17:22:44
 *  Author: USER
 */ 

/******************************
*  very basic rotary encoder library
*  IMPORTANT: usage notes
*  hardware: 
*  encoder's terminal A and terminal B should be connected to pin0 and pin1 of RE_PORT
*  software:
*  lib configures and uses TIMER2. it must not be used or changed outside the library
*  sei command should be called in main 
*/


#ifndef ROTENC_H_
#define ROTENC_H_
#include <avr/io.h>

#define RE_NO_SIGNAL                0
#define RE_CLOCKWISE_STEP           1
#define RE_COUNTER_CLOCKWISE_STEP   2

#define RE_BUTTON_CLICK             1

#define RE_DDR       DDRB
#define RE_PORT      PORTB
#define RE_PIN_REG   PINB

#define RE_BUT_DDR       DDRD
#define RE_BUT_PORT      PORTD
#define RE_BUT_PIN_REG   PIND
#define RE_BUT_PIN       PD2  // PD7 for VSM debug

uint8_t re_rotation_signal;
uint8_t re_button_signal;

void re_init_at_mcu8Mhz();
void re_init_at_mcu1Mhz();

#endif /* ROTENC_H_ */