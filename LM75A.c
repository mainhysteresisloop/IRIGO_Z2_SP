/*
 * LM75A.c
 *
 * Created: 18.07.2017 16:18:05
 *  Author: USER
 */ 


#include "LM75A.h"

#define LM75A_ADDR 0x48							            // 7 bits address A0, A1, A2 tied to GND


extern uint8_t i2c_start(uint8_t address);
extern uint8_t i2c_write(uint8_t data);
extern uint8_t i2c_read_ack(void);
extern uint8_t i2c_read_nack(void);
extern void i2c_stop(void);

static int8_t temp_int_part;
static uint8_t temp_frac_part;


void LM75A_measure() {
	i2c_start(LM75A_ADDR << 1);								// making write address
	i2c_write(0x00);										// pointer to temp register
	i2c_start((LM75A_ADDR << 1) | 0x01);					// making read address
	temp_int_part = i2c_read_ack();							// 1st byte reading ack (MSB)
	temp_frac_part = i2c_read_nack();						// 2nd byte reading nack (LSB)
	i2c_stop();

	temp_frac_part >>= 5;
	
	if(temp_int_part < 0) {
		temp_frac_part = 16 - temp_frac_part;				// transforming 2nd complement form
	}

}

int8_t LM75A_get_int_part() {
	return 	temp_int_part;
}

uint8_t LM75A_get_frac_steps() {
	return temp_frac_part;
}

//--------------------------------------------------------------------------------------------
// returns fractional part of the temperature with 2 digits after dot
//--------------------------------------------------------------------------------------------
uint8_t LM75A_get_hundreds() {
	
	uint8_t res = temp_frac_part * 12;
	
	if (temp_frac_part > 1) {						// apply corrections
		++res;
	}
	
	if (temp_frac_part >2) {
		++res;
	}
	
	if(temp_frac_part > 4) {
		++res;	
	}

	if(temp_frac_part > 6) {
		++res;
	}
	

	return res;
}
