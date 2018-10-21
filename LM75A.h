/*
 * LM75A.h
 *
 * Created: 18.07.2017 16:08:27
 *  Author: USER
 */ 


#ifndef LM75A_H_
#define LM75A_H_

#include <stdint.h>

void LM75A_measure();
int8_t LM75A_get_int_part();
uint8_t LM75A_get_frac_steps();
uint8_t LM75A_get_hundreds();

#endif /* LM75A_H_ */