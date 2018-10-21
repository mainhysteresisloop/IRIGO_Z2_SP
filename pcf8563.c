/*
 * pcf8563.c
 *
 * Created: 18.07.2017 14:52:04
 *  Author: USER
 */ 

#include <stdint.h>
#include "pcf8563.h"
#include <avr/interrupt.h>

#define TRUE  1
#define FALSE 0

#define SM_VAL_MASK   0b01111111   // sec, min mask
#define HD_VAL_MASK   0b00111111   // hour and days mask
#define DOW_VAL_MASK  0b00000111   // day of week mask
#define MON_VAL_MASK  0b00011111   // month mask

#define DOW_MASK    0b01111111  // 

extern uint8_t i2c_start(uint8_t address);
extern uint8_t i2c_write(uint8_t data);
extern uint8_t i2c_read_ack(void);
extern uint8_t i2c_read_nack(void);
extern uint8_t i2c_transmit(uint8_t address, uint8_t* data, uint16_t length);
extern uint8_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length);
extern uint8_t i2c_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
extern uint8_t i2c_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
extern void i2c_stop(void);

extern void UARTPrintInt16(uint16_t i, uint8_t raddix);
extern void UARTPrintUint(uint8_t ui, uint8_t raddix);
extern void UARTPrint( const char *str );
extern void UARTPrintln( const char *str );

//uint8_t rtc_base_year;

//void correct_feb_days(uint8_t year8_t);

uint8_t dec2bcd(uint8_t d) {
	return (d/10 << 4) + (d % 10);
}

uint8_t bcd2dec(uint8_t b) {
	return (b >> 4) * 10 + (b & 0b1111);
}

/*
void pcf8583_set_month_days(rtc_time* rt) {
	*(rtc_days_in_month)     = 31;    //Jan
	*(rtc_days_in_month + 1) = 28;    //Feb
	*(rtc_days_in_month + 2) = 31;    //Mar

	*(rtc_days_in_month + 3) = 30;    //Apr
	*(rtc_days_in_month + 4) = 31;    //May
	*(rtc_days_in_month + 5) = 30;    //Jun

	*(rtc_days_in_month + 6) = 31;    //Jul
	*(rtc_days_in_month + 7) = 31;    //Avg
	*(rtc_days_in_month + 8) = 30;    //Sep
	
	*(rtc_days_in_month + 9) = 31;    //Oct
	*(rtc_days_in_month + 10) = 30;   //Nov
	*(rtc_days_in_month + 11) = 31;   //Dec			
	
	correct_feb_days(rt->year8_t);
// 		31, 28, 31,
// 		30, 31, 30,
// 		31, 31, 30,
// 		31, 30, 31
//	};  
}
*/
//----------------------------------------------------------------------
//  set hours minutes and seconds to pcf8563
//----------------------------------------------------------------------
uint8_t pcf8563_read_byte(uint8_t offset) {
	register uint8_t res;
	i2c_start(PCF8563_ADDR_WRITE);
	i2c_write(offset);
	i2c_start(PCF8563_ADDR_READ);
	res = i2c_read_nack();
	i2c_stop();
	return res;
}

uint8_t pcf8583_write_byte(uint8_t offset, uint8_t value) {
	if(!i2c_start(PCF8563_ADDR_WRITE)) {
		return FALSE;
	}
	i2c_write(offset);
	i2c_write(value);                                        
	i2c_stop();
	return TRUE;
}

//----------------------------------------------------------------------
//  check if year is leap year
//----------------------------------------------------------------------
uint8_t is_leap_year(uint16_t year) {

	if( (year%400 == 0 || year%100 != 0) && (year%4 == 0)) {
		return TRUE;
	}
	return FALSE;
}
/*
//----------------------------------------------------------------------
//  
//----------------------------------------------------------------------
void correct_feb_days(uint8_t year8_t) {
	if (is_leap_year((uint16_t)year8_t + 2000)) {
		rtc_days_in_month[1] = 29;
	} else {
		rtc_days_in_month[1] = 28;
	}	
}
*/
//----------------------------------------------------------------------
//  set time to pcf8583, take data from rtc_time struct
//----------------------------------------------------------------------
void pcf8563_set_time(rtc_time* rt) {
	
	cli();                                                                    // disable interrupts: due to rtc write ongoing transaction 
	if (!i2c_start(PCF8563_ADDR_WRITE)){									  // starting i2c communication
		return;
	}
	i2c_write(PCF8563_SECONDS_REG);											  // write start address
	//	sec, min, hour, day, dow, mon, year
	for(uint8_t i = 0; i < 7; i++) {										  // writing 7 bytes
		i2c_write(dec2bcd(*((uint8_t*)rt + i)));
	}
	i2c_stop();																  // stop communication

	sei();
	
}
//----------------------------------------------------------------------
//  read time from pcf8583to rtc_time struct
//----------------------------------------------------------------------
void pcf8563_get_time(rtc_time *rt) {
	
	cli();                                                                    // disable interrupts: due to rtc read ongoing transaction 

	if(!i2c_start(PCF8563_ADDR_WRITE)) {									  // starting i2c communication
		return;
	}
	i2c_write(PCF8563_SECONDS_REG);											  // write reading start address
	i2c_start(PCF8563_ADDR_READ);											  // start reading
//	sec, min, hour, day, dow, mon, year
	uint8_t i = 0;
	while(i < 6) {															  // read 6 bytes with ack
		*((uint8_t*)rt + i) = i2c_read_ack();
		i++;
	}
	*((uint8_t*)rt + i) = i2c_read_nack();									  // and last byte with nack
	i2c_stop();																  // stop communication

	rt->sec &= SM_VAL_MASK;													  // apply masks for all values 
	rt->min &= SM_VAL_MASK;
	rt->hour &= HD_VAL_MASK;
	rt->mday &= HD_VAL_MASK;
	rt->dow &= DOW_VAL_MASK;
	rt->mon &= MON_VAL_MASK;

	i = 0;
	while (i < 7) {															  // decoding values from bcd code to decimal 
		*((uint8_t*)rt + i) = bcd2dec(*((uint8_t*)rt + i));
		++i;
	}
	sei();
}
//----------------------------------------------------------------------
//  print time for debug
//----------------------------------------------------------------------
void pcf8563_print_time(rtc_time *rt) {
	UARTPrintUint(rt->mday, 10); 
	UARTPrint("-");
	UARTPrintUint(rt->mon, 10);
	UARTPrint("-");
	UARTPrintUint(rt->year8_t, 10);
	UARTPrint(" ");
	UARTPrintUint(rt->dow, 10); 
	UARTPrint(" ");
	UARTPrintUint(rt->hour, 10);
	UARTPrint(":");
	UARTPrintUint(rt->min, 10);
	UARTPrint(":");
	UARTPrintUint(rt->sec, 10);
	UARTPrintln("");
}
//----------------------------------------------------------------------
//  calc and update day of week
//----------------------------------------------------------------------
void pcf8563_refresh_dow(rtc_time* rt) {
	register uint8_t  d = rt->mday;
	register uint8_t  m = rt->mon;
	register uint16_t y = ((uint16_t)rt->year8_t) + 2000;
	
	register uint8_t dow = (d+=m<3?y--:y-2,23*m/9+d+4+y/4-y/100+y/400)%7;
	
	if (dow) {																//decode from sun, mon, tue... to mon, tue, wed...
		--dow;
	} else {
		dow = 6;
	}

	rt->dow= dow;
}
