/*
 * pcf8563.h
 *
 * Created: 17.07.2017 14:47:32
 *  Author: Sergey Shelepin
 */ 


#ifndef PCF8563_H_
#define PCF8563_H_

// Register addresses
#define PCF8563_CTRL_STATUS_1_REG 0x00
#define PCF8563_CTRL_STATUS_2_REG 0x01
#define PCF8563_SECONDS_REG       0x02
#define PCF8563_MINUTES_REG       0x03
#define PCF8563_HOURS_REG         0x04
#define PCF8563_WEEK_DAY_REG      0x05
#define PCF8563_DAY_REG           0x06
#define PCF8563_MONTHS_REG        0x07
#define PCF8563_YEAR_REG          0x08

/*
#define PCF8583_ALARM_CONTROL_REG 0x08
#define PCF8583_ALARM_100S_REG    0x09
#define PCF8583_ALARM_SECS_REG    0x0A
#define PCF8583_ALARM_MINS_REG    0x0B
#define PCF8583_ALARM_HOURS_REG   0x0C
#define PCF8583_ALARM_DATE_REG    0x0D
#define PCF8583_ALARM_MONTHS_REG  0x0E
#define PCF8583_ALARM_TIMER_REG   0x0F
// Use the first NVRAM address for the year byte.
#define PCF8583_YEAR_REG          0x10
*/

// pcf8563 addresses
#define PCF8563_ADDR_WRITE 0xA2
#define PCF8563_ADDR_READ  0xA3


// !!!! DO NOT TOUCH the sequence !!!
// sequences for pcf8583 and pcf8563 are different!!!
typedef struct {
	uint8_t sec;      // 0 to 59
	uint8_t min;      // 0 to 59
	uint8_t hour;     // 0 to 23
	uint8_t mday;     // 1 to 31
	uint8_t dow;      // 1-7
	uint8_t mon;      // 1 to 12
	uint8_t year8_t;  // year - starting from 2000
} rtc_time;

//uint8_t rtc_days_in_month[12];

//void pcf8583_set_month_days(rtc_time* rt);
void pcf8563_set_time(rtc_time* rt);
void pcf8563_get_time(rtc_time* rt);
void pcf8563_print_time(rtc_time* rt);
//void pcf8583_refresh_dow(rtc_time* rt);
//uint8_t pcf8583_get_base_year();
//void pcf8583_set_base_year(uint8_t by);

// uint8_t pcf8583_read_byte(uint8_t offset);
// uint8_t pcf8583_write_byte(uint8_t offset, uint8_t value);
// uint8_t pcf8583_set_ss_mi_hh24(struct tm* tm_);
// uint8_t pcf8583_get_ss_mi_hh24(struct tm* tm_);

// uint8_t dec2bcd(uint8_t d);
// uint8_t bcd2dec(uint8_t b);

#endif /* PCF8563_H_ */