/*
 * SmartIrrigationCoreBoard20_SP.c
 *
 * Created: 13.07.2017 23:15:31
 * Author : Sergey Shelepin
 
 
 todo: 1. sleep / wake up with watchdog timer 
	   2. check adc code 
	   3. save / load to eeprom
 */ 

//#define VSM_DEBUG
// change #define RE_BUT_PIN       PD2 to PD7 in rotenc.h


#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdlib.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>


#include "SSD1306.h"
#include "UARTSI.h"
#include "rotenc.h"
#include "pcf8563.h"
#include "i2c_master.h"
#include "LM75A.h"
#include "ADC_SI.h"

#define TRUE  1
#define FALSE 0


//#define DEBUG_P

#ifdef DEBUG_P
	#define dp(x)		          UARTPrint(x)
	#define dpln(x)				  UARTPrintln(x)	
	#define dp_ui(x, raddix)	  UARTPrintUint(x, raddix);
	#define dp_val_dec(name, val) UARTPrintlnValDec(name, val);
#else
	#define dp(x)
	#define dpln(x)
	#define dp_ui(x, raddix)		  
	#define dp_val_dec(name, val) 
#endif

// 
// #define ADC_enable()	   ADCSRA |= _BV(ADEN)								// ADC on/off macro
// #define ADC_disable() 	   ADCSRA &= ~_BV(ADEN)

#define watchdog_reset() __asm__ __volatile__ ("wdr")

#define HVV_PIN			PD5   // high valves voltage pin
#define VALVE1_PIN		PB7
#define VALVE2_PIN		PB6

#define HVV_PORT		PORTD
#define HVV_DDR			DDRD
#define VALVES_PORT		PORTB
#define VALVES_DDR		DDRB

#define hvv_on()		HVV_PORT |= _BV(HVV_PIN)
#define hvv_off()		HVV_PORT &= ~_BV(HVV_PIN)
#define valve1_on()		VALVES_PORT |= _BV(VALVE1_PIN)
#define valve1_off()	VALVES_PORT &= ~_BV(VALVE1_PIN)
#define valve2_on()		VALVES_PORT |= _BV(VALVE2_PIN)
#define valve2_off()	VALVES_PORT &= ~_BV(VALVE2_PIN)

#define ADC_CH_SOL_VLT		2
#define ADC_CH_LIPOL_VLT	1
#define ADC_CH_BAT_VLT		0
#define ADC_CH_LIPOL_CUR	3
#define ADC_FRC10_SOL_VLT	1004		// 1023 *10 / <full range = 10.1933>
#define ADC_FRC10_LIPOL_VLT 2307		// 1023 *10 / <full range = 5.06>
#define ADC_FRC10_BAT_VLT	1487//1500		// 1023 *10 / <full range = 8.32>
#define ADC_FRC10_LIPOL_CUR 10			// 1023 *10 / <full range = 1000>   @todo: refactor it 10,23 not 10 but should be parameter to bypass scale 

#define VALVE1_NUM		 0				// not completely definable. this sequence is in wse.vts as well.
#define VALVE2_NUM		 1


#define DS_SET_TIME      0
#define DS_SET_SCHEDULE  1
#define DS_VIEW_SCHEDULE 2
#define DS_MANUAL        3
#define DS_AUTORUN		 4
#define DS_INFO			 5
#define DS_AUTORUN_MAIN  6
#define DS_AUTORUN_SCH   7
#define DS_AUTORUN_INFO  8
#define DS_AUTORUN_START 9
#define DS_AUTORUN_EXIT  10

#define DSS_MAIN_MENU	 0
#define DSS_FIRST_LEVEL  1
#define DSS_SECOND_LEVEL 2
#define DSS_AUTORUN_MENU 3

#define SF_NONE			 0  //------------------------
#define SF_ST_HH		 1
#define SF_ST_MI		 2
#define SF_ST_SS		 3
#define SF_ST_DD		 4 // Set time sequence. Do not touch.
#define SF_ST_MM		 5
#define SF_ST_YY		 6
#define SF_ST_OK		 7
#define SF_ST_CANCEL	 8 //------------------------

#define SF_SS_11_HH 	 9 //------------------------
#define SF_SS_11_MI		10
#define SF_SS_11_DUR	11
#define SF_SS_11_TEMP	12
#define SF_SS_12_HH 	13 
#define SF_SS_12_MI		14
#define SF_SS_12_DUR	15
#define SF_SS_12_TEMP	16
#define SF_SS_21_HH 	17 // Set schedule fields
#define SF_SS_21_MI		18
#define SF_SS_21_DUR	19
#define SF_SS_21_TEMP	20
#define SF_SS_22_HH 	21 
#define SF_SS_22_MI		22
#define SF_SS_22_DUR	23
#define SF_SS_22_TEMP	24 
#define SF_SS_OK	    25 
#define SF_SS_CANCEL	26 //------------------------

#define SF_MV_VLV_1	    27 //------------------------
#define SF_MV_VLV_2	    28 // MANUAL VALVE
#define SF_MV_EXIT	    29 //------------------------

#define SF_AR_START	    30 //------------------------
#define SF_AR_CANCEL    31 // AUTORUN START / CANCEL
#define SF_AR_STOP	    32 //------------------------
#define SF_AR_BACK      33 // AUTORUN STOP / BACK

uint8_t device_state;
uint8_t device_sub_state;
uint8_t selected_field;
uint8_t field_under_change;
uint8_t err_msg_flag;

enum autorun_state_type {AR_NONE, AR_IDLE, AR_WATERING, AR_WATERING_BLOCKED};
enum autorun_state_type autorun_state = AR_NONE;

enum valves_state_type {VLV_CLOSED, VLV_CHARGING, VLV_OPENING, VLV_OPENED};
enum valves_state_type valves_state = VLV_CLOSED;

#define VSR_SIGNAL_MASK			0x03
#define VSR_NO_SIGNAL			0
#define VSR_SIGNAL_OPEN			1
#define VSR_SIGNAL_CLOSE		2
#define VSR_SIGNAL_CLOSE_ALL	3

#define VSR_VALVES_RSH			2

uint8_t vs_reg;				   // Valve signal register

uint8_t lcd_update_flag;

#define WSE_TS_MASK  0x07
#define WSE_VLV_MASK 0xE0
#define WSE_VLV_RSH  5						// right shift 

volatile uint8_t main_register;
#define MR_HS_FLAG				1
#define MR_HS_LCD_RT_UPD_FLAG	2
#define MR_S_RTC_SYNCH_FLAG		3
#define MR_S_ADC_READ_FLAG		4

typedef struct {
	uint8_t vts;			                   // valve (bits 5 - 7) and time slot (bits 0 - 4)
	uint8_t hour;
	uint8_t min;
	uint8_t duration;
	uint8_t min_temp;
} water_schedule_element;

#define WSEA_SIZE 4

water_schedule_element wsea[WSEA_SIZE] = {
	{ 0, 7,  15, 45, 14	},
	{ 1, 21, 05, 15, 20 },
	{32, 14, 00, 45, 12	},
	{33, 16, 05, 15, 23 }
};
uint8_t twsea_seq[WSEA_SIZE] = {0, 1, 2, 3};										// true wsea sequence

water_schedule_element wsea_s[WSEA_SIZE];											// setup buffer 

rtc_time rt = {	30, 14, 7, 18, 2, 7, 17 };
rtc_time rt_s;

uint8_t day_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31,  30, 31, 30, 31 };        // no feb correction. not valid in 2020.

// valve num, idle or watering
struct watering_info{
	uint8_t valve_num;																// valve num to display, i.e. starting from 1
	uint8_t high_time;
	uint8_t low_time;	
	uint8_t min_temp;
};

uint8_t is_manual_wakeup = FALSE;
uint8_t is_WDT_on = FALSE;

uint8_t is_display_on;

uint8_t EEMEM ws_eeprom_config_addr[sizeof(wsea_s)];


#define WI_MODE_IDLE	  1
#define WI_MODE_WATERING  2
#define WI_MODE_BLOCKED   3

struct watering_info w_info;

#define SLEEP_CNT_MAX 20*2															// 20 sec, as we have HS timer
volatile uint8_t sleep_counter;
volatile uint8_t fast_counter;

#define FC_CHARGING_VALUE   43														//must be greater than 32

#ifdef DEBUG_P 
	#define PRR_CONFIG  0  
#else
	#define PRR_CONFIG  0 // (_BV(PRUSART0))
#endif

#define signal_led_ddr_conf()  DDRD |= _BV(PD4)
#define signal_led_on()        PORTD |= _BV(PD4)
#define signal_led_off()       PORTD &= ~_BV(PD4)
#define signal_led_blink()     PORTD ^= _BV(PD4)

uint16_t rt_cur_daymin;

inline void run_timer1_at_500msec();
inline void run_timer0_at_appx_30msec();
void states_processing();
void lcd_processing();
void valves_processing();
void water_cycle_processing();
uint8_t is_irrig_time(uint8_t cur_wse);
uint8_t get_cur_wse_num();
void calc_twses_seq(water_schedule_element* wa);
void close_all_valves_msg();
void sleep_and_wakeup();
void upd_time();
uint16_t get_stop_daymin(uint8_t wse_num);
uint16_t get_start_daymin(uint8_t wse_num);
uint8_t get_cur_wse_seq();
void WDT_int_mode(void);
void WDT_off(void);

int main(void) {
	
	HVV_DDR |= _BV(HVV_PIN);
	VALVES_DDR |= _BV(VALVE1_PIN) | _BV(VALVE2_PIN);
	
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);							// set sleep mode POWER DOWN
	PRR = PRR_CONFIG;
	
	PCMSK2 = _BV(PCINT23);											// set masks for PCINT 0, 1, 23 - RE rotation and button
	PCMSK0 = _BV(PCINT0) | _BV(PCINT1);
	
	signal_led_ddr_conf();
	
	sleep_counter = SLEEP_CNT_MAX;
	
//	re_init_at_mcu8Mhz();
	re_init_at_mcu1Mhz();
	run_timer1_at_500msec();
	run_timer0_at_appx_30msec();
//	ADC_single_conversion_mode_init_8Mhz();
	ADC_single_conversion_mode_init_1Mhz();

#ifdef DEBUG_P
	UARTInitRXTX();												// 9600 @ 1Mhz
	//UARTInitRXTX_conf(103, 1);								// 9600 @ 8Mhz
#endif	

	sei();	

	dpln("Starting...");
	
	ADC_enable();
		
	#ifdef VSM_DEBUG
	ssd1306_init(0x3D, TRUE);  // initialize with the I2C addr 0x3D
	#else
	ssd1306_init(0x3C, FALSE);  // initialize with the I2C addr 0x3C
	#endif

	is_display_on = TRUE;
	
	ssd1306_put_str_narrow(48, 0, "IRIGO", 3);
	ssd1306_put_str_P(70, 25, PSTR("water"), 2);
	ssd1306_put_str_P(104, 44, PSTR("tech"), 1);
	
	ssd1306_render();
	signal_led_on();
	_delay_ms(2000);
	signal_led_off();

	calc_twses_seq(wsea);
	
	for (uint8_t i = 0; i < WSEA_SIZE; i++) {
		dp_ui(twsea_seq[i], 10);
		dp(" ");
	}
	dpln("");
	dpln("Privet!");

	LM75A_measure();
	upd_time();
	
	eeprom_read_block(wsea_s, ws_eeprom_config_addr, sizeof(wsea_s));
	if(wsea_s->duration != 0xFF) {
		memcpy(wsea, wsea_s, sizeof(wsea_s));							// copy to setup buffer
 	} else {
     	pcf8563_set_time(&rt);
 		dpln("no config in EEPROM");
 	}
	
	// Write your code here
	lcd_update_flag = TRUE;
	autorun_state = AR_NONE;
	
	dp_val_dec("state =", device_state);
	dp_val_dec(" sub state =", device_sub_state);
	dpln("");

	
	while (1) {
		
 		states_processing();
 		lcd_processing();

		if(autorun_state != AR_NONE) {
			water_cycle_processing();
		}
		
		valves_processing();
		if(main_register & _BV(MR_S_RTC_SYNCH_FLAG)) {
			upd_time();
			dp("o");
			dp_ui(sleep_counter, 10);
			main_register &= ~_BV(MR_S_RTC_SYNCH_FLAG);
		}
		
		if(rt.sec == 0) {								   // get temperature every minute
			LM75A_measure();							   // @todo: add measure flag because it asks LM75 many time during whole sec
		}
		
// 		if(!sleep_counter) { 
// 			sleep_and_wakeup();	
// 		}

		if(!sleep_counter){
			if(is_display_on) {
				ssd1306_command(SSD1306_DISPLAYOFF);										  //--turn on oled panel
				is_display_on = FALSE;			
			}
		} else {
			if (!is_display_on)	{
				ssd1306_command(SSD1306_DISPLAYON);											 // turn on oled panel
				is_display_on = TRUE;						
				lcd_update_flag = TRUE;
			}
		}
			
	}
}

void upd_time() {
	pcf8563_get_time(&rt);
	rt_cur_daymin = rt.hour*60 + rt.min;
}



void sleep_and_wakeup() {

		if (!is_WDT_on) {
			WDT_int_mode();
			is_WDT_on = TRUE;
		}

		ssd1306_clear();
		ssd1306_render();
		_delay_ms(200);
		ssd1306_command(SSD1306_DISPLAYOFF);										  //--turn on oled panel

		ADC_disable();															 	 // disable ADC
	#ifdef VSM_DEBUG
		PCICR = _BV(PCIE2) | _BV(PCIE0);											 // enable Pin Change interrupts for RE rotation and button
	#else
		PCICR = _BV(PCIE0);															 // enable Pin Change interrupts for RE rotation 
		EIMSK = _BV(INT0);
	#endif		
		PRR |= _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRTIM2) | _BV(PRADC) | _BV(PRUSART0); // shutdown timers 0,1,2, ADC, UART twi?

		is_manual_wakeup = FALSE;
				
		sleep_enable();																 // set sleep enable
		MCUCR |= (1<<BODS) | (1<<BODSE);											 // turn bod off
		MCUCR &= ~(1<<BODSE);														 // must be done right before sleep
		sleep_cpu();																 // sleeping....zzz

		// after waking-up
		sleep_disable();															 // set sleep disable
		PRR = PRR_CONFIG;												    		 // restore PRR register
		upd_time();
		pcf8563_print_time(&rt);
		dp("hi");

		dp_val_dec(" IMWU=", is_manual_wakeup);

		if (is_manual_wakeup) {
			dp(" !!!! ");		
			WDT_off();
			is_WDT_on = FALSE;	
			ADC_enable();																 // enable ADC
			ssd1306_command(SSD1306_DISPLAYON);											 // turn on oled panel
			lcd_update_flag = TRUE;
			sleep_counter = SLEEP_CNT_MAX;
			_delay_ms(200);
			re_rotation_signal = RE_NO_SIGNAL;
			re_button_signal = RE_NO_SIGNAL;
		}

}

ISR(INT0_vect) {
	EIMSK = 0;																		 // disable int0
	is_manual_wakeup = TRUE;
}

ISR(PCINT0_vect) {
	PCICR = 0;																	 // disable Pin Change interrupts
	is_manual_wakeup = TRUE;
}


// -----------------------------------------
//			WDT FUNCTIONS
// -----------------------------------------
void WDT_int_mode(void) {
	cli();
	watchdog_reset();
	MCUSR = 0;
	WDTCSR = _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDIE) |_BV(WDP3) | _BV(WDP0);  // interrupt only mode, 8 sec
	
	sei();
}



void WDT_off(void) {
	cli();
	watchdog_reset();
	// ensure WDRF in MCUSR is 0
	MCUSR &= ~(1<<WDRF);
	// Write logical one to WDCE. Start timed sequence
	WDTCSR |= _BV(WDCE) | _BV(WDE);
	// Stop WDT
	WDTCSR = 0x00;
	sei();
}

ISR(WDT_vect) {
	signal_led_on();
	_delay_ms(5);
	signal_led_off();
}


//----------------------------------------------------------------
//
//----------------------------------------------------------------
uint8_t is_valve_on(uint8_t valve_num) {
// 	if (!(HVV_PORT & _BV(HVV_PIN))) {								// if valve high voltage is off return false
// 		return FALSE;
// 	}
	
	register uint8_t valve_pin = VALVE1_PIN;
	
	if(valve_num == VALVE2_NUM) {									
		valve_pin = VALVE2_PIN;
	}
	
	if (VALVES_PORT & _BV(valve_pin)) {
		return TRUE;
	}
	return FALSE;
}

//----------------------------------------------------------------
void fill_watering_info(uint8_t wse_num) {

	w_info.valve_num = ((wsea[wse_num].vts & WSE_VLV_MASK) >> WSE_VLV_RSH) + 1;      // get valve num

	switch(autorun_state) {
		case AR_WATERING:
		case AR_WATERING_BLOCKED:

			w_info.high_time = (get_stop_daymin(wse_num) - rt_cur_daymin - 1);			// min left
			if(!rt.sec) {																// increment min left if sec is zero
				++w_info.high_time;
			}
			w_info.low_time = rt.sec? 60-rt.sec:0;										// sec left
		break;
		
		case AR_IDLE:
			w_info.high_time = wsea[wse_num].hour;
			w_info.low_time = wsea[wse_num].min;
			w_info.min_temp = wsea[wse_num].min_temp;		
		break;
		
		default:
		break;
		
	}

}
//----------------------------------------------------------------

//----------------------------------------------------------------
// looking to cur_wse 
//----------------------------------------------------------------
void water_cycle_processing() {
	
	if(main_register & _BV(MR_HS_LCD_RT_UPD_FLAG) ) {							// check if we have to update on lcd
		main_register &= ~_BV(MR_HS_LCD_RT_UPD_FLAG);
		lcd_update_flag = TRUE;
		dp_ui(sleep_counter, 10);
		dp("  ");
		dp_ui(fast_counter, 10);
		dpln("");

	}
	
	uint8_t cur_seq = get_cur_wse_seq();	
	uint8_t cur_wse_num = twsea_seq[cur_seq];									// get cur wse
	uint8_t valve_num = (wsea[cur_wse_num].vts & WSE_VLV_MASK) >> WSE_VLV_RSH;  // get valve num
	
	if(is_irrig_time(cur_wse_num)) {											// if now is irrigation time
		
		if (LM75A_get_int_part() >= wsea[cur_wse_num].min_temp					// if cur temp >= min temp 
			&& autorun_state != AR_WATERING_BLOCKED) {							// and currently irrigation is not blocked
			
			autorun_state = AR_WATERING;										// move to watering
			
			if(!is_valve_on(valve_num)) {										// if valve is off
				vs_reg = valve_num << VSR_VALVES_RSH;							// write signal to open
				vs_reg |= VSR_SIGNAL_OPEN;
			}
			
		} else {
			autorun_state = AR_WATERING_BLOCKED;								// else watering is blocked due to low temperature
		}

	} else {																	// if now is not an irrigation time
		
		autorun_state = AR_IDLE;												// put autorun state to idle

		if(cur_seq < WSEA_SIZE ) {
			cur_wse_num = twsea_seq[++cur_seq];									// reuse of cur_wse variable
		} else {
			cur_wse_num = twsea_seq[0];
		}

		fill_watering_info(cur_wse_num);
		
		if(is_valve_on(valve_num)) {											// if valve is on
			vs_reg  = VSR_SIGNAL_CLOSE_ALL;										// close all valves
		} 
	}
	
	fill_watering_info(cur_wse_num);											// fill info to display
	
//	lcd_update_flag = TRUE;
}
//----------------------------------------------------------------
//
//----------------------------------------------------------------
uint16_t get_stop_daymin(uint8_t wse_num) {
	return 	wsea[wse_num].hour*60 + wsea[wse_num].min + wsea[wse_num].duration;
}
//----------------------------------------------------------------
//
//----------------------------------------------------------------
uint16_t get_start_daymin(uint8_t wse_num) {
	return 	wsea[wse_num].hour*60 + wsea[wse_num].min ;
}
//----------------------------------------------------------------
//
//----------------------------------------------------------------
void close_all_valves_msg() {
	vs_reg = VSR_SIGNAL_CLOSE_ALL;
}
//----------------------------------------------------------------
//	
//----------------------------------------------------------------
uint8_t is_irrig_time(uint8_t cur_wse) {

	if (rt_cur_daymin < get_stop_daymin(cur_wse) && rt_cur_daymin >= get_start_daymin(cur_wse))  {
		return TRUE;
	}
	return FALSE;
}
//----------------------------------------------------------------
//
//----------------------------------------------------------------
uint8_t get_cur_wse_seq() {

	for(uint8_t i = 0; i < WSEA_SIZE; i++)	{
		uint8_t ti = twsea_seq[WSEA_SIZE -1 - i];
		if (rt_cur_daymin >= get_start_daymin(ti)) {		
			return WSEA_SIZE -1 - i;
		}
	}
	
	return WSEA_SIZE - 1;
}

//----------------------------------------------------------------
//
//----------------------------------------------------------------
uint8_t get_cur_wse_num() {
	
	uint8_t cur_seq = get_cur_wse_seq();
	return twsea_seq[cur_seq];

}

//----------------------------------------------------------------
//
//----------------------------------------------------------------
void valves_processing() {
	
	switch(vs_reg & VSR_SIGNAL_MASK) { 
		case VSR_NO_SIGNAL:
			return;
		break;
		
		case VSR_SIGNAL_CLOSE_ALL:
			valve1_off();
			valve2_off();
			hvv_off();
			valves_state = VLV_CLOSED;
			vs_reg = 0;
			
			dpln("close all vlv");
		break;
		
		case VSR_SIGNAL_OPEN:
		
			switch(valves_state) {
				case VLV_CLOSED:
					fast_counter = FC_CHARGING_VALUE;
					hvv_on();
					valves_state = VLV_CHARGING;
					if (!sleep_counter) {
						sleep_counter = 8;										// 4 sec should be ok to finish open process
					}
					dpln("charging");
				break;
				
				case VLV_CHARGING:
					lcd_update_flag = TRUE;		
					if (!fast_counter) {
						if ( (vs_reg >> VSR_VALVES_RSH) == VALVE1_NUM) {
							valve1_on();
							dpln("vlv1 opening");
						} else {
							valve2_on();
							dpln("vlv2 open");
						}						
						fast_counter = 30;
						valves_state = VLV_OPENING;
					}
				break;
				
				case VLV_OPENING:
					if (!fast_counter) {
						hvv_off();
						vs_reg = 0;
						dpln("vlv1 opened");
						valves_state = VLV_OPENED;
					}
				break;
				
				case VLV_OPENED:
				break;
				
			}

		break;
/*
		case VSR_SIGNAL_CLOSE:
			if (( vs_reg >> VSR_VALVES_RSH) == VALVE1_NUM) {
				valve1_off();
			} else {
				valve2_off();
			}		
			hvv_off();			
			vs_reg = 0;		
		break;
*/		
	}
}

void cycle_var_if_rotation_signal(uint8_t* var, uint8_t min_val, uint8_t max_val) {

	if (re_rotation_signal)	{
		if (re_rotation_signal == RE_CLOCKWISE_STEP) {
			if(*var < max_val) {
				++(*var);
			} else {
				*var = min_val;
			}
		} else {
			if(*var > min_val) {
			 --(*var); 
			} else {
				*var = max_val;
			}
		}
		sleep_counter = SLEEP_CNT_MAX;
		re_rotation_signal = RE_NO_SIGNAL;
		lcd_update_flag = TRUE;
	}
	
}

uint8_t get_sf_max_val(uint8_t sf) {
	switch(sf) {
		case SF_SS_11_HH: case SF_SS_12_HH: case SF_SS_21_HH: case SF_SS_22_HH:				
			return 23;
		break;
		case SF_SS_11_MI: case SF_SS_12_MI: case SF_SS_21_MI: case SF_SS_22_MI:
			return 59;
		break;
		case SF_SS_11_DUR: case SF_SS_12_DUR: case SF_SS_21_DUR: case SF_SS_22_DUR:
			return 90;
		break;
		case SF_SS_11_TEMP: case SF_SS_12_TEMP: case SF_SS_21_TEMP: case SF_SS_22_TEMP:
			return 45;
		break;
	}
	return 0;
}

uint8_t get_wsea_element_num(uint8_t sf) {
	sf -= SF_SS_11_HH;
	for (uint8_t i = 1; i <= 4; i++) {
		if (sf < WSEA_SIZE*i ) {
			return sf + i;
		}
	}
	return 0; 
}

uint8_t check_sch() {
	
	calc_twses_seq(wsea_s);
	
	uint16_t daymin_start[WSEA_SIZE];
	uint8_t dur[WSEA_SIZE];
	
	for(uint8_t i = 0; i < WSEA_SIZE; i++) {
		uint8_t ti = twsea_seq[i]; 
		daymin_start[i] = wsea_s[ti].hour*60 + wsea_s[ti].min;
		dur[i] = wsea_s[ti].duration;
	}

	uint8_t res = TRUE;
	
	for(uint8_t i = 0; i < WSEA_SIZE - 1; i++) {														// end of [i] has to be before start of [i+1] 
		if(daymin_start[i] + dur[i] > daymin_start[i+1] ) {
			res = FALSE;
		}		
	}
	
	calc_twses_seq(wsea);																				// move back twse_seq
	
	return res;
}

void calc_twses_seq(water_schedule_element* wa) {

	uint16_t daymin_start[WSEA_SIZE];
	
	for(uint8_t i = 0; i < WSEA_SIZE; i++) {
		daymin_start[i] = wa[i].hour*60 + wa[i].min;
		twsea_seq[i] = i;																				// reset sequence in twsea_seq 
	}

	uint8_t tmp8;
	uint16_t tmp16;
		
	for (uint8_t j = 0; j < WSEA_SIZE - 1; j++) {															// bubble sort
		for(uint8_t i = 0; i < WSEA_SIZE - 1 - j; i++) {													
			if(daymin_start[i] > daymin_start[i + 1] ) {
				tmp16 = daymin_start[i+1];
				daymin_start[i+1] = daymin_start[i];
				daymin_start[i] = tmp16;

				tmp8 = twsea_seq[i+1];
				twsea_seq[i+1] = twsea_seq[i];
				twsea_seq[i] = tmp8;
			}
		}
	}
}

//----------------------------------------------------------------------------------------------------------------------------
// checks rotary encoder button signal and updates environment variables
//----------------------------------------------------------------------------------------------------------------------------
uint8_t check_re_bs_upd() {
	if (re_button_signal) {
		re_button_signal = RE_NO_SIGNAL;
		lcd_update_flag = TRUE;
		sleep_counter = SLEEP_CNT_MAX;
		dpln("but pressed");
		return TRUE;
	}
	return FALSE;
}
//----------------------------------------------------------------------------------------------------------------------------
//---------------------------------    STATES PROCESSING								 -------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
void states_processing() {	
	
	switch (device_sub_state) {

		case DSS_AUTORUN_MENU:			
			switch (device_state) {
				case DS_AUTORUN_MAIN:
				case DS_AUTORUN_SCH:
				case DS_AUTORUN_INFO:
					cycle_var_if_rotation_signal(&device_state, DS_AUTORUN_MAIN, DS_AUTORUN_INFO);
					if(check_re_bs_upd()) {
						device_state = DS_AUTORUN_EXIT;
						selected_field = SF_AR_STOP;
					}
				break;
				
				case DS_AUTORUN_START:
					cycle_var_if_rotation_signal(&selected_field, SF_AR_START, SF_AR_CANCEL);
					if(check_re_bs_upd()) {
						if(selected_field == SF_AR_START) {
							autorun_state = AR_IDLE;
							device_state = DS_AUTORUN_MAIN;
							LM75A_measure();
						} else {
							device_sub_state = DSS_MAIN_MENU;
							device_state = DS_AUTORUN;
						}
					}
				break;
				
				case DS_AUTORUN_EXIT:
					cycle_var_if_rotation_signal(&selected_field, SF_AR_STOP, SF_AR_BACK);
					if(check_re_bs_upd()) {
						if(selected_field == SF_AR_STOP) {
							autorun_state = AR_NONE;
							close_all_valves_msg();
							device_sub_state = DSS_MAIN_MENU;
							device_state = DS_AUTORUN;
						} else {
							device_state = DS_AUTORUN_MAIN;
						}
					}
				break;
			}
		break;

		case DSS_MAIN_MENU:
			cycle_var_if_rotation_signal(&device_state, 0, DS_INFO); 
		
			if(check_re_bs_upd()) {														   // main menu button click processing
//				alarms_processing();
				get_cur_wse_num();
				device_sub_state = DSS_FIRST_LEVEL;
				switch(device_state) {													   // set selected field ( pre setup for DSS_FIRST_LEVEL handler)
					case DS_SET_TIME:
						pcf8563_get_time(&rt_s);
						selected_field = SF_ST_HH;
					break;
					case DS_SET_SCHEDULE:
						memcpy(wsea_s, wsea, sizeof(wsea_s));							   // copy to setup buffer
						selected_field = SF_SS_11_HH;
					break;
					
					case DS_MANUAL:
						selected_field = SF_MV_VLV_1;
					break;
					
					case DS_AUTORUN:
						selected_field = SF_AR_START;
						device_state = DS_AUTORUN_START;
						device_sub_state = DSS_AUTORUN_MENU;
					break;
					
					case DS_INFO:
					case DS_VIEW_SCHEDULE:
					default:
						selected_field = SF_NONE;
					break;
				}
			}
		break;

		case DSS_FIRST_LEVEL:
			switch(device_state) {
				
				case DS_SET_TIME:
					if(!field_under_change) {													// cycling time fields
						cycle_var_if_rotation_signal(&selected_field, SF_ST_HH, SF_ST_CANCEL);	
					} else {																	// cycling time values
						switch(selected_field) {
							case SF_ST_HH:
								cycle_var_if_rotation_signal(&rt_s.hour, 0, 23);	
							break;
							
							case SF_ST_MI:
								cycle_var_if_rotation_signal(&rt_s.min, 0, 59);
							break;

							case SF_ST_SS:
								cycle_var_if_rotation_signal(&rt_s.sec, 0, 59);
							break;							

							case SF_ST_DD:
								cycle_var_if_rotation_signal(&rt_s.mday, 1, day_in_month[rt_s.mon]);
							break;
							
							case SF_ST_MM:
								cycle_var_if_rotation_signal(&rt_s.mon, 1, 12);
								if (rt_s.mday > day_in_month[rt_s.mon]) {
									--rt_s.mday;
								}
							break;
							
							case SF_ST_YY:
								cycle_var_if_rotation_signal(&rt_s.year8_t, 0, 99);
							break;
						}
					}
					
					if(check_re_bs_upd()) {
						if(selected_field < SF_ST_OK) {								// if field not OK and not CANCEL
							field_under_change ^= 1;								// inverting fuc flag
							} else {
							if(selected_field == SF_ST_OK) {
								pcf8563_set_time(&rt_s);
							}
							selected_field = SF_NONE;
							device_sub_state = DSS_MAIN_MENU;
						}
					}
				break;
				
				case DS_SET_SCHEDULE:
					if(!field_under_change) {
						cycle_var_if_rotation_signal(&selected_field, SF_SS_11_HH, SF_SS_CANCEL);
					} else {
						uint8_t* wp = ((uint8_t*)wsea_s + get_wsea_element_num(selected_field));
						cycle_var_if_rotation_signal(wp, 0, get_sf_max_val(selected_field));						
					}

					if(check_re_bs_upd()) {
						if(err_msg_flag) {
							err_msg_flag = FALSE;
						} else {
							if(selected_field < SF_SS_OK) {								// if field not OK and not CANCEL
								field_under_change ^= 1;								// inverting fuc flag
							} else {
								if(selected_field == SF_SS_OK) {
									err_msg_flag = FALSE;
									if( check_sch() ) {
										memcpy(wsea, wsea_s, sizeof(wsea));
										eeprom_write_block(wsea_s, ws_eeprom_config_addr, sizeof(wsea_s)); // store configuration in eeprom										
										calc_twses_seq(wsea);
										selected_field = SF_NONE;
										device_sub_state = DSS_MAIN_MENU;
									} else {
										dpln("CHSCH FAIL");										
										err_msg_flag = TRUE;
									}

								} else {												// SF_SS_CANCEL
									selected_field = SF_NONE;
									device_sub_state = DSS_MAIN_MENU;
								}
							}
						}
					}
				break;
				
				case DS_MANUAL:
					if(!field_under_change) {
						cycle_var_if_rotation_signal(&selected_field, SF_MV_VLV_1, SF_MV_EXIT);
					} 
					if(check_re_bs_upd()) {
						switch(selected_field){
							case SF_MV_VLV_1:
								if (!field_under_change) {
									field_under_change = TRUE;
									vs_reg |= (VALVE1_NUM << VSR_VALVES_RSH) | VSR_SIGNAL_OPEN;
								} else {
									field_under_change = FALSE;
								    vs_reg |= VSR_SIGNAL_CLOSE_ALL;									// to be on the safe side
//									vs_reg |= (VALVE1_NUM << VSR_VALVES_RSH) | VSR_SIGNAL_CLOSE;
								}
							break;
							case SF_MV_VLV_2:
								if (!field_under_change) {
									field_under_change = TRUE;
									vs_reg |= (VALVE2_NUM << VSR_VALVES_RSH) | VSR_SIGNAL_OPEN;
								} else {
									field_under_change = FALSE;
								    vs_reg |= VSR_SIGNAL_CLOSE_ALL;									// to be on the safe side
//									vs_reg |= (VALVE2_NUM << VSR_VALVES_RSH) | VSR_SIGNAL_CLOSE;
								}							
							break;
							case SF_MV_EXIT:
								field_under_change = FALSE;
								selected_field = SF_NONE;
								device_sub_state = DSS_MAIN_MENU;
								vs_reg |= VSR_SIGNAL_CLOSE_ALL;									// to be on the safe side
							break;
							
						}
					}
				break;
				
				case DS_INFO:
					if(main_register & _BV(MR_S_ADC_READ_FLAG)) {
						lcd_update_flag = TRUE;
						main_register &= ~_BV(MR_S_ADC_READ_FLAG);
					}
					if (check_re_bs_upd()) {
						device_sub_state = DSS_MAIN_MENU;
					}					
				break;
				case DS_VIEW_SCHEDULE:
					if (check_re_bs_upd()) {
						device_sub_state = DSS_MAIN_MENU;
					}
				break;
								
				default:
				break;
			}

		break;
		
	}
	
	
}

//----------------------------------------------------------------------------------------------------------------------------
// render time setup
//----------------------------------------------------------------------------------------------------------------------------
void render_time_setup() {
	
	uint8_t x_pos = 15;
	uint8_t y_pos = 9;
	
	ssd1306_print_uint(x_pos, y_pos, rt_s.hour, 2, 2);
	ssd1306_put_char(x_pos + 2*12, y_pos, ':', 2);	

	ssd1306_print_uint(x_pos + 3*12 , y_pos, rt_s.min, 2, 2);
	ssd1306_put_char(x_pos + 5*12, y_pos, ':', 2);	
	
	ssd1306_print_uint(x_pos + 6*12 , y_pos, rt_s.sec, 2, 2);

	if (selected_field < 4) {
		ssd1306_fill_rect(x_pos - 1 + (selected_field -1)*(2*17+2), y_pos - 1, 25, 18, INVERSE);
	}
	
	y_pos += 20;

	ssd1306_print_uint(x_pos, y_pos, rt_s.mday, 2, 2);
	ssd1306_put_char(x_pos + 2*12, y_pos, '/', 2);
	
	ssd1306_print_uint(x_pos + 3*12 , y_pos, rt_s.mon, 2, 2);
	ssd1306_put_char(x_pos + 5*12, y_pos, '/', 2);

	ssd1306_print_uint(x_pos + 6*12 , y_pos, rt_s.year8_t, 2, 2);
	
	if (selected_field >=4 && selected_field < 7 ) {
		ssd1306_fill_rect(x_pos -1 + (selected_field-4)*(2*17+2), y_pos - 1, 25, 18, INVERSE);
	}
	
	ssd1306_put_str_P(30, 55, PSTR("ok"), 1);
	ssd1306_put_str_P(80, 55, PSTR("cancel"), 1);

	if (selected_field >= 7 ) {
		ssd1306_fill_rect(0 + (selected_field - 7)*64, 53, 64, 11, INVERSE);
	}
	
}


void render_wse(water_schedule_element* w, uint8_t y_pos) {	
	const uint8_t ch_size = 6;
	ssd1306_print_uint(        0, y_pos, ((w->vts & WSE_VLV_MASK) >> WSE_VLV_RSH) + 1, 1, 1);
	ssd1306_print_uint(ch_size*2, y_pos, (w->vts & WSE_TS_MASK) + 1, 1, 1);
	ssd1306_print_uint(ch_size*5, y_pos, w->hour, 1, 2);
	ssd1306_put_char(ch_size*7, y_pos, '-', 1);
	ssd1306_print_uint(ch_size*8, y_pos, w->min, 1, 2);
	ssd1306_print_uint(ch_size*12, y_pos, w->duration, 1, 2);
	ssd1306_put_char(ch_size*14, y_pos, 'm', 1);
	ssd1306_print_uint(ch_size*17, y_pos, w->min_temp, 1, 2);
	ssd1306_put_char(ch_size*20, y_pos, 'C', 1);
}


void render_sch(uint8_t is_setup_mode) {
	
	if (err_msg_flag) {
		ssd1306_put_str_P(35, 5, PSTR("SCH"), 2);
		ssd1306_put_str_P(25, 25, PSTR("SETUP"), 2);
		ssd1306_put_str_P(25, 45, PSTR("ERROR"), 2);		
		return;
	}

	water_schedule_element* wp;	
	
	is_setup_mode ? (wp = &wsea_s[0]) : (wp = &wsea[0]);
	
	ssd1306_put_str_P(0, 0,  PSTR("V TS START  DUR TEMP>"), 1);
	ssd1306_draw_fast_hline(0, 9, 127, WHITE);	

	render_wse(wp++, 13);
	render_wse(wp++, 23);
	render_wse(wp++, 33);
	render_wse(wp, 43);			
	
	if(!is_setup_mode) {
		return;
	}
	
	ssd1306_put_str_P(30, 55, PSTR("ok"), 1);
	ssd1306_put_str_P(80, 55, PSTR("cancel"), 1);
	
	uint8_t x_pos = 0;
	uint8_t y_pos = 0;	

	if(selected_field < SF_SS_OK) {
		switch(selected_field) {
			case SF_SS_11_HH: case SF_SS_12_HH:	case SF_SS_21_HH: case SF_SS_22_HH:
				x_pos = 5*6-1;
			break;
			case SF_SS_11_MI: case SF_SS_12_MI:	case SF_SS_21_MI: case SF_SS_22_MI:		
				x_pos = 6*8-1;
			break;
			case SF_SS_11_DUR: case SF_SS_12_DUR: case SF_SS_21_DUR: case SF_SS_22_DUR:		
				x_pos = 6*12-1;
			break;	
			case SF_SS_11_TEMP: case SF_SS_12_TEMP: case SF_SS_21_TEMP: case SF_SS_22_TEMP:
				x_pos = 6*17-1;
			break;			
		}
	
		switch (selected_field) {
			case SF_SS_11_HH: case SF_SS_11_MI: case SF_SS_11_DUR: case SF_SS_11_TEMP:		
				y_pos = 12;
			break;
			case SF_SS_12_HH: case SF_SS_12_MI: case SF_SS_12_DUR: 	case SF_SS_12_TEMP:
				y_pos = 22;
			break;		
			case SF_SS_21_HH: case SF_SS_21_MI: case SF_SS_21_DUR: case SF_SS_21_TEMP:
				y_pos = 32;
			break;
			case SF_SS_22_HH: case SF_SS_22_MI: case SF_SS_22_DUR: case SF_SS_22_TEMP:
				y_pos = 42;
			break;
		}
		ssd1306_fill_rect(x_pos, y_pos, 13, 9, INVERSE);
	} else {
		ssd1306_fill_rect(0 + (selected_field - SF_SS_OK)*64, 53, 64, 11, INVERSE);				
	}
	
}

void render_manual_valve() {
	ssd1306_put_str_P(20, 20, PSTR("V1"), 2);
	ssd1306_put_str_P(80, 20, PSTR("V2"), 2);
	ssd1306_put_str_P(50, 55, PSTR("exit"), 1);	
	
	switch(selected_field) {
		case SF_MV_VLV_1:
			ssd1306_fill_rect(19, 19, 24, 16, INVERSE);
			if(field_under_change) {
				ssd1306_put_str_P(12, 40, PSTR("opened"), 1);
			}
		break;
		case SF_MV_VLV_2:
			ssd1306_fill_rect(79, 19, 24, 16, INVERSE);			
			if(field_under_change) {
				ssd1306_put_str_P(72, 40, PSTR("opened"), 1);
			}
		break;
		case SF_MV_EXIT:
			ssd1306_fill_rect(0, 53, 128, 11, INVERSE );
		break;
		
	}
	
}

void render_info() {
	const uint8_t x_pos = 0;
	uint8_t y_pos = 15;
	const uint8_t y_inc = 10;

	char buf[6];
	ADC_get_str(ADC_CH_SOL_VLT, ADC_FRC10_SOL_VLT, buf);
	ssd1306_put_str_P(x_pos, y_pos, PSTR("Solar panel:"), 1);
	ssd1306_put_str(x_pos + 6*15, y_pos, buf, 1);
	ssd1306_put_char(x_pos + 6*20, y_pos, 'v', 1);
	y_pos += y_inc;

	ADC_get_str(ADC_CH_BAT_VLT, ADC_FRC10_BAT_VLT, buf);
	ssd1306_put_str_P(x_pos, y_pos, PSTR("Batteries:"), 1);
	ssd1306_put_str(x_pos + 6*15, y_pos, buf, 1);
	ssd1306_put_char(x_pos + 6*20, y_pos, 'v', 1);
	y_pos += y_inc;
	
	ADC_get_str(ADC_CH_LIPOL_VLT, ADC_FRC10_LIPOL_VLT, buf);
	ssd1306_put_str_P(x_pos, y_pos, PSTR("Li-pol:"), 1);
	ssd1306_put_str(x_pos + 6*15, y_pos, buf, 1);
	ssd1306_put_char(x_pos + 6*20, y_pos, 'v', 1);
	y_pos += y_inc;
	
// 	ADC_get_str(ADC_CH_LIPOL_CUR, ADC_FRC10_LIPOL_CUR, buf);
// 	ssd1306_put_str_P(x_pos, y_pos, PSTR("Li-pol:"), 1);
// 	ssd1306_put_str(x_pos + 6*15, y_pos, buf, 1);
// 	ssd1306_put_str_P(x_pos + 6*19, y_pos, PSTR("mA"), 1);

	
}

void render_autorun_start() {
	ssd1306_put_str_P(30, 20, PSTR("START"), 2);
	ssd1306_put_str_P(40, 50, PSTR("cancel"), 1);	

	switch(selected_field) {
		case SF_AR_START:
			ssd1306_fill_rect(28, 18, 63, 18, INVERSE);		
		break;
		case SF_AR_CANCEL:
			ssd1306_fill_rect(38, 48, 40, 11, INVERSE);				
		break;		
	}	
	
	
}

void render_autorun_exit() {
	ssd1306_put_str_P(30, 20, PSTR("STOP"), 2);
	ssd1306_put_str_P(40, 50, PSTR("back"), 1);

	switch(selected_field) {
		case SF_AR_STOP:
		ssd1306_fill_rect(28, 18, 63, 18, INVERSE);
		break;
		case SF_AR_BACK:
		ssd1306_fill_rect(38, 48, 40, 11, INVERSE);
		break;
	}
	
}

void lcd_print_time_and_temp() {
	ssd1306_print_uint(0, 0, rt.hour, 1, 2);																	//printing time
	ssd1306_put_char(2*6, 0, ':', 1);
	ssd1306_print_uint(3*6, 0, rt.min, 1, 2);
	ssd1306_put_char(5*6, 0, ':', 1);
	ssd1306_print_uint(6*6, 0, rt.sec, 1, 2);
//	ssd1306_put_str_P(20, 10, PSTR(""), 2);
	ssd1306_print_uint(14*6, 0, LM75A_get_int_part(), 1, 2);
	ssd1306_put_char(16*6, 0, '.', 1);
	ssd1306_print_uint(17*6, 0, LM75A_get_hundreds(), 1, 2);
	ssd1306_put_char(19*6, 0, 'c', 1);	
}

void render_autorun_main() {
	lcd_print_time_and_temp();
	
	switch(autorun_state) {
		case AR_WATERING:
		case AR_WATERING_BLOCKED:
			if (autorun_state == AR_WATERING){
				ssd1306_put_str_P(35, 15, PSTR("WATERING"), 1);
			} else {
				ssd1306_put_str_P(20, 15, PSTR("WATERING BLOCKED"), 1);				
			}
			ssd1306_put_str_P(0, 30, PSTR("valve"), 1);
			ssd1306_put_str_P(40, 30, PSTR("min left"), 1);
			ssd1306_print_uint(10, 44, w_info.valve_num, 2, 1);
			ssd1306_print_uint(47, 44, w_info.high_time, 2, 2);					// min left
			ssd1306_print_uint(72, 44, w_info.low_time, 1, 2);					// sec left
		break;
		case AR_IDLE:
			ssd1306_put_str_P(30, 15, PSTR("NEXT SESSION"), 1);
			ssd1306_put_str_P(0, 30, PSTR("valve"), 1);
			ssd1306_put_str_P(40, 30, PSTR("start at"), 1);
			ssd1306_put_str_P(99, 30, PSTR("temp"), 1);
			ssd1306_print_uint(10, 44, w_info.valve_num, 2, 1);
			ssd1306_print_uint(47, 44, w_info.high_time, 2, 2);
			ssd1306_print_uint(72, 44, w_info.low_time, 1, 2);
			ssd1306_print_uint(99, 44, w_info.min_temp, 2, 2);
		break;
		default:
		break;
	}
}

void render_charging() {
	ssd1306_put_str_P(10, 15, PSTR("CHARGING"), 2);
	register uint8_t width = 4*(FC_CHARGING_VALUE-fast_counter);
	if(width > 127) {
		width = 127;
	}
	ssd1306_fill_rect(0, 9, width, 28, INVERSE);
}


//----------------------------------------------------------------------------------------------------------------------------
//---------------------------------    LCD PROCESSING								 ----------------------------------------
//----------------------------------------------------------------------------------------------------------------------------

void lcd_processing() {

	if(main_register & _BV(MR_HS_LCD_RT_UPD_FLAG) && (device_sub_state == DSS_MAIN_MENU)) {	// check if we have to update time on lcd
		main_register &= ~_BV(MR_HS_LCD_RT_UPD_FLAG);
		lcd_update_flag = TRUE;		
	}

	if(!lcd_update_flag) {
		return;
	}
			
	lcd_update_flag = FALSE;
	ssd1306_clear();
	
	if(valves_state == VLV_CHARGING) {
		render_charging();
	} else {
		switch(device_sub_state) {
			
			case DSS_AUTORUN_MENU:
				switch(device_state){
					case DS_AUTORUN_MAIN:
						render_autorun_main();
					break;
					case DS_AUTORUN_SCH:
						render_sch(FALSE);
					break;
					case DS_AUTORUN_INFO:
						render_info();
	//					ssd1306_put_str_P(10, 30, PSTR("AR INFO"), 2);
					break;
					case DS_AUTORUN_START:
						render_autorun_start();
					break;
					case DS_AUTORUN_EXIT:
						render_autorun_exit();
					break;
				}
			break;

			case DSS_MAIN_MENU:
				lcd_print_time_and_temp();
		
				switch(device_state) {
					#define X_MM_POS 10
					#define Y_MM_POS 30
		
					case DS_SET_TIME:
						ssd1306_put_str_P(X_MM_POS, Y_MM_POS, PSTR("SET TIME"), 2);		
					break;
					case DS_SET_SCHEDULE:
						ssd1306_put_str_P(X_MM_POS, Y_MM_POS, PSTR("SET SCH"), 2);
					break;
					case DS_VIEW_SCHEDULE:
						ssd1306_put_str_P(X_MM_POS, Y_MM_POS, PSTR("VIEW SCH"), 2);
					break;		
					case DS_MANUAL:
						ssd1306_put_str_P(X_MM_POS, Y_MM_POS, PSTR("MANUAL"), 2);
					break;
					case DS_AUTORUN:
						ssd1306_put_str_P(X_MM_POS, Y_MM_POS, PSTR("AUTORUN"), 2);
					break;
					case DS_INFO:
						ssd1306_put_str_P(X_MM_POS, Y_MM_POS, PSTR("INFO"), 2);
					break;				
				}

				for(uint8_t i = 0; i <= DS_INFO; i++) {
					uint8_t x_pos;
					uint8_t height = 10;
					uint8_t y_off_set = 0;
					if(i == device_state) {
						x_pos = 123;
						ssd1306_draw_fast_hline(x_pos, i*height + y_off_set, 5, WHITE);
						ssd1306_draw_fast_hline(x_pos, (i + 1)*height +y_off_set,5, WHITE);
					} else {
						x_pos = 127;
					}
					ssd1306_draw_fast_vline(x_pos, i*height+y_off_set, height,WHITE);
				}
			break;
	
			case DSS_FIRST_LEVEL:
				switch (device_state) {
					case DS_SET_TIME:
						render_time_setup();
					break;
					case DS_SET_SCHEDULE:
						render_sch(TRUE);
					break;
					case DS_VIEW_SCHEDULE:
						render_sch(FALSE);
					break;
					case DS_MANUAL:
						render_manual_valve();
					break;
					case DS_INFO:
						render_info();
					break;
					default:
						ssd1306_put_str_P(X_MM_POS, Y_MM_POS, PSTR("FLEVEL"), 2);			
					break;;
				}
			break;
		}
	}
	ssd1306_render();

}

inline void run_timer1_at_500msec() {
	OCR1A  =  8313;//31249;//16625; //31249;                     // hopefully compiler do this 16-bit assignment :)  
	TIMSK1 = _BV(OCIE1A);
//	TCCR1B = _BV(WGM12) | _BV(CS12);			 // run at CTC mode with prescaler = 256   for 8Mhz
	TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10); // run at CTC mode with prescaler = 64     for 1Mhz

}

inline void run_timer0_at_appx_30msec() {
	TIMSK0 = _BV(TOIE0);
//	TCCR0B = _BV(CS02) | _BV(CS00);			     // run at TOV mode with prescaler = 1024
	TCCR0B = _BV(CS02);			     // run at TOV mode with prescaler = 256   60 ms at 1Mhz :(

}

ISR(TIMER0_OVF_vect) {
	if (fast_counter) {
		--fast_counter;
	}
}


ISR(TIMER1_COMPA_vect) {
	main_register ^= _BV(MR_HS_FLAG);
	main_register |= _BV(MR_HS_LCD_RT_UPD_FLAG);
// 	main_register |= _BV(MR_HS_LCD_UPD_FLAG)
// 	| _BV(MR_HS_VALVE_CNT_DOWN_FLAG)
// 	| _BV(MR_HS_SIGNAL_LED_FLAG)
// 	| _BV(MR_HS_LCD_BACKLIGHT_FLAG)
// 	| _BV(MR_HS_MRT_FLAG);
	
	if(main_register & _BV(MR_HS_FLAG)) {                                                    // make 1 sec flags
		main_register |= (_BV(MR_S_RTC_SYNCH_FLAG) | _BV(MR_S_ADC_READ_FLAG));
	}
	
	if(sleep_counter) {
		--sleep_counter;
	}

}



