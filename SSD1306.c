/*
 * SSD1306.c
 *
 * Created: 15.07.2017 21:23:42
 *  Author: USER
 */ 

#define VSM_DEBUG

#define TRUE  1
#define FALSE 0

extern void UARTPrint( const char *str );

#include "SSD1306.h"
#include "i2c_master.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "sifonts.h"

int16_t
_width, _height, // Display w/h as modified by current rotation
cursor_x, cursor_y;

int8_t _i2caddr, _vccstate;//, sid, sclk, dc, rst, cs;

//--------------------------------------------------------------------------------------------------------------------
// ---- command
//--------------------------------------------------------------------------------------------------------------------
void ssd1306_command(uint8_t c) {
	uint8_t control = 0x00;   // Co = 0, D/C = 0
	i2c_start(_i2caddr << 1);
	i2c_write(control);
	i2c_write(c);
	i2c_stop();
	
/*	twi_beginTransmission(_i2caddr);
	twi_write(control);
	twi_write(c);
	twi_endTransmission(1);
	*/

}


//--------------------------------------------------------------------------------------------------------------------
// ---- begin
//--------------------------------------------------------------------------------------------------------------------
void  ssd1306_init(uint8_t i2caddr, uint8_t reset) {

	_vccstate = SSD1306_SWITCHCAPVCC;
	_i2caddr = i2caddr;

	//twi_init();
	i2c_init();

	//-------------------------------------------------------------
	// DO NOT START WITHOUT THIS FUNCTION IN PROTEUS !!!!!!!!  doing reset of the module 
	//-------------------------------------------------------------
	if(reset) {
		DDRD |= _BV(PD4);													// setup reset pin for Proteus debug only
		PORTD |= _BV(PD4);
		_delay_ms(1);														// VDD (3.3V) goes high at start, lets just chill for a ms
		PORTD &= ~_BV(PD4);													// bring reset low
		_delay_ms(10);														// wait 10ms
		PORTD |= _BV(PD4);													// bring out of reset
	}
	//---------------------------------------------------------

	// Init sequence for 128x64 OLED module
	ssd1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
	ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
	ssd1306_command(0x80);                                  // the suggested ratio 0x80
	ssd1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
	ssd1306_command(0x3F);
	ssd1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
	ssd1306_command(0x0);                                   // no offset
	ssd1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
	ssd1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
	
	if (_vccstate == SSD1306_EXTERNALVCC) {
		ssd1306_command(0x10); 
	} else { 
		ssd1306_command(0x14); 
	}
		
	ssd1306_command(SSD1306_MEMORYMODE);                    // 0x20
	ssd1306_command(0x00);                                  // 0x0 act like ks0108
	ssd1306_command(SSD1306_SEGREMAP | 0x1);
	ssd1306_command(SSD1306_COMSCANDEC);
	ssd1306_command(SSD1306_SETCOMPINS);                    // 0xDA
	ssd1306_command(0x12);
	ssd1306_command(SSD1306_SETCONTRAST);                   // 0x81
	
	if (_vccstate == SSD1306_EXTERNALVCC) {
		ssd1306_command(0x9F); 
	} else {
		ssd1306_command(0xCF); 
	}
	ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
	if (_vccstate == SSD1306_EXTERNALVCC) {
		 ssd1306_command(0x22); 
	} else { 
		ssd1306_command(0xF1); 
	}
	ssd1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
	ssd1306_command(0x40);
	ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
	ssd1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6

	ssd1306_command(SSD1306_DISPLAYON);//--turn on oled panel
}

//--------------------------------------------------------------------------------------------------------------------
// ---- display
//--------------------------------------------------------------------------------------------------------------------
void ssd1306_render(void) {
	ssd1306_command(SSD1306_COLUMNADDR);
	ssd1306_command(0);											// Column start address (0 = reset)
	ssd1306_command(SSD1306_LCDWIDTH-1);						// Column end address (127 = reset)
	ssd1306_command(SSD1306_PAGEADDR);
	ssd1306_command(0);											// Page start address (0 = reset)
	ssd1306_command(7);											// Page end address

	uint16_t i = 0;
		i2c_start(_i2caddr << 1);
		i2c_write(0x40);
	
		for ( i = 0; i < (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8); i++) {  		// send a bunch of data in one xmission
			uint8_t x = 0;
			for (x = 0; x < 16; x++) {
				i2c_write(ssd1306_buffer[i++]);
			}
			i--;
/*
		twi_beginTransmission(_i2caddr);
		twi_write(0x40);
		uint8_t x = 0;
		for (x = 0; x < 16; x++) {
			twi_write(ssd1306_buffer[i++]);
		}
		i--;
		twi_endTransmission(1);
*/
	}
		i2c_stop();
}

void ssd1306_clear(){ 
	memset(ssd1306_buffer, 0, 1024);
}


// the most basic function, set a single pixel
void ssd1306_draw_pixel(int16_t x, int16_t y, uint16_t color) {
	if ((x < 0) || (x >= SSD1306_LCDWIDTH) || (y < 0) || (y >= SSD1306_LCDHEIGHT ))
	return;
	// x is which column
	switch (color) {
		case WHITE:   ssd1306_buffer[x+ (y/8)*SSD1306_LCDWIDTH] |=  (1 << (y&7)); break;
		case BLACK:   ssd1306_buffer[x+ (y/8)*SSD1306_LCDWIDTH] &= ~(1 << (y&7)); break;
		case INVERSE: ssd1306_buffer[x+ (y/8)*SSD1306_LCDWIDTH] ^=  (1 << (y&7)); break;
	}
	
}


void ssd1306_put_char(uint8_t x, uint8_t y, char ch, uint8_t size) {

	for (uint8_t i = 0; i < 5; i++) {
		uint8_t line = pgm_read_byte(font5_7 + ch*5 + i);
		for (int8_t j = 0; j < 8; j++) {
			if (line & 0x1) {
				if(size == 1 ) { 
					ssd1306_draw_pixel(x + i, y + j, WHITE);
				} else {
					ssd1306_fill_rect(x+(i*size), y+(j*size), size, size, WHITE);					
				}
			}
			line >>= 1;
		}
		
	}

}


void ssd1306_put_str(uint8_t x, uint8_t y, char* str, uint8_t size) {
	while(*str) {
		ssd1306_put_char(x, y, *str, size);
		x += 6*size;
		++str;
	}
}

void ssd1306_put_str_P(uint8_t x, uint8_t y, PGM_P str, uint8_t size) {
	char ch;
	while(0 != (ch = pgm_read_byte(str++))) {
		ssd1306_put_char(x, y, ch, size);
		x += 6*size;
	}
}

void ssd1306_put_str_narrow(uint8_t x, uint8_t y, char* str, uint8_t size) {
	while(*str) {
		ssd1306_put_char(x, y, *str, size);
		x += 5*size + 1;
		++str;
	}
}



void ssd1306_draw_fast_vline(int8_t x, int8_t y, int8_t h, uint8_t color) {

	if(x >= SSD1306_LCDWIDTH || y >= SSD1306_LCDHEIGHT) {				// do nothing if we're out of the screen
		return;
	}


	// set up the pointer for fast movement through the buffer
	register uint8_t *pBuf = ssd1306_buffer;
	// adjust the buffer pointer for the current row
	pBuf += ((y/8) * SSD1306_LCDWIDTH);
	// and offset x columns in
	pBuf += x;

	// do the first partial byte, if necessary - this requires some masking
	register uint8_t mod = (y&7);
	if(mod) {
		// mask off the high n bits we want to set
		mod = 8-mod;

		// note - lookup table results in a nearly 10% performance improvement in fill* functions
		// register uint8_t mask = ~(0xFF >> (mod));
		static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
		register uint8_t mask = premask[mod];

		// adjust the mask if we're not going to reach the end of this byte
		if( h < mod) {
			mask &= (0XFF >> (mod-h));
		}

		switch (color)
		{
			case WHITE:   *pBuf |=  mask;  break;
			case BLACK:   *pBuf &= ~mask;  break;
			case INVERSE: *pBuf ^=  mask;  break;
		}
		
		// fast exit if we're done here!
		if(h<mod) { return; }

		h -= mod;

		pBuf += SSD1306_LCDWIDTH;
	}


	// write solid bytes while we can - effectively doing 8 rows at a time
	if(h >= 8) {
		if (color == INVERSE)  {          // separate copy of the code so we don't impact performance of the black/white write version with an extra comparison per loop
			do  {
				*pBuf=~(*pBuf);

				// adjust the buffer forward 8 rows worth of data
				pBuf += SSD1306_LCDWIDTH;

				// adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
				h -= 8;
			} while(h >= 8);
		}
		else {
			// store a local value to work with
			register uint8_t val = (color == WHITE) ? 255 : 0;

			do  {
				// write our value in
				*pBuf = val;

				// adjust the buffer forward 8 rows worth of data
				pBuf += SSD1306_LCDWIDTH;

				// adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
				h -= 8;
			} while(h >= 8);
		}
	}

	// now do the final partial byte, if necessary
	if(h) {
		mod = h & 7;
		// this time we want to mask the low bits of the byte, vs the high bits we did above
		// register uint8_t mask = (1 << mod) - 1;
		// note - lookup table results in a nearly 10% performance improvement in fill* functions
		static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
		register uint8_t mask = postmask[mod];
		switch (color) {
			case WHITE:   *pBuf |=  mask;  break;
			case BLACK:   *pBuf &= ~mask;  break;
			case INVERSE: *pBuf ^=  mask;  break;
		}
	}
}


void ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
	for (int16_t i = x; i < x + w; i++) {
		ssd1306_draw_fast_vline(i, y, h, color);
	}	
}



void ssd1306_draw_fast_hline(uint8_t x, uint8_t y, uint16_t w, uint8_t color) {
	if(x >= SSD1306_LCDWIDTH || y >= SSD1306_LCDHEIGHT) {				// do nothing if we're out of the screen
		return;
	}

	// set up the pointer for  movement through the buffer
	register uint8_t *pBuf = ssd1306_buffer;
	// adjust the buffer pointer for the current row
	pBuf += ((y/8) * SSD1306_LCDWIDTH);
	// and offset x columns in
	pBuf += x;

	register uint8_t mask = 1 << (y&7);

	switch (color)
	{
		case WHITE:         while(w--) { *pBuf++ |= mask; }; break;
		case BLACK: mask = ~mask;   while(w--) { *pBuf++ &= mask; }; break;
		case INVERSE:         while(w--) { *pBuf++ ^= mask; }; break;
	}
}



uint8_t ssd1306_print_uint(uint8_t x, uint8_t y, uint8_t ui,  uint8_t size, uint8_t dig_qty ) {
	
	
	register uint8_t p = size*6; // dig pos period
	register uint8_t xpos = 0;
	
	if(dig_qty == 3) {
		ssd1306_put_char(x, y, ui/100 + 48, size);
		++xpos;
	}
	
	if (dig_qty >= 2) {
		ssd1306_put_char(x + xpos*p, y, ui/10 + 48, size);
		++xpos;
	}
	
	ssd1306_put_char(x + xpos*p, y, ui%10 + 48, size);
	
	return ++xpos*p + x;
}