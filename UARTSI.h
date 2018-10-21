/*
 * UARTSI.h
 *
 * Created: 08.05.2016 13:39:24
 * Author: Sergey Shelepin <Sergey.Shelepin@gmail.com>
 * 
 * Supports ATMEGA328 
 */ 

#include <avr/interrupt.h>

#ifndef UARTSI_H_
#define UARTSI_H_

// must be power of 2
#ifndef TX_BUF_SIZE 
# warning "TX_BUF_SIZE not defined "
#define TX_BUF_SIZE 32
#endif

// must be power of 2 
#ifndef RX_BUF_SIZE 
# warning "RX_BUF_SIZE not defined "
#define RX_BUF_SIZE 64
#endif

void UARTInitRXTX();
void UARTInitTX();
void UARTInitRXTX4800();
void UARTInitRXTX_conf(uint8_t ubrr0l, uint8_t u2x0);

void UARTDisableRX();
void UARTEnableRX();

ISR( USART_RX_vect ); // takes ~72 bytes of memory even in TX mode (!) should be put under #ifdef directive to save ~72 bytes of RAM.
ISR( USART_UDRE_vect );

char UARTRXGetChar();
void UARTRXNext();
char UARTRXGetCharI();
uint8_t UARTRXIsEmpty();
void UARTRXFlash();

void UARTPutChar(char ch);
void UARTPrint( const char *str );
void UARTPrintln( const char *str );
void UARTPrint_P( const char *str );
void UARTPrintln_P( const char *str );
void UARTPrintUint(uint8_t ui, uint8_t raddix);
void UARTPrintInt16(int16_t i, uint8_t raddix);
void UARTPrintUint_M(uint8_t ui, uint8_t raddix, uint8_t min_cp);
void UARTPrintBit(uint8_t byte_val, const char *bit_name, uint8_t bit_num);
void UARTPrintValDec(const char *name, uint8_t val);
void UARTPrintlnValDec(const char *name, uint8_t val);
void UARTPrintUintBinary(uint8_t ui);

#endif /* UARTSI_H_ */