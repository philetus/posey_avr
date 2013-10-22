/*
 * uart.c
 *
 * functions to init uart transmit and write xml connect event to uart
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "defines.h"
#include "uart.h"

/*
 * init uart for transmit 8N1 9600
 */
void init_uart( void )
{
	UCSR0A = _BV(U2X0);		/* improves baud rate error @ F_CPU = 1 MHz */
	UBRR0L = (F_CPU / (8 * BAUD_RATE)) - 1;
	UCSR0B = _BV(TXEN0); /* tx enable only */
}

/*
 *  write a single character to uart
 */
int uart_put_char( char c )
{
  if (c == '\n')
    uart_put_char('\r');
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
  return 0; /* so it could be used for fdevopen(), too */
}

/*
 *  write a string created with PSTR macro to uart
 */
void uart_put_PSTR( const char *addr )
{
  char c;

  while ((c = pgm_read_byte(addr++)))
    uart_put_char(c);
}

/*
 *  convert an int to a string and write it to uart
 */
void uart_put_int( uint8_t i )
{
	if( i > 99 )
	{
		uart_put_char( (char) ((i / 100) + 48) );
		i = i % 100;
		if( i < 10 ) uart_put_char( '0' );
	}
	if( i > 9 )
	{
		uart_put_char( (char) ((i / 10) + 48) );
		i = i % 10;
	}
	uart_put_char( (char) (i + 48) );
}

/*
 *  convert an int to a hex string and write it to uart
 */
void uart_put_hex( uint16_t i )
{
	uint8_t next, shift, nonzero;
	
	// start writing chars for each 4 bits once a nonzero value is reached
	for( shift = 0xc; shift < 0xf; shift -= 4 )
	{
	    next = (i >> shift) & 0xf; // consider each 4 bits
	    if( nonzero | (next > 0) | (shift == 0) )
	    {
	        nonzero = TRUE;
	        
	        if( next < 0xa ) uart_put_char( (char) (next + 0x30) );
	        else uart_put_char( (char) (next + 0x57) );
	    }
	}
}

