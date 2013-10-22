/*
 *  strut.c
 * 
 *  bit-bangs strut address and pin number out of a list of avr pins
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

#include <defines.h>
#include <bit_bang.h>


/* table of pins to transmit data out of */
#define TX_PIN_COUNT 22
static struct pin_data tx_pins[TX_PIN_COUNT] =
{
	{ &DDRB, &PORTB, &PINB, PB3 }, //  0.0
	{ &DDRB, &PORTB, &PINB, PB2 }, //  0.1
	{ &DDRB, &PORTB, &PINB, PB0 }, //  0.2
	{ &DDRB, &PORTB, &PINB, PB1 }, //  0.3
	{ &DDRC, &PORTC, &PINC, PC4 }, //  0.4 - 0.4 & 0.5 are backwards on sb v0.3
	{ &DDRB, &PORTB, &PINB, PB6 }, //  0.5
	{ &DDRC, &PORTC, &PINC, PC0 }, //  0.6
	{ &DDRB, &PORTB, &PINB, PB7 }, //  0.7
	{ &DDRC, &PORTC, &PINC, PC1 }, //  0.8
	{ &DDRB, &PORTB, &PINB, PB5 }, //  0.9
	{ &DDRB, &PORTB, &PINB, PB4 }, //  0.10
	{ &DDRD, &PORTD, &PIND, PD4 }, //  1.0
	{ &DDRD, &PORTD, &PIND, PD1 }, //  1.1
	{ &DDRC, &PORTC, &PINC, PC5 }, //  1.2
	{ &DDRC, &PORTC, &PINC, PC2 }, //  1.3
	{ &DDRD, &PORTD, &PIND, PD7 }, //  1.4
	{ &DDRD, &PORTD, &PIND, PD5 }, //  1.5
	{ &DDRD, &PORTD, &PIND, PD3 }, //  1.6
	{ &DDRD, &PORTD, &PIND, PD2 }, //  1.7
	{ &DDRD, &PORTD, &PIND, PD6 }, //  1.8
	{ &DDRC, &PORTC, &PINC, PC3 }, //  1.9
	{ &DDRD, &PORTD, &PIND, PD0 }  //  1.10
};

/* strut address */
static uint8_t strut_address[ADDRESS_SIZE - 1] = { 3, 20 };

extern struct data_packet *tx_packets;

/*
 *  loop and test flags
 */
int main( void )
{
	uint8_t i, j;
	
	init_tx_pins( tx_pins, TX_PIN_COUNT );
	init_bit_bang_timer( TICS_PER_INTERRUPT, PRESCALER );    
	sei(); /* global interrupt enable */
	
	for( i = 0; i < TX_PIN_COUNT; i++ )
	{
		if( !(tx_packets[i].data = malloc(ADDRESS_SIZE * sizeof(uint8_t))) )
			return FALSE;
	}
		
	
	while( 1 )
	{
		for( i = 0; i < TX_PIN_COUNT; i++ )
		{						
			if( !tx_packets[i].new )
			{
				tx_packets[i].length = ADDRESS_SIZE;
				for( j = 0; j < ADDRESS_SIZE - 1; j++ )
					tx_packets[i].data[j] = strut_address[j];
				tx_packets[i].data[ADDRESS_SIZE - 1] = i;
				tx_packets[i].new = TRUE;
				
				DEBUG( uart_put_PSTR(PSTR("set new tx packet\n")) );
			}
		}
	}
}
