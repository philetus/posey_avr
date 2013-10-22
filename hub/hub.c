/*
 *  hub.c
 *  
 *  reads bit-banged contact address on each pin and transmits new
 *  address on change
 *
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <defines.h>
#include <uart.h>
#include <bit_bang.h>

#define MAX_READ_FAILURES 10

/* hub 88.1 */
#define SOCKETS 4
#define PINS_PER_SOCKET 4
#define RX_PIN_COUNT 16
static struct pin_data rx_pin_table[RX_PIN_COUNT] =
{
	{ &DDRB, &PORTB, &PINB, PB2 }, //  0
	{ &DDRB, &PORTB, &PINB, PB3 }, //  1
	{ &DDRB, &PORTB, &PINB, PB6 }, //  2
	{ &DDRB, &PORTB, &PINB, PB7 }, //  3
	{ &DDRC, &PORTC, &PINC, PC0 }, //  4
	{ &DDRC, &PORTC, &PINC, PC1 }, //  5
	{ &DDRC, &PORTC, &PINC, PC4 }, //  6
	{ &DDRC, &PORTC, &PINC, PC5 }, //  7
	{ &DDRB, &PORTB, &PINB, PB0 }, //  8
	{ &DDRB, &PORTB, &PINB, PB1 }, //  9
	{ &DDRD, &PORTD, &PIND, PD4 }, // 10
	{ &DDRD, &PORTD, &PIND, PD5 }, // 11
	{ &DDRD, &PORTD, &PIND, PD2 }, // 12
	{ &DDRD, &PORTD, &PIND, PD3 }, // 13
	{ &DDRD, &PORTD, &PIND, PD6 }, // 14
	{ &DDRD, &PORTD, &PIND, PD7 }  // 15
};

static uint8_t read_failures[RX_PIN_COUNT];
static uint8_t last_address[RX_PIN_COUNT][ADDRESS_SIZE];
static uint8_t hub_address[ADDRESS_SIZE] = { 17, 5, 0 };
static uint8_t nil_address[ADDRESS_SIZE] = { 0, 0, 0 };

// counts interrupts since last event sent
volatile uint16_t last_event_counter[SOCKETS]; 

/*
 * set up timer to trigger events
 */
static void init_event_timer( void )
{
    uint8_t i;
    
    // set timer to CTC mode, OCR2A top
    TCCR2A |= _BV( WGM21 );
    
    // set prescaler to F_CPU / 1024 (~1kHz)
    TCCR2B |= _BV( CS20 ) | _BV( CS21 ) | _BV( CS22 );

    // number of tics before interrupt is triggered
    OCR2A = 200; // ~5Hz
    
    // enable timer 2 compare A interrupt.
    TIMSK2 |= _BV( OCIE2A );
    
    // init last event counter
    for( i = 0; i < SOCKETS; i++ ) last_event_counter[i] = 0;
}

/*
 * increment last event counter
 */
ISR(TIMER2_COMPA_vect)
{
    uint8_t i;
    
    for( i = 0; i < SOCKETS; i++ ) last_event_counter[i]++;
}

static void uart_put_address( uint8_t address[], uint8_t length )
{
	uint8_t i;
	
	for( i = 0; i < length; i++ )
	{
		uart_put_int( address[i] );
		if( i < length - 1 ) uart_put_char( '.' );
	}
}

/*
 * write xml connect event specifying hub and strut addresses connected
 */
void uart_put_socket_event( uint8_t socket )
{
    uint8_t i;
    
    last_event_counter[socket] = 0;
    
	uart_put_PSTR( PSTR("<socket_event hub_address=\"") );
	uart_put_address( hub_address, ADDRESS_SIZE - 1 );
	uart_put_PSTR( PSTR("\" socket=\"") );
	uart_put_int( socket );
	for( i = 0; i < PINS_PER_SOCKET; i++ )
	{
	    uart_put_PSTR( PSTR("\" ") );
	    uart_put_int( i );
	    uart_put_PSTR( PSTR("=\"") );
	    uart_put_address( last_address[(socket*PINS_PER_SOCKET)+i], ADDRESS_SIZE );
	}
	uart_put_PSTR( PSTR("\" />\n") );
}

/*
 * swap most recently read address on given pin into connected table 
 */
static void swap_address( uint8_t pin_number, uint8_t strut_address[] )
{
	uint8_t i, address_changed;
	
	read_failures[pin_number] = 0;
	
	address_changed = FALSE;
	for( i = 0; i < ADDRESS_SIZE; i++ )
	{
		if( last_address[pin_number][i] != strut_address[i] )
			address_changed = TRUE;
	}
	
	if( !address_changed ) return;
	
	// update connected strut for pin number
	for( i = 0; i < ADDRESS_SIZE; i++ )
		last_address[pin_number][i] = strut_address[i];
	uart_put_socket_event( pin_number / PINS_PER_SOCKET );
}

/*
 *
 */
int main( void )
{
	uint8_t i;
	
	init_uart();
	
	DEBUG_WARN( uart_put_PSTR(PSTR("starting hub\n")) );
    
	init_rx_pins( rx_pin_table, RX_PIN_COUNT );
	init_bit_bang_timer( TICS_PER_INTERRUPT, PRESCALER );    
    init_event_timer();
	sei(); /* global interrupt enable */
	
	/* loop and check for address change */
	while( 1 )
	{
		for( i = 0; i < RX_PIN_COUNT; i++ )
		{
			if( rx_packets[i].new )
			{
				hub_address[ADDRESS_SIZE - 1] = i;
				if( rx_packets[i].length < 1 )
				{
					if( read_failures[i]++ > MAX_READ_FAILURES )
					{
						swap_address( i, nil_address );
					}
				}
				else
				{
					swap_address( i, rx_packets[i].data );
				}
				rx_packets[i].new = FALSE;
			}
		}
		
		// check for sockets whose event counters have timed out
		for( i = 0; i < SOCKETS; i++ )
		{
		    if( last_event_counter[i] > 5 ) // no events sent for ~1 second
		    {
		        uart_put_socket_event( i );
		    }
		}
	}
}
