/*
 *  accelerometer_hub.c
 *  
 *  reads bit-banged contact address on each pin and transmits new
 *  address on change
 *
 *  reads accelerometer values on timer and transmits them back to host
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

/* table of pins to transmit data out of */
#define SOCKETS 2
#define PINS_PER_SOCKET 4
#define RX_PIN_COUNT 8
static struct pin_data rx_pin_table[RX_PIN_COUNT] =
{
    { &DDRB, &PORTB, &PINB, PB0 }, //  0
    { &DDRB, &PORTB, &PINB, PB1 }, //  1
    { &DDRB, &PORTB, &PINB, PB2 }, //  2
    { &DDRB, &PORTB, &PINB, PB6 }, //  3
    { &DDRD, &PORTD, &PIND, PD2 }, //  4
    { &DDRD, &PORTD, &PIND, PD3 }, //  5
    { &DDRD, &PORTD, &PIND, PD5 }, //  6
    { &DDRD, &PORTD, &PIND, PD4 }  //  7
};

static uint8_t read_failures[RX_PIN_COUNT];
static uint8_t last_address[RX_PIN_COUNT][ADDRESS_SIZE];
static uint8_t hub_address[ADDRESS_SIZE] = { 69, 9, 0 };
static uint8_t nil_address[ADDRESS_SIZE] = { 0, 0, 0 };

// counts interrupts since last event sent
volatile uint16_t last_event_counter[SOCKETS]; 

/*
 * analog ports to read x, y and z accelerometer values from
 */
static uint8_t xyz_pins[3] = { 0, 1, 2 };
volatile uint8_t current_xyz_pin = 0;
volatile uint16_t xyz_values[3];

/*
 * enable ADC, select ADC clock = F_CPU / 8 (i.e. 125 kHz)
 */
static void init_adc( void )
{
  ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);
}

/*
 * set up timer to trigger accelerometer readings
 */
static void init_xyz_timer( void )
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
    
    // set current xyz pin to 0, otherwise interrupt handler will
    // return immediately
    current_xyz_pin = 0;

    // init last event counter
    for( i = 0; i < SOCKETS; i++ ) last_event_counter[i] = 0;
}

/*
 * initiate analog read on given port
 */
static void init_analog_read( uint8_t port )
{
    ADMUX &= 0xf0; // clear current port value
    ADMUX |= port; // set new port value
    
    ADCSRA |= _BV(ADIE); // set ADC_vect interrupt to fire when read completes
    ADCSRA |= _BV(ADSC); // init read
}

/*
 * init ADC conversion to read accelerometer values
 */
ISR(TIMER2_COMPA_vect)
{
    uint8_t i;
    
    // increment last event counter
    for( i = 0; i < SOCKETS; i++ ) last_event_counter[i]++;

    // if last value has not been transmitted just return
    if( current_xyz_pin != 0 ) return;
    
    // disable this interrupt until x, y and z values have been read
    // and enable adc interrupt
    TIMSK2 &= ~_BV( OCIE2A );
    ADCSRA |= _BV( ADIE );
    
    // init analog read on pin 0
    init_analog_read( xyz_pins[current_xyz_pin] );
}

/*
 * once analog read is started, check all three (x, y & z) pin values
 * before returning control to timer interrupt
 */
ISR(ADC_vect)
{
    // store analog value for current pin
    xyz_values[current_xyz_pin] = ADCW;
    
    // check next pin
    current_xyz_pin++;
    
    // if still reading values start new analog read on next pin
    if( current_xyz_pin < 3 )
    {
        init_analog_read( xyz_pins[current_xyz_pin] );
        return;
    }
    
    // otherwise disable adc interrupt and enable timer interrupt
    ADCSRA &= ~_BV(ADIE);        // disable ADC interrupt
    TIMSK2 |= _BV( OCIE2A );    // enable timer 2 compare A interrupt.
}

/*
 * write address to uart as ascii characters
 */
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
 * write xml connect event specifying accelerometer value
 */
void uart_put_accelerometer_value( void )
{
    uart_put_PSTR( PSTR("<accelerometer_value hub_address=\"") );
    uart_put_address( hub_address, ADDRESS_SIZE - 1 );
    uart_put_PSTR( PSTR("\" x=\"") );
    uart_put_hex( xyz_values[0] );
    uart_put_PSTR( PSTR("\" y=\"") );
    uart_put_hex( xyz_values[1] );
    uart_put_PSTR( PSTR("\" z=\"") );
    uart_put_hex( xyz_values[2] );
    uart_put_PSTR( PSTR("\" />\n") );
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

    DEBUG_WARN( uart_put_PSTR(PSTR("starting hub\n")) );

    init_uart();
    init_adc();
    init_xyz_timer();    
    init_rx_pins( rx_pin_table, RX_PIN_COUNT );
    init_bit_bang_timer( TICS_PER_INTERRUPT, PRESCALER );    
    sei(); /* global interrupt enable */
    
    // loop
    while( TRUE )
    {
        
        // check for address change
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
      
        // check for accelerometer pin counter to run past 2
        if( current_xyz_pin > 2 )
        {
            // send new accelerometer value
            uart_put_accelerometer_value();
            
            // reset current pin
            current_xyz_pin = 0;
        }
    }
}
