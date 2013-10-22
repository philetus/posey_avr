/*
 *  bit_bang.c
 *  
 *  read and write serial data packets from multiple pins of avr in parallel
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/parity.h>

#include "defines.h"
#include "bit_bang.h"

#ifdef DEBUG_ON
#include "uart.h"
#endif

#define SAMPLES_PER_BIT 3 /* number of times to sample each read bit */
#define SYNC_BITS 12 /* min number of high bits to indicate packet start */
#define SYNC_SAMPLES (SYNC_BITS * SAMPLES_PER_BIT)
#define MAX_HEADER_SAMPLES (SYNC_SAMPLES * 2) /* # samples before fail read */

/* set packet size to 0 to send variable length packets */
#define PACKET_SIZE 3 

struct rx_pin_data
{
	/* pin data */
	volatile uint8_t *ddr;
	volatile uint8_t *port;
	volatile uint8_t *pin;
	uint8_t bit;
	
	/* state data */
	int8_t sample_count;
	int8_t bit_count;
	int8_t byte_count;
	int8_t sample_queue[SAMPLES_PER_BIT];
	
	/* packet data */
	uint8_t length; /* length of packet */
	uint8_t byte_buffer; /* checksum byte */
	uint8_t *data_buffer; /* buffer to hold data read */
};

struct tx_pin_data
{
	/* pin data */
	volatile uint8_t *ddr;
	volatile uint8_t *port;
	volatile uint8_t *pin;
	uint8_t bit;
	
	/* state data */
	int8_t bit_count;
	int8_t byte_count;
	
	/* packet data */
	uint8_t length;
	uint8_t byte_buffer;
	uint8_t *data_buffer;
	uint8_t sent; /* flag to indicate finished sending data */
};

static uint8_t rx_init_flag = FALSE;
static uint8_t tx_init_flag = FALSE;

static struct rx_pin_data *rx_data;
static struct tx_pin_data *tx_data;

static uint8_t rx_pin_count; /* length of rx pin data and rx packet tables */
static uint8_t tx_pin_count; /* length of tx pin data and tx packet tables */

static uint8_t interrupt_counter; /* counts samples since last bit transmit */

static void write_high_bit( uint8_t pin_number )
{
	*(tx_data[pin_number].port) |= _BV( tx_data[pin_number].bit );
	DEBUG( uart_put_PSTR(PSTR("1")) );
}

static void write_low_bit( uint8_t pin_number )
{
	*(tx_data[pin_number].port) &= ~_BV( tx_data[pin_number].bit );
	DEBUG( uart_put_PSTR(PSTR("0")) );
}

/*
 * if strut address buffer value is different from strut address, set new 
 * address value and set address changed flag
 *
 * wait until address changed flag is unset by send address to swap address
 * again
 */
static void swap_rx_data_buffer( uint8_t pin_number )
{
	uint8_t i;
	struct rx_pin_data *pin;
	struct data_packet *rx_packet;
	
	DEBUG( uart_put_PSTR(PSTR("swapping pin ")) );
	DEBUG( uart_put_int(pin_number) );
	DEBUG( uart_put_PSTR(PSTR(" data buffer!\n")) );
	
	/* set pin and rx packet for given pin number */
	pin = &rx_data[pin_number];
	rx_packet = &rx_packets[pin_number];
		 
	/* if new data flag is still set on packet last value has not been
	   read yet so do nothing */
	if( (*rx_packet).new )
	{
		DEBUG( uart_put_PSTR(PSTR("last rx packet still new!\n")) );
		return;
	}
	
	/* set packet data length */
	(*rx_packet).length = (*pin).length;
	
	/* realloc space for packet data */
	if( ((*rx_packet).length > 0) &&
		!((*rx_packet).data = realloc((*rx_packet).data, 
									  (*rx_packet).length * sizeof(uint8_t))) )
		DEBUG( uart_put_PSTR(PSTR("failed to realloc rx packet data\n")) );
		
	/* swap data */
	for( i = 0; i < (*rx_packet).length; i++ )
		(*rx_packet).data[i] = (*pin).data_buffer[i];
		
	/* set new data flag on packet */
	(*rx_packet).new = TRUE;
}

/*
 * try to swap new tx packet in from tx packets table
 */
static uint8_t swap_tx_data_buffer( uint8_t pin_number )
{
	uint8_t i;
	struct tx_pin_data *pin;
	struct data_packet *tx_packet;
	
	/* set pin and packet for given pin number */
	pin = &tx_data[pin_number];
	tx_packet = &tx_packets[pin_number];
/*
	DEBUG( if(pin_number == 0) uart_put_PSTR(PSTR("\n")) );
	DEBUG( uart_put_PSTR(PSTR("[a[")) );
	DEBUG( uart_put_int(pin_number) );
	DEBUG( uart_put_PSTR(PSTR(":")) );
	DEBUG( uart_put_int((*pin).byte_count) );
	DEBUG( uart_put_PSTR(PSTR(":")) );
	DEBUG( uart_put_int((*pin).bit_count) );
	DEBUG( uart_put_PSTR(PSTR(":")) );
	DEBUG( uart_put_int((*pin).byte_buffer) );
	DEBUG( uart_put_PSTR(PSTR("] ")) );
*/
	/* if packet new flag is not set return false */
	if( !(*tx_packet).new )
	{
		DEBUG( uart_put_PSTR(PSTR("no new tx packet to swap!\n")) );
		return FALSE;
	}
	
	/* set packet length */
	if( PACKET_SIZE > 0 ) (*pin).length = PACKET_SIZE;
	else (*pin).length = (*tx_packet).length;
	
	/* realloc pin data buffer */
	if( !((*pin).data_buffer = realloc((*pin).data_buffer,
									   (*pin).length * sizeof(uint8_t))) )
		DEBUG_WARN( uart_put_PSTR(PSTR("failed to realloc tx data buffer!\n")) );
	
	/* swap data */
	for( i = 0; i < (*pin).length; i++ )
		(*pin).data_buffer[i] = (*tx_packet).data[i];
	
	/* reset counters */
	(*pin).bit_count = 0;
	(*pin).byte_count = -2;
	
	/* clear new packet and sent flags */
	(*pin).sent = FALSE;
	(*tx_packet).new = FALSE;
/*	
	DEBUG( if(pin_number == 0) uart_put_PSTR(PSTR("\n")) );
	DEBUG( uart_put_PSTR(PSTR("[b[")) );
	DEBUG( uart_put_int(pin_number) );
	DEBUG( uart_put_PSTR(PSTR(":")) );
	DEBUG( uart_put_int((*pin).byte_count) );
	DEBUG( uart_put_PSTR(PSTR(":")) );
	DEBUG( uart_put_int((*pin).bit_count) );
	DEBUG( uart_put_PSTR(PSTR(":")) );
	DEBUG( uart_put_int((*pin).byte_buffer) );
	DEBUG( uart_put_PSTR(PSTR("] ")) );
*/
	return TRUE;
}

static void fail_read_packet( uint8_t pin_number )
{
	struct rx_pin_data *pin;
	
	/* set pin to given index */
	pin = &rx_data[pin_number];
	
	/* set length to 0 and swap data */
	(*pin).length = 0;
	swap_rx_data_buffer( pin_number );
	
	/* reset counters */
	(*pin).sample_count = 0;
	(*pin).byte_count = -2;

	DEBUG( uart_put_PSTR(PSTR("pin ")) );
	DEBUG( uart_put_int(pin_number) );
	DEBUG( uart_put_PSTR(PSTR(" failed to read packet!\n")) );
}

/*
 * function to increment counters at each sample within byte
 */
static void increment_rx_counters( uint8_t pin_number )
{
	uint8_t i, sum;
	struct rx_pin_data *pin;
	
	/* set pin to given index */
	pin = &rx_data[pin_number];
	
	/* increment sample counter, if it is already at max reset */
	if( ++(*pin).sample_count == SAMPLES_PER_BIT )
	{
		(*pin).sample_count = 0;
		
		/* increment bit counter, if it is already on 9 swap byte buffer */
		if( (*pin).bit_count++ == 9 )
		{
			(*pin).bit_count = -1;

			if( (*pin).byte_count == -1 ) /* length byte */
			{
				(*pin).length = (*pin).byte_buffer;
			}
			else if( (*pin).byte_count == (*pin).length ) /* checksum byte */
			{
				sum = 0;
				for( i = 0; i < (*pin).length; i++ )
					sum += (*pin).data_buffer[i];
				if( sum != (*pin).byte_buffer )
				{
						DEBUG( uart_put_PSTR(PSTR("failed checksum!\n")) );
						fail_read_packet( pin_number );
						return;
				}
			}
			else if( (*pin).byte_count > -1 &&
					 (*pin).byte_count < (*pin).length ) /* data byte */
			{
				/* reallocate memory for data buffer on first data byte */
				if( (*pin).byte_count == 0 )
				{
					if( !((*pin).data_buffer = 
							realloc((*pin).data_buffer,
									(*pin).length * sizeof(uint8_t))) )
						DEBUG_WARN( uart_put_PSTR(PSTR("failed to realloc rx data buffer\n")) );
				}
				/* swap data byte into byte buffer */
				(*pin).data_buffer[(*pin).byte_count] = (*pin).byte_buffer;
			}
			else
			{
				DEBUG( uart_put_PSTR(PSTR("unexpected rx byte count!\n")) );
			}
			/* increment byte counter, if checksum has alrady been read swap
			   data buffer */
			if( (*pin).byte_count++ == (*pin).length )
			{
				(*pin).byte_count = -2;
				(*pin).sample_count = 0;
				swap_rx_data_buffer( pin_number );
			}
		}
	}
}

/*
 * function to increment counters at each bit within byte
 */
static void increment_tx_counters( uint8_t pin_number )
{
	uint8_t i;
	struct tx_pin_data *pin;
	
	/* set pin to given index */
	pin = &tx_data[pin_number];
	
	/* when next byte is reached load new byte into byte buffer */
	if( (*pin).bit_count++ == 9 )
	{
		(*pin).bit_count = -1;
		
		/* if entire packet has been sent set sent flag and return */
		if( (*pin).byte_count++ == (*pin).length )
		{
			(*pin).sent = TRUE;
			return;
		}
		
		/* if all data bytes have been sent load checksum into byte buffer */
		else if( (*pin).byte_count == (*pin).length )
		{
			(*pin).byte_buffer = 0;
			
			for( i = 0; i < (*pin).length; i++ )
			{
				(*pin).byte_buffer += (*pin).data_buffer[i];
			}
		}
		
		/* otherwise load next data byte into byte buffer */
		else
		{
			(*pin).byte_buffer = (*pin).data_buffer[(*pin).byte_count];
		}
		
		DEBUG( if(pin_number == 0) uart_put_PSTR(PSTR("\n")) );
		DEBUG( uart_put_PSTR(PSTR("{")) );
		DEBUG( uart_put_int(pin_number) );
		DEBUG( uart_put_PSTR(PSTR(":")) );
		DEBUG( uart_put_int((*pin).byte_count) );
		DEBUG( uart_put_PSTR(PSTR(":")) );
		DEBUG( uart_put_int((*pin).byte_buffer) );
		DEBUG( uart_put_PSTR(PSTR("} ")) );
	}
}

/*
 * poll rx pin and update pin state in rx data table
 */
static void poll( uint8_t pin_number )
{
	uint8_t i;
	struct rx_pin_data *pin;
	
	/* set pin to given index */
	pin = &rx_data[pin_number];

	/* sample pin and rotate value onto sample queue*/
	for( i = SAMPLES_PER_BIT - 1; i > 0; i-- )
		(*pin).sample_queue[i] = (*pin).sample_queue[i - 1];
	(*pin).sample_queue[0] = bit_is_clear( *((*pin).pin), (*pin).bit );
	
	/* byte count: -2 -> header
	 *             -1 -> n bytes of data
	 *             0-(n-1) -> data
	 *			   n -> checksum
	 *
	 * first try to read SYNC_SAMPLES high samples in a row to indicate
	 * the beginning of a new packet
	 */
	if( (*pin).byte_count < -1 )
	{
		/* if sample is high increase sample count */
		if( (*pin).sample_queue[0] )
		{			
			DEBUG( uart_put_PSTR(PSTR(".")) );
			DEBUG( uart_put_int((*pin).sample_count) );

			/* if we have not successfully read a packet header within the 
			   maximimum sample count fail read packet */
			if( (*pin).sample_count++ > MAX_HEADER_SAMPLES )
				fail_read_packet( pin_number );
			return;
		}
		/* otherwise sample is low */
		DEBUG( uart_put_PSTR(PSTR("got low sample after ")) );
		DEBUG( uart_put_int((*pin).sample_count) );
		DEBUG( uart_put_PSTR(PSTR(" samples!\n")) );

		/* if there were enough high samples look for byte */
		if( (*pin).sample_count > SYNC_SAMPLES )
		{
			(*pin).sample_count = 1;
			(*pin).bit_count = -1;

			/* if packet size is fixed set length and jump to first data
			   byte*/
			if( PACKET_SIZE > 0 )
			{
				(*pin).length = PACKET_SIZE;
				(*pin).byte_count = 0;
			}
			/* otherwise read packet length byte */
			else
			{
				(*pin).byte_count = -1;
			}
			DEBUG( uart_put_PSTR(PSTR("[0")) );
			return;
		}			
		/* otherwise fail read because we haven't seen a full header */
		fail_read_packet( pin_number );
		return;
	}
	
	/*
	 * after we have seen packet header read bytes into byte buffer and
	 * take the appropriate action with them once they are read and checked
	 * against the parity bit
	 */
	switch( (*pin).sample_count )
	{
		/* on first sample just print out debugging info */
		case 0:
			DEBUG( if((*pin).bit_count == 0) uart_put_PSTR(PSTR("<")) );
			DEBUG( uart_put_PSTR(PSTR("[")) );
			DEBUG( uart_put_int((*pin).sample_queue[0]) );
			break;

		/* on second sample read bit value */
		case 1:
			DEBUG( uart_put_int((*pin).sample_queue[0]) );

			/* check which bit we are reading */
			switch( (*pin).bit_count )
			{
				/* start bit should be low */
				case -1:
					if( (*pin).sample_queue[0] )
					{
						DEBUG( uart_put_PSTR(PSTR("failed start bit check!\n")) );
						fail_read_packet( pin_number );
						return;
					}
					break;

				/* collect data bits, cases 0-7 fall through to
				   block to write current bit to byte buffer */
				case 0:
				case 1:
				case 2:
				case 3:
				case 4:
				case 5:
				case 6:
				case 7:
					if( (*pin).sample_queue[0] )
						(*pin).byte_buffer |= _BV( (*pin).bit_count );
					else
						(*pin).byte_buffer &= ~_BV( (*pin).bit_count );
					break;

				/* check parity bit */
				case 8:						
					if( (*pin).sample_queue[0] !=
							parity_even_bit((*pin).byte_buffer) )
					{
						DEBUG( uart_put_PSTR(PSTR("failed parity bit check!\n")) );
						fail_read_packet( pin_number );
						return;
					}
					break;

				/* stop bit should be high */
				case 9:
					if( !(*pin).sample_queue[0] )
					{
						DEBUG( uart_put_PSTR(PSTR("failed stop bit check!\n")) );
						fail_read_packet( pin_number );
						return;
					}
					break;
			}
			break;

		/* on last sample analyze sample queue for drift and correct */
		case 2:
			DEBUG( uart_put_int((*pin).sample_queue[0]) );
			DEBUG( uart_put_PSTR(PSTR("]")) );
			DEBUG( if((*pin).bit_count == 7) uart_put_PSTR(PSTR(">")) );

			/* if we have drifted ahead slow down */
			if( ((*pin).sample_queue[0] == (*pin).sample_queue[1]) && 
				((*pin).sample_queue[1] != (*pin).sample_queue[2]) )
			{
				DEBUG( uart_put_PSTR(PSTR("(-)")) );
				return; /* return without incrementing counters */
			}

			/* if we have drifted behind speed up */
			else if( ((*pin).sample_queue[0] != (*pin).sample_queue[1]) && 
					 ((*pin).sample_queue[1] == (*pin).sample_queue[2]) )
			{
				DEBUG( uart_put_PSTR(PSTR("[(+)")) );

				/* increment counters twice and return */
				increment_rx_counters( pin_number );
				if( (*pin).byte_count == -2 ) return; /* fail checksum? */
				increment_rx_counters( pin_number );
				return;
			}
			break;

		default:
			DEBUG_WARN( uart_put_PSTR(PSTR("invalid sample count\n")) );
	}
	/* increment all counters */
	increment_rx_counters( pin_number );
}

/*
 * write next bit to tx pin
 */
static void transmit( uint8_t pin_number )
{
	struct tx_pin_data *pin;
	
	/* set pin to given pin number */
	pin = &tx_data[pin_number];
	
	/* if packet has already been sent check for new data to send,
	   if there is no new data just return */
	if( (*pin).sent )
	{
		swap_tx_data_buffer(pin_number);
		return;
	}
	
	/* first send packet header */
	if( (*pin).byte_count < -1 )
	{
		write_high_bit( pin_number );
		
		/* increment counters */
		if( (*pin).bit_count++ == SYNC_BITS ) /* send one extra */
		{
			(*pin).bit_count = -1;
			
			/* if packet size is fixed jump to first data byte */
			if( PACKET_SIZE > 0 )
			{
				(*pin).byte_count = 0;
				(*pin).byte_buffer = (*pin).data_buffer[0];
			}
			else
			{
				(*pin).byte_count = -1;
				(*pin).byte_buffer = (*pin).length;
			}

			DEBUG( if(pin_number == 0) uart_put_PSTR(PSTR("\n")) );
			DEBUG( uart_put_PSTR(PSTR("{")) );
			DEBUG( uart_put_int(pin_number) );
			DEBUG( uart_put_PSTR(PSTR(":")) );
			DEBUG( uart_put_int((*pin).byte_count) );
			DEBUG( uart_put_PSTR(PSTR(":")) );
			DEBUG( uart_put_int((*pin).byte_buffer) );
			DEBUG( uart_put_PSTR(PSTR("} ")) );
		}
		return;
	}
	
	/* send byte in byte buffer */
	switch( (*pin).bit_count )
	{		
		case -1: /* start bit */
			write_low_bit( pin_number );
			DEBUG( uart_put_PSTR(PSTR("<")) );
			break;

		/* data bits */
		case 0:		
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
			if( ((*pin).byte_buffer >> (*pin).bit_count) & 1 )
				write_high_bit( pin_number );
			else
				write_low_bit( pin_number );
			break;

		case 8: /* parity bit */
			DEBUG( uart_put_PSTR(PSTR(">")) );

			if( parity_even_bit((*pin).byte_buffer) ) 
				write_high_bit( pin_number );
			else write_low_bit( pin_number );
			break;

		case 9: /* stop bit */
			write_high_bit( pin_number );
			break;

		default:
			DEBUG_WARN( uart_put_PSTR(PSTR("invalid bit count!\n")) );
	}

	/* increment counters and load next byte into byte buffer */
	increment_tx_counters( pin_number );
}

/*
 * called to give a list of pins to read data from
 *
 * data will be read into rx packet table
 */
void init_rx_pins( struct pin_data rx_pins[], uint8_t pin_count )
{
	uint8_t i, j;
	
	rx_pin_count = pin_count;
	
	/* malloc space for rx packet list */
	if( !(rx_packets = malloc(rx_pin_count * sizeof(struct data_packet))) )
		DEBUG_WARN( uart_put_PSTR(PSTR("failed to malloc rx packets list\n")) );
	
	/* malloc space for rx pin data table */
	if( !(rx_data = malloc(rx_pin_count * sizeof(struct rx_pin_data))) )
		DEBUG_WARN( uart_put_PSTR(PSTR("failed to malloc rx data list\n")) );
	
	/* init rx pin data table and rx pins */
	for( i = 0; i < rx_pin_count; i++ )
	{
		/* copy pin data */
		rx_data[i].ddr = rx_pins[i].ddr;
		rx_data[i].port = rx_pins[i].port;
		rx_data[i].pin = rx_pins[i].pin;
		rx_data[i].bit = rx_pins[i].bit;
		
		/* set pin as input */
		*(rx_data[i].ddr) &= ~_BV( rx_data[i].bit );
		*(rx_data[i].port) |= _BV( rx_data[i].bit );
		
		/* initialize state values */
		rx_data[i].sample_count = 0;
		rx_data[i].bit_count = -1;
		rx_data[i].byte_count = -2;
		for( j = 0; j < SAMPLES_PER_BIT; j++ ) rx_data[i].sample_queue[j] = -1;

		/* initialize packet data values */
		rx_packets[i].new = FALSE;
		rx_packets[i].length = 0;
		
		rx_packets[i].data = 0;
		rx_data[i].data_buffer = 0;
	}
	
	rx_init_flag = TRUE;
}

/*
 * called to give a list of pins to write data to
 *
 * to send data it should be written to tx packet table and then new
 * packet flag should be set to signal that the data should be copied into
 * the write buffer and sent
 */
void init_tx_pins( struct pin_data tx_pins[], uint8_t pin_count )
{
	uint8_t i;
	
	tx_pin_count = pin_count;
	
	/* malloc space for tx packet list */
	if( !(tx_packets = malloc(tx_pin_count * sizeof(struct data_packet))) )
		DEBUG_WARN( uart_put_PSTR(PSTR("failed to malloc tx packets list\n")) );
		
	/* malloc space for tx pin data */
	if( !(tx_data = malloc(tx_pin_count * sizeof(struct tx_pin_data))) )
		DEBUG_WARN( uart_put_PSTR(PSTR("failed to malloc tx data list\n")) );

	/* init tx pin data table and tx pins */
	for( i = 0; i < tx_pin_count; i++ )
	{
		/* copy pin data */
		tx_data[i].ddr = tx_pins[i].ddr;
		tx_data[i].port = tx_pins[i].port;
		tx_data[i].pin = tx_pins[i].pin;
		tx_data[i].bit = tx_pins[i].bit;
		
		/* set pin as output */
		*(tx_data[i].ddr) |= _BV( tx_data[i].bit );
		
		/* initialize state values */
		tx_data[i].bit_count = -1;
		tx_data[i].byte_count = -1;
		tx_data[i].sent = TRUE;
		
		/* initialize packet data values */
		tx_packets[i].new = FALSE;
		tx_packets[i].length = 0;
		
		tx_packets[i].data = 0;
		tx_data[i].data_buffer = 0;
	}
		
	tx_init_flag = TRUE;
}

/*
 * sets up timer interrupt handler to read data from rx pins into rx packets
 * table and write data from tx packet table to tx pins
 *
 * tics is the number of clock tics to count up to before triggering
 * each interrupt, and prescaler controls the rate of clock tics, which can
 * be either
 *   0 - no prescaler, 1mhz
 *   1 - 1/8 prescaler, 125khz
 *   2 - 1/64 prescaler, 31.3khz
 *   3 - 1/256 prescaler, 7.81 khz
 *   4 - 1/1024 prescaler, 977hz
 */
void init_bit_bang_timer( uint16_t tics, uint8_t prescaler )
{
	/* timer1 settings */
    TCCR1B = _BV( WGM12 ); /* CTC mode, top = OCR1A */
	switch( prescaler )
	{
		case 1: TCCR1B |= _BV( CS11 ); break;			
		case 2: TCCR1B |= _BV( CS10 ) | _BV( CS11 ); break;
		case 3: TCCR1B |=  _BV( CS12 ); break;			
		case 4: TCCR1B |= _BV( CS10 ) | _BV( CS12 ); break;
		default: TCCR1B |= _BV( CS10 ); /* no prescaler */
	}
	
	/* number of tics before interrupt is triggered */
	OCR1A = tics;
	
    /* enable timer 1 compare A interrupt. */
    TIMSK1 = _BV( OCIE1A );
}

/*
 * interrupt handler is triggered to handle rx and tx
 */
ISR( TIMER1_COMPA_vect )
{
	uint8_t i;
	
/*	DEBUG( uart_put_PSTR(PSTR("*")) );
*/	
	/* if tx pins have been initialized set transmit bits at correct interval */
	if( tx_init_flag && (++interrupt_counter == SAMPLES_PER_BIT) )
	{
		interrupt_counter = 0;
		for( i = 0; i < tx_pin_count; i++ )
			transmit( i );
	}
	
	/* if rx pins have been initialized poll each rx pin for sample value */
	if( rx_init_flag )
	{
		for( i = 0; i < rx_pin_count; i++ )
			poll( i );
	}
/*	DEBUG( uart_put_PSTR(PSTR("#")) );
*/
}
