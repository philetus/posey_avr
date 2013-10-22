/*
 *  blink.c
 * 
 *  uses avr timer interrupt to trigger led flag read by mainloop
 *
 *  lists register settings to run at full 1mhz clock or use prescalers
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define LED_PORT PORTB
#define LED_DDR DDRB
#define LED_PIN PINB
#define LED_BIT_0 PB0
#define LED_BIT_1 PB1
#define LED_BIT_2 PB2
#define LED_BIT_3 PB3

/*
 *  set up io pins 
 */
static void init_io( void )
{
    /* enable led pins as output. */
    LED_DDR |= _BV( LED_BIT_0 ) | _BV( LED_BIT_1 ) | 
	           _BV( LED_BIT_2 ) | _BV( LED_BIT_3 );

/*	LED_PORT |= _BV( LED_BIT_0 ) | _BV( LED_BIT_1 ) | 
	            _BV( LED_BIT_2 ) | _BV( LED_BIT_3 );		
*/
}

/*
 *  
 */
int main( void )
{
	
	init_io();
	
	LED_PIN |= _BV( LED_BIT_0 ) | _BV( LED_BIT_1 ) | 
	           _BV( LED_BIT_2 ) | _BV( LED_BIT_3 );
	
	while( 1 );
}
