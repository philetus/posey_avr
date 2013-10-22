/*
 * uart.h
 *
 * functions for uart transmit
 *
 */

/*
 * init uart for transmit 8N1 9600
 */
void init_uart( void );

int uart_put_char( char c );

void uart_put_PSTR( const char *addr );

void uart_put_int( uint8_t i );

void uart_put_hex( uint16_t i );
