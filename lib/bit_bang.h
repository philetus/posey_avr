/*
 * bit_bang.h
 *
 * functions to read and write serial data packets from multiple pins of 
 * avr in parallel
 *
 */

struct pin_data
{
	volatile uint8_t *ddr;
	volatile uint8_t *port;
	volatile uint8_t *pin;
	uint8_t bit;
};

struct data_packet
{
	uint8_t length;	/* length of packet */
	uint8_t new;	/* flag unset after packet is sent / set after read */
	uint8_t *data;	/* pointer to packet data */
};

/*
 * once a packet is read from an rx pin it is written to the rx packets table
 * and the new data flag for that packet is set
 *
 * any packets read from that pin subsequently will be discarded until the new
 * data flag is cleared
 */
struct data_packet *rx_packets;

/*
 * to transmit a data packet the data and the length of the packet should
 * be set and then the new data flag should be set
 *
 * once the data is copied into the appropriate pin's write buffer for sending
 * the new data flag will be cleared to signal that it is safe to write more
 * data
 *
 * changing the length or data of the packet while the new data flag is set
 * may lead to *BADNESS*
 */
struct data_packet *tx_packets;

/*
 * set up pins to read from or write to
 */
void init_rx_pins( struct pin_data rx_pins[], uint8_t pin_count );
void init_tx_pins( struct pin_data tx_pins[], uint8_t pin_count );

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
void init_bit_bang_timer( uint16_t tics, uint8_t prescaler );
