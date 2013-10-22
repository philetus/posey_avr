/*
 * defines.h
 *
 * parameter definitions for flexy avr project
 */

#define TRUE 1
#define FALSE 0

/* #define DEBUG_ON TRUE */
#ifdef DEBUG_ON
#define DEBUG(x) x
#else
#define DEBUG(x)
#endif

/* #define DEBUG_INFO_ON TRUE */
#ifdef DEBUG_INFO_ON
#define DEBUG_INFO(x) x
#else
#define DEBUG_INFO(x)
#endif

/* #define DEBUG_WARN_ON TRUE */
#ifdef DEBUG_WARN_ON
#define DEBUG_WARN(x) x
#else
#define DEBUG_WARN(x)
#endif

#define F_CPU 1000000UL /* avr on-chip oscillator speed is 1mhz */
#define BAUD_RATE 9600UL

#define TICS_PER_INTERRUPT 3000UL /* clock tics before timer interrupt */
#define PRESCALER 0 /* 0 - none / 1 - 1/8 prescaler / 2 - 1/64 prescaler */

#define ADDRESS_SIZE 3 /* length of hub and strut addresses */
