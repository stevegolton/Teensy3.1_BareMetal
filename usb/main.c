#include "common.h"
#include "string.h"
#include "usb.h"

#define BLINK_INTERVAL_MS		(1000)

/**
 * @brief		Delay using a loop.
 * @param[in]	ms		Delay in ms.
 */
static __inline__ void dumbdelay_ms( const uint32_t ms )
{
	uint32_t loops;
	uint32_t index;

	// Calc delay in clock cycles
	loops = ms * ( (uint32_t)mcg_clk_hz / 10000 );

	// Dumb delay
	for ( index = 0; index < loops; index++ );
}

/**
 * @brief		Called by the system when a hard fault is encountered.
 * 				Flashes our led at 20hz indefinitely.
 */
void HardFault_Handler()
{
	for (;;)
	{
		// Do the "hard fault panic" dance
		GPIOC_PSOR = ( 1 << 5 );
		dumbdelay_ms( 20 );
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( 20 );
	}
}

/**
** @brief		Entry point to the program.
*/
int main( void )
{
	DisableInterrupts

	// Initialize on board LED
	PORTC_PCR5 = PORT_PCR_MUX( 0x1 );	// LED is on PC5 (pin 13), config as GPIO (alt = 1)
	GPIOC_PDDR = ( 1 << 5 );			// make this an output pin
	GPIOC_PCOR = ( 1 << 5 );			// start with LED off

	usb_init();

	EnableInterrupts

	// Start blinking
	for ( ;; )
	{
		// Set LED
		GPIOC_PSOR = ( 1 << 5 );
		dumbdelay_ms( BLINK_INTERVAL_MS );

		// Clear LED
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( BLINK_INTERVAL_MS );
	}

	// We definitely should never get here, this return is just to keep the
	// compiler happy
	return  0;
}
