/* ************************************************************************** **
** INCLUDES
** ************************************************************************** */
#include "common.h"			// Common MK20D7 definitions
#include "arm_cm4.h"		// ARM Cortex M4 helper module

/* ************************************************************************** **
** MACROS
** ************************************************************************** */
#define BLINK_INTERVAL_MS		( 500 )
#define BLINK_INTERVAL_PANIC_MS	( 50 )

// Helper macros to turn the Teensy's LED on and off
#define LED_ON					GPIOC_PSOR = ( 1<<5 )
#define LED_OFF					GPIOC_PCOR = ( 1<<5 )

/* ************************************************************************** **
** TYPEDEFS
** ************************************************************************** */

/* ************************************************************************** **
** LOCAL FUNCTION PROTOTYPES
** ************************************************************************** */
static inline void DumbDelayMillis( const uint32_t uiTimeoutMillis );

/* ************************************************************************** **
** LOCAL VARIABLES
** ************************************************************************** */

/* ************************************************************************** **
** API FUNCTIONS
** ************************************************************************** */

/* ************************************************************************** */
int main( void )
/**
 * @brief		Main entry point to the program. This function is linked from
 * 				sysinit.c implicitly by the linker.
 * @returns		Error code.. supposedly, but we should never return from here.
 */
/* -------------------------------------------------------------------------- */
{
	// Turn off interrupts before setup to avoid race conditions
	DisableInterrupts

	// Initialize on board LED
	PORTC_PCR5 = PORT_PCR_MUX( 0x1 );	// LED is on PC5 (pin 13), config as GPIO (alt = 1)
	GPIOC_PDDR = ( 1 << 5 );			// make this an output pin
	GPIOC_PCOR = ( 1 << 5 );			// start with LED off

	// Initialise bare metal USB driver
	//usb_init();

	// Enable interrupts for USB to work
	EnableInterrupts

	// Start blinking
	for ( ;; )
	{
		// Set LED
		LED_ON;
		DumbDelayMillis( BLINK_INTERVAL_MS );

		// Clear LED
		LED_OFF;
		DumbDelayMillis( BLINK_INTERVAL_MS );
	}

	// Should never get here - don't have to return anything!
}

/* ************************************************************************** */
void HardFault_Handler()
/**
 * @brief		Called by the system when a hard fault is encountered.
 * 				Flashes our led at 20hz indefinitely.
 * 				This function is linked from crt0.s implicitly by the linker.
 */
/* -------------------------------------------------------------------------- */
{
	// Initialize on board LED
	PORTC_PCR5 = PORT_PCR_MUX( 0x1 );	// LED is on PC5 (pin 13), config as GPIO (alt = 1)
	GPIOC_PDDR = ( 1 << 5 );			// Make an output

	// Do the "hard fault panic" dance forever
	for ( ;; )
	{
		// Turn on LED
		LED_ON;
		DumbDelayMillis( BLINK_INTERVAL_PANIC_MS );

		// Turn off LED
		LED_OFF;
		DumbDelayMillis( BLINK_INTERVAL_PANIC_MS );
	}

	// Never return
}

/* ************************************************************************** **
** LOCAL FUNCTIONS
** ************************************************************************** */

/* ************************************************************************** */
static inline void DumbDelayMillis( const uint32_t uiTimeoutMillis )
/**
 * @brief		Approximate delay using a for loop. Accuracy will be affected
 * 				by interrupts.
 * @param[in]	ms		Delay in ms.
 */
/* -------------------------------------------------------------------------- */
{
	const uint32_t uiCyclesPerLoop = 10;
	uint32_t uiLoops;
	uint32_t uiIndex;

	// Number of loops = timeout_ms / looptime_ms
	uiLoops = uiTimeoutMillis * ( (uint32_t)mcg_clk_hz / ( uiCyclesPerLoop * 1000 ) );

	// Dumb delay
	for ( uiIndex = 0; uiIndex < uiLoops; uiIndex++ );

	return;
}

