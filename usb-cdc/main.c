/* ************************************************************************** **
** INCLUDES
** ************************************************************************** */
#include "common.h"			// Common MK20D7 definitions
#include "arm_cm4.h"		// ARM Cortex M4 helper module
#include "virtual_com.h"

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
static void USB_Init( void );

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
	DisableInterrupts

	// Initialize on board LED
	PORTC_PCR5 = PORT_PCR_MUX( 0x1 );	// LED is on PC5 (pin 13), config as GPIO (alt = 1)
	GPIOC_PDDR = ( 1 << 5 );			// make this an output pin
	GPIOC_PCOR = ( 1 << 5 );			// start with LED off

	// Init low level registers for the USB device
	USB_Init();

	// Initialise virtual comm app
	TestApp_Init();

	//EnableInterrupts

	// Start blinking
	for ( ;; )
	{
		//TestApp_Task();

#if 0
		// Set LED
		LED_ON;
		DumbDelayMillis( BLINK_INTERVAL_MS );

		// Clear LED
		LED_OFF;
		DumbDelayMillis( BLINK_INTERVAL_MS );
#endif
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

/*****************************************************************************
 *
 *    @name     USB_Init
 *
 *    @brief    This function Initializes USB module
 *
 *    @param    None
 *
 *    @return   None
 *
 ***************************************************************************/
static void USB_Init( void )
{
	//1: Select clock source
	SIM_SOPT2 |= SIM_SOPT2_USBSRC_MASK | SIM_SOPT2_PLLFLLSEL_MASK; //we use MCGPLLCLK divided by USB fractional divider
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(0);// SIM_CLKDIV2_USBDIV(2) | SIM_CLKDIV2_USBFRAC_MASK; //(USBFRAC + 1)/(USBDIV + 1) = (1 + 1)/(2 + 1) = 2/3 for 72Mhz clock

	//2: Gate USB clock
	SIM_SCGC4 |= SIM_SCGC4_USBOTG_MASK;

	//3: Software USB module reset
	USB0_USBTRC0 |= USB_USBTRC0_USBRESET_MASK;
	while (USB0_USBTRC0 & USB_USBTRC0_USBRESET_MASK);

	//5: Clear all ISR flags and enable weak pull downs
	USB0_ISTAT = 0xFF;
	USB0_ERRSTAT = 0xFF;
	USB0_OTGISTAT = 0xFF;
	USB0_USBTRC0 |= 0x40; //a hint was given that this is an undocumented interrupt bit

	//6: Enable USB reset interrupt
	USB0_CTL = USB_CTL_USBENSOFEN_MASK;
	USB0_USBCTRL = 0;

	USB0_INTEN |= USB_INTEN_USBRSTEN_MASK;
	//NVIC_SET_PRIORITY(IRQ(INT_USB0), 112);
	enable_irq(IRQ(INT_USB0));

	//7: Enable pull-up resistor on D+ (Full speed, 12Mbit/s)
	USB0_CONTROL = USB_CONTROL_DPPULLUPNONOTG_MASK;
}

