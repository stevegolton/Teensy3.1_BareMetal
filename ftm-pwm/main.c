/**
 * Example of PWM using FTM 0.
 *
 * Teensy3.1 Pinouts:
 * Rx = pin 0 (PTB16)
 * Tx = pin 1 (PTB17)
 */

#include "common.h"
#include "string.h"

/**
 * @brief		Approximate delay using a simple loop.
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

// FTM divider settings
#define FTM_FC_PS_DIV_1 0
#define FTM_FC_PS_DIV_2 1
#define FTM_FC_PS_DIV_4 2
#define FTM_FC_PS_DIV_8 3
#define FTM_FC_PS_DIV_16 4
#define FTM_FC_PS_DIV_32 5
#define FTM_FC_PS_DIV_64 6
#define FTM_FC_PS_DIV_128 7

/**
 * @brief		Initialize the FTM module 0 channel 0 for PWM.
 *
 * PWM is at 250Hz with a 25% duty cycle.
 */
static void init_ftm0( void )
{
	/* SIM_SCGC6: FTM0=1 - Enable FTM module clock (also set this otherwise hard fault... I don't know why!) */
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

	/* FTM0_MODE:FAULTIE=0,FAULTM=0,CAPTEST=0,PWMSYNC=0,WPDIS=1,INIT=0,FTMEN=0 */
	FTM0_MODE = (FTM_MODE_FAULTM(0x00) | FTM_MODE_WPDIS_MASK); /* Set up mode register */

	/* Basically reset the module completely */
	/* FTM0_SC: TOF=0,TOIE=0,CPWMS=0,CLKS=0,PS=0 */
	FTM0_SC = (FTM_SC_CLKS(0x00) | FTM_SC_PS(0x00)); /* Clear status and control register turning the module off */
	FTM0_CNTIN = FTM_CNTIN_INIT( 0x00 );	/* Clear initial counter register */

	FTM0_MOD = FTM_MOD_MOD( 48000 );		/* Set up modulo register for FTM0 */

	/* Set up channel 0 status and control register to be a PWM channel, edge aligned, starts high becomes low after match */
	FTM0_C0SC = ( FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK );

	/* Set up channel 1 value register */
	FTM0_C0V = FTM_CnV_VAL( 12000 );

	FTM0_OUTINIT = FTM_OUTINIT_CH0OI_MASK; /* Set up Initial State for Channel Output register */

	/* FTM0_MODE: FAULTIE=0,FAULTM=0,CAPTEST=0,PWMSYNC=0,WPDIS=1,INIT=1,FTMEN=0 */
	FTM0_MODE = (FTM_MODE_FAULTM(0x00) | FTM_MODE_WPDIS_MASK | FTM_MODE_INIT_MASK); /* Initialise the Output Channels */

	/* FTM0_SC: TOF=0,TOIE=0,CPWMS=0,CLKS=1,PS=0 */
	FTM0_SC = ( FTM_SC_CLKS( 0x01 ) | FTM_SC_PS( FTM_FC_PS_DIV_4 ) ); /* Set up status and control register */
}

/**
 * Blink the LED 3 times just to let the user know we have passed initialisation
 * and we are about to start doing stuff!
 */
void startup_blink( void )
{
	int idx;

	// Blink the LED 3 times just to let the user know we have passed initialisation and we are about to start doing stuff
	for (idx = 0; idx < 3; idx++) {
		// Set LED
		GPIOC_PSOR = (1 << 5);
		dumbdelay_ms(50);
		// Clear LED
		GPIOC_PCOR = (1 << 5);
		dumbdelay_ms(50);
	}
}

/**
** @brief		Entry point to the program.
*/
int main( void )
{


	// Configure PTC5 to option 1, as GPIOLED is on PC5 (pin 13), config as GPIO (alt = 1)
	PORTC_PCR5 = PORT_PCR_MUX( 0x1 );

	// Configure as general purpose output and clear it
	GPIOC_PDDR = ( 1 << 5 );
	GPIOC_PCOR = ( 1 << 5 );

	// Set up PORTC pin 1 (Teensy pin 22) as FTM0 output 0 (alt = 4).
	PORTC_PCR1 = PORT_PCR_MUX( 0x4 );

	// Initialize FTM0 channel 0 for pwm
	init_ftm0();

	startup_blink();

	// Spin in here forever
	for ( ;; )
	{
		// Set LED
		GPIOC_PSOR = ( 1 << 5 );
		FTM0_C0V = FTM_CnV_VAL( 36000 );
		dumbdelay_ms( 500 );

		// Clear LED
		GPIOC_PCOR = ( 1 << 5 );
		FTM0_C0V = FTM_CnV_VAL( 12000 );
		dumbdelay_ms( 500 );
	}

	// We definitely should never get here, this return is just to keep the
	// compiler happy
	return 0;
}
