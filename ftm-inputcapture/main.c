/**
 * Example of input capture using FTM 0 channel 0.
 *
 * For a decent demonstration I set up a PWM on FTM0CH0 and an input capture on
 * FTM0CH1 so that CH1 may sample the input duty cycle from CH0. In order to get
 * a handle on the output and to check that she is working properly, a serial
 * port has also been configured and the duty cycle is printed in milliseconds
 * to the terminal window.
 *
 * Teensy3.1 Pinouts:
 * UART Rx = Teensy pin 0 (PTB16)
 * UART Tx = Teensy pin 1 (PTB17)
 * FTM1CH0 = Teensy pin 22 (PTC1)
 * FTM1CH1 = Teensy pin 23 (PTC2)
 *
 * In order to test, Teensy pins 22 and 25 should be wired together so that the
 * input capture can actually capture something!
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
		// Do the "hard fault panic" dance
		GPIOC_PSOR = ( 1 << 5 );
		dumbdelay_ms( 20 );
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( 20 );
		dumbdelay_ms( 20 );
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( 20 );
	}
}

/**
 * @brief		Initialises the serial port module, baud rate=115200 8N1, hw
 * 				flow control disabled.
 * @param[in]	channel		UART module's base register pointer.
 */
void init_serial( UART_MemMapPtr channel )
{
	int baud = 115200;
	register uint16_t ubd, brfa;
	uint8_t temp;

	/* Enable the clock to UART0 */
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;

	/* Make sure that the transmitter and receiver are disabled while we
	* change settings.
	*/
	UART_C2_REG( channel ) &= ~( UART_C2_TE_MASK | UART_C2_RE_MASK );

	/* Configure the UART for 8-bit mode, no parity */
	/* We need all default settings, so entire register is cleared */
	UART_C1_REG( channel ) = 0;

	/* Calculate baud settings */
	ubd = (uint16_t)( ( mcg_clk_khz * 1000)/( baud * 16 ) );

	/* Save off the current value of the UARTx_BDH except for the SBR */
	temp = UART_BDH_REG( channel ) & ~( UART_BDH_SBR( 0x1F ) );
	UART_BDH_REG( channel ) = temp | UART_BDH_SBR( ( ( ubd & 0x1F00 ) >> 8 ) );
	UART_BDL_REG( channel ) = (uint8_t)( ubd & UART_BDL_SBR_MASK );

	/* Determine if a fractional divider is needed to get closer to the baud rate */
	brfa = ( ( ( mcg_clk_khz * 32000 ) / ( baud * 16 ) ) - ( ubd * 32 ) );

	/* Save off the current value of the UARTx_C4 register except for the BRFA */
	temp = UART_C4_REG( channel ) & ~( UART_C4_BRFA( 0x1F ) );
	UART_C4_REG( channel ) = temp | UART_C4_BRFA( brfa );

	/* Enable receiver and transmitter */
	UART_C2_REG( channel ) |= ( UART_C2_TE_MASK | UART_C2_RE_MASK );
}

/**
 * @brief		Get a character from the buffer.
 * @param[in]	channel		UART module's base register pointer.
 * @return		The character received from our FIFO.
 */
char uart_getchar( UART_MemMapPtr channel )
{
	/* Wait until character has been received */
	while (!(UART_S1_REG(channel) & UART_S1_RDRF_MASK));

	/* Return the 8-bit data from the receiver */
	return UART_D_REG(channel);
}

/**
 * @brief		Put a character into the tx buffer.
 * @param[in]	channel		UART module's base register pointer.
 * @param[in]	ch			Character to send.
 */
void uart_putchar( UART_MemMapPtr channel, char ch )
{
	/* Wait until space is available in the FIFO */
	while(!(UART_S1_REG(channel) & UART_S1_TDRE_MASK));

	/* Send the character */
	UART_D_REG(channel) = (uint8_t)ch;
}

/**
 * @brief		Put a character into the tx buffer.
 * @param[in]	channel		UART module's base register pointer.
 * @param[in]	ch			Character to send.
 */
void uart_puts( UART_MemMapPtr channel, char *s )
{
	int i;

	for ( i = 0; i < strlen( s ); i++ )
	{
		uart_putchar( channel, s[i] );
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
	FTM0_C0SC = ( FTM_CnSC_MSB_MASK | FTM_CnSC_ELSA_MASK );

	/* Set up channel 1 value register */
	FTM0_C0V = FTM_CnV_VAL( 24000 );

	// First we configure the channel 1 mode register by turning on the rising
	// edge, falling edge and interrupt enable modes
	FTM0_C1SC = ( FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK | FTM_CnSC_CHIE_MASK );

	FTM0_OUTINIT = FTM_OUTINIT_CH0OI_MASK; /* Set up Initial State for Channel Output register */

	/* FTM0_MODE: FAULTIE=0,FAULTM=0,CAPTEST=0,PWMSYNC=0,WPDIS=1,INIT=1,FTMEN=0 */
	FTM0_MODE = (FTM_MODE_FAULTM(0x00) | FTM_MODE_WPDIS_MASK | FTM_MODE_INIT_MASK); /* Initialise the Output Channels */

	// Enable interrupt in NVIC and set priority to 0 */
	NVICICPR1 |= ( 1 << 30 );
	NVICISER1 |= ( 1 << 30 );
	NVICIP62 = 0x00;

	/* FTM0_SC: TOF=0,TOIE=0,CPWMS=0,CLKS=1,PS=0 */
	FTM0_SC = ( FTM_SC_CLKS( 0x01 ) | FTM_SC_PS( FTM_FC_PS_DIV_128 ) | FTM_SC_TOIE_MASK ); /* Set up status and control register */
}

static uint16_t up = 0xFFFF;
static uint16_t down = 0xFFFF;

/**
 * @brief		Called on an FTM interrupt.
 *
 * This function is referenced by name in the vector table and so is called
 * under any FTM0 interrupt. This may include channel interrupts and overflow
 * interrupts.
 *
 */
void FTM0_IRQHandler( void )
{
	// First of all we need to tell which interrupt just happened by checking
	// the appropriate interrupt flag.
	if ( FTM0_C1SC & FTM_CnSC_CHF_MASK )
	{
		// We must clear the interrupt flag here
		FTM0_C1SC &= ~FTM_CnSC_CHF_MASK;

		// Check the state of out input pin and report over serial
		if ( GPIOC_PDIR & ( 0x1 << 2 ) )
		{
			// Rising edge
			up = FTM0_C1V;
			//uart_puts( UART0_BASE_PTR, "rise\n" );
		}
		else
		{
			// Falling edge
			down = FTM0_C1V;
			//uart_puts( UART0_BASE_PTR, "fall\n" );
		}
	}

	if ( FTM0_SC & FTM_SC_TOF_MASK )
	{
		FTM0_SC &= ~FTM_SC_TOF_MASK;
		//uart_puts( UART0_BASE_PTR, "of\n" );
	}
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
 * Prints an unsigned int to a string (decimal) without using sprintf/malloc.
 */
int printuint( char *buf, int maxlen, unsigned int input )
{
	int idx, len;
	unsigned int copy = input;

	// If 0 just put a 0
	if ( input == 0 )
	{
		if ( maxlen < 2 ) return 0;

		buf[0] = '0';
		buf[1] = 0;
		return 2;
	}

	// Work out how long in decimal characters our number is going to be
	len = 0;
	while ( copy > 0 )
	{
		copy /= 10;
		len++;
	}

	// Get out if we are too long
	if ( len >= maxlen ) return 0;

	// Stick a NULL at the end
	buf[len] = 0;

	// Copy off the index of the last character in our array and increment the
	// length to account for the final NULL character
	idx = len - 1;
	len++;

	// Write out the number backwards into the buffer
	while ( input > 0 )
	{
		buf[idx--] = '0' + ( input % 10 );
		input /= 10;
	}

	return len;
}

/**
** @brief		Entry point to the program.
*/
int main( void )
{
	char buf[32];
	int uptime = 0;

	// Configure PTC5 to option 1, as GPIOLED is on PC5 (pin 13), config as GPIO (alt = 1)
	PORTC_PCR5 = PORT_PCR_MUX( 0x1 );

	// Configure as general purpose output and clear it
	GPIOC_PDDR = ( 1 << 5 );
	GPIOC_PCOR = ( 1 << 5 );

	// Set up PORTC pin 1 (Teensy pin 22) as FTM0 output 0 (alt = 4).
	PORTC_PCR1 = PORT_PCR_MUX( 0x4 );
	PORTC_PCR2 = PORT_PCR_MUX( 0x4 );

	// Initialize FTM0 channel 0 for pwm
	init_ftm0();

	// Initialise serial port pins
	PORTB_PCR16 = PORT_PCR_MUX( 0x3 );
	PORTB_PCR17 = PORT_PCR_MUX( 0x3 );

	// Initialise UART module (UART0)
	init_serial( UART0_BASE_PTR );

	uart_puts( UART0_BASE_PTR, "FTM Example\n" );

	startup_blink();

	// Spin in here forever
	for ( ;; )
	{
		// Set LED
		GPIOC_PSOR = ( 1 << 5 );
		dumbdelay_ms( 500 );

		// Clear LED
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( 500 );

		//continue;

		// Print the results of the input capture
		printuint( buf, 32, uptime++ );
		uart_puts( UART0_BASE_PTR, buf );

		uart_puts( UART0_BASE_PTR, ": rising = " );
		printuint( buf, 32, up );
		uart_puts( UART0_BASE_PTR, buf );

		uart_puts( UART0_BASE_PTR, ", falling = " );
		printuint( buf, 32, down );
		uart_puts( UART0_BASE_PTR, buf );

		uart_puts( UART0_BASE_PTR, "\n" );
	}

	// We definitely should never get here, this return is just to keep the
	// compiler happy
	return 0;
}
