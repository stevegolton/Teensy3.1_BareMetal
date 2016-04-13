/**
 * Example using the I2C bus as master - interrupt driven implementation.
 * This example also sets up the uart and prints debugging to it.
 *
 * Teensy3.1 Pinouts:
 * I2C SCL = pin 19 (PTB2)
 * I2C SDA = pin 20 (PTB3)
 * UART Rx = pin 0 (PTB16)
 * UART Tx = pin 1 (PTB17)
 */

#include "common.h"
#include "string.h"
#include "i2c.h"

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

/**100
 * @brief		Put a character into the tx buffer.
 * @param[in]	channel		UART module's base register pointer.
 * @param[in]	ch			Character to send.
 */
static void uart_putchar( UART_MemMapPtr channel, char ch )
{
	/* Wait until space is available in the FIFO */
	while(!(UART_S1_REG(channel) & UART_S1_TDRE_MASK));

	/* Send the character */
	UART_D_REG(channel) = (uint8_t)ch;
}

/**
 * @brief		Put a string into the tx buffer.
 * @param[in]	channel		UART module's base register pointer.
 * @param[in]	ch			Characters to send.
 */
static void uart_puts( UART_MemMapPtr channel, char *s )
{
	int i;

	for ( i = 0; i < strlen( s ); i++ )
	{
		uart_putchar( channel, s[i] );
	}
}

void my_callback(void *data)
{
	/* This callback function gets called once the sequence has been processed. Note that this gets called from an ISR, so
	   it should do as little as possible. */

	uart_puts( UART0_BASE_PTR, "Callback called...\r\n" );
}

#if 0
void I2C0_IRQHandler( void )
{
	if( I2C0_BASE_PTR->S & I2C_S_IICIF_MASK )
	{
		I2C0_BASE_PTR->S |= I2C_S_IICIF_MASK; /* Acknowledge the interrupt request. */
	}
}
#endif

/**
** @brief		Entry point to the program.
*/
int main( void )
{
	int idx;
	char data;
	uint32_t status;
	uint16_t init_sequence[] = {0x3a, 0x0d, I2C_RESTART, 0x3b, I2C_READ};
	uint8_t device_id = 0;        /* will contain the device id after sequence has been processed */

	// Configure PTC5 to option 1, as GPIOLED is on PC5 (pin 13), config as GPIO (alt = 1)
	PORTC_PCR5 = PORT_PCR_MUX( 0x1 );

	// Configure as general purpose output and clear it
	GPIOC_PDDR = ( 1 << 5 );
	GPIOC_PCOR = ( 1 << 5 );

	// Initialise serial port pins
	PORTB_PCR16 = PORT_PCR_MUX( 0x3 );
	PORTB_PCR17 = PORT_PCR_MUX( 0x3 );

	// Initialise UART module (UART0)
	init_serial( UART0_BASE_PTR );

	// Initialize i2c module
	//enable_irq();

	// Enable interrupt in NVIC and set priority to 0
	// See the vector channel assignments in K20P121M100SF2RM.pdf page 69
	// The interupt for I2C0 is number 79:
	// 24 / 32 = 0
	// 24 % 32 = 24
	// Therefore we choose the 0th register and set bit 24 (INT_I2C0 - 16)
	NVICICPR0 |= ( 1 << 24 );
	NVICISER0 |= ( 1 << 24 );
	NVICIP24 = 0x00;

	SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

	PORTB_PCR2 = PORT_PCR_MUX(0x02) | PORT_PCR_ODE_MASK;
	PORTB_PCR3 = PORT_PCR_MUX(0x02) | PORT_PCR_ODE_MASK;

	status = i2c_init( 0, 0x01, 0x20 );

	// Startup blink
	for ( idx = 0; idx < 3; idx++ )
	{
		// Set LED
		GPIOC_PSOR = ( 1 << 5 );
		dumbdelay_ms( 50 );

		// Clear LED
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( 50 );
	}

	uart_puts( UART0_BASE_PTR, "Trying some I2C...\r\n" );
	status = i2c_send_sequence( 0, init_sequence, 5, &device_id, my_callback, (void*)0x1234 );

	// Receive characters from the UART and echo back to the sender
	for ( ;; )
	{
		data = uart_getchar( UART0_BASE_PTR );
		uart_putchar( UART0_BASE_PTR, data );
	}

	// We definitely should never get here, this return is just to keep the
	// compiler happy
	return 0;
}
