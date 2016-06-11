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
#include <stdbool.h>

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

/**
 * Prints an unsigned int to a string (decimal) without using sprintf/malloc.
 *
 * @param[in]	buf		Character buffer to write the number out to.
 * @param[in]	maxlen	Max length of the buffer - ensures we dont write off the end.
 * @param[in]	input	The number to write out.
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

// I2C addresses
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

#define WHOAMI_G_RESPONSE	0xD4
#define WHOAMI_XM_RESPONSE	0x49


////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define WHO_AM_I_G			0x0F
#define CTRL_REG1_G			0x20
#define CTRL_REG2_G			0x21
#define CTRL_REG3_G			0x22
#define CTRL_REG4_G			0x23
#define CTRL_REG5_G			0x24
#define REFERENCE_G			0x25
#define STATUS_REG_G		0x27
#define OUT_X_L_G			0x28
#define OUT_X_H_G			0x29
#define OUT_Y_L_G			0x2A
#define OUT_Y_H_G			0x2B
#define OUT_Z_L_G			0x2C
#define OUT_Z_H_G			0x2D
#define FIFO_CTRL_REG_G		0x2E
#define FIFO_SRC_REG_G		0x2F
#define INT1_CFG_G			0x30
#define INT1_SRC_G			0x31
#define INT1_THS_XH_G		0x32
#define INT1_THS_XL_G		0x33
#define INT1_THS_YH_G		0x34
#define INT1_THS_YL_G		0x35
#define INT1_THS_ZH_G		0x36
#define INT1_THS_ZL_G		0x37
#define INT1_DURATION_G		0x38

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define OUT_TEMP_L_XM		0x05
#define OUT_TEMP_H_XM		0x06
#define STATUS_REG_M		0x07
#define OUT_X_L_M			0x08
#define OUT_X_H_M			0x09
#define OUT_Y_L_M			0x0A
#define OUT_Y_H_M			0x0B
#define OUT_Z_L_M			0x0C
#define OUT_Z_H_M			0x0D
#define WHO_AM_I_XM			0x0F
#define INT_CTRL_REG_M		0x12
#define INT_SRC_REG_M		0x13
#define INT_THS_L_M			0x14
#define INT_THS_H_M			0x15
#define OFFSET_X_L_M		0x16
#define OFFSET_X_H_M		0x17
#define OFFSET_Y_L_M		0x18
#define OFFSET_Y_H_M		0x19
#define OFFSET_Z_L_M		0x1A
#define OFFSET_Z_H_M		0x1B
#define REFERENCE_X			0x1C
#define REFERENCE_Y			0x1D
#define REFERENCE_Z			0x1E
#define CTRL_REG0_XM		0x1F
#define CTRL_REG1_XM		0x20
#define CTRL_REG2_XM		0x21
#define CTRL_REG3_XM		0x22
#define CTRL_REG4_XM		0x23
#define CTRL_REG5_XM		0x24
#define CTRL_REG6_XM		0x25
#define CTRL_REG7_XM		0x26
#define STATUS_REG_A		0x27
#define OUT_X_L_A			0x28
#define OUT_X_H_A			0x29
#define OUT_Y_L_A			0x2A
#define OUT_Y_H_A			0x2B
#define OUT_Z_L_A			0x2C
#define OUT_Z_H_A			0x2D
#define FIFO_CTRL_REG		0x2E
#define FIFO_SRC_REG		0x2F
#define INT_GEN_1_REG		0x30
#define INT_GEN_1_SRC		0x31
#define INT_GEN_1_THS		0x32
#define INT_GEN_1_DURATION	0x33
#define INT_GEN_2_REG		0x34
#define INT_GEN_2_SRC		0x35
#define INT_GEN_2_THS		0x36
#define INT_GEN_2_DURATION	0x37
#define CLICK_CFG			0x38
#define CLICK_SRC			0x39
#define CLICK_THS			0x3A
#define TIME_LIMIT			0x3B
#define TIME_LATENCY		0x3C
#define TIME_WINDOW			0x3D
#define ACT_THS				0x3E
#define ACT_DUR				0x3F

/**
** @brief		Entry point to the program.
*/
int main( void )
{
	int idx;
	char data;
	uint32_t status;


	uint8_t checkbyte = 0;        /* will contain the device id after sequence has been processed */
	char buf[16];

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

	if ( -1 != status )
	{
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

		uart_puts( UART0_BASE_PTR, "I2C module initialized OK, reading whoami bytes from the LSM9DS0\r\n" );

		/* Read WHO_AM_I_G from LSM9DS0_G */
		i2c_read_byte( 0, LSM9DS0_G, WHO_AM_I_G, &checkbyte );

		printuint( buf, 16, checkbyte );
		uart_puts( UART0_BASE_PTR, "Checkbyte = " );
		uart_puts( UART0_BASE_PTR, buf );
		uart_puts( UART0_BASE_PTR, "\r\n" );

		/* Read WHO_AM_I_XM from LSM9DS0_XM */
		i2c_read_byte( 0, LSM9DS0_XM, WHO_AM_I_XM, &checkbyte );

		printuint( buf, 16, checkbyte );
		uart_puts( UART0_BASE_PTR, "Checkbyte = " );
		uart_puts( UART0_BASE_PTR, buf );
		uart_puts( UART0_BASE_PTR, "\r\n" );
	}
	else
	{
		// Startup blink
		for ( ; ; )
		{
			// Set LED
			GPIOC_PSOR = ( 1 << 5 );
			dumbdelay_ms( 200 );

			// Clear LED
			GPIOC_PCOR = ( 1 << 5 );
			dumbdelay_ms( 200 );
		}
	}

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
