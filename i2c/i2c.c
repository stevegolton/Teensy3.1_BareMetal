/*
  Copyright (C) 2014, 2015 Jan Rychter

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"
#include "common.h"
#include "string.h"

/* Pointer to the base of our I2C device's memory address */
static I2C_MemMapPtr i2c_base_ptrs[] = I2C_BASE_PTRS;

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

static int printuint( char *buf, int maxlen, unsigned int input )
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

/* Some local metadata about the channels */
volatile I2C_Channel i2c_channels[I2C_NUMBER_OF_DEVICES];

/**
 * Initializes the i2c module.
 * @param[in]	Index of the I2C module to use.
 * @param[in]	Frequency multiplier to use.
 * @param[in]	Frequency multiplier to use.
 */
uint32_t i2c_init( uint8_t i2c_number, uint8_t mult, uint8_t icr )
{
	I2C_MemMapPtr i2c;

	/* Make a stack pointer to the device we are initialising for convenience */
	i2c = i2c_base_ptrs[i2c_number];

	/* Initialise control register - Enable interrupt for timing signals */
	i2c->C1 = 0;
	i2c->C1 |= I2C_C1_IICEN_MASK;

	/* Initialise frequency register */
	i2c->F &= ~0xf;
	i2c->F |= ( (mult << I2C_F_MULT_SHIFT) | icr );

	return i2c_number;
}

/** Send a sequence of bytes
 *
 */
int32_t i2c_send_sequence( uint32_t channel_number,
						   uint16_t *sequence,
						   uint32_t sequence_length,
						   uint8_t *received_data,
						   void (*callback_fn)(void*),
						   void *user_data )
{
	volatile I2C_Channel *channel = &(i2c_channels[channel_number]);
	I2C_MemMapPtr i2c = i2c_base_ptrs[channel_number];

	if ( channel->status == I2C_BUSY )
	{
		return -1;
	}

	/* reads_ahead does not need to be initialised .. TODO Why? */
	channel->sequence = sequence;
	channel->sequence_end = sequence + sequence_length;
	channel->received_data = received_data;
	channel->status = I2C_BUSY;
	channel->txrx = I2C_WRITING;
	channel->callback_fn = callback_fn;
	channel->user_data = user_data;

	i2c->S |= I2C_S_IICIF_MASK; /* Acknowledge the interrupt request, just in case */
	i2c->C1 = (I2C_C1_IICEN_MASK | I2C_C1_IICIE_MASK);

	/* Generate a start condition and prepare for transmitting. */
	i2c->C1 |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);

	/* Check status register to see if arbitration has worked */
	if( i2c->S & I2C_S_ARBL_MASK )
	{
		/* Error, send sequence cleanup */
		i2c->C1 &= ~(I2C_C1_IICIE_MASK | I2C_C1_MST_MASK | I2C_C1_TX_MASK);
		channel->status = I2C_ERROR;

		return -1;
	}
	else
	{
		/* Success, write the first (address) byte. */
		i2c->D = *channel->sequence++;
	}

	return 0;
}

static bool complete_flag = false;

void my_callback_from_ISR(void *data)
{
	/* This callback function gets called once the sequence has been processed. Note that this gets called from an ISR, so
	   it should do as little as possible. */
	complete_flag = true;
}

int i2c_read_byte( const uint32_t channel_number,
				   const uint8_t device,
				   const uint8_t addr,
				   const uint8_t *data )
{
	uint32_t status;
	uint16_t init_sequence[] = { ( device << 1 ) | I2C_WRITING, addr, I2C_RESTART, ( device << 1 ) | I2C_READING, I2C_READ };
	//uint8_t data = 0;        /* will contain the device id after sequence has been processed */

	complete_flag = false;

	status = i2c_send_sequence( channel_number, init_sequence, 5, data, my_callback_from_ISR, (void*)0x1234 );

	/* Block until the I2C transaction has completed */
	while ( false == complete_flag );

	return status;
}

void I2C0_IRQHandler( void )
{
	volatile I2C_Channel* channel;
	I2C_MemMapPtr i2c;
	uint8_t channel_number;
	uint16_t element;
	uint8_t status;
	char buf[16];

#ifdef ERRATA_1N96F_WORKAROUND
	uint8_t f_register;
#endif

	/* Loop over I2C modules. For the most common use case where I2C_NUMBER_OF_DEVICES == 1, the compiler should optimize
	** this loop out entirely. */
	for ( channel_number = 0; channel_number < I2C_NUMBER_OF_DEVICES; channel_number++ )
	{
		channel = &i2c_channels[channel_number];
		i2c = (I2C_MemMapPtr)i2c_base_ptrs[channel_number];

		//uart_puts( UART0_BASE_PTR, "ISR" );
		//printuint( buf, 16, *channel->sequence );
		//uart_puts( UART0_BASE_PTR, buf );
		//uart_puts( UART0_BASE_PTR, "\r\n" );

		status = i2c->S;

		/* Was the interrupt request from the current I2C module? */
		if( !( status & I2C_S_IICIF_MASK ) )
		{
			continue;                 /* If not, proceed to the next I2C module. */
		}

		i2c->S |= I2C_S_IICIF_MASK; /* Acknowledge the interrupt request. */

		if ( status & I2C_S_ARBL_MASK )
		{
			i2c->S |= I2C_S_ARBL_MASK;
			goto i2c_isr_error;
		}

		if( channel->txrx == I2C_READING )
		{
			switch( channel->reads_ahead )
			{
				case 0:
				{
					/* All the reads in the sequence have been processed (but note that the final data register read still needs to
					   be done below! Now, the next thing is either a restart or the end of a sequence. In any case, we need to
					   switch to TX mode, either to generate a repeated start condition, or to avoid triggering another I2C read
					   when reading the contents of the data register. */
					i2c->C1 |= I2C_C1_TX_MASK;

					/* Perform the final data register read now that it's safe to do so. */
					*channel->received_data++ = i2c->D;

					/* Do we have a repeated start? */
					if ( ( channel->sequence < channel->sequence_end ) && ( *channel->sequence == I2C_RESTART ) )
					{

						/* Issue 6070: I2C: Repeat start cannot be generated if the I2Cx_F[MULT] field is set to a non-zero value. */
		#ifdef ERRATA_1N96F_WORKAROUND
						f_register = i2c->F(i2c_ptr);
						i2c->F = (f_register & 0x3f); /* Zero out the MULT bits (topmost 2 bits). */
		#endif

						i2c->C1 |= I2C_C1_RSTA_MASK; /* Generate a repeated start condition. */

		#ifdef ERRATA_1N96F_WORKAROUND
						i2c->F = f_register;
		#endif
						/* A restart is processed immediately, so we need to get a new element from our sequence. This is safe, because
						 a sequence cannot end with a RESTART: there has to be something after it. Note that the only thing that can
						 come after a restart is an address write. */
						channel->txrx = I2C_WRITING;
						channel->sequence++;
						element = *channel->sequence;
						i2c->D = element;
					}
					else
					{
						goto i2c_isr_stop;
					}
				}
				break;

				case 1:
				{
					i2c->C1 |= I2C_C1_TXAK_MASK; /* do not ACK the final read */
					*channel->received_data++ = i2c->D;
				}
				break;

				default:
				{
					*channel->received_data++ = i2c->D;
				}
				break;
			}

			channel->reads_ahead--;

		}
		else
		{
			/* channel->txrx == I2C_WRITING */
			/* First, check if we are at the end of a sequence. */
			if ( channel->sequence == channel->sequence_end )
			{
				goto i2c_isr_stop;
			}

			if ( status & I2C_S_RXAK_MASK )
			{
				/* We received a NACK. Generate a STOP condition and abort. */
				goto i2c_isr_error;
			}

			/* check next thing in our sequence */
			element = *channel->sequence;

			if ( element == I2C_RESTART )
			{
				/* Do we have a restart? If so, generate repeated start and make sure TX is on. */
				i2c->C1 |= I2C_C1_RSTA_MASK | I2C_C1_TX_MASK;

				/* A restart is processed immediately, so we need to get a new element from our sequence. This is safe, because a
				sequence cannot end with a RESTART: there has to be something after it. */
				channel->sequence++;
				element = *channel->sequence;

				/* Note that the only thing that can come after a restart is a write. */
				i2c->D = element;
			}
			else
			{
				if ( element == I2C_READ )
				{
					channel->txrx = I2C_READING;
					/* How many reads do we have ahead of us (not including this one)? For reads we need to know the segment length
					to correctly plan NACK transmissions. */
					channel->reads_ahead = 1;        /* We already know about one read */
					while ( ( ( channel->sequence + channel->reads_ahead ) < channel->sequence_end ) &&
							(*(channel->sequence + channel->reads_ahead) == I2C_READ))
					{
						channel->reads_ahead++;
					}
					i2c->C1 &= ~I2C_C1_TX_MASK; /* Switch to RX mode. */

					if ( channel->reads_ahead == 1 )
					{
						i2c->C1 |= I2C_C1_TXAK_MASK; /* do not ACK the final read */
					}
					else
					{
						i2c->C1 &= ~(I2C_C1_TXAK_MASK);  /* ACK all but the final read */
					}

					/* Dummy read comes first, note that this is not valid data! This only triggers a read, actual data will come
					in the next interrupt call and overwrite this. This is why we do not increment the received_data
					pointer. */
					*channel->received_data = i2c->D;
					channel->reads_ahead--;
				}
				else
				{
					/* Not a restart, not a read, must be a write. */
					i2c->D = element;
				}
			}
		}

		channel->sequence++;
		continue;

i2c_isr_stop:
		/* Generate STOP (set MST=0), switch to RX mode, and disable further interrupts. */
		i2c->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_IICIE_MASK | I2C_C1_TXAK_MASK);
		channel->status = I2C_AVAILABLE;

		/* Call the user-supplied callback function upon successful completion (if it exists). */
		if ( NULL != channel->callback_fn )
		{
			(*channel->callback_fn)( channel->user_data );
		}
		continue;

i2c_isr_error:
		i2c->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_IICIE_MASK); /* Generate STOP and disable further interrupts. */
		channel->status = I2C_ERROR;
		continue;
	}
}
