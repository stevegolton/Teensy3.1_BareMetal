/**
 * Example of a polling UART implementation. This example simply echos back any
 * characters it receives on UART0.
 *
 * Teensy3.1 Pinouts:
 * Rx = pin 0 (PTB16)
 * Tx = pin 1 (PTB17)
 */

#include "common.h"
#include "string.h"
#include "esp.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "timers.h"

// Define me if you want debugging, remove me for release!
//#define configASSERT( x )     if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

static __inline__ void dumbdelay_ms( const uint32_t ms );
static void exampleTaskHandler( void *pvParameters );
static void timerCallback( TimerHandle_t xTimer );

static TimerHandle_t exampleTimer = NULL;
static int ledstate = 0;

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
void uart_putchar( UART_MemMapPtr channel, char ch )
{
	/* Wait until space is available in the FIFO */
	while(!(UART_S1_REG(channel) & UART_S1_TDRE_MASK));

	/* Send the character */
	UART_D_REG(channel) = (uint8_t)ch;
}

int esp_getc(esp_t *ctx)
{
	return uart_getchar(UART0_BASE_PTR);
}

int esp_write_out(esp_t *ctx, char *buf, int len)
{
	int i;
	for(i = 0; i < len; i++)
	{
		uart_putchar( UART0_BASE_PTR, buf[i] );
	}

	return len;
}

char out[2048];
esp_t esp;

/**
** @brief		Entry point to the program.
*/
int main( void )
{
	int idx;
	char data;

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

	// Create a timer with a period of 500ms
	exampleTimer = xTimerCreate( "exampleTimer", 					/* A text name, purely to help debugging. */
								 ( 500UL / portTICK_PERIOD_MS ),	/* The timer period, in this case 500ms. */
								 pdTRUE,							/* We want this to be a recurring timer so set uxAutoReload to pdTRUE. */
								 ( void * ) 0,						/* The ID is not used, so can be set to anything. */
								 timerCallback						/* The callback function that is called when the timer expires. */
							);

	// Create a task
	xTaskCreate( exampleTaskHandler,			// The task's callback function
				 "Task",						// Task name
				 configMINIMAL_STACK_SIZE,		// We can specify different stack sizes for each task? Cool!
				 NULL,							// Parameter to pass to the callback function, we have nothhing to pass..
				 0,								// Priority, this is our only task so.. lets just use 0
				 NULL );						// We could put a pointer to a task handle here which will be filled in when the task is created

	// Start the tasks and timer running, this should never return freertos will
	// branch directly into the idle task.
	vTaskStartScheduler();

	// We definitely should never get here, this return is just to keep the
	// compiler happy
	return 0;
}

void *esp_malloc(int size)
{
	return pvPortMalloc(size);
}

void esp_free(void *ptr)
{
	vPortFree(ptr);
}

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

#define MSG "Hello from Mr Quadcopter!\n\n"

/**
 * @brief		Example task, simply starts our timer and loops forever.
 */
static void exampleTaskHandler( void *pvParameters )
{
	// Start the timer
	xTimerStart( exampleTimer, 0 );

	esp_create(&esp, esp_getc, esp_write_out);
#if 1


	esp_reset(&esp);

#define STATION_MODE

#ifdef STATION_MODE
	esp_ap_mode(&esp, esp_ap_mode_station);
	esp_ap_disconnect(&esp);
	if(0 != esp_ap_join(&esp, "ssid", "pass"))
	{
		//return EXIT_FAILURE;
	}
#else
	// SoftAP mode
	puts("Configuring ESP as a WiFi AP");
	esp_ap_mode(&esp, esp_ap_mode_ap);

	puts("Configuring soft ap");
	if(0 != esp_ap_configure(&esp, "ESP8266", "password", 9, esp_softap_mode_wpa2_psk))
	{
		return EXIT_FAILURE;
	}
#endif

	// Enable muxing
	esp_cipmux(&esp, 1);

	// Bind to port 8080
	esp_bind(&esp, 80);

	// Wait for connection
	for ( ;; )
	{
		int fd = esp_accept(&esp);

		esp_read(&esp, fd, out, 2048);
		esp_write(&esp, fd, MSG, sizeof(MSG));
		esp_close(&esp, fd);
	}

#endif

	for(;;)
	{
		//
	}
}

/**
 * @brief		Callback for our timer, toggles the led.
 * @param[in]	xTimer		Timer handle.
 */
static void timerCallback( TimerHandle_t xTimer )
{
	if ( 1 == ledstate )
	{
		ledstate = 0;
		GPIOC_PCOR = ( 1 << 5 );
	}
	else
	{
		ledstate = 1;
		GPIOC_PSOR = ( 1 << 5 );
	}
}

/*!
 * \brief	Called by the system when a hard fault is encountered.
 * 			Flashes our led at 20hz indefinitely.
 */
void HardFault_Handler()
{
	for (;;)
	{
		// Do the "hard fault panic" dance
		GPIOC_PSOR = ( 1 << 5 );
		dumbdelay_ms( 50 );
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( 50 );
	}
}

/**
 * @brief		If enabled, this hook will be called in case of a stack
 * 				overflow.
 * @param[in]	pxTask		Task handle.
 * @param[in]	pcTaskName	Pointer to task name.
 */
void vApplicationStackOverflowHook( xTaskHandle pxTask, char *pcTaskName )
{
	/* This will get called if a stack overflow is detected during the context
	 switch.  Set configCHECK_FOR_STACK_OVERFLOWS to 2 to also check for stack
	 problems within nested interrupts, but only do this for debug purposes as
	 it will increase the context switch time. */
	(void)pxTask;
	(void)pcTaskName;
	taskDISABLE_INTERRUPTS();
	/* Write your code here ... */
	for(;;) {}
}

/**
 * @brief		If enabled, this hook will be called by the RTOS for every
 *				tick increment.
 */
void vApplicationTickHook( void )
{
	/* Called for every RTOS tick. */
	/* Write your code here ... */
}

/**
 * @brief		If enabled, this hook will be called when the RTOS is idle.
 *				This might be a good place to go into low power mode.
 */
void vApplicationIdleHook( void )
{
	/* Called whenever the RTOS is idle (from the IDLE task).
	 Here would be a good place to put the CPU into low power mode. */
	/* Write your code here ... */
}

/**
 * @brief		If enabled, the RTOS will call this hook in case memory
 *				allocation failed.
 */
void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	 free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	 internally by FreeRTOS API functions that create tasks, queues, software
	 timers, and semaphores.  The size of the FreeRTOS heap is set by the
	 configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	taskDISABLE_INTERRUPTS();
	/* Write your code here ... */
	for(;;) {}
}
