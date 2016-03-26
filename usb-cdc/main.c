#include "common.h"
#include "string.h"

#include "usb_cdc.h"		/* USB CDC Class Header File */
#include "usb_dciapi.h"
#include "virtual_com.h"
#include "wdt_kinetis.h"
#include "hidef.h"

// FREESCALE USB CRAP... TODO filter this
/******************************************************************************/

#if (defined MCU_MK20D7) || (defined MCU_MK40D7)
    #define MCGOUTCLK_72_MHZ
#endif

#if (defined MCU_MK60N512VMD100) || (defined MCU_MK53N512CMD100)
	#define BSP_CLOCK_SRC   (50000000ul)    /* crystal, oscillator freq. */
#elif (defined MCU_MK40N512VMD100)
   #if (defined KWIKSTIK)
    #define BSP_CLOCK_SRC   (4000000ul)     /* crystal, oscillator freq. */
   #else
	#define BSP_CLOCK_SRC   (8000000ul)     /* crystal, oscillator freq. */
   #endif
#else
    #define BSP_CLOCK_SRC   (8000000ul)     /* crystal, oscillator freq. */
#endif
#define BSP_REF_CLOCK_SRC   (2000000ul)     /* must be 2-4MHz */


#ifndef MCGOUTCLK_72_MHZ
#define PLL_48              (1)
#define PLL_96              (0)
#else
#define PLL_48              (0)
#define PLL_96              (0)
#endif

#ifdef MCGOUTCLK_72_MHZ
	#define BSP_CORE_DIV    (1)
	#define BSP_BUS_DIV     (2)
	#define BSP_FLEXBUS_DIV (2)
	#define BSP_FLASH_DIV   (3)

	// BSP_CLOCK_MUL from interval 24 - 55
	#define BSP_CLOCK_MUL   (36)    // 72MHz
#elif PLL_48
	#define BSP_CORE_DIV    (1)
	#define BSP_BUS_DIV     (1)
	#define BSP_FLEXBUS_DIV (1)
	#define BSP_FLASH_DIV   (2)

	// BSP_CLOCK_MUL from interval 24 - 55
	#define BSP_CLOCK_MUL   (24)    // 48MHz
#elif PLL_96
    #define BSP_CORE_DIV    (1)
    #define BSP_BUS_DIV     (2)
    #define BSP_FLEXBUS_DIV (2)
    #define BSP_FLASH_DIV   (4)
    // BSP_CLOCK_MUL from interval 24 - 55
    #define BSP_CLOCK_MUL   (48)    // 96MHz
#endif

#define BSP_REF_CLOCK_DIV   (BSP_CLOCK_SRC / BSP_REF_CLOCK_SRC)

#define BSP_CLOCK           (BSP_REF_CLOCK_SRC * BSP_CLOCK_MUL)
#define BSP_CORE_CLOCK      (BSP_CLOCK / BSP_CORE_DIV)      /* CORE CLK, max 100MHz */
#define BSP_SYSTEM_CLOCK    (BSP_CORE_CLOCK)                /* SYSTEM CLK, max 100MHz */
#define BSP_BUS_CLOCK       (BSP_CLOCK / BSP_BUS_DIV)       /* max 50MHz */
#define BSP_FLEXBUS_CLOCK   (BSP_CLOCK / BSP_FLEXBUS_DIV)
#define BSP_FLASH_CLOCK     (BSP_CLOCK / BSP_FLASH_DIV)     /* max 25MHz */


#if ((defined __CWCC__) || (defined __IAR_SYSTEMS_ICC__) || (defined __CC_ARM)|| (defined __arm__))
	extern uint_32 ___VECTOR_RAM[];            // Get vector table that was copied to RAM
#elif defined(__GNUC__)
	extern uint_32 __cs3_interrupt_vector[];
#endif
volatile uint_8 kbi_stat;	   /* Status of the Key Pressed */
#ifdef USE_FEEDBACK_ENDPOINT
	extern uint_32 feedback_data;
#endif

#if(!(defined MCU_MK21D5))
/******************************************************************************
*   @name        IRQ_ISR_PORTA
*
*   @brief       Service interrupt routine of IRQ
*
*   @return      None
*
*   @comment
*
*******************************************************************************/
void PORTA_IRQHandler(void)
{
#if defined(MCU_MK20D5)
    NVICICPR1 = 1 << ((40)%32);
    NVICISER1 = 1 << ((40)%32);
#elif defined (MCU_MKL25Z4)
    NVIC_ICPR = 1 << 30;
    NVIC_ISER = 1 << 30;
#else
    NVICICPR2 = 1 << ((87)%32);
    NVICISER2 = 1 << ((87)%32);
#endif
	DisableInterrupts;
#if defined MCU_MKL25Z4
    if(PORTA_ISFR & (1<<4))
    {
        kbi_stat |= 0x02;                 /* Update the kbi state */
        PORTA_ISFR = (1 << 4);            /* Clear the bit by writing a 1 to it */
    }
#else
	if(PORTA_ISFR & (1<<19))
	{
		kbi_stat |= 0x02; 				/* Update the kbi state */
		PORTA_ISFR = (1 << 19);			/* Clear the bit by writing a 1 to it */
	}
#endif
	EnableInterrupts;
}
#endif
#if (!defined MCU_MKL25Z4)&&((!(defined MCU_MK20D5)) || (!(defined _USB_BATT_CHG_APP_H_)))
/******************************************************************************
*   @name        IRQ_ISR
*
*   @brief       Service interrupt routine of IRQ
*
*   @return      None
*
*   @comment
*
*******************************************************************************/
void PORTC_IRQHandler(void)
	{
		#if defined(MCU_MK20D5)
			NVICICPR1 = (uint32_t)(1 << ((42)%32));
			NVICISER1 = (uint32_t)(1 << ((42)%32));
        #elif defined(MCU_MK21D5)
            NVICICPR1 = 1 << ((61)%32);
            NVICISER1 = 1 << ((61)%32);
        #else
			NVICICPR2 = (uint32_t)(1 << ((89)%32));		/* Clear any pending interrupt on PORTC */
			NVICISER2 = (uint32_t)(1 << ((89)%32));		/* Set interrupt on PORTC */
		#endif

		DisableInterrupts;
	#if(defined MCU_MK20D5) || (defined MCU_MK20D7) || (defined MCU_MK40D7)
		if(PORTC_ISFR & (1<<1))
    #elif defined(MCU_MK21D5)
        if(PORTC_ISFR & (1<<6))
	#else
		if(PORTC_ISFR & (1<<5))
	#endif
		{
			kbi_stat |= 0x02; 				/* Update the kbi state */

			/* Clear the bit by writing a 1 to it */
		#if(defined MCU_MK20D5) || (defined MCU_MK20D7) || (defined MCU_MK40D7)
			PORTC_ISFR = (1 << 1);
        #elif defined(MCU_MK21D5)
            PORTC_ISFR = (1 << 6);
		#else
			PORTC_ISFR = (1 << 5);
		#endif
		}

	#if(defined MCU_MK20D5) || (defined MCU_MK20D7) || (defined MCU_MK40D7)
		if(PORTC_ISFR & (1<<2))
    #elif defined(MCU_MK21D5)
        if(PORTC_ISFR & (1<<7))
	#else
		if(PORTC_ISFR & (1<<13))
	#endif
		{
			kbi_stat |= 0x08;				/* Update the kbi state */

			/* Clear the bit by writing a 1 to it */
			#if(defined MCU_MK20D5) || (defined MCU_MK20D7) || (defined MCU_MK40D7)
				PORTC_ISFR = (1 << 2);
            #elif defined(MCU_MK21D5)
                PORTC_ISFR = (1 << 7);
			#else
				PORTC_ISFR = (1 << 13);
			#endif
		}
		EnableInterrupts;
	}
	#endif
//#endif

#if((!defined MCU_MK21D5)&&(!defined MCU_MKL25Z4))
/******************************************************************************
*   @name        IRQ_ISR
*
*   @brief       Service interrupt routine of IRQ
*
*   @return      None
*
*   @comment
*
*******************************************************************************/
void PORTD_IRQHandler(void)
{
	#ifdef MCU_MK20D5
		NVICICPR1 = 1 << ((43)%32);
		NVICISER1 = 1 << ((43)%32);
	#else
		NVICICPR2 = 1 << ((90)%32);
		NVICISER2 = 1 << ((90)%32);
	#endif

	DisableInterrupts;
	if(PORTD_ISFR & (1<<0))
	{
		/* Update the kbi state */
		kbi_stat |= 0x02;	// right click
		PORTD_ISFR = (1 << 0);			/* Clear the bit by writing a 1 to it */
	}
	EnableInterrupts;
}
#endif

/******************************************************************************
*   @name        IRQ_ISR_PORTE
*
*   @brief       Service interrupt routine of IRQ
*
*   @return      None
*
*   @comment
*
*******************************************************************************/
#if((!defined MCU_MK21D5)&&(!defined MCU_MKL25Z4))
void PORTE_IRQHandler(void)
{
	#ifdef MCU_MK20D5
		NVICICPR1 = 1 << ((44)%32);
		NVICISER1 = 1 << ((44)%32);
	#else
		NVICICPR2 = 1 << ((91)%32);
		NVICISER2 = 1 << ((91)%32);
	#endif
	DisableInterrupts;
	if(PORTE_ISFR & (1<<26))
	{
		/* Update the kbi state */
#ifdef MCU_MK70F12
		kbi_stat |= 0x01;	// left click
#else
		kbi_stat |= 0x08;	// move pointer down
#endif
		PORTE_ISFR = (1 << 26);			/* Clear the bit by writing a 1 to it */
	}
	EnableInterrupts;
}
#endif
/******************************************************************************
*   @name        IRQ_ISR_PORTF
*
*   @brief       Service interrupt routine of IRQ
*
*   @return      None
*
*   @comment
*
*******************************************************************************/

/******************************************************************************/

static void USB_Init( void );
void USB0_Init(void);

static int ledstate = 0;
extern uint8_t g_Mem[];

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
** @brief		Entry point to program
*/
int main( void )
{
	int idx;

	// Initialize on board LED
	PORTC_PCR5 = PORT_PCR_MUX( 0x1 );	// LED is on PC5 (pin 13), config as GPIO (alt = 1)
	GPIOC_PDDR = ( 1 << 5 );			// make this an output pin
	GPIOC_PCOR = ( 1 << 5 );			// start with LED off

	// Flash a little startup sequence, this isn't necessary at all, just nice
	// to see a familiar sign before things start breaking!
	for ( idx = 0; idx < 3; idx ++ )
	{
		// Set LED
		GPIOC_PSOR = ( 1 << 5 );
		dumbdelay_ms( 50 );

		// Clear LED
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( 50 );
	}

	// Initialize USB stack (full speed)
	USB0_Init();

	/* Initialize the USB Test Application */
	TestApp_Init();

	for(;;)
	{
		Watchdog_Reset();

		/* Call the application task */
		TestApp_Task();

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

		//dumbdelay_ms( 500 );
	}

	// We definitely should never get here, this return is just to keep the
	// compiler happy
	return  0;
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
		dumbdelay_ms( 20 );
		GPIOC_PCOR = ( 1 << 5 );
		dumbdelay_ms( 20 );
	}
}

void USB0_Init(void)
{
  /* SIM_CLKDIV2: USBDIV=1,USBFRAC=0 */
  SIM_CLKDIV2 = (uint32_t)((SIM_CLKDIV2 & (uint32_t)~(uint32_t)(
                 SIM_CLKDIV2_USBDIV(0x06) |
                 SIM_CLKDIV2_USBFRAC_MASK
                )) | (uint32_t)(
                 SIM_CLKDIV2_USBDIV(0x01)
                ));
  /* SIM_SOPT2: USBSRC=1 */
  SIM_SOPT2 |= SIM_SOPT2_USBSRC_MASK;
  /* SIM_SCGC4: USBOTG=1 */
  SIM_SCGC4 |= SIM_SCGC4_USBOTG_MASK;
  /* USB0_CTL: ODDRST=1 */
  USB0_CTL |= USB_CTL_ODDRST_MASK;
  /* USB0_USBCTRL: SUSP=1,PDE=1,??=0,??=0,??=0,??=0,??=0,??=0 */
  USB0_USBCTRL = (USB_USBCTRL_SUSP_MASK | USB_USBCTRL_PDE_MASK);
  /* USB0_OTGISTAT: IDCHG=1,ONEMSEC=1,LINE_STATE_CHG=1,??=1,SESSVLDCHG=1,B_SESS_CHG=1,??=1,AVBUSCHG=1 */
  USB0_OTGISTAT = USB_OTGISTAT_IDCHG_MASK |
                  USB_OTGISTAT_ONEMSEC_MASK |
                  USB_OTGISTAT_LINE_STATE_CHG_MASK |
                  USB_OTGISTAT_SESSVLDCHG_MASK |
                  USB_OTGISTAT_B_SESS_CHG_MASK |
                  USB_OTGISTAT_AVBUSCHG_MASK |
                  0x12U;
  /* USB0_ISTAT: STALL=1,ATTACH=1,RESUME=1,SLEEP=1,TOKDNE=1,SOFTOK=1,ERROR=1,USBRST=1 */
  USB0_ISTAT = USB_ISTAT_STALL_MASK |
               USB_ISTAT_ATTACH_MASK |
               USB_ISTAT_RESUME_MASK |
               USB_ISTAT_SLEEP_MASK |
               USB_ISTAT_TOKDNE_MASK |
               USB_ISTAT_SOFTOK_MASK |
               USB_ISTAT_ERROR_MASK |
               USB_ISTAT_USBRST_MASK;
  /* USB0_ERRSTAT: BTSERR=1,??=1,DMAERR=1,BTOERR=1,DFN8=1,CRC16=1,CRC5EOF=1,PIDERR=1 */
  USB0_ERRSTAT = USB_ERRSTAT_BTSERR_MASK |
                 USB_ERRSTAT_DMAERR_MASK |
                 USB_ERRSTAT_BTOERR_MASK |
                 USB_ERRSTAT_DFN8_MASK |
                 USB_ERRSTAT_CRC16_MASK |
                 USB_ERRSTAT_CRC5EOF_MASK |
                 USB_ERRSTAT_PIDERR_MASK |
                 0x40U;
  /* USB0_INTEN: STALLEN=1,ATTACHEN=1,RESUMEEN=1,SLEEPEN=1,TOKDNEEN=1,SOFTOKEN=1,ERROREN=1,USBRSTEN=1 */
  USB0_INTEN = USB_INTEN_STALLEN_MASK |
               USB_INTEN_ATTACHEN_MASK |
               USB_INTEN_RESUMEEN_MASK |
               USB_INTEN_SLEEPEN_MASK |
               USB_INTEN_TOKDNEEN_MASK |
               USB_INTEN_SOFTOKEN_MASK |
               USB_INTEN_ERROREN_MASK |
               USB_INTEN_USBRSTEN_MASK;
  /* USB0_ERREN: BTSERREN=0,??=0,DMAERREN=0,BTOERREN=0,DFN8EN=0,CRC16EN=0,CRC5EOFEN=0,PIDERREN=0 */
  USB0_ERREN = 0x00U;
  /* USB0_USBTRC0: USBRESET=0,??=1,USBRESMEN=1,??=0,??=0,??=0,SYNC_DET=0,USB_RESUME_INT=0 */
  USB0_USBTRC0 = (USB_USBTRC0_USBRESMEN_MASK | 0x40U);
  /* USB0_OTGICR: IDEN=0,ONEMSECEN=0,LINESTATEEN=0,??=0,SESSVLDEN=0,BSESSEN=0,??=0,AVBUSEN=0 */
  USB0_OTGICR = 0x00U;
  /* USB0_ADDR: LSEN=0,ADDR=0 */
  USB0_ADDR = USB_ADDR_ADDR(0x00);
  /* USB0_ENDPT0: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT0 = 0x00U;
  /* USB0_ENDPT1: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT1 = 0x00U;
  /* USB0_ENDPT2: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT2 = 0x00U;
  /* USB0_ENDPT3: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT3 = 0x00U;
  /* USB0_ENDPT4: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT4 = 0x00U;
  /* USB0_ENDPT5: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT5 = 0x00U;
  /* USB0_ENDPT6: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT6 = 0x00U;
  /* USB0_ENDPT7: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT7 = 0x00U;
  /* USB0_ENDPT8: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT8 = 0x00U;
  /* USB0_ENDPT9: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT9 = 0x00U;
  /* USB0_ENDPT10: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT10 = 0x00U;
  /* USB0_ENDPT11: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT11 = 0x00U;
  /* USB0_ENDPT12: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT12 = 0x00U;
  /* USB0_ENDPT13: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT13 = 0x00U;
  /* USB0_ENDPT14: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT14 = 0x00U;
  /* USB0_ENDPT15: HOSTWOHUB=0,RETRYDIS=0,??=0,EPCTLDIS=0,EPRXEN=0,EPTXEN=0,EPSTALL=0,EPHSHK=0 */
  USB0_ENDPT15 = 0x00U;
  USB0_BDTPAGE1 = (uint8_t)((((uint32_t)((uint32_t)&g_Mem[0])) >> 0x08) & 0xFEU);
  USB0_BDTPAGE2 = (uint8_t)((((uint32_t)((uint32_t)&g_Mem[0])) >> 0x10) & 0xFFU);
  USB0_BDTPAGE3 = (uint8_t)((((uint32_t)((uint32_t)&g_Mem[0])) >> 0x18) & 0xFFU);
  /* USB0_SOFTHLD: CNT=0 */
  USB0_SOFTHLD = USB_SOFTHLD_CNT(0x00);
  /* USB0_OTGCTL: DPHIGH=0,??=0,DPLOW=0,DMLOW=0,??=0,OTGEN=0,??=0,??=0 */
  USB0_OTGCTL = 0x00U;
  /* USB0_CONTROL: ??=0,??=0,??=0,DPPULLUPNONOTG=0,??=0,??=0,??=0,??=0 */
  USB0_CONTROL = 0x00U;
  /* USB0_CTL: TXSUSPENDTOKENBUSY=0,HOSTMODEEN=0,ODDRST=0,USBENSOFEN=1 */
  USB0_CTL = (uint8_t)((USB0_CTL & (uint8_t)~(uint8_t)(
              USB_CTL_TXSUSPENDTOKENBUSY_MASK |
              USB_CTL_HOSTMODEEN_MASK |
              USB_CTL_ODDRST_MASK
             )) | (uint8_t)(
              USB_CTL_USBENSOFEN_MASK
             ));

  NVICICPR2 |= ( 1 << 9 );
  NVICISER2 |= ( 1 << 9 );
  NVICIP73 = 0x00;
}

static void USB_Init( void )
{
	//SIM_CLKDIV2 &= (uint32_t)(~(SIM_CLKDIV2_USBFRAC_MASK | SIM_CLKDIV2_USBDIV_MASK));

	/* Configure USBFRAC = 0, USBDIV = 0 => frq(USBout) = 2 / 3 * frq(PLLin) */
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(0);

	//SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(2) | SIM_CLKDIV2_USBFRAC_MASK;

	/* 1. Configure USB to be clocked from PLL */
	SIM_SOPT2 |= SIM_SOPT2_USBSRC_MASK | SIM_SOPT2_PLLFLLSEL_MASK;

#if PLL_96
	/* 2. USB freq divider */
	//SIM_CLKDIV2 = 0x02;
#endif /* PLL_96 */

	/* 3. Enable USB-OTG IP clocking */
	SIM_SCGC4 |= (SIM_SCGC4_USBOTG_MASK);

	/* old documentation writes setting this bit is mandatory */
	USB0_USBTRC0 = 0x40;

	/* Configure enable USB regulator for device */
#if(defined MCU_MK20D5)
	SIM_SOPT1CFG |= SIM_SOPT1CFG_URWE_MASK; /* Enable SOPT1 to be written */
#endif
	SIM_SOPT1 |= SIM_SOPT1_USBREGEN_MASK;

	NVICICPR2 = (1 << 9);	/* Clear any pending interrupts on USB */
	NVICISER2 = (1 << 9);	/* Enable interrupts from USB module */

	// Enable interrupt in NVIC and set priority to 0 */
		//NVICICPR2 |= ( 1 << 9 );
		//NVICISER2 |= ( 1 << 9 );
		//NVICIP73 = 0x00;

}
