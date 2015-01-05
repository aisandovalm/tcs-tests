/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//-----------------------------------------------------------------------------
/// \dir "USB HID Mouse Project"
///
/// !!!Purpose
///
/// The USB HID Mouse Project will help you to get familiar with the
/// USB Device Port(UDP) and PIO interface on AT91SAM microcontrollers. Also
/// it can help you to be familiar with the USB Framework that is used for
/// rapid development of USB-compliant class drivers such as USB Humen
/// Interface Device class (HID).
///
/// You can find following information depends on your needs:
/// - Sample usage of USB HID driver and PIO driver.
/// - USB HID driver development based on the AT91 USB Framework.
/// - USB enumerate sequence, the standard and class-specific descriptors and
///   requests handling.
/// - The initialize sequence and usage of UDP interface.
///
/// !See
/// - pio: PIO interface driver
/// - usb: USB Framework, USB HID driver and UDP interface driver
///    - "AT91 USB device framework"
///       - "USBD API"
///    - "hid-mouse"
///       - "USB HID Mouse"
///
/// !!!Requirements
///
/// This package can be used with all Atmel evaluation kits that has UDP
/// interface and have joystick on it.
///
/// !!!Description
///
/// When an EK running this program connected to a host (PC for example), with
/// USB cable, the EK appears as a HID-compliant mouse for the host. Then you
/// can use the joystick on the EK to control the pointer on the host. E.g., to
/// move it.
///
/// !!!Usage
///
/// -# Build the program and download it inside the evaluation board. Please
///    refer to the
///    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
///    SAM-BA User Guide</a>, the
///    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
///    GNU-Based Software Development</a> application note or to the
///    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
///    IAR EWARM User Guide</a>, depending on your chosen solution.
/// -# On the computer, open and configure a terminal application
///    (e.g. HyperTerminal on Microsoft Windows) with these settings:
///   - 115200 bauds
///   - 8 bits of data
///   - No parity
///   - 1 stop bit
///   - No flow control
/// -# Start the application.
/// -# In the terminal window, the following text should appear:
///     \code
///     -- USB Device HID Mouse Project xxx --
///     -- AT91xxxxxx-xx
///     -- Compiled: xxx xx xxxx xx:xx:xx --
///     \endcode
/// -# When connecting USB cable to windows, the LED blinks.
///    Then new "HID Mouse Device" appears in the
///    hardware %device list.
/// -# Once the device is connected and configured, pressing the joystick or 
///    the configurated board buttons move the cursor.
///
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/// \unit
///
/// !Purpose
///
/// This file contains all the specific code for the
/// usb-device-hid-mouse-project
///
/// !Contents
///
/// The code can be roughly broken down as follows:
///    - Configuration functions
///       - VBus_Configure
///       - ConfigurePit
///       - ConfigureWakeUp
///       - PIO & Timer configurations in start of main
///    - Interrupt handlers
///       - ISR_Vbus
///       - ISR_Pit
///       - WakeUpHandler
///    - The main function, which implements the program behavior
///
/// Please refer to the list of functions in the #Overview# tab of this unit
/// for more detailed information.
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#if defined(cortexm3)
#include <systick/systick.h>
#else
#include <pit/pit.h>
#endif
#include <pmc/pmc.h>
#include <irq/irq.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <usb/device/core/USBD.h>
#include <usb/common/core/USBConfigurationDescriptor.h>
#include <usb/device/hid-mouse/HIDDMouseDriver.h>

#include <string.h>

// +++ SPP Old main +++


/* *****************************************************************************
                SOFTWARE API FOR ADC
   ***************************************************************************** */
//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_EnableIt
//* \brief Enable ADC interrupt
//*----------------------------------------------------------------------------
__inline void AT91F_ADC_EnableIt (
	AT91PS_ADC pADC,     // pointer to a ADC controller
	unsigned int flag)   // IT to be enabled
{
	//* Write to the IER register
	pADC->ADC_IER = flag;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_DisableIt
//* \brief Disable ADC interrupt
//*----------------------------------------------------------------------------
__inline void AT91F_ADC_DisableIt (
	AT91PS_ADC pADC, // pointer to a ADC controller
	unsigned int flag) // IT to be disabled
{
	//* Write to the IDR register
	pADC->ADC_IDR = flag;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetStatus
//* \brief Return ADC Interrupt Status
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetStatus( // \return ADC Interrupt Status
	AT91PS_ADC pADC) // pointer to a ADC controller
{
	return pADC->ADC_SR;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetInterruptMaskStatus
//* \brief Return ADC Interrupt Mask Status
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetInterruptMaskStatus( // \return ADC Interrupt Mask Status
	AT91PS_ADC pADC) // pointer to a ADC controller
{
	return pADC->ADC_IMR;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_IsInterruptMasked
//* \brief Test if ADC Interrupt is Masked
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_IsInterruptMasked(
        AT91PS_ADC pADC,   // \arg  pointer to a ADC controller
        unsigned int flag) // \arg  flag to be tested
{
	return (AT91F_ADC_GetInterruptMaskStatus(pADC) & flag);
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_IsStatusSet
//* \brief Test if ADC Status is Set
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_IsStatusSet(
        AT91PS_ADC pADC,   // \arg  pointer to a ADC controller
        unsigned int flag) // \arg  flag to be tested
{
	return (AT91F_ADC_GetStatus(pADC) & flag);
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_CfgModeReg
//* \brief Configure the Mode Register of the ADC controller
//*----------------------------------------------------------------------------
__inline void AT91F_ADC_CfgModeReg (
	AT91PS_ADC pADC, // pointer to a ADC controller
	unsigned int mode)        // mode register
{
	//* Write to the MR register
	pADC->ADC_MR = mode;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetModeReg
//* \brief Return the Mode Register of the ADC controller value
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetModeReg (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	return pADC->ADC_MR;	
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_CfgTimings
//* \brief Configure the different necessary timings of the ADC controller
//*----------------------------------------------------------------------------
__inline void AT91F_ADC_CfgTimings (
	AT91PS_ADC pADC, // pointer to a ADC controller
	unsigned int mck_clock, // in MHz
	unsigned int adc_clock, // in MHz
	unsigned int startup_time, // in us
	unsigned int sample_and_hold_time)	// in ns
{
	unsigned int prescal,startup,shtim;
	
	prescal = mck_clock/(2*adc_clock) - 1;
	startup = adc_clock*startup_time/8 - 1;
	shtim = adc_clock*sample_and_hold_time/1000 - 1;
	
	//* Write to the MR register
	pADC->ADC_MR = ( (prescal<<8) & AT91C_ADC_PRESCAL) | ( (startup<<16) & AT91C_ADC_STARTUP) | ( (shtim<<24) & AT91C_ADC_SHTIM);
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_EnableChannel
//* \brief Return ADC Timer Register Value
//*----------------------------------------------------------------------------
__inline void AT91F_ADC_EnableChannel (
	AT91PS_ADC pADC, // pointer to a ADC controller
	unsigned int channel)        // mode register
{
	//* Write to the CHER register
	pADC->ADC_CHER = channel;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_DisableChannel
//* \brief Return ADC Timer Register Value
//*----------------------------------------------------------------------------
__inline void AT91F_ADC_DisableChannel (
	AT91PS_ADC pADC, // pointer to a ADC controller
	unsigned int channel)        // mode register
{
	//* Write to the CHDR register
	pADC->ADC_CHDR = channel;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetChannelStatus
//* \brief Return ADC Timer Register Value
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetChannelStatus (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	return pADC->ADC_CHSR;	
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_StartConversion
//* \brief Software request for a analog to digital conversion
//*----------------------------------------------------------------------------
__inline void AT91F_ADC_StartConversion (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	pADC->ADC_CR = AT91C_ADC_START;	
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_SoftReset
//* \brief Software reset
//*----------------------------------------------------------------------------
__inline void AT91F_ADC_SoftReset (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	pADC->ADC_CR = AT91C_ADC_SWRST;	
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetLastConvertedData
//* \brief Return the Last Converted Data
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetLastConvertedData (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	return pADC->ADC_LCDR;	
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetConvertedDataCH0
//* \brief Return the Channel 0 Converted Data
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetConvertedDataCH0 (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	return pADC->ADC_CDR0;	
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetConvertedDataCH1
//* \brief Return the Channel 1 Converted Data
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetConvertedDataCH1 (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	return pADC->ADC_CDR1;	
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetConvertedDataCH2
//* \brief Return the Channel 2 Converted Data
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetConvertedDataCH2 (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	return pADC->ADC_CDR2;	
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetConvertedDataCH3
//* \brief Return the Channel 3 Converted Data
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetConvertedDataCH3 (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	return pADC->ADC_CDR3;	
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetConvertedDataCH4
//* \brief Return the Channel 4 Converted Data
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetConvertedDataCH4 (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	return pADC->ADC_CDR4;	
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetConvertedDataCH5
//* \brief Return the Channel 5 Converted Data
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetConvertedDataCH5 (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	return pADC->ADC_CDR5;	
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetConvertedDataCH6
//* \brief Return the Channel 6 Converted Data
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetConvertedDataCH6 (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	return pADC->ADC_CDR6;	
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_ADC_GetConvertedDataCH7
//* \brief Return the Channel 7 Converted Data
//*----------------------------------------------------------------------------
__inline unsigned int AT91F_ADC_GetConvertedDataCH7 (
	AT91PS_ADC pADC // pointer to a ADC controller
	)
{
	return pADC->ADC_CDR7;	
}




#define BUTTON_SAMPLING 1700 // Sampling frequency of buttons

#define BL_OFF(pio) ((pio) & SW1)
#define BR_OFF(pio) ((pio) & SW2)
#define BU_OFF(pio) ((pio) & SW3)
#define BD_OFF(pio) ((pio) & SW4)

#define BL_ON(pio) (!BL_OFF(pio))
#define BR_ON(pio) (!BR_OFF(pio))
#define BU_ON(pio) (!BU_OFF(pio))
#define BD_ON(pio) (!BD_OFF(pio))
#define CLICKL_ON 1
#define CLICKR_ON 2

//////////////////////////////////////////////////////////////////////////////////////////
/* bits */
#define   BIT0        0x00000001
#define   BIT1        0x00000002
#define   BIT2        0x00000004
#define   BIT3        0x00000008
#define   BIT4        0x00000010
#define   BIT5        0x00000020
#define   BIT6        0x00000040
#define   BIT7        0x00000080
#define   BIT8        0x00000100
#define   BIT9        0x00000200
#define   BIT10       0x00000400
#define   BIT11       0x00000800
#define   BIT12       0x00001000
#define   BIT13       0x00002000
#define   BIT14       0x00004000
#define   BIT15       0x00008000
#define   BIT16       0x00010000
#define   BIT17       0x00020000
#define   BIT18       0x00040000
#define   BIT19       0x00080000
#define   BIT20       0x00100000
#define   BIT21       0x00200000
#define   BIT22       0x00400000
#define   BIT23       0x00800000
#define   BIT24       0x01000000
#define   BIT25       0x02000000
#define   BIT26       0x04000000
#define   BIT27       0x08000000
#define   BIT28       0x10000000
#define   BIT29       0x20000000
#define   BIT30       0x40000000
#define   BIT31       0x80000000

/* Give the number of samples, thus, the number of conversions */
#define   NSAMPLE    16

/* ADC field definition for the Mode Register */
#define   TRGEN    (0x0)    // Hardware triggering
#define   TRGSEL   (0x0)    // Use a Timer output signal (on rising edge) from TIOA0 (for this example)
#define   LOWRES   (0x0)    // 10-bit result output
#define   SLEEP    (0x0)    // Normal Mode
#define   PRESCAL  (0x4)    // Max value
#define   STARTUP  (0xc)    // This time period must be higher than 20 µs and not 20 ms
#define   SHTIM    (0x8)    // Must be higher than 3 ADC clock cycles but depends on output
                            // impedance of the analog driver to the ADC input
/* Channel selection */
#define   CHANNEL1  (0)
#define   CHANNEL2  (1)
#define   CHANNEL3  (2)
#define   CHANNEL4  (3)
#define   CHANNEL5  (4)
#define   CHANNEL6  (5)
#define   CHANNEL7  (6)
#define   CHANNEL8  (7)

/* Print view */
#define PRINT_COORDINATES   0
#define PRINT_DIVERSION     1


unsigned char n;
unsigned int j;
int tmp_x;
int tmp_y;
int tmp_z;
unsigned char num_1;
unsigned char num_2;
unsigned char num_3;
unsigned char num_4;
unsigned int Coordinates[3];
int base_x;
int base_y;
int base_z;
char print_view;
char button_flag;

//////////////////////////////////////////////////////////////////////////////////////////


#define BUFFER_1_WRITE 				0x84	// buffer 1 write
#define BUFFER_2_WRITE 				0x87 	// buffer 2 write
#define BUFFER_1_READ 				0x54	// buffer 1 read
#define BUFFER_2_READ 				0x56	// buffer 2 read		
#define B1_TO_MM_PAGE_PROG_WITH_ERASE 		0x83	// buffer 1 to main memory page program with built-in erase
#define B2_TO_MM_PAGE_PROG_WITH_ERASE 		0x86	// buffer 2 to main memory page program with built-in erase
#define B1_TO_MM_PAGE_PROG_WITHOUT_ERASE 	0x88	// buffer 1 to main memory page program without built-in erase
#define B2_TO_MM_PAGE_PROG_WITHOUT_ERASE 	0x89	// buffer 2 to main memory page program without built-in erase
#define MM_PAGE_PROG_THROUGH_B1 		0x82	// main memory page program through buffer 1
#define MM_PAGE_PROG_THROUGH_B2 		0x85	// main memory page program through buffer 2
#define AUTO_PAGE_REWRITE_THROUGH_B1 		0x58	// auto page rewrite through buffer 1
#define AUTO_PAGE_REWRITE_THROUGH_B2 		0x59	// auto page rewrite through buffer 2
#define MM_PAGE_TO_B1_COMP 			0x60	// main memory page compare to buffer 1
#define MM_PAGE_TO_B2_COMP 			0x61	// main memory page compare to buffer 2
#define MM_PAGE_TO_B1_XFER 			0x53	// main memory page to buffer 1 transfer
#define MM_PAGE_TO_B2_XFER 			0x55	// main memory page to buffer 2 transfer
#define	READ_STATUS_REGISTER			0xD7	// read status register
#define CONTINUOUS_ARRAY_READ			0xE8	// continuous read
#define MAIN_MEMORY_PAGE_READ                   0x52	// main page read
#define PAGE_ERASE                              0x81	// page erase

// SPI
AT91PS_SPI    s_pSpi = AT91C_BASE_SPI;
AT91PS_PIO    s_pPio = AT91C_BASE_PIOA;
AT91PS_PMC    s_pPMC = AT91C_BASE_PMC;
AT91PS_PDC    s_pPDC = AT91C_BASE_PDC_SPI;

// ADC
AT91PS_ADC    a_pADC = AT91C_BASE_ADC;
AT91PS_PMC    a_pPMC = AT91C_BASE_PMC;
AT91PS_PIO    a_pPio = AT91C_BASE_PIOA;
AT91PS_SYS   a_pSys = AT91C_BASE_SYS;

// GPIO and other
AT91PS_PIO    u_pPio    = AT91C_BASE_PIOA;
AT91PS_PMC    u_pPMC    = AT91C_BASE_PMC;
AT91PS_USART  u_pUSART0 = AT91C_BASE_US0;
AT91PS_USART  u_pUSART1 = AT91C_BASE_US1;
AT91PS_PDC    u_pPDC0   = AT91C_BASE_PDC_US0;
AT91PS_PDC    u_pPDC1   = AT91C_BASE_PDC_US1;
AT91PS_MC     u_pMC     = AT91C_BASE_MC;
AT91PS_AIC    u_pAic    = AT91C_BASE_AIC;

// Reset
AT91PS_RSTC   m_pRSTC  = AT91C_BASE_RSTC;

// buffer for test
unsigned char WriteBuffer[]   = { 1, 2, 3, 4, 5, 6, 5, 8, 9, 0 };
unsigned char ReadBuffer[]    = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// Simple delay
void Delay (unsigned long a)
{
  while (--a!=0);
}

void InitUSART0(void)
{
  //m_pPio->PIO_PDR = BIT5 | BIT6;  //Disables the PIO from controlling the corresponding pin (enables peripheral control of the pin).
  //m_pPio->PIO_BSR = BIT5 | BIT6;  //Assigns the I/O line to the peripheral B function.
  u_pPio->PIO_PDR = BIT5 | BIT6;
  u_pPio->PIO_ASR = BIT5 | BIT6;
  u_pPio->PIO_BSR = 0;


  //enable the clock of USART
  u_pPMC->PMC_PCER = 1<<AT91C_ID_US0;

  //set baud rate divisor register
  u_pUSART0->US_BRGR = 313; //((48000000)/9600x16)

  //write the Timeguard Register
  u_pUSART0->US_TTGR = 0;

  //Set the USART mode
  u_pUSART0->US_MR = 0x08c0;

  //Enable the RX and TX PDC transfer requests
  u_pPDC0->PDC_PTCR = AT91C_PDC_TXTEN | AT91C_PDC_RXTEN;

  //Enable usart
  u_pUSART0->US_CR = 0x50;

}


void write_char_USART0(unsigned char ch)
{
  while (!(u_pUSART0->US_CSR&AT91C_US_TXRDY)==1);
  u_pUSART0->US_THR = ((ch & 0x1FF));
}


void write_str_USART0(unsigned char* buff) 
{

  unsigned int i = 0x0;

  while(buff[i] != '\0') 
  {
    write_char_USART0(buff[i]);
    i++;
  }
}

void write_report_USART0(void) 
{

  if(print_view == PRINT_COORDINATES) 
  {
    tmp_x = Coordinates[0]/NSAMPLE;
    tmp_y = Coordinates[1]/NSAMPLE;
    tmp_z = Coordinates[2]/NSAMPLE;
  }
  else 
  {
    tmp_x = base_x - (Coordinates[0]/NSAMPLE);
    tmp_y = base_y - (Coordinates[1]/NSAMPLE);
    tmp_z = base_z - (Coordinates[2]/NSAMPLE);
  }



  // X
  if(print_view == PRINT_DIVERSION) 
  {
    write_str_USART0(" dX: ");
    if(tmp_x<0) 
    {
      tmp_x = (tmp_x)*(-1);
      write_char_USART0('-');
    }
  }
  else 
  {
   write_str_USART0("\rX: ");
  }
  num_1 = (tmp_x%10);
  num_2 = (tmp_x%100 - num_1)/10;
  num_3 = (tmp_x%1000 - num_2)/100;
  num_4 = (tmp_x%10000 - num_3)/1000;

  write_char_USART0(num_4+48);
  write_char_USART0(num_3+48);
  write_char_USART0(num_2+48);
  write_char_USART0(num_1+48);


  // X
  if(print_view == PRINT_DIVERSION) 
  {
    write_str_USART0(" dY: ");
    if(tmp_y<0) 
    {
      tmp_y = (tmp_y)*(-1);
      write_char_USART0('-');
    }
  }
  else 
  {
   write_str_USART0(" Y: ");
  }

  num_1 = (tmp_y%10);
  num_2 = (tmp_y%100 - num_1)/10;
  num_3 = (tmp_y%1000 - num_2)/100;
  num_4 = (tmp_y%10000 - num_3)/1000;

  write_char_USART0(num_4+48);
  write_char_USART0(num_3+48);
  write_char_USART0(num_2+48);
  write_char_USART0(num_1+48);


  // Z
  if(print_view == PRINT_DIVERSION) 
  {
    write_str_USART0(" dZ: ");
    if(tmp_z<0) 
    {
      tmp_z = (tmp_z)*(-1);
      write_char_USART0('-');
    }
  }
  else 
  {
   write_str_USART0(" Z: ");
  }

  num_1 = (tmp_z%10);
  num_2 = (tmp_z%100 - num_1)/10;
  num_3 = (tmp_z%1000 - num_2)/100;
  num_4 = (tmp_z%10000 - num_3)/1000;

  write_char_USART0(num_4+48);
  write_char_USART0(num_3+48);
  write_char_USART0(num_2+48);
  write_char_USART0(num_1+48);

}


void ADCInit (void)
{
       // Clear all previous setting and result
       AT91F_ADC_SoftReset (AT91C_BASE_ADC);

       // First step: Set the ADC by writing in Mode register
       AT91F_ADC_CfgModeReg (AT91C_BASE_ADC, (SHTIM << 24) | (STARTUP << 16) | (PRESCAL << 8) |
                            (SLEEP << 5) | (LOWRES <<4) | (TRGSEL << 1) | (TRGEN )) ;

        // Second Step: Select the first and second active channels
        AT91F_ADC_EnableChannel (AT91C_BASE_ADC, (1<<CHANNEL5));
        AT91F_ADC_EnableChannel (AT91C_BASE_ADC, (1<<CHANNEL6));
        AT91F_ADC_EnableChannel (AT91C_BASE_ADC, (1<<CHANNEL7));
}

void GetCoordinates (void) 
{

  Coordinates[0] = 0;
  Coordinates[1] = 0;
  Coordinates[2] = 0;

  for (n=0; n < NSAMPLE; n++)
  {
        // Third Step: Start the conversion
        AT91F_ADC_StartConversion (AT91C_BASE_ADC);
        // Waiting Stop Of conversion by pulling the status bit
        while (!((AT91F_ADC_GetStatus (AT91C_BASE_ADC)) & (1<<CHANNEL5)));
        // Read the ADC output value
        Coordinates[0]     += AT91F_ADC_GetConvertedDataCH4 (AT91C_BASE_ADC);

        // Third Step: Start the conversion
        AT91F_ADC_StartConversion (AT91C_BASE_ADC);
        // Waiting Stop Of conversion by pulling the status bit
        while (!((AT91F_ADC_GetStatus (AT91C_BASE_ADC)) & (1<<CHANNEL6)));
        // Read the ADC output value
        Coordinates[1]     += AT91F_ADC_GetConvertedDataCH5 (AT91C_BASE_ADC);

         // Third Step: Start the conversion
        AT91F_ADC_StartConversion (AT91C_BASE_ADC);
        // Waiting Stop Of conversion by pulling the status bit
        while (!((AT91F_ADC_GetStatus (AT91C_BASE_ADC)) & (1<<CHANNEL7)));
        // Read the ADC output value
        Coordinates[2]     += AT91F_ADC_GetConvertedDataCH6 (AT91C_BASE_ADC);
  }
}


void InitMMA(void)
{
  //enable the clock of the PIO
  a_pPMC->PMC_PCER = 1 << AT91C_ID_PIOA;

  // Config
  // Configure all ports as Input
  a_pPio->PIO_ODR           = 0xffffffff;       Delay(100);
  // Enable all ports
  a_pPio->PIO_PER           = 0xffffffff;       Delay(100);
  // Disable Pull-up resistor
  // SPP +
  //a_pSys->SYSC_PIOA_PPUDR   = 0xffffffff;       Delay(100);
  a_pSys->PIOA_PPUDR   = 0xffffffff;       Delay(100);
  // SPP -


  // Set G_SEL1, G_SEL2 to low and SM to high
  a_pPio->PIO_SODR = 0x20000000;   // Set SM to HIGH
  a_pPio->PIO_CODR = 0x18000000;   // Set G_SEL1, G_SEL2 to LOW

  // Set G_SEL1, G_SEL2 and SM as output
  a_pPio->PIO_OER = 0x38000000;
}

void FlashEnable(void)
{
  // Set PA11 to LOW
  s_pPio->PIO_CODR  = BIT11;
}

void FlashDisable(void)
{
  // Set PA11 to HIGH
  s_pPio->PIO_SODR  = BIT11;
}

void InitSPI(void)
{
  //set functionalite to pins:
  //port0.12 -> MISO
  //port0.13 -> MOSI
  //port0.14 -> SPCK
  s_pPio->PIO_PDR = BIT12 | BIT13 | BIT14;
  s_pPio->PIO_ASR = BIT12 | BIT13 | BIT14;
  s_pPio->PIO_BSR = 0;

  //port0.11 -> NPCS0 -> CS as output and high
  s_pPio->PIO_PER   = BIT11;
  s_pPio->PIO_OER   = BIT11;    // Configure PA11 as output
  s_pPio->PIO_SODR  = BIT11;    // Set PA11 to HIGH

  //s_pPMC->PMC_PCER = AT91C_ID_SPI;        //enable the clock of SPI
  s_pPMC->PMC_PCER = 1 << AT91C_ID_SPI;

  /****  Fixed mode ****/
  s_pSpi->SPI_CR      = 0x81;              //SPI Enable, Sowtware reset
  s_pSpi->SPI_CR      = 0x01;              //SPI Enable


  //pSpi->SPI_MR      = 0xE0099;          //Master mode, fixed select, disable decoder, FDIV=1 (NxMCK), PCS=1110, loopback
  //s_pSpi->SPI_MR    = 0xE0019;          //Master mode, fixed select, disable decoder, FDIV=1 (NxMCK), PCS=1110,
  //s_pSpi->SPI_MR      = 0xE0011;        //Master mode, fixed select, disable decoder, FDIV=0 (MCK), PCS=1110
  s_pSpi->SPI_MR      = 0xE0011;          //Master mode, fixed select, disable decoder, FDIV=0 (MCK), PCS=1101

  s_pSpi->SPI_CSR[0]  = 0x4A02;           //8bit, CPOL=0, ClockPhase=1, SCLK = 200kHz
  //s_pSpi->SPI_CSR[0]  = 0x4A03;           //8bit, CPOL=1, ClockPhase=1, SCLK = 200kHz
  //s_pSpi->SPI_CSR[0]  = 0x4A01;           //8bit, CPOL=1, ClockPhase=0, SCLK = 200kHz
  //s_pSpi->SPI_CSR[0]  = 0x4A00;           //8bit, CPOL=0, ClockPhase=0, SCLK = 200kHz

  s_pPDC->PDC_PTCR = AT91C_PDC_TXTEN | AT91C_PDC_RXTEN;

  s_pSpi->SPI_PTCR = AT91C_PDC_TXTEN | AT91C_PDC_RXTEN;
}

unsigned char WriteByte(unsigned char data)
{
    unsigned int spib;

    while((s_pSpi->SPI_SR & AT91C_SPI_TDRE) == 0);      // Wait for the transfer to complete
    s_pSpi->SPI_TDR = (data & 0xFFFF);                  // Send the data

    while((s_pSpi->SPI_SR & AT91C_SPI_RDRF) == 0);      // Wait until the character can be sent
    spib = ((s_pSpi->SPI_RDR) & 0xFFFF);                // Get the data received

    return spib;
}

unsigned char FlashBusy(void)
{

    unsigned char result;

    FlashEnable();

    // Send RDSR - Read Status Register opcode
    WriteByte(READ_STATUS_REGISTER);

    // Get register contents
    result = WriteByte(0x0);

    FlashDisable();

    if(result&0x80) return 0;
    else return 1;

}

void ByteToBuffer(unsigned int byte, unsigned int address)
{
	unsigned int 	addr	= 0;

	addr = address%264;	

	// Wait for write to complete
	while( FlashBusy() );	

	// perform to transfer

	// Set the Write Enable latch
	FlashEnable();

	WriteByte(BUFFER_1_WRITE);
	
	WriteByte(0x00);
	
	WriteByte((*((unsigned char*)&addr +1)));
	
	WriteByte(((unsigned char)addr));

	// Send the byte to write
	WriteByte(byte);
	
	FlashDisable();

	// Wait for write to complete
	while( FlashBusy() );
}

void FlashReadArray(unsigned long address, unsigned char *buffer, unsigned char length)
{

    unsigned char ch;

    unsigned int addr	= 0;
    unsigned int page	= 0;

    page = address/264;
    addr = address%264;

    page<<=1;	

    FlashEnable();

    // Send READ opcode
    WriteByte(CONTINUOUS_ARRAY_READ);
	
    // 24 bit page and address
    WriteByte((*((unsigned char*)&page+1)));

    WriteByte((((unsigned char)page&0xFE)|(*((unsigned char*)&addr+1) & 0x01)));

    WriteByte((unsigned char)addr);

    // 32 don't care bits
    WriteByte(0x00);
    WriteByte(0x00);
    WriteByte(0x00);
    WriteByte(0x00);

    while(length--)
    {
       ch = WriteByte(0x0);
       *buffer++ = ch;
       write_str_USART0("\n\r\t Read -> "); write_char_USART0(ch+48);
    };
	


    FlashDisable();
}


void WriteBufferToMainMemory(unsigned int address)
{
	
    unsigned int page	= 0;

    page = address/264;

    page<<=1;

    // Wait for write to complete
    while( FlashBusy() );	

    FlashEnable();

    //command
    WriteByte(B1_TO_MM_PAGE_PROG_WITH_ERASE);

    //6 reserved + 2 high address
    WriteByte((*((unsigned char*)&page+1)));

    //7 low address bits + 1 don't care
    WriteByte(((unsigned char)page));

    //don't care 8 bits
    WriteByte(0x00);

    FlashDisable();

    // Wait for write to complete
    while( FlashBusy() );
}

void ReadBufferFromMainMemory(unsigned int address) 
{

  unsigned int page	= 0;

  page = address/264;

  page<<=1;

  // Wait for write to complete
  while( FlashBusy() );	

  FlashEnable();

  //command
  WriteByte(MM_PAGE_TO_B1_XFER);

  //6 reserved + 2 high address
  WriteByte((*((unsigned char*)&page+1)));

  //7 low address bits + 1 don't care
  WriteByte(((unsigned char)page));

  //don't care 8 bits
  WriteByte(0x00);

  FlashDisable();

  // Wait for write to complete
  while( FlashBusy() );
}

unsigned char TestFlash(void)
{

    // Stat led
    s_pPio->PIO_PER   = BIT31;
    s_pPio->PIO_OER   = BIT31;    // Configure PA11 as output
    s_pPio->PIO_SODR  = BIT31;    // Set PA11 to HIGH

    char i;

    // Test led and flash memory
    InitSPI();

    // Write WriteBuffer
    for(i=0; i<10; i++) 
    {
      ByteToBuffer(WriteBuffer[i], i);
    }
    WriteBufferToMainMemory(0);

    // Read ReadBuffer
    FlashReadArray(0, ReadBuffer, 10);

    // check for Correct Read
    for(i=0; i<10; i++) 
    {

      if(WriteBuffer[i]!=ReadBuffer[i]) 
      {
        return 0;
      }
    }

    return 1;
}

// --- SPP Old main ---

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

/// Speed of pointer movement X
#define SPEED_X             4

/// Speed of pointer movement Y
#define SPEED_Y             4

/// Delay for pushbutton debouncing (ms)
#define DEBOUNCE_TIME      10

/// PIT period value (uSeconds)
#define PIT_PERIOD        1000

/// Use for power management
#define STATE_IDLE    0
/// The USB device is in suspend state
#define STATE_SUSPEND 4
/// The USB device is in resume state
#define STATE_RESUME  5

//------------------------------------------------------------------------------
//         Internal variables
//------------------------------------------------------------------------------
/// State of USB, for suspend and resume
unsigned char USBState = STATE_IDLE;

#if defined(PINS_JOYSTICK)
/// List of pinsJoystick to configure for the application.
static Pin pinsJoystick[] = {PINS_JOYSTICK};
#else
/// List of pinsJoystick (push button) to configure for the application.
static Pin pinsJoystick[] = {PINS_PUSHBUTTONS};
#endif

//------------------------------------------------------------------------------
//         Remote wake-up support (optional)
//------------------------------------------------------------------------------
#if (BOARD_USB_BMATTRIBUTES == USBConfigurationDescriptor_BUSPOWERED_RWAKEUP) \
    || (BOARD_USB_BMATTRIBUTES == USBConfigurationDescriptor_SELFPOWERED_RWAKEUP)

#define WAKEUP_CONFIGURE()  ConfigureWakeUp()

/// Button for Wake-UP the USB device.
static const Pin pinWakeUp = PIN_PUSHBUTTON_1;

/// Debounce count (in ms)
static unsigned long debounceCounter = DEBOUNCE_TIME;

#if !defined(cortexm3)
//------------------------------------------------------------------------------
/// Interrupt service routine for the PIT. Debounces the wake-up pin input.
//------------------------------------------------------------------------------
static void ISR_Pit(void)
{
    unsigned long pisr = 0;

    // Read the PISR
    pisr = PIT_GetStatus() & AT91C_PITC_PITS;

    if (pisr != 0) 
    {

        // Read the PIVR. It acknowledges the IT
        PIT_GetPIVR();
    }

    // Button released
    if (PIO_Get(&pinWakeUp)) 
    {

        debounceCounter = DEBOUNCE_TIME;
    }
    // Button still pressed
    else 
    {

        debounceCounter--;
    }

    // End of debounce time
    if (debounceCounter == 0) 
    {

        debounceCounter = DEBOUNCE_TIME;
        PIT_DisableIT();
        AT91C_BASE_PITC->PITC_PIMR &= ~AT91C_PITC_PITEN;
        HIDDMouseDriver_RemoteWakeUp();
    }
}

//------------------------------------------------------------------------------
/// Configures the PIT to generate 1ms ticks.
//------------------------------------------------------------------------------
static void ConfigurePit(void)
{
    // Initialize and enable the PIT
    PIT_Init(PIT_PERIOD, BOARD_MCK / 1000000);

    // Disable the interrupt on the interrupt controller
    IRQ_DisableIT(AT91C_ID_SYS);

    // Configure the AIC for PIT interrupts
    IRQ_ConfigureIT(AT91C_ID_SYS, 0, ISR_Pit);

    // Enable the interrupt on the interrupt controller
    IRQ_EnableIT(AT91C_ID_SYS);

    // Enable the interrupt on the pit
    PIT_EnableIT();

    // Enable the pit
    PIT_Enable();
}
#else

//------------------------------------------------------------------------------
/// Interrupt service routine for the system tick. Debounces the wake-up pin input.
//------------------------------------------------------------------------------
void SysTick_Handler(void)
{
    // Button released
    if (PIO_Get(&pinWakeUp)) 
    {

        debounceCounter = DEBOUNCE_TIME;
    }
    // Button still pressed
    else 
    {

        debounceCounter--;
    }

    // End of debounce time
    if (debounceCounter == 0) 
    {

        // Disable debounce timer
        SysTick_Configure(1, BOARD_MCK/1000, 0);

        debounceCounter = DEBOUNCE_TIME;
        HIDDMouseDriver_RemoteWakeUp();
    }
}

//------------------------------------------------------------------------------
/// Configures the SYS Tisk to generate 1ms ticks.
//------------------------------------------------------------------------------
static void ConfigureSysTick(void)
{
    // 1ms System Tick
    SysTick_Configure(1, BOARD_MCK/1000, SysTick_Handler);
}

#endif

//------------------------------------------------------------------------------
/// Interrupt service routine for the remote wake-up pin. Starts the debouncing
/// sequence.
//------------------------------------------------------------------------------
static void WakeUpHandler(const Pin *pin)
{
    TRACE_DEBUG("Wake-up handler\n\r");

    // Check current level on the remote wake-up pin
    if (!PIO_Get(&pinWakeUp))
    {

      #if defined(at91sam3uek)
        ConfigureSysTick();
      #else
        ConfigurePit();
      #endif
    }
}

//------------------------------------------------------------------------------
/// Configures the wake-up pin to generate interrupts.
//------------------------------------------------------------------------------
static void ConfigureWakeUp(void)
{
    TRACE_INFO("Wake-up configuration\n\r");

    // Configure PIO
    PIO_Configure(&pinWakeUp, 1);
    PIO_ConfigureIt(&pinWakeUp, WakeUpHandler);
    PIO_EnableIt(&pinWakeUp);
}

#else
    #define WAKEUP_CONFIGURE()
#endif

//------------------------------------------------------------------------------
//         VBus monitoring (optional)
//------------------------------------------------------------------------------
#if defined(PIN_USB_VBUS)

#define VBUS_CONFIGURE()  VBus_Configure()

/// VBus pin instance.
static const Pin pinVbus = PIN_USB_VBUS;

//------------------------------------------------------------------------------
/// Handles interrupts coming from PIO controllers.
//------------------------------------------------------------------------------
static void ISR_Vbus(const Pin *pPin)
{
    TRACE_INFO("VBUS ");

    // Check current level on VBus
    if (PIO_Get(&pinVbus)) 
    {

        TRACE_INFO("conn\n\r");
        USBD_Connect();
    }
    else 
    {

        TRACE_INFO("discon\n\r");
        USBD_Disconnect();
    }
}

//------------------------------------------------------------------------------
/// Configures the VBus pin to trigger an interrupt when the level on that pin
/// changes.
//------------------------------------------------------------------------------
static void VBus_Configure( void )
{
    TRACE_INFO("VBus configuration\n\r");

    // Configure PIO
    PIO_Configure(&pinVbus, 1);
    PIO_ConfigureIt(&pinVbus, ISR_Vbus);
    PIO_EnableIt(&pinVbus);

    // Check current level on VBus
    if (PIO_Get(&pinVbus)) 
    {

        // if VBUS present, force the connect
        TRACE_INFO("conn\n\r");
        USBD_Connect();
    }
    else 
    {
        USBD_Disconnect();
    }
}

#else
    #define VBUS_CONFIGURE()    USBD_Connect()
#endif //#if defined(PIN_USB_VBUS)

static unsigned char ButtonsMonitor(unsigned char *pBtnStatus,
                                    signed char *pDx,
                                    signed char *pDy)
{
    unsigned char isChanged = 0;

#if defined(JOYSTICK_LCLIC)
    // Left Click
    if (PIO_Get(&pinsJoystick[JOYSTICK_LCLIC]) == 0) 
    {
  
        if ((*pBtnStatus & HIDDMouse_LEFT_BUTTON) == 0) 
        {
  
            printf("LDn ");
            *pBtnStatus |= HIDDMouse_LEFT_BUTTON;
            isChanged = 1;
        }
    }
    else if (*pBtnStatus & HIDDMouse_LEFT_BUTTON) 
    {
  
        printf("Lup ");
        *pBtnStatus &= ~HIDDMouse_LEFT_BUTTON;
        isChanged = 1;
    }
#endif
    
#if defined(JOYSTICK_RCLIC)
      // Right Click
      if (PIO_Get(&pinsJoystick[JOYSTICK_RCLIC]) == 0) 
      {
    
          if ((*pBtnStatus & HIDDMouse_RIGHT_BUTTON) == 0) 
          {
    
              printf("RDn ");
              *pBtnStatus |= HIDDMouse_RIGHT_BUTTON;
              isChanged = 1;
          }
      }
      else if (*pBtnStatus & HIDDMouse_RIGHT_BUTTON) 
      {
    
          printf("Rup ");
          *pBtnStatus &= ~HIDDMouse_RIGHT_BUTTON;
          isChanged = 1;
      }
#endif

#if defined(JOYSTICK_LEFT) && defined(JOYSTICK_RIGHT)
      // - Movment buttons, Joystick or Push buttons
      // Left
      if (PIO_Get(&pinsJoystick[JOYSTICK_LEFT]) == 0) 
      {
    
          *pDx = -SPEED_X;
          isChanged = 1;
      }
      // Right
      else if (PIO_Get(&pinsJoystick[JOYSTICK_RIGHT]) == 0) 
      {
    
          *pDx = SPEED_X;
          isChanged = 1;
      }
      else 
      {
          *pDx = 0;
      }
#endif
    
#if defined(JOYSTICK_UP) && defined(JOYSTICK_DOWN)
      // Up
      if (PIO_Get(&pinsJoystick[JOYSTICK_UP]) == 0) 
      {
    
          *pDy = -SPEED_Y;
          isChanged = 1;
      }
      // Down
      else if (PIO_Get(&pinsJoystick[JOYSTICK_DOWN]) == 0) 
      {
    
          *pDy = SPEED_Y;
          isChanged = 1;
      }
      else 
      {
    
          *pDy = 0;
      }
#endif

    return isChanged;
}

//------------------------------------------------------------------------------
/// Put the CPU in 32kHz, disable PLL, main oscillator
/// Put voltage regulator in standby mode
//------------------------------------------------------------------------------
void LowPowerMode(void)
{
    // MCK=48MHz to MCK=32kHz
    // MCK = SLCK/2 : change source first from 48 000 000 to 18. / 2 = 9M
    AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_PRES_CLK_2;
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY ) );
    // MCK=SLCK : then change prescaler
    AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_CSS_SLOW_CLK;
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY ) );
    // disable PLL
    AT91C_BASE_PMC->PMC_PLLR = 0;
    // Disable Main Oscillator
    AT91C_BASE_PMC->PMC_MOR = 0;

    // Voltage regulator in standby mode : Enable VREG Low Power Mode
    AT91C_BASE_VREG->VREG_MR |= AT91C_VREG_PSTDBY;

    PMC_DisableProcessorClock();
}

//------------------------------------------------------------------------------
/// Put voltage regulator in normal mode
/// Return the CPU to normal speed 48MHz, enable PLL, main oscillator
//------------------------------------------------------------------------------
void NormalPowerMode(void)
{
    // Voltage regulator in normal mode : Disable VREG Low Power Mode
    AT91C_BASE_VREG->VREG_MR &= (AT91_REG)(~AT91C_VREG_PSTDBY);

    // MCK=32kHz to MCK=48MHz
    // enable Main Oscillator
    AT91C_BASE_PMC->PMC_MOR = (( (AT91C_CKGR_OSCOUNT & (0x06 <<8)) | AT91C_CKGR_MOSCEN ));
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS ) );

    // enable PLL@96MHz
    AT91C_BASE_PMC->PMC_PLLR = ((AT91C_CKGR_DIV & 0x0E) |
         (AT91C_CKGR_PLLCOUNT & (28<<8)) |
         (AT91C_CKGR_MUL & (0x48<<16)));
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCK ) );
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY ) );
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1 ;
    // MCK=SLCK/2 : change prescaler first
    AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_PRES_CLK_2;
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY ) );
    // MCK=PLLCK/2 : then change source
    AT91C_BASE_PMC->PMC_MCKR |= AT91C_PMC_CSS_PLL_CLK  ;
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY ) );

}

//------------------------------------------------------------------------------
//         Callbacks re-implementation
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Invoked when the USB device leaves the Suspended state. By default,
/// configures the LEDs.
//------------------------------------------------------------------------------
void USBDCallbacks_Resumed(void)
{
    // Initialize LEDs
    LED_Configure(USBD_LEDPOWER);
    LED_Set(USBD_LEDPOWER);
    LED_Configure(USBD_LEDUSB);
    LED_Clear(USBD_LEDUSB);
    USBState = STATE_RESUME;
}

//------------------------------------------------------------------------------
/// Invoked when the USB device gets suspended. By default, turns off all LEDs.
//------------------------------------------------------------------------------
void USBDCallbacks_Suspended(void)
{
    // Turn off LEDs
    LED_Clear(USBD_LEDPOWER);
    LED_Clear(USBD_LEDUSB);
    if (USBD_GetState() >= USBD_STATE_CONFIGURED)
        USBState = STATE_SUSPEND;
}

//------------------------------------------------------------------------------
//         Exported function
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Initializes the system and then monitors buttons, sending the
/// corresponding character when one is pressed.
//------------------------------------------------------------------------------

int main(void)
{
    unsigned char bmButtons = 0;

    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- USB Device HID Mouse Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    // SPP +
    int i;

    button_flag = 0;

    //Enable RESET
    m_pRSTC->RSTC_RCR = 0xA5000008;
    m_pRSTC->RSTC_RMR = 0xA5000001;
    Delay(1000);

    // Init USB device
    //AT91F_USB_Open();

    // Configure the RTT:
    *AT91C_RTTC_RTMR = BUTTON_SAMPLING;

    // Set in PIO mode and Configure in Input
    //AT91F_PIOA_CfgPMC();

    // InitMAM
    InitMMA();

    // InitADC
    ADCInit();

    // UART0 Init
    InitUSART0();

    // Stat led
    s_pPio->PIO_PER   = BIT31;
    // Configure PA31 as output
    s_pPio->PIO_OER   = BIT31;
    // Set PA31 to HIGH
    s_pPio->PIO_SODR  = BIT31;


    // Test flash
    if(TestFlash()) 
    {
      for(i=0; i<20; i++) 
      {
        s_pPio->PIO_CODR  = BIT31; Delay(150000);
        s_pPio->PIO_SODR  = BIT31; Delay(150000);
      }
    }

    // Button
    // Configure P20 as input
    s_pPio->PIO_ODR   = BIT20;
    // Enable
    s_pPio->PIO_PER   = BIT20;

    Delay(1000);

    // CALIBRATE WHEN BUTT IS PRESS
    // while((s_pPio->PIO_PDSR&BIT20)==BIT20);

    Delay(1000);
    // SPP -

    // If they are present, configure Vbus & Wake-up pins
    PIO_InitializeInterrupts(0);

    WAKEUP_CONFIGURE();

    // If there is on board power, switch it off
  #ifdef PIN_USB_POWER_ENB
  { const Pin pinUsbPwr = PIN_USB_POWER_ENB;
    PIO_Configure(&pinUsbPwr, 1);
  }
  #endif

    // Initialize key statuses and configure push buttons
#if defined(at91cap9dk)
    const Pin pinRow0 = PIN_KEYBOARD_ROW0;
    PIO_Configure(&pinRow0, 1);
#endif       
    PIO_Configure(pinsJoystick, PIO_LISTSIZE(pinsJoystick));

    // HID driver initialization
    HIDDMouseDriver_Initialize();

    // connect if needed
    VBUS_CONFIGURE();
    
    // Infinite loop
    while (1) 
    {
       // SPP +
       #define TIMEOUT  100
       static int Timeout = TIMEOUT;
        GetCoordinates();
        base_y = Coordinates[0]/NSAMPLE;
        base_x = Coordinates[1]/NSAMPLE;
        base_z = Coordinates[2]/NSAMPLE;
       // SPP -

        if( USBState == STATE_SUSPEND ) 
        {
            TRACE_DEBUG("suspend  !\n\r");
            USBState = STATE_IDLE;
            LowPowerMode();
        }
        if( USBState == STATE_RESUME ) 
        {
            // Return in normal MODE
            NormalPowerMode();
            USBState = STATE_IDLE;
            TRACE_DEBUG("resume !\n\r");
        }
        if (USBD_GetState() < USBD_STATE_CONFIGURED)
            continue;

        if (!Timeout)
        {
        
            unsigned char status;

            do 
            {
               status = HIDDMouseDriver_ChangePoints(bmButtons, base_x, base_y);
            }
            while (status != USBD_STATUS_SUCCESS);
            Timeout = TIMEOUT;
        }
        else
           Timeout--;
        
    }
}


