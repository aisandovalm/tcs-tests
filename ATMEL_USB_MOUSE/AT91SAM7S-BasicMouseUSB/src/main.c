//*--------------------------------------------------------------------------------------
//*      ATMEL Microcontroller Software Support  -  ROUSSET  -
//*--------------------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*--------------------------------------------------------------------------------------
//* File Name           : USB HID example
//* Object              :
//* Translator          :
//* 1.0 05/Oct/04 ODi	: CReation
//* 1.1 04/Nov/04 JPP	: Add led1 On at power supply
//*--------------------------------------------------------------------------------------

#include "board.h"
#include "dbgu.h"
#include "hid_enumerate.h"

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

struct _AT91S_HID 	HID;

//*----------------------------------------------------------------------------
//* \fn    AT91F_USB_Open
//* \brief This function Open the USB device
//*----------------------------------------------------------------------------
void AT91F_USB_Open(void)
{
    // Set the PLL USB Divider
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1 ;

    // Specific Chip USB Initialisation
    // Enables the 48MHz USB clock UDPCK and System Peripheral USB Clock
    AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_UDP;
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_UDP);

    // Enable UDP PullUp (USB_DP_PUP) : enable & Clear of the corresponding PIO
    // Set in PIO mode and Configure in Output
    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA,AT91C_PIO_PA25);
    // Clear for set the Pul up resistor
    AT91F_PIO_SetOutput(AT91C_BASE_PIOA,AT91C_PIO_PA25);
    //AT91F_PIO_ClearOutput(AT91C_BASE_PIOA,AT91C_PIO_PA25);

    //AT91F_PIO_CfgInput(AT91C_BASE_PIOA,AT91C_PIO_PA24);
    //AT91F_PIO_CfgPullup(AT91C_BASE_PIOA,~AT91C_PIO_PA24);
    //while(!(AT91F_PIO_GetInput(AT91C_BASE_PIOA)&AT91C_PIO_PA24));

    // CDC Open by structure initialization
    AT91F_HID_Open(&HID, AT91C_BASE_UDP);
}


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
AT91PS_SYSC   a_pSys = AT91C_BASE_SYSC;

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
void Delay (unsigned long a) {
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


void write_str_USART0(unsigned char* buff) {

  unsigned int i = 0x0;

  while(buff[i] != '\0') {
    write_char_USART0(buff[i]);
    i++;
  }
}

void write_report_USART0(void) {

  if(print_view == PRINT_COORDINATES) {
    tmp_x = Coordinates[0]/NSAMPLE;
    tmp_y = Coordinates[1]/NSAMPLE;
    tmp_z = Coordinates[2]/NSAMPLE;
  }
  else {
    tmp_x = base_x - (Coordinates[0]/NSAMPLE);
    tmp_y = base_y - (Coordinates[1]/NSAMPLE);
    tmp_z = base_z - (Coordinates[2]/NSAMPLE);
  }



  // X
  if(print_view == PRINT_DIVERSION) {
    write_str_USART0(" dX: ");
    if(tmp_x<0) {
      tmp_x = (tmp_x)*(-1);
      write_char_USART0('-');
    }
  }
  else {
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
  if(print_view == PRINT_DIVERSION) {
    write_str_USART0(" dY: ");
    if(tmp_y<0) {
      tmp_y = (tmp_y)*(-1);
      write_char_USART0('-');
    }
  }
  else {
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
  if(print_view == PRINT_DIVERSION) {
    write_str_USART0(" dZ: ");
    if(tmp_z<0) {
      tmp_z = (tmp_z)*(-1);
      write_char_USART0('-');
    }
  }
  else {
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

void GetCoordinates (void) {

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


void InitMMA(void) {

  //enable the clock of the PIO
  a_pPMC->PMC_PCER = 1 << AT91C_ID_PIOA;

  // Config
  // Configure all ports as Input
  a_pPio->PIO_ODR           = 0xffffffff;       Delay(100);
  // Enable all ports
  a_pPio->PIO_PER           = 0xffffffff;       Delay(100);
  // Disable Pull-up resistor
  a_pSys->SYSC_PIOA_PPUDR   = 0xffffffff;       Delay(100);


  // Set G_SEL1, G_SEL2 to low and SM to high
  a_pPio->PIO_SODR = 0x20000000;   // Set SM to HIGH
  a_pPio->PIO_CODR = 0x18000000;   // Set G_SEL1, G_SEL2 to LOW

  // Set G_SEL1, G_SEL2 and SM as output
  a_pPio->PIO_OER = 0x38000000;

}

void FlashEnable(void) {
  // Set PA11 to LOW
  s_pPio->PIO_CODR  = BIT11;
}

void FlashDisable(void) {
  // Set PA11 to HIGH
  s_pPio->PIO_SODR  = BIT11;
}

void InitSPI(void) {

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

unsigned char WriteByte(unsigned char data) {

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

void ByteToBuffer(unsigned int byte, unsigned int address) {
	
	unsigned int 	addr	= 0;
	unsigned int 	page	= 0;

	page = address/264;
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

void FlashReadArray(unsigned long address, unsigned char *buffer, unsigned char length) {

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


void WriteBufferToMainMemory(unsigned int address) {
	
    unsigned int addr	= 0;
    unsigned int page	= 0;

    page = address/264;
    addr = address%264;	

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

void ReadBufferFromMainMemory(unsigned int address) {

  unsigned int addr	= 0;
  unsigned int page	= 0;

  page = address/264;
  addr = address%264;	

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

unsigned char TestFlash(void) {

    // Stat led
    s_pPio->PIO_PER   = BIT31;
    s_pPio->PIO_OER   = BIT31;    // Configure PA11 as output
    s_pPio->PIO_SODR  = BIT31;    // Set PA11 to HIGH

    char i;

    // Test led and flash memory
    InitSPI();

    // Write WriteBuffer
    for(i=0; i<10; i++) {
      ByteToBuffer(WriteBuffer[i], i);
    }
    WriteBufferToMainMemory(0);

    // Read ReadBuffer
    FlashReadArray(0, ReadBuffer, 10);

    // check for Correct Read
    for(i=0; i<10; i++) {

      if(WriteBuffer[i]!=ReadBuffer[i]) {
        return 0;
      }
    }

    return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////

//*--------------------------------------------------------------------------------------
//* Function Name       : main
//* Object              :
//*--------------------------------------------------------------------------------------
int main ( void )
{
//    char button = 0;
//    int x = 0, y = 0, j = 0;
//    int adc;
//    int i = 0;
    int x,y,z,i;

    button_flag = 0;

    //Init trace DBGU
    //AT91F_DBGU_Init();
    //AT91F_DBGU_Printk("\n\r-I- BasicUSB 1.1 (USB_DP_PUP) \n\r0) Set Pull-UP 1) Clear Pull UP\n\r");

    //Enable RESET
    m_pRSTC->RSTC_RCR = 0xA5000008;
    m_pRSTC->RSTC_RMR = 0xA5000001;
    Delay(1000);

    // Init USB device
    AT91F_USB_Open();

    // Configure the RTT:
    *AT91C_RTTC_RTMR = BUTTON_SAMPLING;

    // Set in PIO mode and Configure in Input
    AT91F_PIOA_CfgPMC();
    // AT91F_PIO_CfgInput(AT91C_BASE_PIOA, (SW1|SW2));

   // Wait for the end of enumeration
   // while (!HID.IsConfigured(&HID));

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
    if(TestFlash()) {

      for(i=0; i<20; i++) {
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

    // Get coordinates
    GetCoordinates();

    base_y = Coordinates[0]/NSAMPLE;
    base_x = Coordinates[1]/NSAMPLE;
    base_z = Coordinates[2]/NSAMPLE;


    // Start waiting some cmd
    while (1) {

      // Check enumeration
      if (HID.IsConfigured(&HID)) {

        // check button status
        if((s_pPio->PIO_PDSR&BIT20)==0) {

          // Set PA31 to LOW
          s_pPio->PIO_CODR  = BIT31;

          // set button flag
          button_flag = 0x01;

          Delay(1000);

        }
        else {

          // Set PA31 to HIGH
          // s_pPio->PIO_SODR  = BIT31;

          // clear button flag
          button_flag = 0x00;

        }


        GetCoordinates();

        Delay(10000);

//*        if((j++)==100) {
//*
//*         print_view = PRINT_COORDINATES;
//*         write_report_USART0();
//*
//*         // print_view = PRINT_DIVERSION;
//*         // write_report_USART0();
//*
//*         j=0;
//*        }

        y = 478 - Coordinates[0]/NSAMPLE;
        // y = base_y - Coordinates[0]/NSAMPLE;

        if(y>40) {
          if(y<50)
            HID.SendReport(&HID, button_flag, 0, -1);
          else if (y<100)
            HID.SendReport(&HID, button_flag, 0, -2);
          else if (y<150)
            HID.SendReport(&HID, button_flag, 0, -3);
          else if (y<200)
            HID.SendReport(&HID, button_flag, 0, -6);
          else
            HID.SendReport(&HID, button_flag, 0, -16);
        }
        else if(y<-40){
        if(y>-50)
            HID.SendReport(&HID, button_flag, 0, 1);
          else if (y>-100)
            HID.SendReport(&HID, button_flag, 0, 2);
          else if (y>-150)
            HID.SendReport(&HID, button_flag, 0, 3);
          else if (y>-200)
            HID.SendReport(&HID, button_flag, 0, 6);
          else
            HID.SendReport(&HID, button_flag, 0, 16);

        }

        x = 565 - Coordinates[1]/NSAMPLE;
        // x = base_x - Coordinates[1]/NSAMPLE;

        if(x>40) {
          if(x<50)
            HID.SendReport(&HID, button_flag, -1, 0);
          else if (x<100)
            HID.SendReport(&HID, button_flag, -2, 0);
          else if (x<150)
            HID.SendReport(&HID, button_flag, -3, 0);
          else if (x<200)
            HID.SendReport(&HID, button_flag, -6, 0);
          else
            HID.SendReport(&HID, button_flag, -16, 0);
        }
        else if(x<-40){
          if(x>-50)
            HID.SendReport(&HID, button_flag, 1, 0);
          else if (x>-100)
            HID.SendReport(&HID, button_flag, 2, 0);
          else if (x>-150)
            HID.SendReport(&HID, button_flag, 3, 0);
          else if (x>-200)
            HID.SendReport(&HID, button_flag, 6, 0);
          else
            HID.SendReport(&HID, button_flag, 16, 0);

        }

        // just test Z coordinate
        z = Coordinates[2]/NSAMPLE;

        if(z<300) {
          s_pPio->PIO_CODR  = BIT31;
        }
        else {
          s_pPio->PIO_SODR  = BIT31;
        }

        Delay(20);



//*      x = ((150) - GetADC4())/20;
//*      HID.SendReport(&HID, button, x, 0);
//*      Delay(50);
//*      HID.SendReport(&HID, button, ((base_y) - GetADC5())/50, 0);
//*      Delay(50);
//*      HID.SendReport(&HID, button, ((base_z) - GetADC6())/50, 0);
//*      Delay(50);
//*
//*       button = 0;
//*
//*       for(j=0; j<100; j++) {
//*           HID.SendReport(&HID, button, 1, 0);
//*           Delay(50);
//*       }
//*
//*       for(j=0; j<100; j++) {
//*           HID.SendReport(&HID, button, 0, 1);
//*           Delay(50);
//*       }
//*
//*       for(j=0; j<100; j++) {
//*         HID.SendReport(&HID, button, -1, 0);
//*         Delay(50);
//*       }
//*
//*       for(j=0; j<100; j++) {
//*         HID.SendReport(&HID, button, 0, -1);
//*         Delay(50);
//*       }

      }
   }
}


