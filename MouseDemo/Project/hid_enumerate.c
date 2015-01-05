//*----------------------------------------------------------------------------
//*      ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : cdc_enumerate.c
//* Object              : Handle HID enumeration
//*
//* 1.0 Oct 05 2004 	: ODi Creation
//*----------------------------------------------------------------------------
#include "board.h"
#include "hid_enumerate.h"

typedef unsigned char  uchar;
typedef unsigned short ushort;
typedef unsigned int   uint;

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define EP_NUMBER 1


const unsigned short mouseDescriptor[] = {
  0x0105, // Usage Page (Generic Desktop)
  0x0209, // Usage (Mouse)
  0x01A1, // Collection (Application)
  0x0109, //  Usage (Pointer)
  0x00A1, //  Collection (Physical)
  0x0905, //    Usage Page (Buttons)
  0x0119, //    Usage Minimumù (01)
  0x0329, //    Usage Maximum (03)
  0x0015, //    Logical Minimum (0)
  0x0125, //    Logical Maximum (1)
  0x0395, //    Report Count (3)
  0x0175, //    Report Size (1)
  0x0281, //    3 Button bits
  0x0195, //    Report Count (1)
  0x0575, //    Report Size (6)
  0x0181, //    6 bit padding
  0x0105, //    Generic desktop
  0x3009, //    Usage (X)
  0x3109, //    Usage(Y)
  0x8115, //    Logical Minimum (-127)
  0x7F25, //    Logical Maximum (127)
  0x0875, //    Report Size (8)
  0x0295, //    Report Count (2)
  0x0681, //    2 position bytes
  0xC0C0
};

// Check http://www.usb.org/developers/hidpage/#Class_Definition
const char devDescriptor[] = {
	/* Device descriptor */
	0x12,   // bLength
	0x01,   // bDescriptorType
	0x10,   // bcdUSBL
	0x01,   //
	0x00,   // bDeviceClass:
	0x00,   // bDeviceSubclass:
	0x00,   // bDeviceProtocol:
	0x08,   // bMaxPacketSize0
	0xFF,   // idVendorL
	0xFF,   //
	0x00,   // idProductL
	0x00,   //
	0x01,   // bcdDeviceL
	0x00,   //
	0x00,   // iManufacturer    // 0x01
	0x00,   // iProduct
	0x00,   // SerialNumber
	0x01    // bNumConfigs
};

const char cfgDescriptor[] = {
	/* ============== CONFIGURATION 1 =========== */
	/* Configuration 1 descriptor */
	0x09,   // CbLength
	0x02,   // CbDescriptorType
	0x22,   // CwTotalLength 2 EP + Control
	0x00,
	0x01,   // CbNumInterfaces
	0x01,   // CbConfigurationValue
	0x00,   // CiConfiguration
	0xA0,   // CbmAttributes Bus powered + Remote Wakeup
	0x32,   // CMaxPower: 100mA

	/* Mouse Interface Descriptor Requirement */
	0x09, // bLength
	0x04, // bDescriptorType
	0x00, // bInterfaceNumber
	0x00, // bAlternateSetting
	0x01, // bNumEndpoints
	0x03, // bInterfaceClass: HID code
	0x01, // bInterfaceSubclass
	0x02, // bInterfaceProtocol: Mouse
	0x00, // iInterface

	/* HID Descriptor */
	0x09, // bLength
	0x21, // bDescriptor type: HID Descriptor Type
	0x00, // bcdHID
	0x01,
	0x00, // bCountryCode
	0x01, // bNumDescriptors
	0x22, // bDescriptorType
	sizeof(mouseDescriptor), // wItemLength
	0x00,

	/* Endpoint 1 descriptor */
	0x07,   // bLength
	0x05,   // bDescriptorType
	0x80 + EP_NUMBER,   // bEndpointAddress, Endpoint 01 - OUT
	0x03,   // bmAttributes      INT
	0x04,   // wMaxPacketSize: 3 bytes (button, x, y)
	0x00,
	0x0A    // bInterval
};



/* USB standard request code */
#define STD_GET_STATUS_ZERO           0x0080
#define STD_GET_STATUS_INTERFACE      0x0081
#define STD_GET_STATUS_ENDPOINT       0x0082

#define STD_CLEAR_FEATURE_ZERO        0x0100
#define STD_CLEAR_FEATURE_INTERFACE   0x0101
#define STD_CLEAR_FEATURE_ENDPOINT    0x0102

#define STD_SET_FEATURE_ZERO          0x0300
#define STD_SET_FEATURE_INTERFACE     0x0301
#define STD_SET_FEATURE_ENDPOINT      0x0302

#define STD_SET_ADDRESS               0x0500
#define STD_GET_DESCRIPTOR            0x0680
#define STD_SET_DESCRIPTOR            0x0700
#define STD_GET_CONFIGURATION         0x0880
#define STD_SET_CONFIGURATION         0x0900
#define STD_GET_INTERFACE             0x0A81
#define STD_SET_INTERFACE             0x0B01
#define STD_SYNCH_FRAME               0x0C82

/* HID Class Specific Request Code */
#define STD_GET_HID_DESCRIPTOR        0x0681
#define STD_SET_IDLE                  0x0A21

static uchar AT91F_UDP_IsConfigured(AT91PS_HID);
static void AT91F_HID_SendReport(AT91PS_HID, char button, signed char x, signed char y);
static void AT91F_HID_Enumerate(AT91PS_HID);


//*----------------------------------------------------------------------------
//* \fn    AT91F_HID_Open
//* \brief
//*----------------------------------------------------------------------------
AT91PS_HID AT91F_HID_Open(AT91PS_HID pHid, AT91PS_UDP pUdp)
{
	pHid->pUdp = pUdp;
	pHid->currentConfiguration = 0;
	pHid->IsConfigured = AT91F_UDP_IsConfigured;
	pHid->SendReport   = AT91F_HID_SendReport;
	return pHid;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_UDP_IsConfigured
//* \brief Test if the device is configured and handle enumeration
//*----------------------------------------------------------------------------
static uchar AT91F_UDP_IsConfigured(AT91PS_HID pHid)
{
	AT91PS_UDP pUDP = pHid->pUdp;
	AT91_REG isr = pUDP->UDP_ISR;

	if (isr & AT91C_UDP_ENDBUSRES) {
		pUDP->UDP_ICR = AT91C_UDP_ENDBUSRES;
		// reset all endpoints
		pUDP->UDP_RSTEP  = 0xf;
		pUDP->UDP_RSTEP  = 0;
		// Enable the function
		pUDP->UDP_FADDR = AT91C_UDP_FEN;
		// Configure endpoint 0
		pUDP->UDP_CSR[0] = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_CTRL);
	}
	else if (isr & AT91C_UDP_EPINT0) {
		pUDP->UDP_ICR = AT91C_UDP_EPINT0;
		AT91F_HID_Enumerate(pHid);
	}
	return pHid->currentConfiguration;
}


//*----------------------------------------------------------------------------
//* \fn    AT91F_HID_SendCoordinates
//* \brief Send Data through the control endpoint
//*----------------------------------------------------------------------------
static void AT91F_HID_SendReport(AT91PS_HID pHid, char button, signed char x, signed char y)
{
	AT91PS_UDP pUdp = pHid->pUdp;
	
	// Send report to the host
	pUdp->UDP_FDR[EP_NUMBER] = button;
	pUdp->UDP_FDR[EP_NUMBER] = x;
	pUdp->UDP_FDR[EP_NUMBER] = y;
	pUdp->UDP_CSR[EP_NUMBER] |= AT91C_UDP_TXPKTRDY;

	// Wait for the end of transmission
	while ( !(pUdp->UDP_CSR[EP_NUMBER] & AT91C_UDP_TXCOMP) )
		AT91F_UDP_IsConfigured(pHid);
		
	// Clear AT91C_UDP_TXCOMP flag
	if (pUdp->UDP_CSR[EP_NUMBER] & AT91C_UDP_TXCOMP) {
		pUdp->UDP_CSR[EP_NUMBER] &= (AT91_REG)(~(AT91C_UDP_TXCOMP));
		while (pUdp->UDP_CSR[EP_NUMBER] & AT91C_UDP_TXCOMP);
	}
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_USB_SendData
//* \brief Send Data through the control endpoint
//*----------------------------------------------------------------------------
static void AT91F_USB_SendData(AT91PS_UDP pUdp, const char *pData, uint length)
{
	uint cpt = 0;
	AT91_REG csr;

	do {
		cpt = MIN(length, 8);
		length -= cpt;

		while (cpt--)
			pUdp->UDP_FDR[0] = *pData++;

		if (pUdp->UDP_CSR[0] & AT91C_UDP_TXCOMP) {
			pUdp->UDP_CSR[0] &= (AT91_REG)(~(AT91C_UDP_TXCOMP));
			while (pUdp->UDP_CSR[0] & AT91C_UDP_TXCOMP);
		}

		pUdp->UDP_CSR[0] |= AT91C_UDP_TXPKTRDY;
		do {
			csr = pUdp->UDP_CSR[0];

			// Data IN stage has been stopped by a status OUT
			if (csr & AT91C_UDP_RX_DATA_BK0) {
				pUdp->UDP_CSR[0] &= (AT91_REG)(~(AT91C_UDP_RX_DATA_BK0));
				return;
			}
		} while ( !(csr & AT91C_UDP_TXCOMP) );

	} while (length);

	if (pUdp->UDP_CSR[0] & AT91C_UDP_TXCOMP) {
		pUdp->UDP_CSR[0] &= (AT91_REG)(~(AT91C_UDP_TXCOMP));
		while (pUdp->UDP_CSR[0] & AT91C_UDP_TXCOMP);
	}
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_USB_SendZlp
//* \brief Send zero length packet through the control endpoint
//*----------------------------------------------------------------------------
void AT91F_USB_SendZlp(AT91PS_UDP pUdp)
{
	pUdp->UDP_CSR[0] |= AT91C_UDP_TXPKTRDY;
	while ( !(pUdp->UDP_CSR[0] & AT91C_UDP_TXCOMP) );
	pUdp->UDP_CSR[0] &= (AT91_REG)(~(AT91C_UDP_TXCOMP));
	while (pUdp->UDP_CSR[0] & AT91C_UDP_TXCOMP);
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_USB_SendStall
//* \brief Stall the control endpoint
//*----------------------------------------------------------------------------
void AT91F_USB_SendStall(AT91PS_UDP pUdp)
{
	pUdp->UDP_CSR[0] |= AT91C_UDP_FORCESTALL;
	while ( !(pUdp->UDP_CSR[0] & AT91C_UDP_ISOERROR) );
	pUdp->UDP_CSR[0] &= (AT91_REG)(~(AT91C_UDP_FORCESTALL | AT91C_UDP_ISOERROR));
	while (pUdp->UDP_CSR[0] & (AT91C_UDP_FORCESTALL | AT91C_UDP_ISOERROR));
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_HID_Enumerate
//* \brief This function is a callback invoked when a SETUP packet is received
//*----------------------------------------------------------------------------
static void AT91F_HID_Enumerate(AT91PS_HID pHid)
{
	AT91PS_UDP pUDP = pHid->pUdp;
	uchar bmRequestType, bRequest;
	ushort wValue, wIndex, wLength, wStatus;


	if ( !(pUDP->UDP_CSR[0] & AT91C_UDP_RXSETUP) )
		return;

	bmRequestType = pUDP->UDP_FDR[0];
	bRequest      = pUDP->UDP_FDR[0];
	wValue        = (pUDP->UDP_FDR[0] & 0xFF);
	wValue       |= (pUDP->UDP_FDR[0] << 8);
	wIndex        = (pUDP->UDP_FDR[0] & 0xFF);
	wIndex       |= (pUDP->UDP_FDR[0] << 8);
	wLength       = (pUDP->UDP_FDR[0] & 0xFF);
	wLength      |= (pUDP->UDP_FDR[0] << 8);

	if (bmRequestType & 0x80) {
		pUDP->UDP_CSR[0] |= AT91C_UDP_DIR;
		while ( !(pUDP->UDP_CSR[0] & AT91C_UDP_DIR) );
	}
	pUDP->UDP_CSR[0] &= (AT91_REG)(~AT91C_UDP_RXSETUP);
	while ( (pUDP->UDP_CSR[0]  & AT91C_UDP_RXSETUP)  );

	// Handle supported standard device request Cf Table 9-3 in USB specification Rev 1.1
	switch ((bRequest << 8) | bmRequestType) {
	case STD_GET_DESCRIPTOR:
		if (wValue == 0x100)       // Return Device Descriptor
			AT91F_USB_SendData(pUDP, devDescriptor, MIN(sizeof(devDescriptor), wLength));
		else if (wValue == 0x200)  // Return Configuration Descriptor
			AT91F_USB_SendData(pUDP, cfgDescriptor, MIN(sizeof(cfgDescriptor), wLength));
		else
			AT91F_USB_SendStall(pUDP);
		break;
	case STD_SET_ADDRESS:
		AT91F_USB_SendZlp(pUDP);
		pUDP->UDP_FADDR = (AT91C_UDP_FEN | wValue);
		pUDP->UDP_GLBSTATE  = (wValue) ? AT91C_UDP_FADDEN : 0;
		break;
	case STD_SET_CONFIGURATION:
		pHid->currentConfiguration = wValue;
		AT91F_USB_SendZlp(pUDP);
		pUDP->UDP_GLBSTATE  = (wValue) ? AT91C_UDP_CONFG : AT91C_UDP_FADDEN;
		pUDP->UDP_CSR[EP_NUMBER] = (wValue) ? (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_IN)   : 0;
		break;
	case STD_GET_CONFIGURATION:
		AT91F_USB_SendData(pUDP, (char *) &(pHid->currentConfiguration), sizeof(pHid->currentConfiguration));
		break;
	case STD_GET_STATUS_ZERO:
		wStatus = 0;
		AT91F_USB_SendData(pUDP, (char *) &wStatus, sizeof(wStatus));
		break;
	case STD_GET_STATUS_INTERFACE:
		wStatus = 0;
		AT91F_USB_SendData(pUDP, (char *) &wStatus, sizeof(wStatus));
		break;
	case STD_GET_STATUS_ENDPOINT:
		wStatus = 0;
		wIndex &= 0x0F;
		if ((pUDP->UDP_GLBSTATE & AT91C_UDP_CONFG) && (wIndex <= 3)) {
			wStatus = (pUDP->UDP_CSR[wIndex] & AT91C_UDP_EPEDS) ? 0 : 1;
			AT91F_USB_SendData(pUDP, (char *) &wStatus, sizeof(wStatus));
		}
		else if ((pUDP->UDP_GLBSTATE & AT91C_UDP_FADDEN) && (wIndex == 0)) {
			wStatus = (pUDP->UDP_CSR[wIndex] & AT91C_UDP_EPEDS) ? 0 : 1;
			AT91F_USB_SendData(pUDP, (char *) &wStatus, sizeof(wStatus));
		}
		else
			AT91F_USB_SendStall(pUDP);
		break;
	case STD_SET_FEATURE_ZERO:
		AT91F_USB_SendStall(pUDP);
	    break;
	case STD_SET_FEATURE_INTERFACE:
		AT91F_USB_SendZlp(pUDP);
		break;
	case STD_SET_FEATURE_ENDPOINT:
		wIndex &= 0x0F;
		if ((wValue == 0) && wIndex && (wIndex <= 3)) {
			pUDP->UDP_CSR[wIndex] = 0;
			AT91F_USB_SendZlp(pUDP);
		}
		else
			AT91F_USB_SendStall(pUDP);
		break;
	case STD_CLEAR_FEATURE_ZERO:
		AT91F_USB_SendStall(pUDP);
	    break;
	case STD_CLEAR_FEATURE_INTERFACE:
		AT91F_USB_SendZlp(pUDP);
		break;
	case STD_CLEAR_FEATURE_ENDPOINT:
		wIndex &= 0x0F;
		if ((wValue == 0) && wIndex && (wIndex <= 3)) {
			if (wIndex == 1)
				pUDP->UDP_CSR[1] = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_OUT);
			else if (wIndex == 2)
				pUDP->UDP_CSR[2] = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_IN);
			else if (wIndex == 3)
				pUDP->UDP_CSR[3] = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_ISO_IN);
			AT91F_USB_SendZlp(pUDP);
		}
		else
			AT91F_USB_SendStall(pUDP);
		break;

	// handle HID class requests
	case STD_GET_HID_DESCRIPTOR:
		if (wValue == 0x2200)       // Return Mouse Descriptor
			AT91F_USB_SendData(pUDP, (const char *) mouseDescriptor, MIN(sizeof(mouseDescriptor), wLength));
		else
			AT91F_USB_SendStall(pUDP);
		break;

	case STD_SET_IDLE:
		AT91F_USB_SendZlp(pUDP);
		break;
		
	default:
		AT91F_USB_SendStall(pUDP);
	    break;
	}
}
