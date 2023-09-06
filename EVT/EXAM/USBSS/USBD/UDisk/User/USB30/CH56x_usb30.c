/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "CH56x_usb30.h"
#include "CH56x_common.h"
#include "CH56xusb30_LIB.h"

/* Global Define */
#define UsbSetupBuf     ((PUSB_SETUP)endp0RTbuff)

/* Global Variable */
__attribute__ ((aligned(16))) UINT8	endp0RTbuff[512] __attribute__((section(".DMADATA")));               //Endpoint 0 data send receive buffer
__attribute__ ((aligned(16))) UINT8	endp1RTbuff[MAX_BUF_BLOCK*512] __attribute__((section(".DMADATA"))); //Endpoint 1 data send buffer
__attribute__ ((aligned(16))) UINT8	endp2RTbuff[MAX_BUF_BLOCK*512] __attribute__((section(".DMADATA"))); //Endpoint 2 data send buffer

UINT8V RWsta = 0xff;
UINT8V cswflag = 0;
UINT16V reqblocknum = 0;
UINT16V writeblocknum;
UINT32V curWaddr = 0,curRaddr = 0;

static  UINT32      SetupLen=0;
static  UINT8       SetupReqCode = 0;
static  PUINT8      pDescr;

/*Superspeed device descriptor*/
const UINT8 DeviceDescriptor[] =
{
    0x12,   // bLength
    0x01,   // DEVICE descriptor type
    0x00,   // 3.00
    0x03,
    0x00,   // device class
    0x00,   // device sub-class
    0x00,   // vendor specific protocol
    0x09,   // max packet size 512B
    0x86,   // vendor id-0x1A86(qinheng)
    0x1a,
    0x10,   // product id
    0xfe,
    0x00,   //bcdDevice 0x0011
    0x01,
    0x01,   // manufacturer index string
    0x02,   // product index string
    0x03,   // serial number index string
    0x01    // number of configurations
};

/*Superspeed Configuration Descriptor*/
const UINT8 ConfigDescriptor[] =
{
    0x09,   // length of this descriptor
    0x02,   // CONFIGURATION (2)
    0x2c,   // total length includes endpoint descriptors (should be 1 more than last address)
    0x00,   // total length high byte
    0x01,   // number of interfaces
    0x01,   // configuration value for this one
    0x00,   // configuration - string is here, 0 means no string
    0x80,   // attributes - bus powered, no wakeup
    0x64,   // max power - 800 ma is 100 (64 hex)

    0x09,   // length of the interface descriptor
    0x04,   // INTERFACE (4)
    0x00,   // Zero based index 0f this interface
    0x00,   // Alternate setting value (?)
    0x02,   // Number of endpoints (not counting 0)
    0x08,   // Interface class, ff is vendor specific
    0x06,   // Interface sub-class
    0x50,   // Interface protocol
    0x00,   // Index to string descriptor for this interface

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x81,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 1024 bytes
    0x04,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x06,   // length of this endpoint compansion descriptor
    0x30,
    0x00,   // max burst size
    0x00,   // no stream
    0x00,
    0x00,

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x02,   // endpoint direction (00 is out) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 1024 bytes
    0x04,   // max packet size - high
    0x00,    // polling interval in milliseconds (1 for iso)

    0x06,   // length of this endpoint compansion descriptor
    0x30,
    0x00,   // max burst size
    0x00,   // no stream
    0x00,
    0x00,

};

/*String Descriptor Lang ID*/
const UINT8 StringLangID[] =
{
    0x04,   // this descriptor length
    0x03,   // descriptor type
    0x09,   // Language ID 0 low byte
    0x04    // Language ID 0 high byte
};

/*String Descriptor Vendor*/
const UINT8 StringVendor[] =
{
    0x08,   // length of this descriptor
    0x03,
    'W',
    0x00,
    'C',
    0x00,
    'H',
    0x00
};

/*String Descriptor Product*/
const UINT8 StringProduct[] =
{
    0x1e,   // length of this descriptor
    0x03,
    'U',
    0x00,
    'S',
    0x00,
    'B',
    0x00,
    '3',
    0x00,
    '.',
    0x00,
    '0',
    0x00,
    ' ',
    0x00,
    'U',
    0x00,
    'D',
    0x00,
    'I',
    0x00,
    'S',
    0x00,
    'K',
    0x00,
    '!',
    0x00,
    ' ',
    0x00
};

/*String Descriptor Serial*/
UINT8 StringSerial[] =
{
    0x16,   // length of this descriptor
    0x03,
    '0',
    0x00,
    '1',
    0x00,
    '2',
    0x00,
    '3',
    0x00,
    '4',
    0x00,
    '5',
    0x00,
    '6',
    0x00,
    '7',
    0x00,
    '8',
    0x00,
    '9',
    0x00,
};

const UINT8 OSStringDescriptor[] =
{
    0x12,   // length of this descriptor
    0x03,
    'M',
    0x00,
    'S',
    0x00,
    'F',
    0x00,
    'T',
    0x00,
    '1',
    0x00,
    '0',
    0x00,
    '0',
    0x00,
    0x01,
    0x00
};

const UINT8 BOSDescriptor[] =
{
    0x05,   // length of this descriptor
    0x0f,   // CONFIGURATION (2)
    0x16,   // total length includes endpoint descriptors (should be 1 more than last address)
    0x00,   // total length high byte
    0x02,   // number of device cap

    0x07,
    0x10,   // DEVICE CAPABILITY type
    0x02,   // USB2.0 EXTENSION
    0x06,
    0x00,
    0x00,
    0x00,

    0x0a,   // length of this descriptor
    0x10,   // DEVICE CAPABILITY type
    0x03,   // superspeed usb device capability
    0x00,   //
    0x0e,   // ss/hs/fs
    0x00,
    0x01,   // the lowest speed is full speed
    0x0a,   // u1 exit latency is 10us
    0xff,   // u1 exit latency is 8us
    0x07
};

const UINT8 MSOS20DescriptorSet[] =
{
    //
    // Microsoft OS 2.0 Descriptor Set Header
    //
    0x0A, 0x00,             // wLength - 10 bytes
    0x00, 0x00,             // MSOS20_SET_HEADER_DESCRIPTOR
    0x00, 0x00, 0x03, 0x06, // dwWindowsVersion  0x06030000 for Windows Blue
    0x48, 0x00,             // wTotalLength  72 bytes
    //
    // Microsoft OS 2.0 Registry Value Feature Descriptor
    //
    0x3E, 0x00,             // wLength - 62 bytes
    0x04, 0x00,             // wDescriptorType  4 for Registry Property
    0x04, 0x00,             // wPropertyDataType - 4 for REG_DWORD
    0x30, 0x00,              // wPropertyNameLength  48 bytes
    0x53, 0x00, 0x65, 0x00, // Property Name - SelectiveSuspendEnabled
    0x6C, 0x00, 0x65, 0x00,
    0x63, 0x00, 0x74, 0x00,
    0x69, 0x00, 0x76, 0x00,
    0x65, 0x00, 0x53, 0x00,
    0x75, 0x00, 0x73, 0x00,
    0x70, 0x00, 0x65, 0x00,
    0x6E, 0x00, 0x64, 0x00,
    0x45, 0x00, 0x6E, 0x00,
    0x61, 0x00, 0x62, 0x00,
    0x6C, 0x00, 0x65, 0x00,
    0x64, 0x00, 0x00, 0x00,
    0x04, 0x00,             // wPropertyDataLength  4 bytes
    0x01, 0x00, 0x00, 0x00  // PropertyData - 0x00000001
};


const UINT8 PropertyHeader[] =
{
        0x8e, 0x00, 0x00, 0x00, 0x00, 01, 05, 00, 01, 00,
        0x84, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x00, 0x00,
        0x28, 0x00,
        0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00, 0x49, 0x00, 0x6e,
        0x00, 0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x66, 0x00, 0x61, 0x00, 0x63, 0x00, 0x65, 0x00, 0x47, 0x00, 0x55, 0x00, 0x49, 0x00, 0x44, 0x00, 0x00, 0x00,

        0x4e, 0x00, 0x00, 0x00, //L"{12345678-1234-1234-1234-123456789ABC"
        0x7b, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00, 0x35, 0x00, 0x36, 0x00, 0x37, 0x00, 0x38, 0x00,
        0x2d, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00,
        0x2d ,0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00,
        0x2d, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00, 0x2d, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00,
        0x34, 0x00, 0x35, 0x00, 0x36, 0x00, 0x37, 0x00, 0x38, 0x00, 0x39, 0x00, 0x41, 0x00, 0x42, 0x00, 0x43, 0x00,
        0x7d, 0x00, 0x00, 0x00
};

const UINT8 CompactId[] =
{
        0x28, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x57, 0x49, 0x4e, 0x55, 0x53, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

UINT8 GetStatus[] =
{
        0x01, 0x00
};

/*******************************************************************************
 * @fn      USB30D_init
 *
 * @brief   USB3.0 initialization
 *
 * @return  None
 */
void USB30D_init(FunctionalState sta) {
    UINT16 i;
    if (sta) {
        USB30_Device_Init();

        USBSS->UEP0_DMA     = (UINT32)(UINT8 *)endp0RTbuff;
        USBSS->UEP1_TX_DMA  = (UINT32)(UINT8 *)endp1RTbuff;
        USBSS->UEP2_TX_DMA  = (UINT32)(UINT8 *)endp2RTbuff;

        USBSS->UEP1_RX_DMA  = (UINT32)(UINT8 *)endp1RTbuff;
        USBSS->UEP2_RX_DMA  = (UINT32)(UINT8 *)endp2RTbuff;


        USBSS->UEP_CFG = EP0_R_EN | EP0_T_EN | EP1_R_EN | EP1_T_EN | EP2_R_EN | EP2_T_EN | EP3_R_EN | EP3_T_EN
                       | EP4_R_EN | EP4_T_EN | EP5_R_EN | EP5_T_EN | EP6_R_EN | EP6_T_EN | EP7_R_EN | EP7_T_EN;// set end point rx/tx enable

//        USB30_OUT_Set(ENDP_1, ACK, NUMP_4);    // endpoint1 receive setting
        USB30_OUT_Set(ENDP_2, ACK, NUMP_4);
    }
    else {
        USB30_Switch_Powermode(POWER_MODE_3);
        USBSS->LINK_CFG = PIPE_RESET | LFPS_RX_PD;
        USBSS->LINK_CTRL = GO_DISABLED | POWER_MODE_3;
        USBSS->LINK_INT_CTRL = 0;
        USBSS->USB_CONTROL = USB_FORCE_RST | USB_ALL_CLR;
    }
}

/*******************************************************************************
 * @fn      USB30_NonStandardReq
 *
 * @brief   USB3.0 Nonstandard request processing function
 *
 * @return  Length
 */
UINT16 USB30_NonStandardReq()
{
		SetupReqCode = UsbSetupBuf->bRequest;
		SetupLen = UsbSetupBuf->wLength;
		UINT16 len = 0;
		switch(SetupReqCode)
		{
		case 0xfe:
		    pDescr = (PUINT8)endp0RTbuff;
		    endp0RTbuff[0]=0;
		    SetupLen = 1;
		    break;
		 case 0x02:																		//用户定义命令
			switch(UsbSetupBuf->wIndexL)
			{
				case 0x05:																//
					 if(SetupLen>SIZE_PropertyHeader) SetupLen  = SIZE_PropertyHeader;
					 pDescr = (PUINT8)PropertyHeader;
					 break;
				default:
					SetupReqCode = INVALID_REQ_CODE;
					return USB_DESCR_UNSUPPORTED;
					break;
			}
			break;
		default:
			printf("stall\n");
			SetupReqCode = INVALID_REQ_CODE;
			return USB_DESCR_UNSUPPORTED;
			break;
		}
		len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen;// This transmission length
		memcpy(endp0RTbuff, pDescr,len );
		SetupLen -= len;
		pDescr += len;
		return len;
}

/*******************************************************************************
* @fn      USB30_StandardReq
*
* @brief   USB3.0 Standard request
*
* @return  Length
*/
UINT16 USB30_StandardReq()
{
	SetupReqCode = UsbSetupBuf->bRequest;
	SetupLen = UsbSetupBuf->wLength;
	UINT16 len = 0;

    if( (UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD )
    {
        len = USB30_NonStandardReq();
    }
    else
    {
        switch(SetupReqCode)
        {
                case USB_GET_DESCRIPTOR:
                    switch(UsbSetupBuf->wValueH)
                    {
                            case USB_DESCR_TYP_DEVICE:
                                    if(SetupLen>SIZE_DEVICE_DESC) SetupLen  = SIZE_DEVICE_DESC;
                                    pDescr = (PUINT8)DeviceDescriptor;
                                    break;
                            case    USB_DESCR_TYP_CONFIG:
                                    if(SetupLen > SIZE_CONFIG_DESC) SetupLen = SIZE_CONFIG_DESC;
                                    pDescr = (PUINT8)ConfigDescriptor;
                                    break;
                            case USB_DESCR_TYP_BOS:
                                    if(SetupLen > SIZE_BOS_DESC) SetupLen = SIZE_BOS_DESC;
                                    pDescr = (PUINT8)BOSDescriptor;
                                    break;
                            case    USB_DESCR_TYP_STRING:
                                    switch(UsbSetupBuf->wValueL)
                                    {
                                    case USB_DESCR_LANGID_STRING:
                                        if(SetupLen > SIZE_STRING_LANGID) SetupLen = SIZE_STRING_LANGID;
                                        pDescr = (PUINT8)StringLangID;
                                        break;
                                    case USB_DESCR_VENDOR_STRING:
                                        if(SetupLen > SIZE_STRING_VENDOR) SetupLen = SIZE_STRING_VENDOR;
                                        pDescr = (PUINT8)StringVendor;
                                        break;
                                    case USB_DESCR_PRODUCT_STRING:
                                        if(SetupLen > SIZE_STRING_PRODUCT) SetupLen = SIZE_STRING_PRODUCT;
                                        pDescr = (PUINT8)StringProduct;
                                        break;
                                    case USB_DESCR_SERIAL_STRING:
                                        if(SetupLen > SIZE_STRING_SERIAL) SetupLen = SIZE_STRING_SERIAL;
                                        pDescr = (PUINT8)StringSerial;
                                        break;
                                    case USB_DESCR_OS_STRING:
                                        if(SetupLen >SIZE_STRING_OS) SetupLen = SIZE_STRING_OS;
                                        pDescr = (PUINT8)OSStringDescriptor;
                                        break;
                                    default:
                                        len = USB_DESCR_UNSUPPORTED;                                //Unsupported descriptor
                                        SetupReqCode = INVALID_REQ_CODE;                            //Invalid request code
                                        break;
                                    }
                                    break;
                            default:
                                len = USB_DESCR_UNSUPPORTED;//Unsupported descriptor
                                SetupReqCode = INVALID_REQ_CODE;
                                break;
                    }
                    len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen;
                    memcpy(endp0RTbuff, pDescr,len );
                    SetupLen -= len;
                    pDescr += len;
                    break;
                case USB_SET_ADDRESS:
                    SetupLen = UsbSetupBuf->wValueL;
                    break;
                case 0x31:
                    SetupLen = UsbSetupBuf->wValueL;
                    break;
                case 0x30:
                    break;
                case USB_SET_CONFIGURATION:
                    break;
                case USB_GET_STATUS:
                    len=2;
                    endp0RTbuff[0]=0x01;
                    endp0RTbuff[1]=0x00;
                    SetupLen = 0;
                    break;
                case USB_CLEAR_FEATURE:
                    switch( UsbSetupBuf->wIndexL )
                    {
                        case 0x82:
                            USB30_IN_ClearIT( ENDP_2 );
                            USB30_IN_Set( ENDP_2, DISABLE , STALL , 0, 0 );
                            break;
                        case 0x02:
                            USB30_OUT_ClearIT( ENDP_2 );
                            USB30_OUT_Set( ENDP_2 , STALL , 0 );
                            break;
                        case 0x81:
                            USB30_IN_ClearIT( ENDP_1 );
                            USB30_IN_Set( ENDP_1, DISABLE , STALL , 0, 0 );
                            break;
                        case 0x01:
                            USB30_OUT_ClearIT( ENDP_1 );
                            USB30_OUT_Set( ENDP_1 , STALL , 0 );
                            break;
                        default:
                            SetupLen = USB_DESCR_UNSUPPORTED;
                            break;
                    }
                    break;
                case USB_SET_FEATURE:
                    break;
                case USB_SET_INTERFACE:
                    break;
                default:
                        len =USB_DESCR_UNSUPPORTED;                                     //return stall
                        SetupReqCode = INVALID_REQ_CODE;
                        printf(" stall \n");
                            break;
                }
    }
	return len;
}

/*******************************************************************************
 * @fn      EP0_IN_Callback
 *
 * @brief   USB3.0 Endpoint 0 IN transaction callback
 *
 * @return  Send length
 */
UINT16 EP0_IN_Callback(void)
{

	UINT16 len = 0;
	switch(SetupReqCode)
	{
		case USB_GET_DESCRIPTOR:
			 len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen;
			 memcpy(  endp0RTbuff, pDescr,len );
			 SetupLen -= len;
			 pDescr += len;
			 break;
	}
	return len;
}

/*******************************************************************************
 * @fn      EP0_OUT_Callback
 *
 * @brief   USB3.0 Endpoint 0 OUT transaction callback
 *
 * @return  Length
 */
UINT16 EP0_OUT_Callback()
{
	return 0;
}

/*******************************************************************************
 * @fn      USB30_Setup_Status
 *
 * @brief   USB3.0 Control transfer status stage callback
 *
 * @return  None
 */
void USB30_Setup_Status( void)
{
	switch(SetupReqCode)
	{
		case USB_SET_ADDRESS:
		    USB30_Device_Setaddress(SetupLen );// SET ADDRESS
			 break;
		case 0x31:
		    break;
	}
}


/*******************************************************************************
 * @fn      EP1_IN_Callback
 *
 * @brief   USB3.0 endpoint1 in callback.
 *
 * @return   None
 */
void EP1_IN_Callback()
{
    uint16_t s,i;
    USB30_IN_ClearIT( ENDP_1 );
    if(cswflag){    cswflag = 0;    Ucsw(0);    }
    else{
        USB30_IN_ClearIT(ENDP_2);
        USB30_OUT_Set( ENDP_2 , ACK , 1 );
        USB30_Send_ERDY( ENDP_2 | OUT , 1 );
    }

}

/*******************************************************************************
 * @fn      EP2_OUT_Callback
 *
 * @brief   USB3.0 endpoint2 out callback.
 *
 * @return   None
 */
void EP2_OUT_Callback()
{
    UINT16 len;
    UINT8 s,i;
    UINT8 *p;
    UINT8 status;
    UINT32 LBAaddr;
    USB30_OUT_ClearIT(ENDP_2);
    USBSS->UEP2_RX_DMA = (UINT32)(UINT8 *)( endp2RTbuff );

    if(*(UINT32 *)endp2RTbuff == 0x43425355){                          //CBW
       *(uint32_t *)&CSW_PARAMETER[4]   = *(uint32_t *)&endp2RTbuff[4];//Assign value to CSW check
       switch(endp2RTbuff[0xf]){  //Determine SCSI commands
           case 0x2a:             //write
                LBAaddr=(endp2RTbuff[0xf+2]<<24)|(endp2RTbuff[0xf+3]<<16)|(endp2RTbuff[0xf+4]<<8)|(endp2RTbuff[0xf+5]);
                writeblocknum=(endp2RTbuff[0xf+7]<<8)|(endp2RTbuff[0xf+8]);
                PFIC_DisableIRQ(USBSS_IRQn);
                if((RWsta != 2) || (curWaddr != LBAaddr)){
                    curWaddr = LBAaddr;
                    if(RWsta != 0xff)
                        cmd12( );
                    write_start(writeblocknum, LBAaddr,1);  //Continuous write needs to be sent cmd
                }
                else{
                    write_start(writeblocknum, LBAaddr,0);
                }
                PFIC_EnableIRQ(USBSS_IRQn);                 //enable USBSS interrupt
                len=0;   //send CSW
                RWsta = 2;
                curWaddr += writeblocknum;
                break;
            case 0x28:   //read
                LBAaddr=(endp2RTbuff[0xf+2]<<24)|(endp2RTbuff[0xf+3]<<16)|(endp2RTbuff[0xf+4]<<8)|(endp2RTbuff[0xf+5]);
                reqblocknum=(endp2RTbuff[0xf+7]<<8)|(endp2RTbuff[0xf+8]);
                PFIC_DisableIRQ(USBSS_IRQn);
                if((RWsta != 1) || (curRaddr != LBAaddr)){
                    curRaddr = LBAaddr;
                    if(RWsta != 0xff)
                        cmd12( );
                    read_start(reqblocknum, LBAaddr,1);
                }
                else{
                    curRaddr = LBAaddr;
                    read_start(reqblocknum, LBAaddr,0);
                }
                PFIC_EnableIRQ(USBSS_IRQn);
                len=0;   //send CSW
                curRaddr += reqblocknum;
                RWsta = 1;
                break;
            case 0x12:
                if(endp2RTbuff[0xf+4]==0x24){
                    memcpy(endp1RTbuff, (uint8_t *)INQUIRY1,36 );
                    len=36;
                }
                else if(endp2RTbuff[0xf+4]==0x38){
                    memcpy(endp1RTbuff, (uint8_t *)s1238,56 );
                    len=56;
                }
                else if(endp2RTbuff[0xf+4]==0x40){
                   memcpy(endp1RTbuff, (uint8_t *)INQUIRY1,0x40 );
                   len=0x40;
               }
                else if(endp2RTbuff[0xf+4]==0x2f){
                    memcpy(endp1RTbuff, (uint8_t *)LONG_INQUIRY,255 );
                    len=0x2f;
                }
                else{
                    memcpy(endp1RTbuff, (uint8_t *)LONG_INQUIRY,255 );
                    len=255;
                }
                cswflag = 1;
                break;
            case 0x23:
                memcpy(endp1RTbuff, (uint8_t *)&FORMAT_CAPACITIES,12 );
                len=12;
                cswflag = 1;
                break;
            case 0x25:
                memcpy(endp1RTbuff, (uint8_t *)&READ_CAPACITY,8 );
                len=8;
                cswflag = 1;
                break;
            case 0x1a:
                if(endp2RTbuff[0xf+2]==0x3f){
                    memcpy(endp1RTbuff, (uint8_t *)s1a03,36 );
                    len=36;
                }
                else{
                    *(uint32_t *)endp1RTbuff= 0x00000003;
                    len=4;
                }
                cswflag = 1;
                break;
            case 0x00:
                len=0;
                break;
            case 0x35:
                len=0;
                break;
            case 0x1E:
                if(RWsta != 0xff){
                    cmd12( );
                    RWsta = 0xff;
                }
                len=0;
                break;
            case 0x03:
                memcpy(endp1RTbuff, (uint8_t *)s03,18 );
                len=18;
                cswflag = 1;
                break;
        }
          if(len){
              USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)( endp1RTbuff );
              USB30_IN_Set( ENDP_1 ,ENABLE, ACK , 1 , len );
              USB30_Send_ERDY( ENDP_1 | IN , 1 );

          }
          else{
              Ucsw(0);
          }
    }
    else{
        printf("errr\n");
    }
}

/***************Endpointn IN/OUT Transaction Processing*******************/
void EP1_OUT_Callback()
{
    ;
}

void EP3_OUT_Callback()
{
    ;
}
void EP4_OUT_Callback()
{
    ;
}
void EP5_OUT_Callback()
{
    ;
}
void EP6_OUT_Callback()
{
    ;
}
void EP7_OUT_Callback()
{
    ;
}
void EP2_IN_Callback()
{

  ;
}

void EP3_IN_Callback()
{

   ;
}
void EP4_IN_Callback()
{
    ;
}
void EP5_IN_Callback()
{

    ;
}
void EP6_IN_Callback()
{
    ;
}
void EP7_IN_Callback()
{
   ;
}

void  USB30_ITP_Callback(UINT32 ITPCounter)
{

}
