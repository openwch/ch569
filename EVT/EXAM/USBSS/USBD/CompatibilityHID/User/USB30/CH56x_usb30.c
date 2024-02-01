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

#include "CH56x_common.h"
#include "CH56xUSB30_LIB.H"
#include "CH56x_usb20.h"
#include "CH56x_usb30.h"


/* Global Variable */
UINT8V        Tx_Lmp_Port = 0;
UINT8V        Link_Sta = 0;
static UINT32 SetupLen = 0;
static UINT8  SetupReqCode = 0;
static PUINT8 pDescr;

__attribute__((aligned(16))) UINT8 endp0RTbuff[512] __attribute__((section(".DMADATA")));  //Endpoint 0 data receiving and sending buffer
__attribute__((aligned(16))) UINT8 HIDbuff[1024*ENDP_INTERRUPT_BURST_LEVEL] __attribute__((section(".DMADATA")));


/*Superspeed device descriptor*/
const UINT8 SS_DeviceDescriptor[] =
{
        0x12, // bLength
        0x01, // DEVICE descriptor type
        0x00, // 3.00
        0x03,
        0xff, // device class
        0x80, // device sub-class
        0x55, // vendor specific protocol
        0x09, // max packet size 512B
        0x86, // vendor id-0x1A86(qinheng)
        0x1A,
        0x07, // product id 0xfe07
        0xFE,
        0x00, //bcdDevice 0x0011
        0x01,
        0x01, // manufacturer index string
        0x02, // product index string
        0x03, // serial number index string
        0x01  // number of configurations
};

/*Superspeed Configuration Descriptor*/
const UINT8 SS_ConfigDescriptor[] =
{
        /* Configuration Descriptor */
        0x09,                           // bLength
        0x02,                           // bDescriptorType
        0x35, 0x00,                     // wTotalLength
        0x01,                           // bNumInterfaces
        0x01,                           // bConfigurationValue
        0x00,                           // iConfiguration (String Index)
        0x80,                           // bmAttributes
        0x64,                           // bMaxPower 70mA

        /* Interface Descriptor */
        0x09,                           // bLength
        0x04,                           // bDescriptorType (Interface)
        0x00,                           // bInterfaceNumber 0
        0x00,                           // bAlternateSetting
        0x02,                           // bNumEndpoints 2
        0x03,                           // bInterfaceClass
        0x00,                           // bInterfaceSubClass
        0x00,                           // bInterfaceProtocol
        0x00,                           // iInterface (String Index)

        /* HID Descriptor */
        0x09,                           // bLength
        0x21,                           // bDescriptorType
        0x11, 0x01,                     // bcdHID
        0x00,                           // bCountryCode
        0x01,                           // bNumDescriptors
        0x22,                           // bDescriptorType
        0x22, 0x00,                     // wDescriptorLength

        /* Endpoint Descriptor */
        0x07,                           // bLength
        0x05,                           // bDescriptorType
        0x01,                           // bEndpointAddress: OUT Endpoint 1
        0x03,                           // bmAttributes
        0x00, 0x04,                     // wMaxPacketSize
        0x01,                           // bInterval: 1mS

        0x06, // length of this endpoint compansion descriptor
        0x30,
        ENDP_INTERRUPT_BURST_LEVEL - 1, // max burst size
        0x00,                          // no stream
        0x00,
        0x00,

        /* Endpoint Descriptor */
        0x07,                           // bLength
        0x05,                           // bDescriptorType
        0x82,                           // bEndpointAddress: IN Endpoint 2
        0x03,                           // bmAttributes
        0x00, 0x04,                     // wMaxPacketSize
        0x01,                           // bInterval: 1mS

        0x06, // length of this endpoint compansion descriptor
        0x30,
        ENDP_INTERRUPT_BURST_LEVEL - 1, // max burst size
        0x00,                          // no stream
        0x00,
        0x00,
};

/* HID Report Descriptor */
const UINT8  SS_HIDReportDesc[ ] =
{
    0x06, 0x00, 0xFF,               // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,                     // Usage (0x01)
    0xA1, 0x01,                     // Collection (Application)
    0x09, 0x02,                     //   Usage (0x02)
    0x26, 0xFF, 0x00,               //   Logical Maximum (255)
    0x75, 0xc0,                     //   Report Size (32)
    0x15, 0x00,                     //   Logical Minimum (0)
    0x95, 0x80,                     //   Report Count (128)
    0x81, 0x06,                     //   Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x02,                     //   Usage (0x02)
    0x15, 0x00,                     //   Logical Minimum (0)
    0x26, 0xFF, 0x00,               //   Logical Maximum (255)
    0x75, 0xc0,                     //   Report Size (32)
    0x95, 0x80,                     //   Report Count (128)
    0x91, 0x06,                     //   Output (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,                           // End Collection
};

/*String Descriptor Lang ID*/
const UINT8 StringLangID[] =
{
        0x04, // this descriptor length
        0x03, // descriptor type
        0x09, // Language ID 0 low byte
        0x04  // Language ID 0 high byte
};

/*String Descriptor Vendor*/
const UINT8 StringVendor[] =
    {
        0x08, // length of this descriptor
        0x03,
        'W',
        0x00,
        'C',
        0x00,
        'H',
        0x00};

/*String Descriptor Product*/
const UINT8 StringProduct[] =
    {
        38,         //38 bytes in length
        0x03,       //Type code
        0x57, 0x00, //W
        0x43, 0x00, //C
        0x48, 0x00, //H
        0x20, 0x00, //
        0x55, 0x00, //U
        0x53, 0x00, //S
        0x42, 0x00, //B
        0x33, 0x00, //3
        0x2e, 0x00, //.
        0x30, 0x00, //0
        0x20, 0x00, //
        0x44, 0x00, //D
        0x45, 0x00, //E
        0x56, 0x00, //V
        0x49, 0x00, //I
        0x43, 0x00, //C
        0x45, 0x00, //E
        0x20, 0x00};

/*String Descriptor Serial*/
UINT8 StringSerial[] =
    {
        0x16, // length of this descriptor
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
        0x12, // length of this descriptor
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
        0x00};

const UINT8 BOSDescriptor[] =
    {
        0x05, // length of this descriptor
        0x0f, // CONFIGURATION (2)
        0x16, // total length includes endpoint descriptors (should be 1 more than last address)
        0x00, // total length high byte
        0x02, // number of device cap

        //dev_cap_descriptor1
        0x07,
        0x10, // DEVICE CAPABILITY type
        0x02, // USB2.0 EXTENSION
        0x06,
        0x00,
        0x00,
        0x00,

        //dev_cap_descriptor2
        0x0a, // length of this descriptor
        0x10, // DEVICE CAPABILITY type
        0x03, // superspeed usb device capability
        0x00, //
        0x0e, // ss/hs/fs
        0x00,
        0x01, // the lowest speed is full speed
        0x0a, // u1 exit latency is 10us
        0xff, // u1 exit latency is 8us
        0x07};

const UINT8 MSOS20DescriptorSet[] =
    {
        // Microsoft OS 2.0 Descriptor Set Header
        0x0A, 0x00,             // wLength - 10 bytes
        0x00, 0x00,             // MSOS20_SET_HEADER_DESCRIPTOR
        0x00, 0x00, 0x03, 0x06, // dwWindowsVersion  0x06030000 for Windows Blue
        0x48, 0x00,             // wTotalLength  72 bytes
                                // Microsoft OS 2.0 Registry Value Feature Descriptor
        0x3E, 0x00,             // wLength - 62 bytes
        0x04, 0x00,             // wDescriptorType  4 for Registry Property
        0x04, 0x00,             // wPropertyDataType - 4 for REG_DWORD
        0x30, 0x00,             // wPropertyNameLength  48 bytes
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
        0x04, 0x00,            // wPropertyDataLength 4 bytes
        0x01, 0x00, 0x00, 0x00 // PropertyData - 0x00000001
};

const UINT8 PropertyHeader[] =
    {
        0x8e, 0x00, 0x00, 0x00, 0x00, 01, 05, 00, 01, 00,
        0x84, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x00, 0x00,
        0x28, 0x00,
        0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00, 0x49, 0x00, 0x6e,
        0x00, 0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x66, 0x00, 0x61, 0x00, 0x63, 0x00, 0x65, 0x00, 0x47, 0x00, 0x55, 0x00, 0x49, 0x00, 0x44, 0x00, 0x00, 0x00,

        0x4e, 0x00, 0x00, 0x00,
        0x7b, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00, 0x35, 0x00, 0x36, 0x00, 0x37, 0x00, 0x38, 0x00,
        0x2d, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00,
        0x2d, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00,
        0x2d, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00, 0x2d, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00,
        0x34, 0x00, 0x35, 0x00, 0x36, 0x00, 0x37, 0x00, 0x38, 0x00, 0x39, 0x00, 0x41, 0x00, 0x42, 0x00, 0x43, 0x00,
        0x7d, 0x00, 0x00, 0x00};

const UINT8 CompactId[] =
    {
        0x28, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x57, 0x49, 0x4e, 0x55, 0x53, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

UINT8 GetStatus[] =
    {
        0x01, 0x00};

/*******************************************************************************
 * @fn      USB30D_init
 *
 * @brief   USB3.0 initialization
 *
 * @return  None
 */
void USB30D_init(FunctionalState sta)
{
    UINT16 i, s;
    if(sta)
    {
        USB30_Device_Init();

        USBSS->UEP_CFG = EP0_R_EN | EP0_T_EN | EP1_R_EN | EP2_T_EN ; // set end point rx/tx enable

        USBSS->UEP0_DMA = (UINT32)(UINT8 *)endp0RTbuff;

        USBSS->UEP1_RX_DMA = (UINT32)(UINT8 *)HIDbuff;
        USBSS->UEP2_TX_DMA = (UINT32)(UINT8 *)HIDbuff;


        USB30_OUT_Set(ENDP_1, ACK, ENDP_INTERRUPT_BURST_LEVEL); // endpoint1 receive setting
        USB30_IN_Set(ENDP_2, ENABLE, NRDY, 0, 0); // endpoint2 send setting
    }
    else
    {
        USB30_Switch_Powermode(POWER_MODE_2);
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
    UINT8 endp_dir;

    SetupReqCode = UsbSetupBuf->bRequest;
    SetupLen = UsbSetupBuf->wLength;
    endp_dir = UsbSetupBuf->bRequestType & 0x80;
    UINT16 len = 0;
#if 0
    printf("NS:%02x %02x %02x %02x %02x %02x %02x %02x\n", endp0RTbuff[0], endp0RTbuff[1],
            endp0RTbuff[2], endp0RTbuff[3], endp0RTbuff[4], endp0RTbuff[5],
            endp0RTbuff[6], endp0RTbuff[7]);
#endif
    switch(SetupReqCode)
    {
        case 0x02:                                                  //User defined commands
            switch(UsbSetupBuf->wIndexL)
            {
                case 0x05:
                    if(SetupLen > SIZE_PropertyHeader)
                        SetupLen = SIZE_PropertyHeader;
                    pDescr = (PUINT8)PropertyHeader;
                    break;
                default:
                    SetupReqCode = INVALID_REQ_CODE;
                    return USB_DESCR_UNSUPPORTED;
                    break;
            }
            break;
        default:
            SetupReqCode = INVALID_REQ_CODE;
            return USB_DESCR_UNSUPPORTED;
            break;
    }
    len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen;     // Current transmission length
    if(endp_dir)
    {
        memcpy(endp0RTbuff, pDescr, len);                           // Load upload data
        pDescr += len;
    }
    SetupLen -= len;
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
#if 0
    printf("S:%02x %02x %02x %02x %02x %02x %02x %02x\n", endp0RTbuff[0], endp0RTbuff[1],
            endp0RTbuff[2], endp0RTbuff[3], endp0RTbuff[4], endp0RTbuff[5],
            endp0RTbuff[6], endp0RTbuff[7]);
#endif

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
                        if(SetupLen > SIZE_DEVICE_DESC)
                            SetupLen = SIZE_DEVICE_DESC;
                        pDescr = (PUINT8)SS_DeviceDescriptor;
                        break;
                    case USB_DESCR_TYP_CONFIG:
                        if(SetupLen > SIZE_CONFIG_DESC)
                            SetupLen = SIZE_CONFIG_DESC;
                        pDescr = (PUINT8)SS_ConfigDescriptor;
                        break;
                    case USB_DESCR_TYP_BOS:
                        if(SetupLen > SIZE_BOS_DESC)
                            SetupLen = SIZE_BOS_DESC;
                        pDescr = (PUINT8)BOSDescriptor;
                        break;
                    case USB_DESCR_TYP_STRING:
                        switch(UsbSetupBuf->wValueL)
                        {
                            case USB_DESCR_LANGID_STRING:
                                if(SetupLen > SIZE_STRING_LANGID)
                                    SetupLen = SIZE_STRING_LANGID;
                                pDescr = (PUINT8)StringLangID;
                                break;
                            case USB_DESCR_VENDOR_STRING:
                                if(SetupLen > SIZE_STRING_VENDOR)
                                    SetupLen = SIZE_STRING_VENDOR;
                                pDescr = (PUINT8)StringVendor;
                                break;
                            case USB_DESCR_PRODUCT_STRING:
                                if(SetupLen > SIZE_STRING_PRODUCT)
                                    SetupLen = SIZE_STRING_PRODUCT;
                                pDescr = (PUINT8)StringProduct;
                                break;
                            case USB_DESCR_SERIAL_STRING:
                                if(SetupLen > SIZE_STRING_SERIAL)
                                    SetupLen = SIZE_STRING_SERIAL;
                                pDescr = (PUINT8)StringSerial;
                                break;
                            case USB_DESCR_OS_STRING:
                                if(SetupLen > SIZE_STRING_OS)
                                    SetupLen = SIZE_STRING_OS;
                                pDescr = (PUINT8)OSStringDescriptor;
                                break;
                            default:
                                len = USB_DESCR_UNSUPPORTED;     //Unsupported descriptor
                                SetupReqCode = INVALID_REQ_CODE; //Invalid request code
                                break;
                        }
                        break;
                    case USB_DESCR_TYP_HID:
                        pDescr =(UINT8 *) &SS_ConfigDescriptor[18];
                        SetupLen = 9;
                        break;

                    case USB_DESCR_TYP_REPORT:
                        pDescr =(UINT8 *) SS_HIDReportDesc;
                        SetupLen = 34;
                        break;
                    default:
                        len = USB_DESCR_UNSUPPORTED; //Unsupported descriptor
                        SetupReqCode = INVALID_REQ_CODE;
                        break;
                }
                len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen; //Current transmission length
                memcpy(endp0RTbuff, pDescr, len);                           //Load upload data
                SetupLen -= len;
                pDescr += len;
                break;
            case USB_SET_ADDRESS:
                SetupLen = UsbSetupBuf->wValueL; // Temporary USB device address
                break;
            case 0x31:
                SetupLen = UsbSetupBuf->wValueL; // Temporary USB device address
                break;
            case 0x30:
                break;
            case USB_SET_CONFIGURATION:
                break;
            case USB_GET_STATUS:
                len = 2;
                endp0RTbuff[0] = 0x01;
                endp0RTbuff[1] = 0x00;
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
            case 0x0a:
                SetupLen = 0;
                break;
            default:
                len = USB_DESCR_UNSUPPORTED;  //return stall£¬unsupported command
                SetupReqCode = INVALID_REQ_CODE;
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
            memcpy(endp0RTbuff, pDescr, len);
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
UINT16 EP0_OUT_Callback(void)
{
    UINT16 len;
    return len;
}

/*******************************************************************************
 * @fn      USB30_Setup_Status
 *
 * @brief   USB3.0 Control transfer status stage callback
 *
 * @return  None
 */
void USB30_Setup_Status(void)
{
    switch(SetupReqCode)
    {
        case USB_SET_ADDRESS:
            USB30_Device_Setaddress(SetupLen); // SET ADDRESS
            break;
        case 0x31:
            break;
    }
}

/*******************************************************************************
 * @fn      USBSS_IRQHandler
 *
 * @brief   USB3.0 Interrupt Handler.
 *
 * @return   None
 */
void USBSS_IRQHandler(void) //USBSS interrupt service
{
    USB30_IRQHandler();
}

/*******************************************************************************
 * @fn      TMR0_IRQHandler
 *
 * @brief   USB3.0 Connection failure timeout processing
 *
 * @return  None
 */
void TMR0_IRQHandler()
{
    R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;

    if( Link_Sta == LINK_STA_1 )
    {
        Link_Sta = 0;
        PFIC_DisableIRQ(USBSS_IRQn);
        PFIC_DisableIRQ(LINK_IRQn);
        USB30D_init(DISABLE);
        PRINT("USB3.0 disable\n");
        return;
    }
    if( Link_Sta != LINK_STA_3 )
    {
        PFIC_DisableIRQ(USBSS_IRQn);
        PFIC_DisableIRQ(LINK_IRQn);
        USB30D_init(DISABLE);
        //        PRINT("USB2.0\n");
        R32_USB_CONTROL = 0;
        PFIC_EnableIRQ(USBHS_IRQn);
        USB20_Device_Init(ENABLE);
    }
    Link_Sta = LINK_STA_1;
    R8_TMR0_INTER_EN = 0;
    PFIC_DisableIRQ(TMR0_IRQn);
    R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
}

/*******************************************************************************
 * @fn      LINK_IRQHandler
 *
 * @brief   USB3.0 Link Interrupt Handler.
 *
 * @return   None
 */
void LINK_IRQHandler() //USBSS link interrupt service
{
    if(USBSS->LINK_INT_FLAG & LINK_Ux_EXIT_FLAG) // device enter U2
    {
        USBSS->LINK_CFG = CFG_EQ_EN | TX_SWING | DEEMPH_CFG | TERM_EN;
        USB30_Switch_Powermode(POWER_MODE_0);
        USBSS->LINK_INT_FLAG = LINK_Ux_EXIT_FLAG;
    }
    if(USBSS->LINK_INT_FLAG & LINK_RDY_FLAG) // POLLING SHAKE DONE
    {
        USBSS->LINK_INT_FLAG = LINK_RDY_FLAG;
        if(Tx_Lmp_Port) // LMP, TX PORT_CAP & RX PORT_CAP
        {
            USBSS->LMP_TX_DATA0 = LINK_SPEED | PORT_CAP | LMP_HP;
            USBSS->LMP_TX_DATA1 = UP_STREAM | NUM_HP_BUF;
            USBSS->LMP_TX_DATA2 = 0x0;
            Tx_Lmp_Port = 0;
        }
        /*Successful USB3.0 communication*/
        Link_Sta = LINK_STA_3;
        PFIC_DisableIRQ(TMR0_IRQn);
        R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
        R8_TMR0_INTER_EN = 0;
        PFIC_DisableIRQ(USBHS_IRQn);
        USB20_Device_Init(DISABLE);
    }

    if(USBSS->LINK_INT_FLAG & LINK_INACT_FLAG)
    {
        Link_Sta = 0;
        PFIC_EnableIRQ(USBSS_IRQn);
        PFIC_EnableIRQ(LINK_IRQn);
        PFIC_EnableIRQ(TMR0_IRQn);
        R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
        TMR0_TimerInit( 67000000 );
        USB30D_init(ENABLE);
        USBSS->LINK_INT_FLAG = LINK_INACT_FLAG;
        USB30_Switch_Powermode(POWER_MODE_2);
    }
    if(USBSS->LINK_INT_FLAG & LINK_DISABLE_FLAG) // GO DISABLED
    {
        USBSS->LINK_INT_FLAG = LINK_DISABLE_FLAG;
        Link_Sta = LINK_STA_1;
        USB30D_init(DISABLE);
        PFIC_DisableIRQ(USBSS_IRQn);
        R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
        R8_TMR0_INTER_EN = 0;
        PFIC_DisableIRQ(TMR0_IRQn);
        PFIC_EnableIRQ(USBHS_IRQn);
        USB20_Device_Init(ENABLE);
    }
    if(USBSS->LINK_INT_FLAG & LINK_RX_DET_FLAG)
    {
        USBSS->LINK_INT_FLAG = LINK_RX_DET_FLAG;
        USB30_Switch_Powermode(POWER_MODE_2);
    }
    if(USBSS->LINK_INT_FLAG & TERM_PRESENT_FLAG) // term present , begin POLLING
    {
        USBSS->LINK_INT_FLAG = TERM_PRESENT_FLAG;
        if(USBSS->LINK_STATUS & LINK_PRESENT)
        {
            USB30_Switch_Powermode(POWER_MODE_2);
            USBSS->LINK_CTRL |= POLLING_EN;
        }
        else
        {
            USBSS->LINK_INT_CTRL = 0;
        }
    }
    if(USBSS->LINK_INT_FLAG & LINK_TXEQ_FLAG) // POLLING SHAKE DONE
    {
        Tx_Lmp_Port = 1;
        USBSS->LINK_INT_FLAG = LINK_TXEQ_FLAG;
        USB30_Switch_Powermode(POWER_MODE_0);
    }
    if(USBSS->LINK_INT_FLAG & WARM_RESET_FLAG)
    {
        USBSS->LINK_INT_FLAG = WARM_RESET_FLAG;
        USB30_Switch_Powermode(POWER_MODE_2);
        USBSS->LINK_CTRL |= TX_WARM_RESET;
        while(USBSS->LINK_STATUS & RX_WARM_RESET);
        USBSS->LINK_CTRL &= ~TX_WARM_RESET;
        USB30_Device_Setaddress(0);

    }
    if(USBSS->LINK_INT_FLAG & HOT_RESET_FLAG) //The host may send hot reset,Note the configuration of the endpoint
    {
        USBSS->USB_CONTROL |= 1 << 31;
        USBSS->LINK_INT_FLAG = HOT_RESET_FLAG; // HOT RESET begin
        USBSS->UEP0_TX_CTRL = 0;
        USB30_IN_Set(ENDP_1, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_2, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_3, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_4, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_5, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_6, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_7, DISABLE, NRDY, 0, 0);
        USB30_OUT_Set(ENDP_1, NRDY, 0);
        USB30_OUT_Set(ENDP_2, NRDY, 0);
        USB30_OUT_Set(ENDP_3, NRDY, 0);
        USB30_OUT_Set(ENDP_4, NRDY, 0);
        USB30_OUT_Set(ENDP_5, NRDY, 0);
        USB30_OUT_Set(ENDP_6, NRDY, 0);
        USB30_OUT_Set(ENDP_7, NRDY, 0);

        USB30_Device_Setaddress(0);
        USBSS->LINK_CTRL &= ~TX_HOT_RESET;     // HOT RESET end
    }
    if(USBSS->LINK_INT_FLAG & LINK_GO_U1_FLAG) // device enter U1
    {
        USB30_Switch_Powermode(POWER_MODE_1);
        USBSS->LINK_INT_FLAG = LINK_GO_U1_FLAG;
    }
    if(USBSS->LINK_INT_FLAG & LINK_GO_U2_FLAG) // device enter U2
    {
        USB30_Switch_Powermode(POWER_MODE_2);
        USBSS->LINK_INT_FLAG = LINK_GO_U2_FLAG;
    }
    if(USBSS->LINK_INT_FLAG & LINK_GO_U3_FLAG) // device enter U2
    {
        USB30_Switch_Powermode(POWER_MODE_2);
        USBSS->LINK_INT_FLAG = LINK_GO_U3_FLAG;
    }
}

/***************Endpointn IN Transaction Processing*******************/
/*******************************************************************************
 * @fn      EP1_IN_Callback
 *
 * @brief   USB3.0 endpoint1 in callback.
 *
 * @return   None
 */
void EP1_IN_Callback(void)
{

}

/*******************************************************************************
 * @fn      EP2_IN_Callback
 *
 * @brief   USB3.0 endpoint2 in callback.
 *
 * @return   None
 */
void EP2_IN_Callback(void)
{
    UINT8 nump;
    nump = USB30_IN_Nump(ENDP_2);
    if(nump == 0)
    {
        USB30_IN_ClearIT(ENDP_2);
        USBSS->UEP2_TX_DMA = (UINT32)(UINT8 *)HIDbuff;

        USB30_OUT_Set(ENDP_1, ACK, ENDP_INTERRUPT_BURST_LEVEL);
        USB30_Send_ERDY(ENDP_1 | OUT, ENDP_INTERRUPT_BURST_LEVEL);
    }
    else if(nump == 1)
    {
        USB30_IN_ClearIT(ENDP_2);
        USB30_IN_Set(ENDP_2, ENABLE, ACK, 1, 1024);
        USB30_Send_ERDY(ENDP_2 | IN, 1);
    }
    else if(nump == 2)
    {
        USB30_IN_ClearIT(ENDP_2);
        USB30_IN_Set(ENDP_2, ENABLE, ACK, 2, 1024);
        USB30_Send_ERDY(ENDP_2 | IN, 2);
    }
}

/*******************************************************************************
 * @fn      EP3_IN_Callback
 *
 * @brief   USB3.0 endpoint3 in callback.
 *
 * @return   None
 */
void EP3_IN_Callback(void)
{

}

/*******************************************************************************
 * @fn      EP4_IN_Callback
 *
 * @brief   USB3.0 endpoint4 in callback.
 *
 * @return   None
 */
void EP4_IN_Callback(void)
{

}
/*******************************************************************************
 * @fn      EP5_IN_Callback
 *
 * @brief   USB3.0 endpoint5 in callback.
 *
 * @return   None
 */
void EP5_IN_Callback(void)
{

}
/*******************************************************************************
 * @fn      EP6_IN_Callback
 *
 * @brief   USB3.0 endpoint6 in callback.
 *
 * @return   None
 */
void EP6_IN_Callback(void)
{

}
/*******************************************************************************
 * @fn      EP7_IN_Callback
 *
 * @brief   USB3.0 endpoint7 in callback.
 *
 * @return   None
 */
void EP7_IN_Callback(void)
{

}

/***************Endpointn OUT Transaction Processing*******************/
/*******************************************************************************
 * @fn      EP1_OUT_Callback
 *
 * @brief   USB3.0 endpoint1 out callback.
 *
 * @return   None
 */
void EP1_OUT_Callback(void)
{
    UINT16 rx_len, i;
    UINT8  nump;
    UINT8  status;
    USB30_OUT_Status(ENDP_1, &nump, &rx_len, &status); //Get the number of received packets,rxlen is the package length of the last package
    if(nump == 0)
    {
        USB30_OUT_ClearIT(ENDP_1);
        USBSS->UEP1_RX_DMA = (UINT32)(UINT8 *)HIDbuff;

        USB30_IN_Set(ENDP_2, ENABLE, ACK, ENDP_INTERRUPT_BURST_LEVEL, 1024);
        USB30_Send_ERDY(ENDP_2 | IN, ENDP_INTERRUPT_BURST_LEVEL);
    }
    else if(nump == 1)
    {
        USB30_OUT_ClearIT(ENDP_1);
        USB30_OUT_Set(ENDP_1, ACK, 1);
        USB30_Send_ERDY(ENDP_1 | OUT, 1);
    }
    else if(nump == 2)
    {
        USB30_OUT_ClearIT(1);
        USB30_OUT_Set(ENDP_1, ACK, 2);
        USB30_Send_ERDY(ENDP_1 | OUT, 2);
    }
}

/*******************************************************************************
 * @fn      EP2_OUT_Callback
 *
 * @brief   USB3.0 endpoint2 out callback.
 *
 * @return   None
 */
void EP2_OUT_Callback(void)
{

}
/*******************************************************************************
 * @fn      EP3_OUT_Callback
 *
 * @brief   USB3.0 endpoint3 out callback.
 *
 * @return   None
 */
void EP3_OUT_Callback(void)
{

}
/*******************************************************************************
 * @fn      EP4_OUT_Callback
 *
 * @brief   USB3.0 endpoint4 out callback.
 *
 * @return   None
 */
void EP4_OUT_Callback(void)
{

}
/*******************************************************************************
 * @fn      EP5_OUT_Callback
 *
 * @brief   USB3.0 endpoint5 out callback.
 *
 * @return   None
 */
void EP5_OUT_Callback(void)
{

}
/*******************************************************************************
 * @fn      EP6_OUT_Callback
 *
 * @brief   USB3.0 endpoint6 out callback.
 *
 * @return   None
 */
void EP6_OUT_Callback(void)
{

}
/*******************************************************************************
 * @fn      EP7_OUT_Callback
 *
 * @brief   USB3.0 endpoint7 out callback.
 *
 * @return  None
 */
void EP7_OUT_Callback(void)
{

}

/*******************************************************************************
 * @fn      USB30_ITP_Callback
 *
 * @brief   USB3.0 ITP callback function
 *
 * @return  None
 */
void USB30_ITP_Callback(UINT32 ITPCounter)
{
}
