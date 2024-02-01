/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb20.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_usb20.h"
#include "CH56x_usb30.h"
#include "CH56xUSB30_LIB.H"
#include "SW_UDISK.h"

/* Global Variable */
UINT16V  U20_EndpnMaxSize = 512;
UINT16V  SetupReqLen=0;            //Host request data length
UINT16V  SetupLen = 0;             //Data length actually sent or received in data phase
UINT32V seq_num = 0;
DevInfo_Typedef  g_devInfo;
static UINT8V SetupReqType = 0;    //Host request descriptor type
static UINT8V SetupReq = 0;        //Host request descriptor type
static PUINT8 pDescr;

__attribute__ ((aligned(16))) UINT8 vendor_buff[16]  __attribute__((section(".DMADATA")));
/* Function declaration */
void USBHS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

const UINT8 hs_device_descriptor[] =
{
    0x12,   // bLength
    0x01,   // DEVICE descriptor type
    0x00,   // 2.00
    0x02,
    0x00,   // device class
    0x00,   // device sub-class
    0x00,   // vendor specific protocol
    0x40,   // max packet size 512B
    0x86, // vendor id-0x1A86(qinheng)
    0x1A,
    0x10, // product id 0xfe10
    0xFE,
    0x00,   //bcdDevice 0x0111
    0x11,
    0x01,   // manufacturer index string
    0x02,   // product index string
    0x00,   // serial number index string
    0x01    // number of configurations
};

const UINT8 hs_config_descriptor[] =
{
        /* Configuration Descriptor */
        0x09,                                                                        // bLength
        0x02,                                                                        // bDescriptorType (Configuration)
        0x20, 0x00,                                                                  // wTotalLength 32
        0x01,                                                                        // bNumInterfaces 1
        0x01,                                                                        // bConfigurationValue
        0x00,                                                                        // iConfiguration (String Index)
        0xC0,                                                                        // bmAttributes Self Powered
        0x32,                                                                        // bMaxPower 100mA

        /*****************************************************************/
        /* Interface Descriptor(UDisk) */
        0x09,                                                                        // bLength
        0x04,                                                                        // bDescriptorType (Interface)
        0x00,                                                                        // bInterfaceNumber 0
        0x00,                                                                        // bAlternateSetting
        0x02,                                                                        // bNumEndpoints 2
        0x08,                                                                        // bInterfaceClass
        0x06,                                                                        // bInterfaceSubClass
        0x50,                                                                        // bInterfaceProtocol
        0x00,                                                                        // iInterface (String Index)

        /* Endpoint Descriptor */
        0x07,                                                                        // bLength
        0x05,                                                                        // bDescriptorType (Endpoint)
        0x82,                                                                        // bEndpointAddress (IN/D2H)
        0x02,                                                                        // bmAttributes (Bulk)
        0x00, 0x02,                                                                  // wMaxPacketSize 512
        0x00,                                                                        // bInterval 0 (unit depends on device speed)

        /* Endpoint Descriptor */
        0x07,                                                                        // bLength
        0x05,                                                                        // bDescriptorType (Endpoint)
        0x03,                                                                        // bEndpointAddress (OUT/H2D)
        0x02,                                                                        // bmAttributes (Bulk)
        0x00, 0x02,                                                                  // wMaxPacketSize 512
        0x00,                                                                        // bInterval 0 (unit depends on device speed)
};

/* Language Descriptor */
const UINT8 hs_string_descriptor0[] =
{
    0x04,   // this descriptor length
    0x03,   // descriptor type
    0x09,   // Language ID 0 low byte
    0x04    // Language ID 0 high byte
};

/* Manufacturer Descriptor */
const UINT8 hs_string_descriptor1[] =
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

/* Product Descriptor */
const UINT8 hs_string_descriptor2[]=
{
    38,         //38 bytes
    0x03,       //0x03
    0x57, 0x00, //W
    0x43, 0x00, //C
    0x48, 0x00, //H
    0x20, 0x00, //
    0x55, 0x00, //U
    0x53, 0x00, //S
    0x42, 0x00, //B
    0x32, 0x00, //2
    0x2e, 0x00, //.
    0x30, 0x00, //0
    0x20, 0x00, //
    0x44, 0x00, //D
    0x45, 0x00, //E
    0x56, 0x00, //V
    0x49, 0x00, //I
    0x43, 0x00, //C
    0x45, 0x00, //E
    0x20, 0x00  //
};

const UINT8 hs_bos_descriptor[] =
{
    0x05,   // length of this descriptor
    0x0f,   // CONFIGURATION (2)
    0x16,   // total length includes endpoint descriptors (should be 1 more than last address)
    0x00,   // total length high byte
    0x02,       // number of device cap

    0x07,
    0x10,   // DEVICE CAPABILITY type
    0x02,   // USB2.0 EXTENSION
    0x02,
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



/*******************************************************************************
 * @fn     USB20_Endp_Init
 *
 * @brief  USB2.0 Endpoint initialization
 *
 * @return  None
 */
void USB20_Endp_Init ( )	// USBHS device endpoint initial
{
    R8_UEP2_3_MOD = RB_UEP2_TX_EN | RB_UEP3_RX_EN;

    R16_UEP0_MAX_LEN = 64;
    R16_UEP2_MAX_LEN = 512;
    R16_UEP3_MAX_LEN = 512;

    R32_UEP0_RT_DMA = (UINT32)(UINT8 *)endp0RTbuff;

	R32_UEP2_TX_DMA = (UINT32)(UINT8 *)UDisk_In_Buf;
	R32_UEP3_RX_DMA = (UINT32)(UINT8 *)UDisk_Out_Buf;

	R16_UEP0_T_LEN = 0;
	R8_UEP0_TX_CTRL = UEP_T_RES_NAK;
	R8_UEP0_RX_CTRL = 0;

    R16_UEP2_T_LEN = U20_MAXPACKET_LEN;
    R8_UEP2_TX_CTRL = UEP_T_RES_NAK | RB_UEP_T_AUTOTOG;
    R8_UEP3_RX_CTRL = UEP_R_RES_ACK | RB_UEP_R_AUTOTOG;
}

/*******************************************************************************
 * @fn      USB20_Device_Init
 *
 * @brief   USB2.0 Device initialization
 *
 * @param   sta - ENABLE / DISABLE
 *
 * @return   None
 */
void USB20_Device_Init ( FunctionalState sta )  // USBHS device initial
{
    UINT16 i;
    UINT32 *p;
    if(sta)
    {
        R8_USB_CTRL = 0;
        R8_USB_CTRL =  UCST_HS | RB_DEV_PU_EN | RB_USB_INT_BUSY |RB_USB_DMA_EN;
        R8_USB_INT_EN = RB_USB_IE_SETUPACT | RB_USB_IE_TRANS | RB_USB_IE_SUSPEND  |RB_USB_IE_BUSRST ;
        USB20_Endp_Init();
    }
    else
    {
        R8_USB_CTRL = RB_USB_CLR_ALL | RB_USB_RESET_SIE;
    }
}

/*******************************************************************************
 * @fn      USB20_Device_setaddress
 *
 * @brief   USB2.0 Set device address
 *
 * @param   address
 *
 * @return  None
 **/
void USB20_Device_Setaddress( UINT32 address )
{
    R8_USB_DEV_AD = address; // SET ADDRESS
}

/*******************************************************************************
 * @fn       U20_NonStandard_Request_Deal
 *
 * @brief   USB2.0 Interrupt Handler.
 *
 * @return   None
 */
UINT16 U20_NonStandard_Request_Deal()
{
  UINT16 len = 0;

  return len;
}

/*******************************************************************************
 * @fn       U20_Standard_Request_Deal
 *
 * @brief    USB2.0 standard request deal
 *
 * @return   None
 */
UINT16 U20_Standard_Request_Deal()
{
  UINT16 len = 0;
  UINT8 endp_dir;
  SetupLen = 0;
  endp_dir = UsbSetupBuf->bRequestType & 0x80;
  switch( SetupReq )
  {
    case USB_GET_DESCRIPTOR:
    {
        switch( UsbSetupBuf->wValueH )
        {
            case USB_DESCR_TYP_DEVICE:
                pDescr = (UINT8 *)hs_device_descriptor;
                SetupLen = ( SetupReqLen > sizeof(hs_device_descriptor) )? sizeof(hs_device_descriptor):SetupReqLen;
                break;
            case USB_DESCR_TYP_CONFIG:
                pDescr = (UINT8 *)hs_config_descriptor;
                SetupLen = ( SetupReqLen > sizeof(hs_config_descriptor) )? sizeof(hs_config_descriptor):SetupReqLen;
                break;
            case USB_DESCR_TYP_STRING:
                switch( UsbSetupBuf->wValueL )
                {
                    case USB_DESCR_LANGID_STRING:

                        pDescr = (UINT8 *)hs_string_descriptor0;
                        SetupLen = ( SetupReqLen > sizeof(hs_string_descriptor0) )? sizeof(hs_string_descriptor0):SetupReqLen;
                        break;
                    case USB_DESCR_VENDOR_STRING:
                        pDescr = (UINT8 *)hs_string_descriptor1;
                        SetupLen = ( SetupReqLen > sizeof(hs_string_descriptor1) )? sizeof(hs_string_descriptor1):SetupReqLen;
                        break;
                    case USB_DESCR_PRODUCT_STRING:
                        pDescr =(UINT8 *) hs_string_descriptor2;
                        SetupLen = ( SetupReqLen > sizeof(hs_string_descriptor2) )? sizeof(hs_string_descriptor2):SetupReqLen;;
                        break;
                    case USB_DESCR_SERIAL_STRING:
                        break;
                    default:
                        SetupLen = USB_DESCR_UNSUPPORTED;
                        break;
                }
                break;
            case USB_DESCR_TYP_BOS:
                 pDescr =(UINT8 *) hs_bos_descriptor;
                 SetupLen = ( SetupReqLen > sizeof(hs_bos_descriptor) )? sizeof(hs_bos_descriptor):SetupReqLen;
                 break;
            default :
                SetupLen = USB_DESCR_UNSUPPORTED;
                break;
        }
    }
        break;
    case USB_SET_ADDRESS:
        g_devInfo.dev_addr = UsbSetupBuf->wValueL;
        break;
    case USB_GET_CONFIGURATION:
        endp0RTbuff[ 0 ] = g_devInfo.dev_config_value;
        SetupLen = 1;
        break;

    case USB_SET_CONFIGURATION:
        if( (R8_USB_SPD_TYPE & RB_USBSPEED_MASK)  == UST_FS )
        {
            U20_EndpnMaxSize = 64;
        }
        else if( (R8_USB_SPD_TYPE & RB_USBSPEED_MASK) == UST_LS )
        {
            U20_EndpnMaxSize = 8;
        }
        g_devInfo.dev_config_value = UsbSetupBuf->wValueL;
        g_devInfo.dev_enum_status = 0x01;
        break;
    case USB_CLEAR_FEATURE:
        if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
        {
            switch( UsbSetupBuf->wIndexL )
            {
                case 0x82:
                    R16_UEP2_T_LEN= 0;
                    R8_UEP2_TX_CTRL = UEP_T_RES_NAK | RB_UEP_T_TOG_0;
                    break;
                case 0x02:
                    R8_UEP2_TX_CTRL = UEP_R_RES_ACK | RB_UEP_R_TOG_0;
                    break;
                case 0x81:
                    R16_UEP1_T_LEN = 0;
                    R8_UEP1_TX_CTRL = UEP_T_RES_NAK | RB_UEP_T_TOG_0;
                    break;
                case 0x01:
                    R8_UEP1_RX_CTRL = UEP_T_RES_ACK | RB_UEP_R_TOG_0;
                    break;
                default:
                    SetupLen = USB_DESCR_UNSUPPORTED;
                    break;
            }
        }
        else if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
        {
            if( (  UsbSetupBuf->wValueL ) == 1 )
            {
                g_devInfo.dev_sleep_status &= ~0x01;
            }
        }
        else
        {
            SetupLen = USB_DESCR_UNSUPPORTED;
        }
        break;
    case USB_SET_FEATURE:
        if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 )
        {
            if( UsbSetupBuf->wValueL == 0x01 )
            {
                if( hs_config_descriptor[ 7 ] & 0x20 )
                {
                    g_devInfo.dev_sleep_status = 0x01;
                }
                else
                {
                    SetupLen = USB_DESCR_UNSUPPORTED;
                }
            }
            else
            {
                SetupLen = USB_DESCR_UNSUPPORTED;
            }
        }
        else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )
        {
            if( UsbSetupBuf->wValueL == 0x00 )
            {
                switch( UsbSetupBuf->wIndexL )
                {
                    case 0x82:
                        R8_UEP2_TX_CTRL = ( R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK ) | UEP_T_RES_STALL;
                        break;

                    case 0x02:
                        R8_UEP2_RX_CTRL = ( R8_UEP2_RX_CTRL & ~RB_UEP_RRES_MASK ) | UEP_R_RES_STALL;
                        break;

                    case 0x81:
                        R8_UEP1_TX_CTRL = ( R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK ) | UEP_T_RES_STALL;
                        break;

                    case 0x01:
                        R8_UEP1_RX_CTRL = ( R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK ) | UEP_R_RES_STALL;
                        break;

                    default:
                        SetupLen = USB_DESCR_UNSUPPORTED;
                        break;
                }
            }
            else
            {
                SetupLen = USB_DESCR_UNSUPPORTED;
            }
        }
        else
        {
            SetupLen = USB_DESCR_UNSUPPORTED;
        }
        break;
    case USB_GET_INTERFACE:
        break;
    case USB_SET_INTERFACE:
        break;
    case USB_GET_STATUS:
        endp0RTbuff[ 0 ] = 0x00;
        endp0RTbuff[ 1 ] = 0x00;
        SetupLen = 2;
        if( UsbSetupBuf->wIndexL == 0x81 )
        {
            if( ( R8_UEP1_TX_CTRL & RB_UEP_TRES_MASK ) == UEP_T_RES_STALL )
            {
                endp0RTbuff[ 0 ] = 0x01;
                SetupLen = 1;
            }
        }
        else if( UsbSetupBuf->wIndexL == 0x01 )
        {
            if( ( R8_UEP1_RX_CTRL & RB_UEP_RRES_MASK ) == UEP_R_RES_STALL )
            {
                endp0RTbuff[ 0 ] = 0x01;
                SetupLen = 1;
            }
        }
        else if( UsbSetupBuf->wIndexL == 0x82 )
        {
            if( ( R8_UEP2_TX_CTRL & RB_UEP_TRES_MASK ) == UEP_T_RES_STALL )
            {
                endp0RTbuff[ 0 ] = 0x01;
                SetupLen = 1;
            }
        }
        else if( UsbSetupBuf->wIndexL == 0x02 )
        {
            if( ( R8_UEP2_RX_CTRL & RB_UEP_RRES_MASK ) == UEP_R_RES_STALL )
            {
                endp0RTbuff[ 0 ] = 0x01;
                SetupLen = 1;
            }
        }
        break;
    default:
        SetupLen = USB_DESCR_UNSUPPORTED;
        break;
   }

  if( (SetupLen != USB_DESCR_UNSUPPORTED) && (SetupLen != 0))
  {
      len = ( SetupLen >= U20_UEP0_MAXSIZE ) ? U20_UEP0_MAXSIZE : SetupLen;
      if(endp_dir)
      {
          memcpy( endp0RTbuff, pDescr, len );
          pDescr += len;
      }
      SetupLen -= len;
  }
   return len;
}

/*******************************************************************************
 * @fn       USBHSD_IRQHandler
 *
 * @brief    USB2.0 Interrupt Handler.
 *
 * @return   None
 */
void USBHS_IRQHandler(void)			                                //USBHS interrupt service
{
	UINT32 end_num;
	UINT32 rx_token;
	UINT16 ret_len,i;
	UINT16 rxlen;
	UINT8  *p8;
	UINT8 int_flg;
	int_flg = R8_USB_INT_FG;
	if( int_flg & RB_USB_IF_SETUOACT )                   //SETUP interrupt
	{
#if 0
		 printf("SETUP :");
		 p8 = (UINT8 *)endp0RTbuff;
		 for(i=0; i<8; i++)  { printf("%02x ", *p8++); }
		 printf("\n");
#endif
		 SetupReqType = UsbSetupBuf->bRequestType;
		 SetupReq = UsbSetupBuf->bRequest;
		 SetupReqLen = UsbSetupBuf->wLength;            //Data length

		 /*Analyze host requests*/
		 if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD)
         {
             ret_len =  U20_NonStandard_Request_Deal();
         }
         else
         {
             ret_len = U20_Standard_Request_Deal();
         }
		 if(ret_len == 0xFFFF)
         {
              R16_UEP0_T_LEN = 0;
              R8_UEP0_TX_CTRL = UEP_T_RES_STALL ;
              R8_UEP0_RX_CTRL = UEP_R_RES_STALL ;
         }
         else
         {
              R16_UEP0_T_LEN = ret_len;
              R8_UEP0_TX_CTRL = UEP_T_RES_ACK | RB_UEP_T_TOG_1;
              R8_UEP0_RX_CTRL = UEP_R_RES_ACK | RB_UEP_R_TOG_1;
         }
		R8_USB_INT_FG = RB_USB_IF_SETUOACT;              // clear int flag
	}
	/*Transaction transfer complete interrupt*/
	else if( int_flg & RB_USB_IF_TRANSFER )
	{
		end_num =   R8_USB_INT_ST & 0xf;
		rx_token = ( (R8_USB_INT_ST )>>4) & 0x3;
#if 0
		if( !(R8_USB_INT_ST & RB_USB_ST_TOGOK) )
		{
			printf(" TOG MATCH FAIL : ENDP %x token %x \n", end_num, rx_token);
		}
#endif
		switch( end_num )
		{
		   case 0:
                if( rx_token == PID_IN )
                {
                    ret_len = U20_Endp0_IN_Callback();
                    if(ret_len == 0)
                    {
                        R8_UEP0_RX_CTRL = UEP_R_RES_ACK | RB_UEP_R_TOG_1;
                        R16_UEP0_T_LEN = 0;
                        R8_UEP0_TX_CTRL = 0;
                    }
                    else
                    {
                        R16_UEP0_T_LEN = ret_len;
                        R8_UEP0_TX_CTRL ^= RB_UEP_T_TOG_1;
                        R8_UEP0_TX_CTRL = ( R8_UEP0_TX_CTRL &~RB_UEP_TRES_MASK )| UEP_T_RES_ACK ;
                    }
                }
                else if( rx_token == PID_OUT )
                {
                    SetupLen -= SetupLen > R16_USB_RX_LEN ? R16_USB_RX_LEN :SetupLen;
                    if( SetupLen > 0 )
                    {
                       R8_UEP0_RX_CTRL ^=RB_UEP_R_TOG_1;
                       R8_UEP0_RX_CTRL = ( R8_UEP0_RX_CTRL &~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;

                    }
                    else
                    {
                        R16_UEP0_T_LEN = 0;
                        R8_UEP0_TX_CTRL = UEP_T_RES_ACK | RB_UEP_T_TOG_1;
                        R8_UEP0_RX_CTRL = 0 ;
                    }
                }
                break;
		   case 1:
		       break;
		   case 2:
               if( rx_token == PID_IN )
               {
                   R32_UEP2_TX_DMA = (UINT32)(UINT8 *)UDisk_In_Buf;
                   R8_UEP2_TX_CTRL = (R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_NAK;
                   UDISK_In_EP_Deal();
               }
		       break;
		   case 3:
		       if( rx_token == PID_OUT )
               {
		           R32_UEP3_RX_DMA = (UINT32)(UINT8 *)UDisk_Out_Buf;
                   R8_UEP3_RX_CTRL = (R8_UEP3_RX_CTRL & ~RB_UEP_TRES_MASK) | UEP_R_RES_NAK;

                   rxlen = R16_USB_RX_LEN;

                   UDISK_Out_EP_Deal(UDisk_Out_Buf,rxlen);
                   if( UDISK_OutPackflag == 0 )
                   {
                       R8_UEP3_RX_CTRL = (R8_UEP3_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;
                   }
               }
		       break;
           case 4:
                break;
           case 5:
                break;
           case 6:
                break;
           case 7:
                break;
           default:
                break;

		}
		R8_USB_INT_FG = RB_USB_IF_TRANSFER;
	}
	else if( int_flg & RB_USB_IF_BUSRST )
	{
		USB20_Endp_Init();
		USB20_Device_Setaddress( 0 );
		R8_USB_INT_FG = RB_USB_IF_BUSRST;
        if( Link_Sta == LINK_STA_1 )
        {
            PFIC_EnableIRQ(USBSS_IRQn);
            PFIC_EnableIRQ(LINK_IRQn);
            PFIC_EnableIRQ(TMR0_IRQn);
            R8_TMR0_INTER_EN = 1;
            TMR0_TimerInit( 67000000 );
            USB30D_init(ENABLE);
       }
	}
	else if( int_flg & RB_USB_IF_SUSPEND )
	{
		R8_USB_INT_FG = RB_USB_IF_SUSPEND;
	}
}

/*******************************************************************************
 * @fn       U20_Endp0_IN_Callback
 *
 * @brief    U20_Endp0_IN_Callback Handler.
 *
 * @return   None
 */
UINT16 U20_Endp0_IN_Callback(void)
{
    UINT16 len = 0;
    switch(SetupReq)
    {
      case USB_GET_DESCRIPTOR:
          len = SetupLen >= U20_UEP0_MAXSIZE ? U20_UEP0_MAXSIZE : SetupLen;
          memcpy(endp0RTbuff, pDescr, len);
          SetupLen -= len;
          pDescr += len;
          break;
      case USB_SET_ADDRESS:
          USB20_Device_Setaddress(g_devInfo.dev_addr);
          break;
      default:
          break;
    }
    return len;
}


