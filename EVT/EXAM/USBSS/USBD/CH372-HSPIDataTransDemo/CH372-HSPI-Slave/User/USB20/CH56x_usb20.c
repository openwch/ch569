/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb20.c
* Author             : WCH
* Version            : V1.0
* Date               : 2022/11/18
* Description        :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_usb20.h"
#include "MAIN.h"
#include "CH56xusb30_lib.h"

/* Global Variable */
UINT16V  U20_EndpnMaxSize = 512;
UINT16V  SetupReqLen=0;            //Host request data length
UINT16V  SetupLen = 0;             //Data length actually sent or received in data phase
static UINT8V SetupReqType = 0;    //Host request descriptor type
static UINT8V SetupReq = 0;        //Host request descriptor type
static PUINT8 pDescr;
UINT32V seq_num = 0;
DevInfo_Typedef  g_devInfo;
extern UINT8V Link_Sta;

/* Function declaration */
void USBHS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));           //USB2.0 interrupt service

const UINT8 hs_device_descriptor[ ] =
{
    0x12,   // bLength
    0x01,   // DEVICE descriptor type
    0x00,   // 2.00
    0x02,
    0x00,   // device class
    0x00,   // device sub-class
    0x00,   // vendor specific protocol
    0x40,   // max packet size 64B
    0x86,   // vendor id-0x1A86(qinheng)
    0x1A,
    0x37,   // product id 0x5537
    0x55,
    0x00,   //bcdDevice 0x0111
    0x11,
    0x01,   // manufacturer index string
    0x02,   // product index string
    0x00,   // serial number index string
    0x01    // number of configurations
};

const UINT8 hs_config_descriptor[ ] =
{
    0x09,   // length of this descriptor
    0x02,   // CONFIGURATION (2)
    0x20,   // total length includes endpoint descriptors (should be 1 more than last address)
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
    0xFF,   // Interface class, ff is vendor specific
    0xFF,   // Interface sub-class
    0xFF,   // Interface protocol
    0x00,   // Index to string descriptor for this interface

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x81,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x01,   // endpoint direction (00 is out) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,    // polling interval in milliseconds (1 for iso)

};

/* Language Descriptor */
const UINT8 hs_string_descriptor0[ ] =
{
    0x04,   // this descriptor length
    0x03,   // descriptor type
    0x09,   // Language ID 0 low byte
    0x04    // Language ID 0 high byte
};

/* Manufacturer Descriptor */
const UINT8 hs_string_descriptor1[ ] =
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
const UINT8 hs_string_descriptor2[ ]=
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

/*******************************************************************************
 * @fn     USB20_Endp_Init
 *
 * @brief   USB2.0 Endpoint initialization
 *
 * @return  None
 */
void USB20_Endp_Init( void )
{
    R8_UEP4_1_MOD = RB_UEP1_RX_EN | RB_UEP1_TX_EN;

    R16_UEP0_MAX_LEN = 64;
    R16_UEP1_MAX_LEN = 512;

    R32_UEP0_RT_DMA = (UINT32)(UINT8 *)endp0RTbuff;
    R32_UEP1_TX_DMA = (UINT32)(UINT8 *)DEF_ENDP1_TX_DMA_ADDR;
	R32_UEP1_RX_DMA = (UINT32)(UINT8 *)DEF_ENDP1_RX_DMA_ADDR;

	R16_UEP0_T_LEN = 0;
	R8_UEP0_TX_CTRL = UEP_T_RES_NAK;
	R8_UEP0_RX_CTRL = 0;

	R16_UEP1_T_LEN = 0;
	R8_UEP1_TX_CTRL = UEP_T_RES_ACK |RB_UEP_T_TOG_1;
	R8_UEP1_RX_CTRL = UEP_R_RES_ACK |RB_UEP_R_TOG_0;
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
void USB20_Device_Init ( FunctionalState sta )
{
    UINT16 i;
    UINT32 *p;
    if( sta )
    {
        R8_USB_CTRL = 0;
        R8_USB_CTRL =  UCST_HS | RB_DEV_PU_EN | RB_USB_INT_BUSY | RB_USB_DMA_EN;
        R8_USB_INT_EN = RB_USB_IE_SETUPACT | RB_USB_IE_TRANS | RB_USB_IE_SUSPEND  | RB_USB_IE_BUSRST ;
        USB20_Endp_Init( );
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
 * @brief    USB2.0 Interrupt Handler.
 *
 * @return   None
 */
UINT16 U20_NonStandard_Request_Deal( void )
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
UINT16 U20_Standard_Request_Deal( void )
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
                SetupLen = ( SetupReqLen > sizeof( hs_device_descriptor ) )? sizeof( hs_device_descriptor ):SetupReqLen;
                break;
            case USB_DESCR_TYP_CONFIG:
                pDescr = (UINT8 *)hs_config_descriptor;
                SetupLen = ( SetupReqLen > sizeof( hs_config_descriptor ) )? sizeof( hs_config_descriptor ):SetupReqLen;
                break;
            case USB_DESCR_TYP_STRING:
                switch( UsbSetupBuf->wValueL )
                {
                    case USB_DESCR_LANGID_STRING:

                        pDescr = (UINT8 *)hs_string_descriptor0;
                        SetupLen = ( SetupReqLen > sizeof( hs_string_descriptor0 ) )? sizeof( hs_string_descriptor0 ):SetupReqLen;
                        break;
                    case USB_DESCR_VENDOR_STRING:
                        pDescr = (UINT8 *)hs_string_descriptor1;
                        SetupLen = ( SetupReqLen > sizeof( hs_string_descriptor1 ) )? sizeof( hs_string_descriptor1 ):SetupReqLen;
                        break;
                    case USB_DESCR_PRODUCT_STRING:
                        pDescr =(UINT8 *) hs_string_descriptor2;
                        SetupLen = ( SetupReqLen > sizeof( hs_string_descriptor2 ) )? sizeof( hs_string_descriptor2 ):SetupReqLen;;
                        break;
                    case USB_DESCR_SERIAL_STRING:
                        break;
                    default:
                        SetupLen = USB_DESCR_UNSUPPORTED;
                        break;
                }
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
        if( ( R8_USB_SPD_TYPE & RB_USBSPEED_MASK )  == UST_FS )
        {
            U20_EndpnMaxSize = 64;
        }
        else if( ( R8_USB_SPD_TYPE & RB_USBSPEED_MASK ) == UST_LS )
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
            if(  UsbSetupBuf->wValueL == 1 )
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
  /* Judge whether it can be handled normally */
  if( ( SetupLen != USB_DESCR_UNSUPPORTED ) && ( SetupLen != 0 ) )
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
void USBHS_IRQHandler(void)
{
	UINT32 end_num;
	UINT32 rx_token;
	UINT16 ret_len,i;
	UINT16 rxlen;
	UINT8  *p8;
	UINT8 int_flg;

    static UINT32 remanlen;
    UINT32 len;
    UINT32 offset;
    UINT16 packnum;
    UINT16 rx_len;

	int_flg = R8_USB_INT_FG;
	/* SETUP Interrupt */
	if( int_flg & RB_USB_IF_SETUOACT )
	{
#if 0
		 printf("SETUP :");
		 p8 = (UINT8 *)endp0RTbuff;
		 for(i=0; i<8; i++)  { printf("%02x ", *p8++); }
		 printf("\n");
#endif
		 SetupReqType = UsbSetupBuf->bRequestType;
		 SetupReq = UsbSetupBuf->bRequest;
		 /* Data length */
		 SetupReqLen = UsbSetupBuf->wLength;

		 /* Analyze host requests */
		 if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD)
         {
             ret_len =  U20_NonStandard_Request_Deal( );
         }
         else
         {
             ret_len = U20_Standard_Request_Deal( );
         }
		 /* Unsupported descriptor */
		 if( ret_len == 0xFFFF )
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
		/* clear int flag */
		R8_USB_INT_FG = RB_USB_IF_SETUOACT;
	}
	/* Transaction transfer complete interrupt */
	else if( int_flg & RB_USB_IF_TRANSFER )
	{

		end_num =   R8_USB_INT_ST & 0xf;
		rx_token = ( R8_USB_INT_ST >> 4 ) & 0x3;
#if 0
		if( !(R8_USB_INT_ST & RB_USB_ST_TOGOK) )
		{
			printf(" TOG MATCH FAIL : ENDP %x token %x \n", end_num, rx_token);
		}
#endif
		switch( end_num )  /*Endpoint number*/
		{
		   case 0:
                if( rx_token == PID_IN )
                {
                    ret_len = U20_Endp0_IN_Callback( );
                    /*Data sending completed*/
                    if( ret_len == 0 )
                    {
                        R8_UEP0_RX_CTRL = UEP_R_RES_ACK | RB_UEP_R_TOG_1;
                        R16_UEP0_T_LEN = 0;
                        R8_UEP0_TX_CTRL = 0;
                    }
                    else
                    {
                        R16_UEP0_T_LEN = ret_len;
                        R8_UEP0_TX_CTRL ^= RB_UEP_T_TOG_1;
                        R8_UEP0_TX_CTRL = ( R8_UEP0_TX_CTRL &~RB_UEP_TRES_MASK ) | UEP_T_RES_ACK ;
                    }
                }
                else if( rx_token == PID_OUT )
                {
                    SetupLen -= SetupLen > R16_USB_RX_LEN ? R16_USB_RX_LEN :SetupLen;
                    if( SetupLen > 0 )
                    {
                       R8_UEP0_RX_CTRL ^= RB_UEP_R_TOG_1;
                       R8_UEP0_RX_CTRL = ( R8_UEP0_RX_CTRL &~ RB_UEP_RRES_MASK) | UEP_R_RES_ACK;

                    }
                    /*No data reception*/
                    else
                    {
                        R16_UEP0_T_LEN = 0;
                        R8_UEP0_TX_CTRL = UEP_T_RES_ACK | RB_UEP_T_TOG_1;
                        R8_UEP0_RX_CTRL = 0 ;
                    }
                }
                break;
		   case 1:
		       if( rx_token == PID_IN )
		       {
		           /* Batch transmission mode processing */
                   /* Switching USB DMA addresses */
                   HSPI_Rx_Data_DealAddr += 512;

                   if( HSPI_Rx_Data_DealAddr >= DEF_HPSI_RX_DMA_ADDR_MAX )
                   {
                       HSPI_Rx_Data_DealAddr = DEF_HPSI_DMA_RX_ADDR0;
                   }
                   R32_UEP1_TX_DMA = (UINT32)(UINT8 *)HSPI_Rx_Data_DealAddr;

                   /* Calculate the remaining amount of data not uploaded */
                   R8_HSPI_INT_EN = 0x00;

                   HSPI_Rx_Data_RemainLen -= Endp1_Up_LastPackLen;
                   remanlen = HSPI_Rx_Data_RemainLen;
                   R8_HSPI_INT_EN = HSPI_Int_En_Save;

		           /* Determine if there is still data to upload */
		           if( remanlen )
		           {
		               /* Calculate the length and number of packages uploaded this time */
		               offset = ( DEF_HPSI_RX_DMA_ADDR_MAX - HSPI_Rx_Data_DealAddr );//Remaining buffer size
		               if( remanlen >= 512 )//The data to be sent is greater than or equal to 512
		               {
		                   /* Determine if the buffer offset is sufficient, and if not, send it to the end of the buffer at most */
		                   if( offset >= 512 )//The remaining buffer is greater than or equal to 4k
		                   {
		                       Endp1_Up_LastPackLen = 512;
		                       len = 512;
		                   }
		                   else
		                   {
		                       Endp1_Up_LastPackLen = offset;
		                       Endp1_Up_LastPackNum = Endp1_Up_LastPackLen / 512;
		                       len = Endp1_Up_LastPackLen % 512;
		                       if( len == 0 )
		                       {
		                           len = 512;
		                       }
		                   }
		               }
		               else//Not full package
		               {
		                   /* Determine if the buffer offset is sufficient, and if not, send it to the end of the buffer at most*/
		                   if( offset >= remanlen )
		                   {
		                       Endp1_Up_LastPackLen = remanlen;
		                   }
		                   else
		                   {
		                       Endp1_Up_LastPackLen = offset;
		                   }
		                   len = Endp1_Up_LastPackLen % 512;
		                   if( len == 0 )
		                   {
		                       len = 512;
		                   }
		               }

		               /* Enable USB data upload*/
	                   R16_UEP1_T_LEN = len;
	                   R8_UEP1_TX_CTRL ^= RB_UEP_T_TOG_1;
	                   R8_UEP1_TX_CTRL = ( R8_UEP1_TX_CTRL &~ RB_UEP_TRES_MASK ) | UEP_T_RES_ACK;

		               /* Set endpoint 1 upload status */
		               Endp1_Up_Status = 0x01;
		           }
		           else
		           {
		               if( HSPI_RX_StopFlag == 1 )
		               {
		                   HSPI_Rx_Data_LoadAddr = DEF_HPSI_DMA_RX_ADDR0;
		                   HSPI_Rx_Data_DealAddr = DEF_HPSI_DMA_RX_ADDR0;

		                   R32_HSPI_RX_ADDR0 = HSPI_Rx_Data_LoadAddr;
		                   R32_HSPI_RX_ADDR1 = HSPI_Rx_Data_LoadAddr + DEF_HSPI_DMA_PACK_LEN;
		                   R32_UEP1_TX_DMA = (UINT32)(UINT8 *)HSPI_Rx_Data_DealAddr;
		                   HSPI_RX_StopFlag = 0;
		               }

		               /* Determine if there is still data to upload */
	                   R8_UEP1_TX_CTRL = ( R8_UEP1_TX_CTRL &~ RB_UEP_TRES_MASK ) | UEP_T_RES_NAK;
		               Endp1_Up_Status = 0x00;
		           }
		       }
		       else if(rx_token == PID_OUT)
		       {
		           /* Endpoint 1 download processing */
		           /* Obtain the number of received packets */
		           /* Switch buffer address */
		           HSPI_Tx_Data_LoadAddr += 512;
		           if( HSPI_Tx_Data_LoadAddr >= DEF_HPSI_TX_DMA_ADDR_MAX )
		           {
		               HSPI_Tx_Data_LoadAddr = DEF_HPSI_DMA_TX_ADDR0;
		           }
		           R32_UEP1_RX_DMA = (UINT32)(UINT8 *)HSPI_Tx_Data_LoadAddr;

		           /* Calculate the amount of data to be downloaded this time */
		           R8_HSPI_INT_EN = 0x00;
		           rx_len = R16_USB_RX_LEN;
		           if( rx_len == 512 )
		           {
		               remanlen = 512;
		           }
		           else
		           {
		               remanlen = rx_len;

		               /* Set USB download pause flag */
		               USB_Down_StopFlag = 0x01;
		           }
		           HSPI_Tx_Data_RemainLen += remanlen;
		           remanlen = DEF_ENDP1_RX_BUF_LEN - HSPI_Tx_Data_RemainLen;
		           R8_HSPI_INT_EN = HSPI_Int_En_Save;

		           /* Determine if there is enough buffer space to continue downloading data */
		           if( ( remanlen >= 512 ) && ( USB_Down_StopFlag == 0x00 ) )
		           {
		               /* Calculate the length and number of packets allowed for download this time */
		               packnum = 1;
		               if( remanlen < 512 )
		               {
		                   packnum = remanlen / 512;
		               }

		               /* Determine whether the buffer offset is sufficient, and if not, limit it */
		               remanlen = ( DEF_HPSI_TX_DMA_ADDR_MAX - HSPI_Tx_Data_LoadAddr );
		               if( remanlen < DEF_ENDP1_IN_BURST_LEVEL * 512 )
		               {
		                   packnum = remanlen / 512;
		               }

		               /* Notify the computer to continue downloading N packets of data */
	                   R8_UEP1_RX_CTRL ^= RB_UEP_R_TOG_1;
	                   R8_UEP1_RX_CTRL = ( R8_UEP1_RX_CTRL &~ RB_UEP_RRES_MASK ) | UEP_R_RES_ACK;

		               Endp1_Down_Status = 0x00;
		           }
		           else
		           {
		               /* Notify the computer to pause downloading */
		               packnum = 0;
                       R8_UEP1_RX_CTRL ^= RB_UEP_R_TOG_1;
                       R8_UEP1_RX_CTRL = ( R8_UEP1_RX_CTRL &~ RB_UEP_RRES_MASK ) | UEP_R_RES_NAK;
		               Endp1_Down_Status = 0x01;
		           }

		           /* Refresh computer download timeout */
		           Endp1_Down_IdleCount = 0x00;
		       }
		       break;
           default:
                break;

		}
		/*Clearing USB transaction completion interrupt*/
		R8_USB_INT_FG = RB_USB_IF_TRANSFER;
	}
	/*Bus reset interrupt*/
	else if( int_flg & RB_USB_IF_BUSRST )
	{
		USB20_Endp_Init( );
		USB20_Device_Setaddress( 0 );
		R8_USB_INT_FG = RB_USB_IF_BUSRST;
        if( Link_Sta == LINK_STA_1 )
        {
            PFIC_EnableIRQ(USBSS_IRQn);
            PFIC_EnableIRQ(LINK_IRQn);
            PFIC_EnableIRQ(TMR0_IRQn);
            R8_TMR0_INTER_EN = 1;
            TMR0_TimerInit( 67000000 );/*About 0.5 seconds*/
            USB30D_init(ENABLE);
       }
	}
	/*Suspend interrupt*/
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
UINT16 U20_Endp0_IN_Callback( void )
{
    UINT16 len = 0;
    switch( SetupReq )
    {
      case USB_GET_DESCRIPTOR:
          len = SetupLen >= U20_UEP0_MAXSIZE ? U20_UEP0_MAXSIZE : SetupLen;
          memcpy( endp0RTbuff, pDescr, len );
          SetupLen -= len;
          pDescr += len;
          break;
      case USB_SET_ADDRESS:
          USB20_Device_Setaddress( g_devInfo.dev_addr );
          break;
      default:
          break;
    }
    return len;
}


