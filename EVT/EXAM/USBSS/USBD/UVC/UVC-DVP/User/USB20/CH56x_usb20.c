 /********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb20.c
* Author             : WCH
* Version            : V1.1
* Date               : 2020/12/23
* Description        :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_usb20.h"
#include "CH56x_usb30.h"
#include "UVCLIB.H"
/* Global Variable */
UINT32V seq_num = 0;
DevInfo_Typedef  g_devInfo;
static UINT16V  SetupReqLen=0;     //Host request data length
static UINT16V  SetupLen = 0;      //Data length actually sent or received in data phase
static UINT8V SetupReqType = 0;    //Host request descriptor type
static UINT8V SetupReq = 0;        //Host request descriptor type
static PUINT8 pDescr;
extern UINT8V Link_sta;

__attribute__ ((aligned(16))) UINT8 vendor_buff[16]  __attribute__((section(".DMADATA"))); //Data sending and receiving buffer of endpoint 0
/* Function declaration */
void USBHS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* Device Descriptor */
const UINT8 hs_device_descriptor[] =
{
    0x12,   // bLength
    0x01,   // DEVICE descriptor type
    0x00,   // 2.00
    0x02,
    0xef,
    0x02,
    0x01,   //Device Dvp_Class
    0x40,
    0x86,   // vendor id-0x1A86(qinheng)
    0x1A,
    0x1c,
    0xfe,   //PID
    0x00,
    0x01,   //bcdDevice
    0x01,   // manufacturer index string
    0x02,   // product index string
    0x00,   // serial number index string
    0x01    // number of configurations
};

/* Configuration Descriptor */
const UINT8 hs_config_descriptor[] =
{
        0x09,
        0x02,
        0xec,
        0x01,
        0x02,
        0x01,
        0x00,
        0xa0,
        0x4b,
        /* Interface Association Descriptor */
        0x08,
        0x0b,
        0x00,
        0x02,
        0x0e,
        0x03,
        0x00,
        0x08,

        /* Standard VC Interface Descriptor */
        0x09,
        0x04,
        0x00,
        0x00,
        0x00,
        0x0e,
        0x01,
        0x00,
        0x08,
        /* Dvp_Class-specific VC Interface Descriptor */
        0x0d,
        0x24,
        0x01,
        0x00,
        0x01,
        0x4d,
        0x00,
        0x00,
        0x0e,
        0x27,
        0x07,
        0x01,
        0x01,
        /* Input Terminal Descriptor (Composite) */
        0x12,
        0x24,
        0x02,
        0x01,
        0x01,
        0x02,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x03,
        0x00,
        0x00,
        0x00,

        0x0b,
        0x24,
        0x05,
        0x03,
        0x01,
        0x00,
        0x00,
        0x02,
        0x00,
        0x00,
        0x00,
        /* Output Terminal Descriptor */
        0x09,
        0x24,
        0x03,
        0x02,
        0x01,
        0x01,
        0x00,
        0x04,
        0x00,
        /* Extension Unit Descriptor */
        0x1a,
        0x24,
        0x06,
        0x04,
        0x5b,
        0x12,
        0x6d,
        0xc6,
        0x80,
        0x04,
        0x44,
        0x08,
        0x8c,
        0x26,
        0xb8,
        0x36,
        0x3a,
        0x84,
        0xde,
        0x63,
        0x03,
        0x01,
        0x03,
        0x01,
        0x07,
        0x00,
        /* Standard VS Interface Descriptor */
        0x09,
        0x04,
        0x01,
        0x00,
        0x00,
        0x0e,
        0x02,
        0x00,
        0x00,
        /* VS Interface Input Header Descriptor */
        0x0e,
        0x24,
        0x01,
        0x01,
        0x6c,    //l
        0x01,    //h
        0x81,
        0x00,
        0x02,
        0x00,
        0x00,
        0x00,
        0x01,
        0x00,

        /* Dvp_Class-specific VS Format Descriptor--- VS_FORMAT_MJPEG */
        0x0b,                                                                       /* bLength */
        0x24,                                                                       /* bDescriptorType£ºCS_INTERFACE */
        0x06,                                                                       /* bDescriptorSubtype£º VS_FORMAT_MJPEG */
        0x01,                                                                       /* bFormatIndex£ºFirst (and only) format descriptor  */
        0x05,                                                                       /* bNumFrameDescriptors£º12 frame descriptor for this format follows */
        0x01,                                                                       /* bmFlags£ºUses fixed size samples */
        0x01,                                                                       /* bDefaultFrameIndex£ºDefault frame index is 1. */
        0x00,                                                                       /* bAspectRatioX */
        0x00,                                                                       /* bAspectRatioY */
        0x00,                                                                       /* bmInterlaceFlags */
        0x00,                                                                       /* bCopyProtect */

        /* 800*600@15fps */
        0x1e,                                                                       /* bLength: 10+(4*n)-4+m */
        0x24,                                                                       /* bDescriptorType: CS_INTERFACE descriptor type */
        0x07,                                                                       /* bDescriptorSubtype: VS_FRAME_MJPEG */
        0x01,                                                                       /* bFrameIndex: First (and only) frame descriptor */
        0x00,                                                                       /* bmCapabilities */
        0x20,0x03,                                                                  /* Width of frame£º800 */
        0x58,0x02,                                                                  /* Height of frame£º 600 */
        0x00,0xA0,0xBB,0x0D,                                                        /* Min bit rate in bits/s */
        0x00,0xA0,0xBB,0x0D,                                                        /* Max bit rate in bits/s 800*600*2*30fps * 8*/
        0x00,0xA6,0x0E,0x00,                                                        /* Maximum video or still frame size, in bytes£º 800*600*2  */
        0x2a,0x2c,0x0a,0x00,                                                        /* Default frame interval is 15fps */
        0x01,                                                                       /* bFrameIntervalType: Continuous frame interval */
        0x2a,0x2c,0x0a,0x00,                                                        /* Default frame interval is 15fps */

        /* 352*288@15fps */
        0x1e,
        0x24,
        0x07,
        0x02,
        0x00,
        0x60,0x01,
        0x20,0x01,
        0x00,0x80,0xE6,0x02,
        0x00,0x80,0xE6,0x02,
        0x00,0x18,0x03,0x00,
        0x2a,0x2c,0x0a,0x00,
        0x01,
        0x2a,0x2c,0x0a,0x00,

        /* 640*480@15fps */
        0x1e,
        0x24,
        0x07,
        0x03,
        0x00,
        0x80,0x02,
        0xe0,0x01,
        0x00,0x00,0xca,0x08,
        0x00,0x00,0xca,0x08,
        0x00,0x60,0x09,0x00,
        0x2a,0x2c,0x0a,0x00,
        0x01,
        0x2a,0x2c,0x0a,0x00,

        /* 1024*768@15fps */
        0x1e,
        0x24,
        0x07,
        0x04,
        0x00,
        0x00,0x04,
        0x00,0x03,
        0x00,0x00,0x80,0x16,
        0x00,0x00,0x80,0x16,
        0x00,0x00,0x18,0x00,
        0x2a,0x2c,0x0a,0x00,
        0x01,
        0x2a,0x2c,0x0a,0x00,

        /* 1600*1200@15fps */
        0x1e,
        0x24,
        0x07,
        0x05,
        0x00,
        0x40,0x06,
        0xb0,0x04,
        0x00,0x80,0xee,0x36,
        0x00,0x80,0xee,0x36,
        0x00,0x98,0x3a,0x00,
        0x2a,0x2c,0x0a,0x00,
        0x01,
        0x2a,0x2c,0x0a,0x00,


        /* Color Matching Descriptor */
        0x06,
        0x24,
        0x0d,
        0x01,
        0x01,
        0x04,

        /* Dvp_Class-specific VS Format Descriptor----VS_FORMAT_UNCOMPRESSED */
        0x1b,
        0x24,
        0x04,
        0x02,
        0x05,
        0x59,
        0x55,
        0x59,
        0x32,
        0x00,
        0x00,
        0x10,
        0x00,
        0x80,
        0x00,
        0x00,
        0xaa,
        0x00,
        0x38,
        0x9b,
        0x71,
        0x10,
        0x01,
        0x00,
        0x00,
        0x00,
        0x00,

        /* Dvp_Class-specific VS Frame Descriptor*/
        /* 640*480@15fps */
        0x1e,
        0x24,
        0x05,
        0x01,
        0x00,
        0x80,0x02,
        0xe0,0x01,
        0x00,0x00,0xca,0x08,
        0x00,0x00,0xca,0x08,
        0x00,0x60,0x09,0x00,
        0x2a,0x2c,0x0a,0x00,
        0x01,
        0x2a,0x2c,0x0a,0x00,

        /* 800*600@15fps */
        0x1e,
        0x24,
        0x05,
        0x02,
        0x00,
        0x20,0x03,
        0x58,0x02,
        0x00,0xA0,0xBB,0xD,
        0x00,0xA0,0xBB,0xD,
        0x00,0xA6,0xE,0x00,
        0x2a,0x2c,0x0a,0x00,
        0x01,
        0x2a,0x2c,0x0a,0x00,

        /* 176*144@15fps */
        0x1e,
        0x24,
        0x05,
        0x03,
        0x00,
        0xb0,0x00,
        0x90,0x00,
        0x00,0xa0,0xb9,0x00,
        0x00,0xa0,0xb9,0x00,
        0x00,0xc6,0x00,0x00,
        0x2a,0x2c,0x0a,0x00,
        0x01,
        0x2a,0x2c,0x0a,0x00,

        /* 320*240@15fps */
        0x1e,
        0x24,
        0x05,
        0x04,
        0x00,
        0x40,0x01,
        0xf0,0x00,
        0x00,0x80,0x32,0x02,
        0x00,0x80,0x32,0x02,
        0x00,0x58,0x02,0x00,
        0x2a,0x2c,0x0a,0x00,
        0x01,
        0x2a,0x2c,0x0a,0x00,

        /* 352*288@15fps */
        0x1e,
        0x24,
        0x05,
        0x05,
        0x00,
        0x60,0x01,
        0x20,0x01,
        0x00,0x80,0xE6,0x02,
        0x00,0x80,0xE6,0x02,
        0x00,0x18,0x03,0x00,
        0x2a,0x2c,0x0a,0x00,
        0x01,
        0x2a,0x2c,0x0a,0x00,

        /* Color Matching Descriptor */
        0x06,
        0x24,
        0x0d,
        0x01,
        0x01,
        0x04,

        0x09,0x04,0x01,0x01,0x01,0x0e,0x02,0x00,0x00,
        0x07,0x05,0x81,0x05,0x00,0x14,0x01,

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


/*******************************************************************************
 * @fn     USB20_Endp_Init
 *
 * @brief   USB2.0 Endpoint initialization
 *
 * @return  None
 */
void USB20_Endp_Init () // USBHS device endpoint initial
{
    R8_UEP4_1_MOD = RB_UEP1_TX_EN;

    R16_UEP0_MAX_LEN = 64;
    R16_UEP1_MAX_LEN = 1024;

    R32_UEP0_RT_DMA = (UINT32)(UINT8 *)endp0RTbuff;
    R32_UEP1_TX_DMA = (UINT32)(UINT8 *)endp1RTbuff;

    R16_UEP0_T_LEN = 0;
    R8_UEP0_TX_CTRL = UEP_T_RES_NAK;
    R8_UEP0_RX_CTRL = 0;

    R16_UEP1_T_LEN = 0;
    R8_UEP1_TX_CTRL = RB_UEP_TRES_NO;

    R8_UEP3_TX_CTRL = UEP_T_RES_NAK;
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
    if(sta)
    {
        R8_USB_CTRL = 0;
        R8_USB_CTRL =  UCST_HS | RB_DEV_PU_EN | RB_USB_INT_BUSY |RB_USB_DMA_EN;
        R8_USB_INT_EN = RB_USB_IE_SETUPACT | RB_USB_IE_TRANS | RB_USB_IE_SUSPEND  |RB_USB_IE_BUSRST |RB_USB_IE_ISOACT;
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
    R8_USB_DEV_AD = address;
}

/*******************************************************************************
 * @fn       U20_NonStandard_Request_Deal
 *
 * @brief    USB2.0 Interrupt Handler.
 *
 * @return   None
 */
UINT16 U20_NonStandard_Request_Deal(){
    SetupLen = SetupReqLen;
    UINT16 len = 0xFFFF;
    /*Upload data*/
    if(UsbSetupBuf->bRequestType & 0x80){
        len = UVC_NonStandardReq(&pDescr);
        if(len != 0xFFFF){
            len = SetupLen >= U20_UEP0_MAXSIZE ? U20_UEP0_MAXSIZE : SetupLen;
            memcpy(endp0RTbuff, pDescr,len );
            SetupLen -= len;
            pDescr += len;
        }
    }else{
        /*set cur*/
        if(UsbSetupBuf->bRequestType==0x21 && UsbSetupBuf->bRequest == 0x01){
            len = 0;
        }
    }
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
            default :
                SetupLen = USB_DESCR_UNSUPPORTED;
                break;
        }
    }
        break;
    case USB_SET_ADDRESS:
        g_devInfo.dev_addr = UsbSetupBuf->wValueL;
        break;
    case 0x31:
        SetupLen = UsbSetupBuf->wValueL;
        break;
    case 0x30:
        break;
    case USB_GET_CONFIGURATION:
        endp0RTbuff[ 0 ] = g_devInfo.dev_config_value;
        SetupLen = 1;
        break;
    case USB_SET_CONFIGURATION:
        break;
    case USB_CLEAR_FEATURE:
        break;
    case USB_SET_FEATURE:
        break;
    case USB_GET_INTERFACE:
        break;
    case USB_SET_INTERFACE:
        HS_CtrlCamera();
        break;
    case USB_GET_STATUS:
        break;
    default:
        SetupLen = USB_DESCR_UNSUPPORTED;
        break;
   }
  /* Judge whether it can be handled normally */
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
void USBHS_IRQHandler(void)
{
    UINT32 end_num;
    UINT32 rx_token;
    UINT16 ret_len,i;
    UINT16 rxlen;
    UINT8  *p8;
    UINT8 int_flg;
    static UINT8 format = 0;
    static UINT8 frame = 0;

    int_flg = R8_USB_INT_FG;
    /*SETUP Interrupt*/
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
         /*Data length*/
         SetupReqLen = UsbSetupBuf->wLength;

         /*Analyze host requests*/
         if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD)
         {
             ret_len =  U20_NonStandard_Request_Deal();
         }
         else
         {
             ret_len = U20_Standard_Request_Deal();
         }
         /*Unsupported descriptor*/
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
        /*clear int flag*/
        R8_USB_INT_FG = RB_USB_IF_SETUOACT;
    }
    else if( int_flg & RB_USB_IF_ISOACT )
    {
        R8_USB_INT_FG = RB_USB_IF_ISOACT ;
        HS_Endp1_ISOHander();
    }
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
        /*Endpoint number*/
        switch( end_num )
        {
           case 0:
                /*Endpoint zero send complete interrupt*/
                if( rx_token == PID_IN )
                {
                    ret_len = U20_Endp0_IN_Callback();
                    /*Data sending completed*/
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
                    /* Switch video format or resolution */
                    if(R16_USB_RX_LEN == 0x1A)
                    {
                        if( (format != *(UINT8 *)(endp0RTbuff+2)) || (frame != *(UINT8 *)(endp0RTbuff+3))){
                            format = *(UINT8 *)(endp0RTbuff+2);
                            frame = *(UINT8 *)(endp0RTbuff+3);
                            Get_Curr[2] = format;
                            Get_Curr[3] = frame;
                            Formatchange_flag = *(UINT8 *)(endp0RTbuff+2);
                            OV2640_Format_Mode(Formatchange_flag);
                            OV2640_Change_Resolution(format,frame);
                        }
                    }
                }
                break;
           case 1:
               if( rx_token == PID_IN )
               {
                   HS_Endp1_Hander();
               }
               else if(rx_token == PID_OUT)
               {

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
        USB20_Endp_Init();
        USB20_Device_Setaddress( 0 );
        R8_USB_INT_FG = RB_USB_IF_BUSRST;
        if( Link_sta == 1 )
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
