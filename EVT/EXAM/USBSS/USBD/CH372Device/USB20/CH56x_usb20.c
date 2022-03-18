/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb20.c
* Author             : WCH
* Version            : V1.1
* Date               : 2020/12/23
* Description        :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_usb20.h"
#include "CH56x_usb30.h"
#include "CH56xUSB30_LIB.H"

/*
 * 此例程为USB 2.0 device使用例程
 *  主机通过端点1下传后， 可通过端点1上传一包
 *  主机可直接连续从端点2 取数据
 */

/* Global define */

/* Global Variable */
UINT16V  U20_EndpnMaxSize = 512;
UINT16V  SetupReqLen=0;            //主机请求数据长度
UINT16V  SetupLen = 0;             //数据阶段实际发送或接收的数据长度
UINT32V seq_num = 0;
DevInfo_Typedef  g_devInfo;
static UINT8V SetupReqType = 0;    //主机请求描述符类型
static UINT8V SetupReq = 0;        //主机请求描述符类型
static PUINT8 pDescr;
extern UINT8V link_sta;

__attribute__ ((aligned(16))) UINT8 vendor_buff[16]  __attribute__((section(".DMADATA"))); //端点0数据收发缓冲区
/* Function declaration */
void USBHS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));           //USB2.0 interrupt service

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

const UINT8 hs_config_descriptor[] =
{
    0x09,   // length of this descriptor
    0x02,   // CONFIGURATION (2)
    0x74,   // total length includes endpoint descriptors (should be 1 more than last address)
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
    0x0e,   // Number of endpoints (not counting 0)
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

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x82,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x02,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x83,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x03,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x84,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x04,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x85,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x05,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x86,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x06,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x87,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x07,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00    // polling interval in milliseconds (1 for iso)
};
// ====================================
// ===== String Descriptor Lang ID=====
// ====================================
const UINT8 hs_string_descriptor0[] =
{
    0x04,   // this descriptor length
    0x03,   // descriptor type
    0x09,   // Language ID 0 low byte
    0x04    // Language ID 0 high byte
};
// ====================================
// =====   String Descriptor 1    =====
// ====================================
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
// ====================================
// =====   String Descriptor 2    =====
// ====================================
const UINT8 hs_string_descriptor2[]=
{
    38,         //该描述符的长度为38字节
    0x03,       //字符串描述符的类型编码为0x03
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
 * @brief   USB2.0端点初始化
 *
 * @return  None
 */
void USB20_Endp_Init ()	// USBHS device endpoint initial
{
    R8_UEP4_1_MOD = RB_UEP1_RX_EN | RB_UEP1_TX_EN | RB_UEP4_RX_EN | RB_UEP4_TX_EN;
    R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_RX_EN | RB_UEP3_TX_EN;
    R8_UEP5_6_MOD = RB_UEP5_RX_EN | RB_UEP5_TX_EN | RB_UEP6_RX_EN | RB_UEP6_TX_EN;
    R8_UEP7_MOD  = RB_UEP7_RX_EN |RB_UEP7_TX_EN;                                       //endpoint send and  receive enable

    R16_UEP0_MAX_LEN = 64;
    R16_UEP1_MAX_LEN = 512;
    R16_UEP2_MAX_LEN = 512;
    R16_UEP3_MAX_LEN = 512;
    R16_UEP4_MAX_LEN = 512;
    R16_UEP5_MAX_LEN = 512;
    R16_UEP6_MAX_LEN = 512;
    R16_UEP7_MAX_LEN = 512;

    R32_UEP0_RT_DMA = (UINT32)(UINT8 *)endp0RTbuff;

    R32_UEP1_TX_DMA = (UINT32)(UINT8 *)endp1RTbuff;
	R32_UEP1_RX_DMA = (UINT32)(UINT8 *)endp1RTbuff;

	R32_UEP2_TX_DMA = (UINT32)(UINT8 *)endp2RTbuff;
	R32_UEP2_RX_DMA = (UINT32)(UINT8 *)endp2RTbuff;

	R32_UEP3_TX_DMA = (UINT32)(UINT8 *)endp3RTbuff;
	R32_UEP3_RX_DMA = (UINT32)(UINT8 *)endp3RTbuff;

	R32_UEP4_TX_DMA = (UINT32)(UINT8 *)endp4RTbuff;
	R32_UEP4_RX_DMA = (UINT32)(UINT8 *)endp4RTbuff;

	R32_UEP5_TX_DMA = (UINT32)(UINT8 *)endp5RTbuff;
	R32_UEP5_RX_DMA = (UINT32)(UINT8 *)endp5RTbuff;

	R32_UEP6_TX_DMA = (UINT32)(UINT8 *)endp6RTbuff;
	R32_UEP6_RX_DMA = (UINT32)(UINT8 *)endp6RTbuff;

	R32_UEP7_TX_DMA = (UINT32)(UINT8 *)endp7RTbuff;
	R32_UEP7_RX_DMA = (UINT32)(UINT8 *)endp7RTbuff;

	R16_UEP0_T_LEN = 0;
	R8_UEP0_TX_CTRL = 0;
	R8_UEP0_RX_CTRL = 0;

	R16_UEP1_T_LEN = 0;
	R8_UEP1_TX_CTRL = UEP_T_RES_NAK ;
	R8_UEP1_RX_CTRL = UEP_R_RES_ACK |RB_UEP_R_TOG_0;

    R16_UEP2_T_LEN = U20_MAXPACKET_LEN;
    R8_UEP2_TX_CTRL = UEP_T_RES_ACK | RB_UEP_T_TOG_0;
    R8_UEP2_RX_CTRL = UEP_R_RES_ACK | RB_UEP_R_TOG_0;

    R16_UEP3_T_LEN = 0;
    R8_UEP3_TX_CTRL = UEP_T_RES_NAK;
    R8_UEP3_RX_CTRL = UEP_R_RES_NAK;

    R16_UEP4_T_LEN = 0;
    R8_UEP4_TX_CTRL = UEP_T_RES_NAK ;
    R8_UEP4_RX_CTRL = UEP_R_RES_NAK;

    R16_UEP5_T_LEN = 0;
    R8_UEP5_TX_CTRL = UEP_T_RES_NAK;
    R8_UEP5_RX_CTRL = UEP_R_RES_NAK;

    R16_UEP6_T_LEN = 0;
    R8_UEP6_TX_CTRL = UEP_T_RES_NAK;
    R8_UEP6_RX_CTRL = UEP_R_RES_NAK;

    R16_UEP7_T_LEN = 0;
    R8_UEP7_TX_CTRL = UEP_T_RES_NAK;
    R8_UEP7_RX_CTRL = UEP_R_RES_NAK;
}
/*******************************************************************************
 * @fn      USB20_Device_Init
 *
 * @brief   USB2.0设备初始化
 *
 * @param   sta -
 *            ENABLE - 使能   
 *            DISABLE - 关闭
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
 * @brief   USB2.0设置设备地址
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
        switch( ( ( UsbSetupBuf->wValue.w ) >> 8 ) )
        {
            case USB_DESCR_TYP_DEVICE:              /*获取设备描述符 */
                pDescr = (UINT8 *)hs_device_descriptor;
                SetupLen = ( SetupReqLen > sizeof(hs_device_descriptor) )? sizeof(hs_device_descriptor):SetupReqLen;
                break;
            case USB_DESCR_TYP_CONFIG:               /*获取配置描述符 */
                pDescr = (UINT8 *)hs_config_descriptor;
                SetupLen = ( SetupReqLen > sizeof(hs_config_descriptor) )? sizeof(hs_config_descriptor):SetupReqLen;
                break;
            case USB_DESCR_TYP_STRING:                /*获取字符串描述符*/
                switch( ( UsbSetupBuf->wValue.w ) & 0xff )
                {
                    case USB_DESCR_LANGID_STRING:                        /*语言字符串描述符 */

                        pDescr = (UINT8 *)hs_string_descriptor0;
                        SetupLen = ( SetupReqLen > sizeof(hs_string_descriptor0) )? sizeof(hs_string_descriptor0):SetupReqLen;
                        break;
                    case USB_DESCR_VENDOR_STRING:                        /*产商字符串描述符 */
                        pDescr = (UINT8 *)hs_string_descriptor1;
                        SetupLen = ( SetupReqLen > sizeof(hs_string_descriptor1) )? sizeof(hs_string_descriptor1):SetupReqLen;
                        break;
                    case USB_DESCR_PRODUCT_STRING:                        /*产品字符串描述符 */
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
    case USB_SET_ADDRESS:             /*设置地址 */
        g_devInfo.dev_addr = ( UsbSetupBuf->wValue.w )& 0xff;
        break;
    case USB_GET_CONFIGURATION:       /*获取配置值 */
        endp0RTbuff[ 0 ] = g_devInfo.dev_config_value;
        SetupLen = 1;
        break;

    case USB_SET_CONFIGURATION:       /*设置配置值 */
        if( (R8_USB_SPD_TYPE & RB_USBSPEED_MASK)  == UST_FS )
        {
            U20_EndpnMaxSize = 64;
        }
        else if( (R8_USB_SPD_TYPE & RB_USBSPEED_MASK) == UST_LS )
        {
            U20_EndpnMaxSize = 8;
        }
        g_devInfo.dev_config_value = ( UsbSetupBuf->wValue.w ) & 0xff;
        g_devInfo.dev_enum_status = 0x01;
        break;
    case USB_CLEAR_FEATURE:             /*清除特性 */
        if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )  /*清除端点 */
        {
            switch( ( UsbSetupBuf->wIndex.w ) & 0xff )      /* wIndexL */
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
            if( ( ( UsbSetupBuf->wValue.w ) & 0xff ) == 1 )        /* wIndexL */
            {
                g_devInfo.dev_sleep_status &= ~0x01;
            }
        }
        else
        {
            SetupLen = USB_DESCR_UNSUPPORTED;
        }
        break;
    case USB_SET_FEATURE:        /*设置特性 */
        if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 )            /*设置设备 */
        {
            if( UsbSetupBuf->wValue.w == 0x01 )
            {
                if( hs_config_descriptor[ 7 ] & 0x20 )                    /*设置唤醒使能标志 */
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
        else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )            /*设置端点 */
        {
            if( UsbSetupBuf->wValue.w == 0x00 )                           /*设置指定端点STALL */
            {
                switch( ( UsbSetupBuf->wIndex.w ) & 0xff )
                {
                    case 0x82:        /*设置端点2 IN STALL */
                        R8_UEP2_TX_CTRL = ( R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK ) | UEP_T_RES_STALL;
                        break;

                    case 0x02:        /*设置端点2 OUT Stall */
                        R8_UEP2_RX_CTRL = ( R8_UEP2_RX_CTRL & ~RB_UEP_RRES_MASK ) | UEP_R_RES_STALL;
                        break;

                    case 0x81:         /*设置端点1 IN STALL */
                        R8_UEP1_TX_CTRL = ( R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK ) | UEP_T_RES_STALL;
                        break;

                    case 0x01:        /*设置端点1 OUT STALL */
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
    case USB_GET_STATUS:              /*根据当前端点实际状态进行应答 */
        endp0RTbuff[ 0 ] = 0x00;
        endp0RTbuff[ 1 ] = 0x00;
        SetupLen = 2;
        if( UsbSetupBuf->wIndex.w == 0x81 )
        {
            if( ( R8_UEP1_TX_CTRL & RB_UEP_TRES_MASK ) == UEP_T_RES_STALL )
            {
                endp0RTbuff[ 0 ] = 0x01;
                SetupLen = 1;
            }
        }
        else if( UsbSetupBuf->wIndex.w == 0x01 )
        {
            if( ( R8_UEP1_RX_CTRL & RB_UEP_RRES_MASK ) == UEP_R_RES_STALL )
            {
                endp0RTbuff[ 0 ] = 0x01;
                SetupLen = 1;
            }
        }
        else if( UsbSetupBuf->wIndex.w == 0x82 )
        {
            if( ( R8_UEP2_TX_CTRL & RB_UEP_TRES_MASK ) == UEP_T_RES_STALL )
            {
                endp0RTbuff[ 0 ] = 0x01;
                SetupLen = 1;
            }
        }
        else if( UsbSetupBuf->wIndex.w == 0x02 )
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
  /* 判断是否可以正常处理 */
  if( (SetupLen != USB_DESCR_UNSUPPORTED) && (SetupLen != 0))
  {
      len = ( SetupLen >= U20_UEP0_MAXSIZE ) ? U20_UEP0_MAXSIZE : SetupLen;                //返回发送长度
      if(endp_dir)
      {
          memcpy( endp0RTbuff, pDescr, len );                                               //数据阶段为IN 准备好数据至缓冲区
          pDescr += len;
      }
      SetupLen -= len;                                                                     //剩余未发送数据长度
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
void USBHS_IRQHandler(void)			                                //USBHS interrupt severice
{
	UINT32 end_num;
	UINT32 rx_token;
	UINT16 ret_len,i;
	UINT16 rxlen;
	UINT8  *p8;
	UINT8 int_flg;
	int_flg = R8_USB_INT_FG;
	if( int_flg & RB_USB_IF_SETUOACT )                   //SETUP中断
	{
#if 0
		 printf("SETUP :");
		 p8 = (UINT8 *)endp0RTbuff;
		 for(i=0; i<8; i++)  { printf("%02x ", *p8++); }
		 printf("\n");
#endif
		 SetupReqType = UsbSetupBuf->bRequestType;
		 SetupReq = UsbSetupBuf->bRequest;
		 SetupReqLen = UsbSetupBuf->wLength;   //数据长度

         /*分析主机请求*/
		 if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD)   //非标准请求
         {
             ret_len =  U20_NonStandard_Request_Deal();
         }
         else                                                                        //标准请求
         {
             ret_len = U20_Standard_Request_Deal();
         }
		 if(ret_len == 0xFFFF)                                                  //不支持的描述符
         {
              R16_UEP0_T_LEN = 0;
              R8_UEP0_TX_CTRL = UEP_T_RES_STALL ;
              R8_UEP0_RX_CTRL = UEP_R_RES_STALL ;
         }
         else
         {
              R16_UEP0_T_LEN = ret_len;
              R8_UEP0_TX_CTRL = UEP_T_RES_ACK | RB_UEP_T_TOG_1;           //数据过程或状态过程
              R8_UEP0_RX_CTRL = UEP_R_RES_ACK | RB_UEP_R_TOG_1;
         }
		R8_USB_INT_FG = RB_USB_IF_SETUOACT;              // clear int flag
	}
	else if( int_flg & RB_USB_IF_TRANSFER )        //事务传输完成中断
	{
		end_num =   R8_USB_INT_ST & 0xf;                 //当前产生中断的端点
		rx_token = ( (R8_USB_INT_ST )>>4) & 0x3;           //中断类型  00: OUT, 01:SOF, 10:IN, 11:SETUP
#if 0
		if( !(R8_USB_INT_ST & RB_USB_ST_TOGOK) )
		{
			printf(" TOG MATCH FAIL : ENDP %x token %x \n", end_num, rx_token);
		}
#endif
		switch( end_num )                                 //端点号
		{
		   case 0:
                if( rx_token == PID_IN )                                        //端点零 发送完成中断
                {
                    ret_len = U20_Endp0_IN_Callback();
                    if(ret_len == 0)                                            //数据发送完成
                    {
                        R8_UEP0_RX_CTRL = UEP_R_RES_ACK | RB_UEP_R_TOG_1;       //状态过程为OUT
                        R16_UEP0_T_LEN = 0;                                     //清发送长度
                        R8_UEP0_TX_CTRL = 0;                                    //清发送控制器
                    }
                    else
                    {
                        R16_UEP0_T_LEN = ret_len;
                        R8_UEP0_TX_CTRL ^= RB_UEP_T_TOG_1;
                        R8_UEP0_TX_CTRL = ( R8_UEP0_TX_CTRL &~RB_UEP_TRES_MASK )| UEP_T_RES_ACK ;
                    }
                }
                else if( rx_token == PID_OUT )                                   //端点零 接收完成中断
                {
                    SetupLen -= SetupLen > R16_USB_RX_LEN ? R16_USB_RX_LEN :SetupLen;
                    if( SetupLen > 0 )                                                          //还有数据待接收
                    {
                       R8_UEP0_RX_CTRL ^=RB_UEP_R_TOG_1;
                       R8_UEP0_RX_CTRL = ( R8_UEP0_RX_CTRL &~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;

                    }
                    else                        //无数据接收
                    {
                        R16_UEP0_T_LEN = 0;                                      //状态过程发零长度包
                        R8_UEP0_TX_CTRL = UEP_T_RES_ACK | RB_UEP_T_TOG_1;        //状态过程为IN
                        R8_UEP0_RX_CTRL = 0 ;                                    //无数据接收  接收状态零
                    }
                }
                break;
		   case 1:
		       if( rx_token == PID_IN )
		       {
                   R16_UEP1_T_LEN = 0;
                   R8_UEP1_TX_CTRL ^= RB_UEP_T_TOG_1;                                               //翻转下次准备发送的同步触发位
                   R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_NAK;

                   R8_UEP1_RX_CTRL ^= RB_UEP_R_TOG_1;                                                //先翻转1号端点接收期望的同步触发位
                   R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;          //1号端点接收置ACK
		       }
		       else if(rx_token == PID_OUT)
		       {
		           rxlen = R16_USB_RX_LEN;
                   R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_NAK;     //1号端点接收置NAK

		           R32_UEP1_TX_DMA = (UINT32)(UINT8 *)endp1RTbuff;
		           R16_UEP1_T_LEN =  rxlen;                                                     //设置1号端点发送长度
		           R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL &~RB_UEP_TRES_MASK)|UEP_T_RES_ACK;        //启动发送 置ACK
		       }
		       break;
		   case 2:
		       if(rx_token == PID_IN)
		       {
                   //翻转下次准备发送的同步触发位 bulk传输 data0 data1来回翻转
                   R32_UEP2_TX_DMA = (UINT32)(UINT8 *)endp2RTbuff;
                   R8_UEP2_TX_CTRL ^= RB_UEP_T_TOG_1;

                   //端点状态置成ACK,若发送长度不变 则不需要重新写 R16_UEP2_T_LEN寄存器
                   R8_UEP2_TX_CTRL = (R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
		       }
		       else if(rx_token == PID_OUT)
		       {
		           R32_UEP2_RX_DMA = (UINT32)(UINT8 *)endp2RTbuff;
		           R8_UEP2_RX_CTRL ^= RB_UEP_R_TOG_1;
		           R8_UEP2_RX_CTRL = (R8_UEP2_RX_CTRL &~RB_UEP_RRES_MASK)|UEP_R_RES_ACK;
		       }
		       break;
		   case 3:
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
		R8_USB_INT_FG = RB_USB_IF_TRANSFER;               //清USB事务完成中断
	}
	else if( int_flg & RB_USB_IF_BUSRST )           //总线复位中断
	{
		USB20_Endp_Init();
		USB20_Device_Setaddress( 0 );                          //总线复位 地址清0
		R8_USB_INT_FG = RB_USB_IF_BUSRST;
        if( link_sta == 1 )
        {
            PFIC_EnableIRQ(USBSS_IRQn);
            PFIC_EnableIRQ(LINK_IRQn);
            PFIC_EnableIRQ(TMR0_IRQn);
            R8_TMR0_INTER_EN = 1;
            TMR0_TimerInit( 67000000 );   //约0.5秒
            USB30D_init(ENABLE);
       }
	}
	else if( int_flg & RB_USB_IF_SUSPEND )         //挂起中断
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


