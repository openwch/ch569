/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56X_ENMU.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/


#ifndef __CH56X_ENMU_H__
#define __CH56X_ENMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#define ERR_SUCCESS             0x14
#define ERR_USB_UNKNOWN         0xFA
#define ERR_USB_TRANSFER        0x20    /* NAK/STALL等更多错误码在0x20~0x2F */
#define ERR_USB_CONNECT         0x15    /* 检测到USB设备连接事件,已经连接 */
#define ERR_USB_DISCON          0x16    /* 检测到USB设备断开事件,已经断开 */
#define ERR_USB_BUF_OVER        0x17    /* USB传输的数据有误或者数据太多缓冲区溢出 */

#define ERR_USB_LENGTH          0x18
#define ERR_USB_DATAERR         0x19


/* 以下状态代码1XH用于USB主机方式的操作状态代码,可以自行定义任意值,下面的定义仅为了兼容CH37x */
#define	ERR_USB_UNKNOWN			  			0xFA			/* 未知错误,不应该发生的情况,需检查硬件或者程序错误 */
#ifndef	USB_INT_SUCCESS
#define	USB_OPERATE_SUCCESS			  		0x00			/* USB操作成功 */
#define	USB_OPERATE_ERROR			  		0x01			/* USB操作失败 */
#define	USB_PARAMETER_ERROR			  		0x03									/* 参数错误, 范围溢出 */
#define	USB_INT_CONNECT_U20					0x13			/* 检测到USB设备连接事件 */
#define	USB_INT_SUCCESS						0x14			/* USB事务或者传输操作成功 */
#define	USB_INT_CONNECT						0x15			/* 检测到USB设备连接事件 */
#define	USB_INT_DISCONNECT				    0x16			/* 检测到USB设备断开事件 */
#define	USB_INT_BUF_OVER					0x17			/* USB控制传输的数据太多, 缓冲区溢出 */
#define	USB_INT_DISK_ERR					0x1F			/* USB存储器操作失败 */
//#define	USB_INT_DISK_ERR1					0x20			/* USB存储器操作失败 */
#define	USB_CH56XUSBTIMEOUT				    0x28			/* USB设备超时 */
#endif

#define U30_PID_OUT                     0x00			/* OUT */
#define U30_PID_IN                      0x01			/* IN */

// USB CONTROL
#define DMA_EN				(1<<0)
#define USB_ALL_CLR			(1<<1)
#define USB_FORCE_RST		(1<<2)
#define INT_BUSY_EN			(1<<3)
#define DEV_PU_EN			(1<<4)	

#define HOST_MODE			(1<<7)

#define FULL_SPEED			(0<<5)
#define HIGH_SPEED			(1<<5)
#define LOW_SPEED			(2<<5)

// UHOST

#define HOST_TX_EN			(1<<14)
#define HOST_RX_EN			(1<<11)

#define SEND_BUS_RESET		(UINT32)(1<<8)
#define SEND_BUS_SUSPEND	(UINT32)(1<<9)
#define SEND_BUS_RESUME		(UINT32)(1<<10)
//#define LINK_READY			(UINT32)(1<<14)
#define SEND_SOF_EN			(UINT32)(1<<15)

// INT_EN
#define USB2_DETECT_EN		(1<<16)
#define USB2_ACT_EN			(1<<17)
#define USB2_SUSP_EN		(1<<18)
#define USB2_SOF_EN			(1<<19)
#define USB2_OVER_EN		(1<<20)
#define USB2_SETUP_EN		(1<<21)
#define USB2_ISO_EN			(1<<22)

#define USB2_ATTACH			(1<<9)

// INT_FLAG
#define USB2_DETECT_FLAG	(1<<16)
#define USB2_ACT_FLAG		(1<<17)
#define USB2_SUSP_FLAG		(1<<18)
#define USB2_SOF_FLAG		(1<<19)
#define USB2_OVER_FLAG		(1<<20)
#define USB2_SETUP_FLAG		(1<<21)
#define USB2_ISO_FLAG		(1<<22)

#define EP_T_RES_MASK		(3<<16)
#define EP_T_RES_ACK		(0<<16)
#define EP_T_RES_NYET		(1<<16)
#define EP_T_RES_NAK		(2<<16)
#define EP_T_RES_STALL		(3<<16)

#define EP_T_TOG_0			(0<<19)
#define EP_T_TOG_1			(1<<19)			

#define EP_R_RES_MASK		(3<<24)
#define EP_R_RES_ACK		(0<<24)
#define EP_R_RES_NYET		(1<<24)
#define EP_R_RES_NAK		(2<<24)
#define EP_R_RES_STALL		(3<<24)

#define EP_R_TOG_0			(0<<19)
#define EP_R_TOG_1			(1<<27)

#define TOG_MATCH			(UINT32)(1<<30)

// 00: OUT, 01:SOF, 10:IN, 11:SETUP
#define PID_OUT		0
#define PID_SOF		1
#define PID_IN		2
//#define PID_SETUP	3

#ifndef U30USB_ENDP_DESCR
typedef struct _USB_ENDPOINT_DESCRIPTOR_U30 /*端点描述符*/
{
	UINT8  bLength;
	UINT8  bDescriptorType;
	UINT8  bEndpointAddress;
	UINT8  bmAttributes;
	UINT8  wMaxPacketSizeL;
	UINT8  wMaxPacketSizeH;
	UINT8  bInterval;

	UINT8  bLength1;				//3.0的备份描述符
	UINT8  bDescriptorType1;
	UINT8  bMaxBurst1;
	UINT8  bmAttributes1;
	UINT8  wBytesPerInterval_L;
	UINT8  wBytesPerInterval_H;
}USB_ENDP_DESCR_U30, *PUSB_ENDP_DESCR_U30;
#endif

#ifndef USB_CFG_DESCR_LONG_U30
typedef struct _USB_CONFIG_DESCRIPTOR_LONG_U30
{
	USB_CFG_DESCR  cfg_descr;
	USB_ITF_DESCR  itf_descr;
	USB_ENDP_DESCR_U30 endp_descr[2];
}USB_CFG_DESCR_LONG_U30, *PUSB_CFG_DESCR_LONG_U30;
#endif


/* USB device classes */
#ifndef USB_DEV_CLASS_HUB
#define USB_DEV_CLASS_RESERVED  		0x00
#define USB_DEV_CLASS_AUDIO     		0x01
#define USB_DEV_CLASS_COMMUNIC  		0x02
#define USB_DEV_CLASS_HUMAN_IF  		0x03
#define USB_DEV_CLASS_MONITOR   		0x04
#define USB_DEV_CLASS_PHYSIC_IF 		0x05
#define USB_DEV_CLASS_POWER     		0x06
#define USB_DEV_CLASS_PRINTER   		0x07
#define USB_DEV_CLASS_STORAGE   		0x08
#define USB_DEV_CLASS_HUB       		0x09
#define USB_DEV_CLASS_VEN_SPEC  		0xFF
#endif
/* USB descriptor type */
#ifndef USB_DEV_DESCR_TYPE
#define USB_DEV_DESCR_TYPE      		0x01
#define USB_CFG_DESCR_TYPE      		0x02
#define USB_STR_DESCR_TYPE      		0x03
#define USB_ITF_DESCR_TYPE      		0x04
#define USB_ENDP_DESCR_TYPE     		0x05
#define USB_PWR_DESCR_TYPE      		0x06
#define USB_DEV_QUAL_DESCR_TYPE 		0x06
#define USB_CFG_PWR_DESCR_TYPE  		0x07
#define USB_ITF_PWR_DESCR_TYPE  		0x08
#define USB_HID_DESCR_TYPE      		0x21
#define USB_REPORT_DESCR_TYPE   		0x22
#define USB_PHYSIC_DESCR_TYPE   		0x23
#endif
#define USB_U30_SPEED					0x02
#define USB_U20_SPEED					0x01
extern UINT8   gDeviceUsbType;															/* USB连接状态，01--USB2.0&1.1  02--USB3.0*/


#define	Usb_Tx_DMAaddr	0x20030000
#define pUsb_Tx_DMAaddr	( ( PUINT8 )0x20030000 )
#define	Usb_Rx_DMAaddr	0x20030400
#define pUsb_Rx_DMAaddr	( ( PUINT8 )0x20030400 )


/* USB主机控制传输SETUP包结构体指针定义 */
extern USB_SETUP_REQ   *Ctl_Setup;
extern UINT8V   gDeviceConnectstatus;													/* USB连接状态 */
extern void  CH56X_USB30_LmpInit( void );
extern UINT8 U30HOST_SetConfig( UINT8 cfg );
extern UINT8 U30HSOT_Enumerate( UINT8 *pbuf );
extern UINT8 U30HOST_SetAddress( UINT8 addr );
extern UINT8 U30HOST_ClearEndpStall( UINT8 endp );
extern UINT8 U30HOST_CofDescrAnalyse( UINT8 *pbuf );
extern UINT8 U30OST_GetDeviceDescr( UINT8 *buf ,UINT16 *len );
extern UINT8 U30HOST_GetConfigDescr( UINT8 *buf ,UINT16 *len );
extern UINT8 U30HostCtrlTransfer( PUINT8 ReqBuf, PUINT8 DatBuf, PUINT16 RetLen ) ; // 执行控制传输,ReqBuf指向8字节请求码,DatBuf为收发缓冲区


extern UINT8 U20OST_GetDeviceDescr( UINT8 *buf ,UINT16 *len );
extern UINT8 U20HOST_GetConfigDescr( UINT8 *buf ,UINT16 *len );
extern UINT8 U20HOST_SetAddress( UINT8 addr );
extern UINT8 U20HOST_SetConfig( UINT8 cfg );
extern UINT8 U20HOST_ClearEndpStall( UINT8 endp );
extern UINT8 U20HostCtrlTransfer(UINT8 *ReqBuf, UINT8 *DataBuf, UINT16 *RetLen );
extern UINT8 U20USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT32 timeout );
extern UINT8 U20HSOT_Enumerate( UINT8 *pbuf );


#ifdef __cplusplus
}
#endif

#endif
