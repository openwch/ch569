/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56XUSB30_LIB.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
*******************************************************************************/
#ifndef CH56XUSB30_LIB_H_
#define CH56XUSB30_LIB_H_

#include "core_riscv.h"

/**********device status***********/
typedef enum _DEVICE_STATE
{
  UNCONNECTED,
  ATTACHED,
  POWERED,
  SUSPENDED,
  ADDRESSED,
  CONFIGURED
} DEVICE_STATE;
/*********************/
typedef union
{
  UINT16 w;
  struct BW
  {
    UINT8 bb1; //低字节
    UINT8 bb0;
  }
  bw;
} UINT16_UINT8;
/**********standard request command***********/
typedef struct __PACKED
{
    UINT8  bRequestType;
    UINT8  bRequest;
    UINT16_UINT8 wValue;
    UINT16_UINT8 wIndex;
    UINT16 wLength;
} *PUSB_SETUP;

#define ENDP0_MAXPACK		512

// status response
#define NRDY_TP				0
#define ACK_TP				0x01
#define STALL_TP			0x02
#define INVALID				0x03

// number of NUMP
#define NUMP_0				0x00
#define NUMP_1				0x01
#define NUMP_2				0x02
#define NUMP_3				0x03
#define NUMP_4				0x04
#define NUMP_5				0x05
#define NUMP_6				0x06


/* USB endpoint direction */
#define endp_out			0x00
#define endp_in				0x80
/* USB endpoint serial number */
#define endp_0				0x00
#define endp_1				0x01
#define endp_2				0x02
#define endp_3				0x03
#define endp_4				0x04
#define endp_5				0x05
#define endp_6				0x06
#define endp_7				0x07


#define USB_DESCR_TYP_BOS       0x0f
#define USB_DESCR_UNSUPPORTED   0xffff
#define INVALID_REQ_CODE 0xFF
/* string descriptor type */
#ifndef USB_DESCR_STRING
#define USB_DESCR_LANGID_STRING    0x00
#define USB_DESCR_VENDOR_STRING    0x01
#define USB_DESCR_PRODUCT_STRING   0x02
#define USB_DESCR_SERIAL_STRING    0x03
#define USB_DESCR_OS_STRING        0xee
#endif
/*******************************************************************************
 * @fn     USB30_initDevice
 *
 * @brief  USB3.0 Device初始化
 *
 * @return   None
 */
extern UINT32 USB30_initDevice(void);

/*******************************************************************************
 * @fn     set_ISO_delay
 *
 * @brief  标准请求，同步传输发送/接收包的延迟时间量
 *
 * @return   None
 */
extern void set_ISO_delay( UINT32 dly );

/*******************************************************************************
 * @fn     USB30_setEndpDMA
 *
 * @brief  设置端点DMA地址
 *
 * @param  num -  端点号   最高位表方向，低四位为端点号
 *                addr - 将要设置的DMA地址
 *
 * @return   None
 */
extern void USB30_setEndpDMA(UINT8 num,UINT8 *addr);

/*******************************************************************************
 * @fn     USB30_setISOEndp
 *
 * @brief  设置端点DMA地址
 *
 * @param  num -  端点号   最高位表方向，低四位为端点号
 *                Status - enable or disable
 *
 * @return   None
 */
extern void USB30_setISOEndp(UINT8 num,FunctionalState Status );

/*******************************************************************************
 * @fn     UART1_DefInit
 *
 * @brief  使能端点
 *
 * @param  num -  端点号   最高位表方向，低四位为端点号
 *                Status - enable or disable
 *
 * @return   None
 */
extern void USB30_setEnableEndp(UINT8 num,FunctionalState Status );

/*******************************************************************************
 * @fn     USB30_getRxCount
 *
 * @brief  获取端点接收数据长度
 *
 * @param  endp -  端点号
 *                nump - 端点能够接收的数据包个数
 *                len - 端点接收的长度，对于突发传输表示端点接收最后一包的数据长度
 *
 * @return   None
 */
extern void USB30_getRxCount(UINT8 endp,UINT8 *nump,UINT16 *len,UINT8 *status);

/*******************************************************************************
 * @fn     USB30_getRxCount
 *
 * @brief  获取端点接收数据长度
 *
 * @param  endp -  将要设置的端点号
 *                 status - NREDY_TP/ACK_TP/STALL_TP/INVALID
 *                nump - 端点0能接收到数据包的个数，端点1~7突发传输能接收数据包的个数
 *
 * @return   None
 */
extern void USB30_setRxCtrl(UINT8 endp,UINT8 status,UINT8 nump);

/*******************************************************************************
 * @fn     USB30_getRxCount
 *
 * @brief  获取端点接收数据长度
 *
 * @param  endp -  端点号
 *                 status - NREDY_TP/ACK_TP/STALL_TP/INVALID
 *                nump - 端点0能接收到数据包的个数，端点1~7突发传输能接收数据包的个数
 *                TxLen - 端点发送数据长度
 *
 * @return   None
 */
extern void USB30_setTxCtrl(UINT8 endp,FunctionalState lpf,UINT8 status,UINT8 nump,UINT16 TxLen);

/*******************************************************************************
 * @fn     USB30_getTxNump
 *
 * @brief  获取实际突发包数
 *
 * @param  endp -  端点号
 *
 * @return   当前突发包数
 */
extern UINT8 USB30_getTxNump(UINT8 endp);

/*******************************************************************************
 * @fn     USB30_getRxCount
 *
 * @brief  获取端点接收数据长度
 *
 * @param  endp -  端点号
 *                nump - 端点接收或发送的数据包长度
 *
 * @return   None
 */
extern void USB30_setFCCTRL(UINT8 endp,UINT8 nump);

/*******************************************************************************
 * @fn     set_device_address
 *
 * @brief  设置设备地址
 *
 * @param  address -  将要设置的地址
 *
 * @return   None
 */
extern void set_device_address( UINT32 address );

/*******************************************************************************
 * @fn     USB30_linkIRQHandler
 *
 * @brief  USB3.0 link中断处理
 *
 * @return   None
 */
extern void USB30_linkIRQHandler (void);

/*******************************************************************************
 * @fn     USB30_enableITP
 *
 * @brief  USB ITP使能
 *
 * @param  Status-  enable/disable
 *
 * @return   None
 */
extern void USB30_enableITP(FunctionalState Status);

/*******************************************************************************
 * @fn     USB30_usbssIRQHandler
 *
 * @brief  USB3.0中断处理
 *
 * @return   None
 */
extern void USB30_usbssIRQHandler();

/*******************************************************************************
 * @fn     USB30_StandardReq
 *
 * @brief  USB设备模式标准请求命令处理
 *
 * @return   主机请求设备发送的数据长度
 */
extern UINT16 USB30_StandardReq();

/*******************************************************************************
 * @fn     USB30_NonStandardReq
 *
 * @brief  USB设备模式非标准请求命令处理
 *
 * @return   主机请求设备发送的数据长度
 */
extern UINT16 USB30_NonStandardReq();

/*******************************************************************************
 * @fn     EP0_IN_Callback
 *
 * @brief  端点0 IN传输完成回调函数
 *
 * @return   一次IN传输响应发送的数据长度
 */
extern UINT16 EP0_IN_Callback();

/*******************************************************************************
 * @fn     EP0_OUT_Callback
 *
 * @brief  端点0 OUT传输完成回调函数
 *
 * @return   0
 */
extern UINT16 EP0_OUT_Callback();

/*******************************************************************************
 * @fn     USB30_SetupStatus
 *
 * @brief  控制传输状态阶段
 *
 * @return   None
 */
extern void USB30_SetupStatus();

/*******************************************************************************
 * @fn     ITP_Callback
 *
 * @brief  ITP回调函数
 *
 * @return   None
 */
extern void  ITP_Callback(UINT32 ITPCounter);

/*******************************************************************************
 * @fn     EPn_IN_Callback()
 *
 * @brief  端点n IN事务处理回调函数
 *
 * @return   None
 */
extern void  EP1_IN_Callback();
extern void  EP2_IN_Callback();
extern void  EP3_IN_Callback();
extern void  EP4_IN_Callback();
extern void  EP5_IN_Callback();
extern void  EP6_IN_Callback();
extern void  EP7_IN_Callback();

/*******************************************************************************
 * @fn     EPn_IN_Callback()
 *
 * @brief  端点n OUT事务处理回调函数
 *
 * @return   None
 */
extern void  EP1_OUT_Callback();
extern void  EP2_OUT_Callback();
extern void  EP3_OUT_Callback();
extern void  EP4_OUT_Callback();
extern void  EP5_OUT_Callback();
extern void  EP6_OUT_Callback();
extern void  EP7_OUT_Callback();

#endif /* USB30_CH56XUSB30_LIB_H_ */
