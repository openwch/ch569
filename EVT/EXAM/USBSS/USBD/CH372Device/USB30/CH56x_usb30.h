/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30.h
* Author             : WCH
* Version            : V1.1
* Date               : 2020/12/23
* Description        :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef USB30_CH56X_USB30_H_
#define USB30_CH56X_USB30_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "CH56x_common.h"


/* Global define */
#define DEF_ENDP1_OUT_BURST_LEVEL       4
#define DEF_ENDP1_IN_BURST_LEVEL        4
#define DEF_ENDP2_OUT_BURST_LEVEL       4
#define DEF_ENDP2_IN_BURST_LEVEL        4
#define DEF_ENDP3_OUT_BURST_LEVEL       4
#define DEF_ENDP3_IN_BURST_LEVEL        4
#define DEF_ENDP4_OUT_BURST_LEVEL       4
#define DEF_ENDP4_IN_BURST_LEVEL        4
#define DEF_ENDP5_OUT_BURST_LEVEL       4
#define DEF_ENDP5_IN_BURST_LEVEL        4
#define DEF_ENDP6_OUT_BURST_LEVEL       4
#define DEF_ENDP6_IN_BURST_LEVEL        4
#define DEF_ENDP7_OUT_BURST_LEVEL       4
#define DEF_ENDP7_IN_BURST_LEVEL        4

#define SIZE_DEVICE_DESC      		 18
#define SIZE_CONFIG_DESC       		 200
#define SIZE_STRING_LANGID     		 4
#define SIZE_STRING_VENDOR     	     8
#define SIZE_STRING_PRODUCT    	     38
#define SIZE_STRING_SERIAL      	 22
#define SIZE_BOS_DESC      			 22
#define SIZE_STRING_OS      		 18
#define SIZE_PropertyHeader			 0x8E
#define SIZE_CompactId				 0x28
#define SIZE_MSOS20DescriptorSet     72
#define SIZE_GetStatus				 2

/* Global Variable */
extern __attribute__ ((aligned(16))) UINT8  endp0RTbuff[512] __attribute__((section(".DMADATA")));    //数据发送/接收缓冲区
extern __attribute__ ((aligned(16))) UINT8  endp1RTbuff[4096] __attribute__((section(".DMADATA"))); //数据发送/接收缓冲区
extern __attribute__ ((aligned(16))) UINT8  endp2RTbuff[4096] __attribute__((section(".DMADATA"))); //数据发送/接收缓冲区
extern __attribute__ ((aligned(16))) UINT8  endp3RTbuff[4096] __attribute__((section(".DMADATA"))); //数据发送/接收缓冲区
extern __attribute__ ((aligned(16))) UINT8  endp4RTbuff[4096] __attribute__((section(".DMADATA"))); //数据发送/接收缓冲区
extern __attribute__ ((aligned(16))) UINT8  endp5RTbuff[4096] __attribute__((section(".DMADATA"))); //数据发送/接收缓冲区
extern __attribute__ ((aligned(16))) UINT8  endp6RTbuff[4096] __attribute__((section(".DMADATA"))); //数据发送/接收缓冲区
extern __attribute__ ((aligned(16))) UINT8  endp7RTbuff[4096] __attribute__((section(".DMADATA"))); //数据发送/接收缓冲区

/* Function declaration */
void USB30D_init(FunctionalState sta);
void TMR0_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));
void LINK_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));
void USBSS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));           //USB3.0 interrupt service

#ifdef __cplusplus
}
#endif

#endif /* USER_USB30_DESC_H_ */


