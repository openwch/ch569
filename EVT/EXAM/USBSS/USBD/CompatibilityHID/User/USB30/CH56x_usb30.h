/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30.h
* Author             : WCH
* Version            : V1.0
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef USB30_CH56X_USB30_H_
#define USB30_CH56X_USB30_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "CH56x_common.h"

/* Global define */
#define ENDP_INTERRUPT_BURST_LEVEL   3                    //The maximum number of burst package at the USB3.0 interrupt endpoint is 3


#define SIZE_DEVICE_DESC      		 18
#define SIZE_CONFIG_DESC       		 0x35
#define SIZE_STRING_LANGID     		 4
#define SIZE_STRING_VENDOR     	     8
#define SIZE_STRING_PRODUCT    	     38
#define SIZE_STRING_SERIAL      	 22
#define SIZE_BOS_DESC      			 22
#define SIZE_STRING_OS      		 18
#define SIZE_HIDREPORTDES            34
#define SIZE_PropertyHeader			 0x8E
#define SIZE_CompactId				 0x28
#define SIZE_MSOS20DescriptorSet     72
#define SIZE_GetStatus				 2

/* Global Variable */
extern __attribute__ ((aligned(16))) UINT8  endp0RTbuff[512] __attribute__((section(".DMADATA")));
extern __attribute__ ((aligned(16))) UINT8  HIDbuff[1024*ENDP_INTERRUPT_BURST_LEVEL] __attribute__((section(".DMADATA")));


/* Function declaration */
void USB30D_init(FunctionalState sta);
void TMR0_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));
void LINK_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));
void USBSS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

#ifdef __cplusplus
}
#endif

#endif /* USER_USB30_DESC_H_ */


