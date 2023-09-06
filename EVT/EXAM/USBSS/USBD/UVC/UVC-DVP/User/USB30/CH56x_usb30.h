/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30.h
* Author             : WCH
* Version            : V1.1
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
#include "CH56x_usb30_LIB.h"
/* Global define */
#define SIZE_DEVICE_DESC      		    18
#define SIZE_STRING_LANGID     		    4
#define SIZE_BOS_DESC      			    22
#define SIZE_STRING_OS      		    18
#define SIZE_PropertyHeader			    0x8E
#define SIZE_CompactId				    0x28
#define SIZE_MSOS20DescriptorSet        72
#define UsbSetupBuf     ((PUSB_SETUP)endp0RTbuff)

/* Global Variable */
extern __attribute__ ((aligned(16))) UINT8 endp0RTbuff[512] __attribute__((section(".DMADATA")));
extern __attribute__ ((aligned(16))) UINT8 endp1RTbuff[1024] __attribute__((section(".DMADATA")));

extern UINT8V Link_sta;
/* Function declaration */
void USB30D_init(FunctionalState sta);
void TMR0_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));
void LINK_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));
void USBSS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


#ifdef __cplusplus
}
#endif

#endif /* USER_USB30_DESC_H_ */


