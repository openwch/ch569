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
#include "CH56xusb30_lib.h"


/* Global define */
#define DEF_ENDP1_OUT_BURST_LEVEL       4
#define DEF_ENDP1_IN_BURST_LEVEL        3

#define UsbSetupBuf     ((PUSB_SETUP)endp0RTbuff)//endpoint0

#define SIZE_DEVICE_DESC      		 18
#define SIZE_CONFIG_DESC       		 0x2c
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

#define LINK_STA_1  (1<<0)
#define LINK_STA_3  (1<<2)
/* Global Variable */
extern UINT8V USB30_rec_flag;
extern UINT8V Link_Sta;

extern UINT32V Endp1_Up_LastPackNum;                                            /* The number of packets uploaded by endpoint 1 */
extern UINT32V Endp1_Up_LastPackLen;                                            /* The length of the last packet uploaded by endpoint 1 */
extern UINT32V Endp1_Up_Status;                                                 /* Endpoint 1 upload status: 0: free; 1: Uploading; */
extern UINT32V Endp1_Down_LastPackNum;                                          /* The number of packets last downloaded by endpoint 1 */
extern UINT32V Endp1_Down_LastPackLen;                                          /* The length of the last packet transmitted by endpoint 1 */
extern UINT32V Endp1_Down_Status;                                               /* Endpoint 1 download status: 0: download; 1: Stop downloading; */
extern UINT32V Endp1_Down_IdleCount;                                            /* Endpoint 1 downlink idle timing; */
extern UINT8V  USB_Down_StopFlag;                                               /* USB Download pause flag(USB Received a non-full packet,must wait until the HSPI is sent before continuing to download) */
extern UINT8V  HSPI_RX_StopFlag;                                                /* HSPI Receive pause flag(HSPI Received a non-full packet,You must wait for the USB upload to complete before continuing to receive) */

extern __attribute__ ((aligned(16))) UINT8  endp0RTbuff[512] __attribute__((section(".DMADATA")));
extern __attribute__ ((aligned(16))) UINT8  endp1RTbuff[4096] __attribute__((section(".DMADATA")));

/* Function declaration */
void USB30D_init( FunctionalState sta );
void TMR0_IRQHandler( void ) __attribute__((interrupt("WCH-Interrupt-fast")));
void LINK_IRQHandler( void ) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBSS_IRQHandler( void ) __attribute__((interrupt("WCH-Interrupt-fast")));

#ifdef __cplusplus
}
#endif

#endif /* USER_USB30_DESC_H_ */


