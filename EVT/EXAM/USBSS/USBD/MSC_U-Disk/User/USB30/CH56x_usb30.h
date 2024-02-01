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
#define DEF_ENDP2_IN_BURST_LEVEL        1
#define DEF_ENDP3_OUT_BURST_LEVEL       1

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

#define UDISKSIZE 1024*4*10
/* Global Variable */
extern __attribute__ ((aligned(16))) UINT8  endp0RTbuff[512] __attribute__((section(".DMADATA")));
extern __attribute__ ((aligned(16))) UINT8  UDisk_In_Buf[UDISKSIZE] __attribute__((section(".DMADATA")));
extern __attribute__ ((aligned(16))) UINT8  UDisk_Out_Buf[UDISKSIZE] __attribute__((section(".DMADATA")));
extern UINT8V Link_Sta;

typedef struct
{
    volatile uint32_t OUT_LoadPtr;//Current buffer location
    volatile uint32_t OUT_TotalPack;
    volatile uint32_t OUT_RemainPack;//The remaining number of unprocessed downstream packets
    volatile uint32_t OUT_RemainPackPtr;//Buffer location for remaining unprocessed download packets
    volatile uint32_t OUT_LastPack;
    volatile uint32_t IN_LoadPtr;//Current buffer location
    volatile uint32_t IN_RemainPack;//The remaining number of unprocessed downstream packets
    volatile uint32_t IN_RemainPackPtr;//Buffer location for remaining unprocessed upload packets
    volatile uint32_t IN_TotalPack;
    volatile uint32_t IN_UploadFlag;
    volatile uint32_t IN_LastPack;
}USB_DEAL;
extern USB_DEAL USBDeal;

/* Function declaration */
void USB30D_init(FunctionalState sta);
void TMR0_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));
void LINK_IRQHandler() __attribute__((interrupt("WCH-Interrupt-fast")));
void USBSS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

#ifdef __cplusplus
}
#endif

#endif /* USER_USB30_DESC_H_ */


