/********************************** (C) COPYRIGHT *******************************
* File Name          : usb30_desc.h
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef USB30_USB30_DESC_H_
#define USB30_USB30_DESC_H_
#include "CH56x_usb30.h"

/* Global define */
#define SIZE_DEVICE_DESC      		 18
#define SIZE_CONFIG_DESC       		 0x14b
#define SIZE_STRING_LANGID     		 4
#define SIZE_STRING_VENDOR     	     38
#define SIZE_STRING_PRODUCT    	     26
#define SIZE_STRING_SERIAL      	 22
#define SIZE_BOS_DESC      			 22
#define SIZE_STRING_OS      		 18
#define SIZE_PropertyHeader			 0x8E
#define SIZE_CompactId			     0x28
#define SIZE_MSOS20DescriptorSet     72
#define SIZE_GetStatus				 2

/* Global Variable */
extern const UINT8 DeviceDescriptor[SIZE_DEVICE_DESC];
extern const UINT8 ConfigDescriptor[SIZE_CONFIG_DESC];
extern const UINT8 OSStringDescriptor[SIZE_STRING_OS];
extern const UINT8 BOSDescriptor[SIZE_BOS_DESC];
extern const UINT8 StringLangID [SIZE_STRING_LANGID];
extern const UINT8 StringVendor [SIZE_STRING_VENDOR];
extern const UINT8 StringProduct[SIZE_STRING_PRODUCT];
extern UINT8 StringSerial [SIZE_STRING_SERIAL];
extern UINT8 GetStatus[SIZE_GetStatus];
extern const UINT8 PropertyHeader[SIZE_PropertyHeader];
extern const UINT8 CompactId[SIZE_CompactId];
extern const UINT8 MSOS20DescriptorSet[SIZE_MSOS20DescriptorSet];

#endif /* USB30_USB30_DESC_H_ */
