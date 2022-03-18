/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb20.h
* Author             : WCH
* Version            : V1.1
* Date               : 2021/11/3
* Description        :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef USB20_CH56X_USB20_H_
#define USB20_CH56X_USB20_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Global define */
#define U20_MAXPACKET_LEN       512
#define U20_UEP0_MAXSIZE        64
// 00: OUT, 01:SOF, 10:IN, 11:SETUP
#define PID_OUT     0
#define PID_SOF     1
#define PID_IN      2

typedef struct __attribute__((packed))
{
   UINT8 dev_speed;
   UINT8 dev_addr;
   UINT8 dev_config_value;
   UINT8 dev_sleep_status;
   UINT8 dev_enum_status;
}DevInfo_Typedef;

//#define PID_SETUP	3
extern const UINT8 hs_device_descriptor[];
extern const UINT8 hs_config_descriptor[];
extern const UINT8 hs_string_descriptor0[];
extern const UINT8 hs_string_descriptor1[];
extern const UINT8 hs_string_descriptor2[];
extern const UINT8 hs_bos_descriptor[];

void   USB20_Device_Init ( FunctionalState sta );  // USBSS device initial
UINT16 U20_NonStandard_Request_Deal();
UINT16 U20_Standard_Request_Deal();
UINT16 U20_Endp0_IN_Callback(void);


#ifdef __cplusplus
}
#endif

#endif

