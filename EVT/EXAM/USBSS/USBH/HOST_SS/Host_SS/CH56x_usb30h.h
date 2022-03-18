/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30h.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2020/12/23
* Description        : USB3.0
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef HOST_CH56X_USB30H_H_
#define HOST_CH56X_USB30H_H_

#ifdef __cplusplus
extern "C" {
#endif

#define USBSS_ENDP0_MAXSIZE   512
#define ENDP_0                0
#define ENDP_1                1
#define ENDP_2                2
#define ENDP_3                3
#define ENDP_4                4
#define ENDP_5                5
#define ENDP_6                6
#define ENDP_7                7


#define USB_U30_SPEED         (0x02)
#define USB_U20_SPEED         (0x01)
#define USB_INT_CONNECT_U20   (0x13)            /* 检测到USB设备连接事件 */
#define USB_INT_SUCCESS       (0x14)             /* USB事务或者传输操作成功 */
#define USB_INT_CONNECT       (0x15)             /* 检测到USB设备连接事件 */
#define USB_INT_DISCONNECT    (0x16)            /* 检测到USB设备断开事件 */

typedef struct __PACKED  _HOST_STATUS
{
    UINT8 InSeqNum[8];
    UINT8 InMaxBurstSize[8];
    UINT8 OutSeqNum[8];
    UINT8 OutMaxBurstSize[8];
}DevInfo,PDevInfo;

extern UINT8V device_link_status;

extern const UINT8 get_descriptor[];
extern const UINT8 get_cfg_descriptor[];
extern const UINT8 get_cfg_descriptor_all[];
extern const UINT8 get_bos_descriptor[];
extern const UINT8 get_string_descriptor0[];
extern const UINT8 get_string_descriptor1[];
extern const UINT8 get_string_descriptor2[];
extern const UINT8 get_string_descriptor3[];
extern const UINT8 get_interface[];
extern const UINT8 set_configuration[];
extern const UINT8 set_address[];
extern const UINT8 set_isoch_delay[];
extern const UINT8 set_sel[];
extern const UINT8 tx_sel_data[];
extern const UINT8 set_feature_U1[];
extern const UINT8 set_feature_U2[];

extern __attribute__ ((aligned(16))) UINT8 endpRXbuff[4096] __attribute__((section(".DMADATA"))); //数据接收缓冲区
extern __attribute__ ((aligned(16))) UINT8 endpTXbuff[4096] __attribute__((section(".DMADATA"))); //数据发送缓冲区

extern void   USB30_Host_Enum(void);
extern void   USB30H_init (FunctionalState s);
extern void   USB30_link_status(UINT8 s);
extern UINT16 USBSS_INTransaction(UINT8 seq_num,UINT8 *recv_packnum ,UINT8 endp_num);
extern UINT8  USBSS_OUTTransaction(UINT8 seq_num,UINT8 send_packnum ,UINT8 endp_num,UINT32 txlen);

UINT16 USBSS_CtrlTransaciton(UINT8 *databuf); UINT16 GetDEV_Descriptor(void);
UINT16 GetConfig_Descriptor(void);
void Set_Address(UINT8 addr);
void Set_IsochDelay(void);
void Set_Sel(void);
void Set_Configuration(void);
void Analysis_Descr(UINT8 *pdesc, UINT16 l);

void LINK_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBSS_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));

#ifdef __cplusplus
}
#endif

#endif /* USER_USB30H_PROCESS_H_ */
