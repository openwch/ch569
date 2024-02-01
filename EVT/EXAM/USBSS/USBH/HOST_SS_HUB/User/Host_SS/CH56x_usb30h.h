/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30h.h
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
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

#define ERR_SUCCESS1            0x14
#define ERR_USB_LENGTH          0x18
#define ERR_USB_DATAERR         0x19

#define USB_OPERATE_SUCCESS                 0x00            /* USB Operation succeeded */
#define USB_OPERATE_ERROR                   0x01            /* USB operation failed */
#define USB_PARAMETER_ERROR                 0x03            /* Parameter error, range overflow */
#define USB_INT_CONNECT_U20                 0x13            /* USB device connection event detected */
#define USB_INT_SUCCESS                     0x14            /* USB transaction or transfer operation succeeded */
#define USB_INT_CONNECT                     0x15            /* USB device connection event detected */
#define USB_INT_DISCONNECT                  0x16            /* USB device disconnect event detected */
#define USB_INT_BUF_OVER                    0x17            /* Too much data transferred by USB control, buffer overflow */
#define USB_INT_DISK_ERR                    0x1F            /* USB memory operation failed */
#define USB_INT_DISK_ERR1                   0x20            /* USB memory operation failed */
#define USB_CH56XUSBTIMEOUT                 0x28            /* USB device timeout */

#define USB30_OUT_DISCONNECT        0xFA
#define USB30_IN_DISCONNECT         0xAAFF

#define U30_PID_OUT                     0x00            /* OUT */
#define U30_PID_IN                      0x01            /* IN */

typedef struct __PACKED  _HOST_STATUS
{
    UINT8 InSeqNum[8];
    UINT8 InMaxBurstSize[8];
    UINT8 OutSeqNum[8];
    UINT8 OutMaxBurstSize[8];
}DevInfo,PDevInfo;

typedef struct _USB_ENDPOINT_DESCRIPTOR_U30 /*Endpoint descriptor*/
{
    UINT8  bLength;
    UINT8  bDescriptorType;
    UINT8  bEndpointAddress;
    UINT8  bmAttributes;
    UINT8  wMaxPacketSizeL;
    UINT8  wMaxPacketSizeH;
    UINT8  bInterval;

    UINT8  bLength1;                //3.0 EndpointCompanion descriptor
    UINT8  bDescriptorType1;
    UINT8  bMaxBurst1;
    UINT8  bmAttributes1;
    UINT8  wBytesPerInterval_L;
    UINT8  wBytesPerInterval_H;
}USB_ENDP_DESCR_U30, *PUSB_ENDP_DESCR_U30;

typedef struct _USB_CONFIG_DESCRIPTOR_LONG_U30
{
    USB_CFG_DESCR  cfg_descr;
    USB_ITF_DESCR  itf_descr;
    USB_ENDP_DESCR_U30 endp_descr[2];
}USB_CFG_DESCR_LONG_U30, *PUSB_CFG_DESCR_LONG_U30;

extern UINT8V device_link_status;
extern UINT8V gDeviceConnectstatus;                                                    /* USB connection status */
extern UINT8  gDeviceUsbType;                                                      /* 01--USB2.0&1.1  02--USB3.0*/
extern UINT8  gDeviceClassType;

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

extern __attribute__ ((aligned(16))) UINT8 endpRXbuff[512] __attribute__((section(".DMADATA")));
extern __attribute__ ((aligned(16))) UINT8 endpTXbuff[512] __attribute__((section(".DMADATA")));

extern __attribute__ ((aligned(16))) UINT8 Test_Buf[3072] __attribute__((section(".DMADATA")));



UINT8 USB30HOST_ClearEndpStall( UINT8 endp );
void USB30Host_Enum(UINT8 depth,UINT8 addr,  UINT8 port );
void USB30_link_status(UINT8 s);
UINT16 USB30HOST_INTransaction(UINT8 seq_num,UINT8 *recv_packnum ,UINT8 endp_num,UINT16 *status);
UINT8  USB30HOST_OUTTransaction(UINT8 seq_num,UINT8 send_packnum ,UINT8 endp_num,UINT32 txlen);
UINT16 USB30HOST_CtrlTransaciton(UINT8 *databuf);
UINT8 U30HOST_GetHubDescr( UINT8 *buf ,UINT16 *len );
UINT8 U30HOST_GetDeviceStstus( void );
UINT8 U30HOST_GetPortStstus( UINT8 depth,UINT8 port );
UINT8 U30HOST_SetHubDepth( UINT8 depth );
UINT8 U30HOST_SetPortFeatrue( UINT8 port );
UINT8 USB30HSOT_Enumerate_Hotrst( UINT8 *pbuf );
UINT8  USBSS_HUB_Main_Process( UINT8 depth ,UINT8 addr,UINT8 uplevelport,UINT32 routestring,UINT8 portnum);
UINT8 USBSS_HUBCheckPortConnect( UINT8 depth,UINT8 port );

void LINK_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBSS_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));

#ifdef __cplusplus
}
#endif

#endif /* USER_USB30H_PROCESS_H_ */
