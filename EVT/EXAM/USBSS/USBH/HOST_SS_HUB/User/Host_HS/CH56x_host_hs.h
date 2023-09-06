/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_host_hs.h
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef HOST_HS_CH56X_HOST_HS_H_
#define HOST_HS_CH56X_HOST_HS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hub.h"

#define U20_MAXPACKET_LEN       512
#define WAIT_TIME               10000000

#ifndef  ENDPOINT_NUM
#define  ENDP_0                 0
#define  ENDP_1                 1
#define  ENDP_2                 2
#define  ENDP_3                 3
#define  ENDP_4                 4
#define  ENDP_5                 5
#define  ENDP_6                 6
#define  ENDP_7                 7
#endif
/*******************TRANSFER STATUS***********************/
#ifndef ERR_SUCCESS
#define ERR_SUCCESS           (0x00)
#define ERR_USB_CONNECT       (0x15)
#define ERR_USB_DISCON        (0x16)
#define ERR_USB_BUF_OVER      (0x17)
#define ERR_USB_DISK_ERR      (0x1F)
#define ERR_USB_TRANSFER      (0x20)
#define ERR_USB_UNSUPPORT     (0xFB)
#define ERR_USB_UNKNOWN       (0xFE)
#define ERR_AOA_PROTOCOL      (0x41)
#define ERR_TIME_OUT          (0xFF)
#endif

typedef struct
{
    UINT16  OutEndpMaxSize;
    UINT16  InEndpMaxSize;
    UINT8   InEndpNum;
    UINT8   InEndpCount;
    UINT8   InTog;
    UINT8   OutEndpNum;
    UINT8   OutTog;
    UINT8   OutEndpCount;
}DEVENDP;

typedef struct  __attribute__((packed))
{
     DEVENDP DevEndp;
     UINT8   DeviceStatus;
     UINT8   DeviceAddress;
     UINT8   DeviceSpeed;
     UINT8   DeviceType;
     UINT8   DeviceEndp0Size;
     UINT8   DeviceCongValue;

 }DEV_INFO_Typedef,*pDEV_INFO_Typedef;


extern UINT8 Global_Index;
extern DEV_INFO_Typedef  thisUsbDev;

void  CopySetupReqPkg( const UINT8 *pReqPkt );
void  USB20HOST_SetBusReset(void);
void  USB20Host_Init(FunctionalState sta);
UINT8 USB20HOST_CtrlTransfer(UINT8 *ReqBuf, UINT8 *DataBuf, UINT8 *RetLen );
UINT8 USB20HOST_ClearEndpStall( UINT8 endp );
UINT8 USB20HOST_Transact( UINT8 endp_pid, UINT8 toggle,UINT32 timeout);
UINT8 USB20Host_Enum(UINT8 depth,UINT8 *Databuf);
UINT8 U20HOST_Enumerate( UINT8 depth,UINT8 *pbuf,UINT8 addr, UINT8 port);
void USBHS_Analysis_Descr(pDEV_INFO_Typedef pusbdev,PUINT8 pdesc, UINT16 l);

extern void HubAnalysis_Descr(PHUB_Port_Info portn,PUINT8 pdesc, UINT16 l);
extern UINT8  USBHS_HUB_Main_Process( UINT8 depth ,UINT8 addr ,UINT8 uplevelport,UINT8 portnum);


#ifdef __cplusplus
}
#endif

#endif /* HOST_HS_CH56X_HOST_HS_H_ */

