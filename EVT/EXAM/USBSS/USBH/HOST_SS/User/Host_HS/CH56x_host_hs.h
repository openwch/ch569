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

//#include "core_riscv.h"
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


 void  CopySetupReqPkg( const UINT8 *pReqPkt );
 void  SetBusReset(void);
 void  USBHS_CurrentAddr( UINT8 addr );
 void  USBHS_Host_Init(FunctionalState sta);
 void  USBHS_Analysis_Descr(pDEV_INFO_Typedef pusbdev,PUINT8 pdesc, UINT16 l);

 UINT8 USBHS_Transact( UINT8 endp_pid, UINT8 toggle,UINT32 timeout);
 UINT8 USBHS_HostCtrlTransfer(UINT8 *databuf,PUINT16 len);
 UINT8 USBHS_Host_Enum(UINT8 *Databuf);
 UINT8 CtrlGetDevDescr( UINT8 *databuf );
 UINT8 CtrlGetConfigDescr( UINT8 *databuf );
 UINT8 CtrlSetAddress( UINT8 addr );
 UINT8 CtrlSetConfig( UINT8 cfg_val );

#ifdef __cplusplus
}
#endif

#endif /* HOST_HS_CH56X_HOST_HS_H_ */

