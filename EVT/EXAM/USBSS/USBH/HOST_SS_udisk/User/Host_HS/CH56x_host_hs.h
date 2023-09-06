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
    UINT16  InEndpMaxSize;          // IN  Maximum packet size of endpoint
    UINT8   InEndpNum;              // IN  endpoint number
    UINT8   InEndpCount;            // IN  endpoint count
    UINT8   InTog;                  // IN Synchronization flag
    UINT8   OutEndpNum;             // OUT endpoint number
    UINT8   OutTog;
    UINT8   OutEndpCount;           // IN  endpoint count
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
 void  USB20HOST_SetBusReset(void);
 void  USB20Host_Init(FunctionalState sta);
 UINT8 USB20HOST_CtrlTransfer(UINT8 *ReqBuf, UINT8 *DataBuf, UINT8 *RetLen );
 UINT8 USB20HOST_ClearEndpStall( UINT8 endp );
 UINT8 USB20HOST_Transact( UINT8 endp_pid, UINT8 toggle,UINT32 timeout);
 UINT8 USB20Host_Enum(UINT8 *Databuf);


#ifdef __cplusplus
}
#endif

#endif /* HOST_HS_CH56X_HOST_HS_H_ */

