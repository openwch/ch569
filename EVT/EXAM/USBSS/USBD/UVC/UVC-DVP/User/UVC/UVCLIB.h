/********************************** (C) COPYRIGHT *******************************
* File Name          : UVCLIB.h
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef UVCLIB_H_
#define UVCLIB_H_

#include "dvp.h"
#include "CH56x_usb30.h"

/* Global define */
#define FORMAT_MJPEG    1
#define FORMAT_YUV2     2
#define SOURCECLOCK     ((~(SysTick->CNT))&0xffffffff)

#define UVC_HEADERSIZE  16
/* Global Variable */
extern UINT8V Get_Curr[26];
extern UINT8V Formatchange_flag;
extern UINT16V Resolution_width;
extern UINT16V Resolution_height;
extern UINT16V Dvp_DataSize;
extern PUINT8 Dvp_Recaddr;

extern const UINT16 ov2640_JPEGframe_resolution_USB20[5][2];
extern const UINT16 ov2640_JPEGframe_resolution_USB30[5][2];
extern const UINT16 ov2640_YUVframe_resolution_USB20[5][2];
extern const UINT16 ov2640_YUVframe_resolution_USB30[5][2];


/* Function declaration */
UINT16 UVC_NonStandardReq(UINT8 **pDescr);
void ClearError(void);
void SS_CtrlCamera(void);
void DVP_Hander(void);
void SS_Endp1_Hander(void);
void HS_CtrlCamera(void);
void HS_Endp1_ISOHander(void);
void HS_Endp1_Hander(void);



#endif /* UVC_UVCLIB_H_ */
