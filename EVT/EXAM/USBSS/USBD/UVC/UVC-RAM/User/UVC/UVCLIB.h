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

#include "CH56x_usb30.h"

/* Global define */
#define FORMAT_MJPEG    1
#define FORMAT_YUV2     2
#define SOURCECLOCK     ((~(SysTick->CNT))&0xffffffff)
#define PACKSIZE_3      (1024*3 - 12 + 1 )
#define PACKSIZE_2      (1024*2 - 12 + 1 )
#define PACKSIZE_1      (1024*1 - 12 + 1 )
#define BURSTMAXSIZE    12

/* Global Variable */
extern UINT8V Get_Curr[26];
extern UINT16V Resolution_width;
extern UINT16V Resolution_height;

/* Function declaration */
void FillYUVdata( void );
UINT16 UVC_NonStandardReq(UINT8 **pDescr);
void ClearError();
void CtrlCamera();
void DVP_Hander(void);
void Endp1_ITPHander(void);
void Endp1_Hander(void);
void Endp1_ISOHander_Hs(void);
void Endp1_Hander_Hs(void);
void CtrlCamera_Hs();
void Switch_Resolution( UINT8 frameindex );

#endif /* UVC_UVCLIB_H_ */
