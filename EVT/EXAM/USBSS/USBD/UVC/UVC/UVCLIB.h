/********************************** (C) COPYRIGHT *******************************
* File Name          : UVCLIB.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
*******************************************************************************/


#ifndef UVCLIB_H_
#define UVCLIB_H_

#include "usb30_porp.h"
#include "dvp.h"

extern UINT16 Res_NonStandardReq(UINT8 **pDescr);

extern void clearError();
extern void ctrlCamera();
extern void DVP_Hander(void);
extern void endp1_Hander(void);

#endif /* UVC_UVCLIB_H_ */
