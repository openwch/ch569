/********************************** (C) COPYRIGHT *******************************
* File Name          : timer.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef _timer_h__
#define _timer_h__

#ifndef UINT8
#define UINT8 unsigned char
#endif

#ifndef UINT16
#define UINT16 unsigned short
#endif

#ifndef UINT32
#define UINT32 unsigned long
#endif

void TMR0_init(unsigned long n);
void tmr1_init(void);
unsigned int gettime(void);

#endif
