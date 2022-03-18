/********************************** (C) COPYRIGHT *******************************
* File Name          : mem_malloc.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : CH565/569 千兆以太网收发器收发队列及 TCP/IP协议栈内存申请函数
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef MEM_MALLOC_H
#define MEM_MALLOC_H

#include "CH56xSFR.h"

extern ETH_DMADESCTypeDef  *DMATxDescToSet;                         /* 当前发送描述符指针 */
extern ETH_DMADESCTypeDef  *DMARxDescToGet;                         /* 当前接收描述符指针 */

#endif
