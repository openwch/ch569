/********************************** (C) COPYRIGHT *******************************
* File Name          : MAIN.H
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
#include <stdio.h>
#include <string.h>													
#include "string.h"
#include <HAL.h>
#include "CH56x_common.h"
#include <HSPI.h>

/*******************************************************************************/
#define DEF_PROG_VERSION  	   	   0x01

/*******************************************************************************/
#define DEF_FUN_PACK_SN_EN         0x00

/******************************************************************************/
extern UINT32V Dbg_Idle_TimeCount;
extern UINT64V Dbg_HSPI_Tx_TLen;
extern UINT64V Dbg_HSPI_Rx_TLen;

/********************************************************************************/

#ifdef __cplusplus
}
#endif

#endif

/*********************************END OF FILE**********************************/
