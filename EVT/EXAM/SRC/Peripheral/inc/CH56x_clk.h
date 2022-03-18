/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_clk.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/


#ifndef __CH56x_CLK_H__
#define __CH56x_CLK_H__

#ifdef __cplusplus
 extern "C" {
#endif


enum
{
	CLK_SOURCE_PLL_15MHz = 15,      //上电默认情况	
	CLK_SOURCE_PLL_30MHz = 30,
	CLK_SOURCE_PLL_60MHz = 60,
	CLK_SOURCE_PLL_80MHz = 80,
	CLK_SOURCE_PLL_96MHz = 96,
	CLK_SOURCE_PLL_120MHz = 120,
	
};


void SystemInit(uint32_t systemclck);			/* 系统时钟初始化 */
UINT32 GetSysClock( void );						/* 获取当前系统时钟 */

	
	 
#ifdef __cplusplus
}
#endif

#endif  // __CH56x_CLK_H__	

