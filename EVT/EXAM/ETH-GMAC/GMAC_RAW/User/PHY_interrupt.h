/********************************** (C) COPYRIGHT *******************************
* File Name          : PHY_interrupt.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : CH565/569 千兆以太网物理层中断演示代码
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef __PHY_interrupt_h__
#define __PHY_interrupt_h__


typedef enum
{
	link_not_ok,
	link_ok,
}phy_link_status_t;

typedef enum
{
	linked_10M=10,
	linked_100M=100,
	linked_1000M=1000,
}phy_link_speed_t;

typedef enum
{
	half_duples,
	full_duples,
}phy_link_duples_t;

typedef struct
{
	phy_link_status_t phy_link_status;
	phy_link_speed_t  phy_link_speed;
	phy_link_duples_t phy_link_duples;
//	UINT8 ETH_reinit_flag;
}globe_eth_status_t;

void phy_int_cheak_init(void);


#endif
