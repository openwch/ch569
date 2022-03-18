/********************************** (C) COPYRIGHT *******************************
* File Name          : PHY_interrupt.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : CH565/569 千兆以太网物理层中断演示代码
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "ethernet_driver.h"
#include "ethernet_config.h"
#if USE_ETH_PHY_INTERRUPT /* 如果定义了使用物理层中断 */
#include "PHY_interrupt.h"

globe_eth_status_t globe_eth_status; /* 全局以太网状态 */

/*******************************************************************************
 * @fn       phy_int_cheak_init
 *
 * @brief    Initializes the GPIO interrupt(PB15).
 *
 * @return   None
 **/
void phy_int_cheak_init(void)
{
	UINT16 RegValue;

	/* 使能PB15低电平中断 */
	GPIOB_ITModeCfg( 15, GPIO_ITMode_LowLevel );          /* PB15设为GPIO中断触发引脚，低电平触发中断  */
	R32_PB_PU = (1<<15);                                  /* PB15上拉 */
	R8_GPIO_INT_FLAG = 0xff;                              /* 清中断标志位 */
	PFIC_EnableIRQ(GPIO_IRQn);                            /* 使能GPIO中断 */

	/* 使能PHY各中断 */
#ifdef USE_RTL8211FS
    ETH_WritePHYRegister(PHY_ADDRESS, 31,0x0a42 );        /* 切换页 */
    RegValue = ETH_ReadPHYRegister(PHY_ADDRESS, 0x12);    /* 读取物理层INER寄存器 */
    ETH_WritePHYRegister(PHY_ADDRESS, 0x12,0xffff );      /* 写INER寄存器 */
    /* 读取初始中断状态并清零 */
    ETH_WritePHYRegister(PHY_ADDRESS, 31,0x0a43 );        /* 切换页 */
    ETH_ReadPHYRegister(PHY_ADDRESS, 0x1d);
    /* 读清物理层INSR寄存器 */
    RegValue = ETH_ReadPHYRegister(PHY_ADDRESS, 0x1a);
    /* 写初始的连接状态 */
	if(RegValue & 0x0008)
	{
		globe_eth_status.phy_link_duples=full_duples;
		printf("phy ini init:full_duples.\n");
	}
	else
	{
		globe_eth_status.phy_link_duples=half_duples;
		printf("phy ini init:half_duples.\n");
	}
	if(RegValue & 0x0004)
	{
		globe_eth_status.phy_link_status=link_ok;
		printf("phy ini init:link_ok.\n");
	}
	else
	{
		globe_eth_status.phy_link_status=link_not_ok;
		printf("phy ini init:link_not_ok.\n");
	}
	if( RegValue & 0x0030 == 0x0000 )
	{
		globe_eth_status.phy_link_speed=linked_10M;
		printf("phy ini init:initial link speed:10M.\n");
	}
	else if(RegValue & 0x0030 == 0x0010)
	{
		globe_eth_status.phy_link_speed=linked_100M;
		printf("phy ini init:initial link speed:100M.\n");
	}
	else if(RegValue & 0x0030 == 0x0020)
	{
		globe_eth_status.phy_link_speed=linked_1000M;
		printf("phy ini init:initial link speed:1000M.\n");
	}
#endif
}

/*******************************************************************************
 * @fn       cheak_phy_int
 *
 * @brief    cheak phy interrupt.
 *
 * @return   None
 */
void cheak_phy_int(void)
{
	UINT16 RegValue;

#ifdef USE_RTL8211FS
    ETH_WritePHYRegister(PHY_ADDRESS, 31,0x0a43 );    /* 切换页 */
    RegValue = ETH_ReadPHYRegister(PHY_ADDRESS, 0x1d);/* 读取物理层INSR寄存器 */
    if(RegValue & 0x0001)                               /* 自动协商出错 */
    {
    	printf("phy int:auto-negotiation error!\n");
    }
    if(RegValue & 0x0008)                               /* 自动协商完成 */
    {
//    	printf("phy int:auto-negotiation completed!\n");
    }
    if(RegValue & 0x0080)                               /* 物理层检测到PME/WOL事件 */
    {
    	printf("phy int:PME/WOL event.\n");
    }
    if(RegValue & 0x0010)                               /* 物理层连接状态改变 */
    {
    	globe_eth_status_t tem_eth_status;              /* 实时连接状态 */

        RegValue = ETH_ReadPHYRegister(PHY_ADDRESS, 0x1a);
    	printf("phy int:link status changed!physr:0x%04x.\n",RegValue);
        /* 获取最新的连接状态 */
		if(RegValue & 0x0004)
		{
			tem_eth_status.phy_link_status=link_ok;
			globe_eth_status.phy_link_status=link_ok;
			printf("phy int:eth linked!\n");
		}
		else
		{
			tem_eth_status.phy_link_status=link_not_ok;
			globe_eth_status.phy_link_status=link_not_ok;
			printf("phy int:eth disconnect!\n");
		}
		if(tem_eth_status.phy_link_status == link_ok)
		{
			if(RegValue & 0x0008)
			{
				tem_eth_status.phy_link_duples=full_duples;
				if( tem_eth_status.phy_link_duples != globe_eth_status.phy_link_duples )
				{
					printf("link duples changed!.full_duples");
					globe_eth_status.phy_link_duples = full_duples;
				}
			}
			else
			{
				tem_eth_status.phy_link_duples=half_duples;
				if( tem_eth_status.phy_link_duples != globe_eth_status.phy_link_duples )
				{
					printf("phy int:link status has changed to half_duples!\n   Error：CH565/569 does not suppose half_duples!\n");
					globe_eth_status.phy_link_duples = half_duples;
				}
			}
			if( (RegValue & 0x0030) == 0x0000 )
			{
				tem_eth_status.phy_link_speed=linked_10M;
			}
			if( (RegValue & 0x0030) == 0x0010)
			{
				tem_eth_status.phy_link_speed=linked_100M;
			}
			if( (RegValue & 0x0030) == 0x0020)
			{
				tem_eth_status.phy_link_speed=linked_1000M;
			}
			/* 和全局状态作比较 */
			if(globe_eth_status.phy_link_speed !=  tem_eth_status.phy_link_speed)
			{
				globe_eth_status.phy_link_speed = tem_eth_status.phy_link_speed;
				printf("phy int:link speed changed!\nrealtime link speed: %d Mbps.\n",tem_eth_status.phy_link_speed);
//				globe_eth_status.ETH_reinit_flag|=8;/* 需要重新初始化 */
				while(ETH_MACDMA_reConfig()!=1)
				{
					printf("Ethernet restart failed!retrying..\n");
				}
				printf("ETH_MACDMA_reConfig finished!\n");
			}
		}
    	printf("\n");
    }
#endif
}

/*******************************************************************************
 * @fn      GPIO_IRQHandler
 *
 * @brief   This function handles GPIO exception.
 *
 * @return  None
 */
void GPIO_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void GPIO_IRQHandler(void)
{
//	printf("GPIO_IRQHandler.\n");
	if(GPIOB_15_ReadITFlagBit())
	{
		cheak_phy_int();
	}
	GPIOB_15_ClearITFlagBit();
}

#endif
