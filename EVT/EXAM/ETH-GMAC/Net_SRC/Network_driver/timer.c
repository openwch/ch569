/********************************** (C) COPYRIGHT *******************************
* File Name          : timer.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include <CH569Net_lib.h>                         /* 库头文件 */
#include "timer.h"
#include "core_riscv.h"
#include "string.h"
#include "stdio.h"


UINT16 globe_send_flag_cnt;

/* *****************************************************************************
 * @fn     TMR0_init
 *
 * @brief 定时器初始化
 *
 * @param  定时周期
 *
 * @return  无
 */
void TMR0_init(UINT32 n)
{
	R32_TMR0_CNT_END = n;                                     /* 设置终端计数 */
	R8_TMR0_CTRL_MOD = RB_TMR_MODE_IN;			              /* 使能计数模式 */
	R8_TMR0_INTER_EN  |= RB_TMR_IE_CYC_END;		              /* 使能计数结束中断 */
	R8_TMR0_CTRL_MOD=RB_TMR_COUNT_EN;	                      /* 使能定时器 */
	PFIC_SetPriority(TMR0_IRQn,0xa0);                         /* 设优先级 */
	PFIC_EnableIRQ(TMR0_IRQn);                                /* 开启定时器中断 */
}

/* *****************************************************************************
 * @fn    Net_TimerIsr
 *
 * @brief每进一次中断的定时器中断函数
 *
 * @param 定时周期
 *
 * @return  无
 **/
/* 在填入值为系统主频除以100的情况下，每10毫秒进一次中断 */
void Net_TimerIsr(void)
{
    CH569NET_TimeIsr(CH569NETTIMEPERIOD);                                  /* 定时器中断服务函数 */

}
//static UINT32 LINSHI;
/* *****************************************************************************
 * @fn    TMR0_IRQHandler
 *
 * @brief 定时器中断函数
 *
 * @return  无
 */
void TMR0_IRQHandler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));
void TMR0_IRQHandler(void)
{
	if(R8_TMR0_INT_FLAG & RB_TMR_IF_CYC_END)
	{
		R8_TMR0_INTER_EN |= RB_TMR_IF_CYC_END;
		R8_TMR0_INT_FLAG =  RB_TMR_IF_CYC_END;
		Net_TimerIsr();
		globe_send_flag_cnt++;
	}
}


//为了测试时间开定时器
void tmr1_init(void)
{
	R32_TMR1_CNT_END=0x3FFFFFF;
	R8_TMR1_CTRL_MOD = RB_TMR_MODE_IN;
	R8_TMR1_CTRL_MOD = RB_TMR_COUNT_EN;
}

unsigned int gettime(void)
{
	unsigned int a=0;
	a=R32_TMR1_COUNT;
	return a;
}





