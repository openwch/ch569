/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_sys.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description        : This file contains all the functions prototypes for 
*                      SystemCoreClock, UART Printf , Delay functions .
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#include "CH56x_common.h"


static uint8_t  p_us=0;
static uint16_t p_ms=0;

/*******************************************************************************
 * @fn     Delay_Init
 *
 * @brief  Initializes Delay Funcation.
 *
 * @param  systemclck - 系统时钟 Hz
 *
 * @return   None
 **/
void Delay_Init(uint32_t systemclck)
{
	p_us=systemclck/8000000;
	p_ms=(uint16_t)p_us*1000;
}

/*******************************************************************************
 * @fn     mDelayuS
 *
 * @brief  Microsecond Delay Time.
 *
 * @param  n - Microsecond number.
 *
 * @return   None
 **/
void mDelayuS(uint32_t n)
{
	uint32_t i;

	SysTick->CTLR = 0;
	i = (uint32_t)n*p_us;

	SysTick->CMP = i;
	SysTick->CTLR = (1<<8)|(1<<0);

    while((SysTick->CNTFG & (1<<1)) != (1<<1));
    SysTick->CNTFG &= ~(1<<1);
}

/*******************************************************************************
 * @fn     mDelaymS
 *
 * @brief  Millisecond Delay Time.
 *
 * @param  n - Millisecond number.
 *
 * @return   None
 **/
void mDelaymS(uint32_t n)
{
	uint32_t i;

	SysTick->CTLR = 0;
	i = (uint32_t)n*p_ms;

	SysTick->CMP = i;
	SysTick->CTLR = (1<<8)|(1<<0);

    while((SysTick->CNTFG & (1<<1)) != (1<<1));
    SysTick->CNTFG &= ~(1<<1);
}

/*******************************************************************************
 * @fn     SYS_GetInfoSta
 *
 * @brief  获取当前系统信息状态
 *
 * @param  i -
 * @return  stat
 **/
UINT8 SYS_GetInfoSta( SYS_InfoStaTypeDef i )
{
	return (R8_RST_BOOT_STAT&(1<<i));
}

/*******************************************************************************
 * @fn     SYS_ResetExecute
 *
 * @brief  执行系统软件复位
 * 
 * @return   None
 **/
void SYS_ResetExecute( void )
{
	R8_SAFE_ACCESS_SIG = 0x57; // enable safe access mode
	R8_SAFE_ACCESS_SIG = 0xa8;
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET | 0x40;
    R8_SAFE_ACCESS_SIG = 0;
}

/*******************************************************************************
 * @fn     WWDG_ITCfg
 *
 * @brief  看门狗定时器溢出中断使能
 *
 * @param  s -
 *           DISABLE - 溢出不中断      
 *           ENABLE - 溢出中断
 *
 * @return   None
 **/
void  WWDG_ITCfg( UINT8 s )
{
	R8_SAFE_ACCESS_SIG = 0x57; // enable safe access mode
	R8_SAFE_ACCESS_SIG = 0xa8;
	if(s == DISABLE)		R8_RST_WDOG_CTRL=(R8_RST_WDOG_CTRL & (~RB_WDOG_INT_EN)) | 0x40;
	else 					R8_RST_WDOG_CTRL|=RB_WDOG_INT_EN | 0x40;
	R8_SAFE_ACCESS_SIG = 0;
}

/*******************************************************************************
 * @fn     WWDG_ResetCfg
 *
 * @brief  看门狗定时器复位功能
 *
 * @param  s -
 *           DISABLE - 溢出不复位      
 *           ENABLE - 溢出系统复位
 *
 * @return   None
 **/
void WWDG_ResetCfg( UINT8 s )
{
	R8_SAFE_ACCESS_SIG = 0x57; // enable safe access mode
	R8_SAFE_ACCESS_SIG = 0xa8;
	if(s == DISABLE)		R8_RST_WDOG_CTRL=(R8_RST_WDOG_CTRL & (~RB_WDOG_RST_EN)) | 0x40;
	else 					R8_RST_WDOG_CTRL|=RB_WDOG_RST_EN | 0x40;
	R8_SAFE_ACCESS_SIG = 0;
}

/*******************************************************************************
 * @fn     WWDG_ClearFlag
 * @brief  清除看门狗中断标志，重新加载计数值也可清除
 * @param  None
 * @return   None
 **/
void WWDG_ClearFlag( void )
{
	R8_SAFE_ACCESS_SIG = 0x57; // enable safe access mode
	R8_SAFE_ACCESS_SIG = 0xa8;
	R8_RST_WDOG_CTRL |= RB_WDOG_INT_FLAG | 0x40;
	R8_SAFE_ACCESS_SIG = 0;
}


#if( defined  DEBUG)
/*******************************************************************************
 * @fn     _write
 *
 * @brief  Support Printf Function
 *
 * @param  *buf: UART send Data.
 *           size - Data length
 *
 * @return   size - Data length
 **/
 __attribute__((used))
int _write(int fd, char *buf, int size)
{
	int i;
	
	for(i=0; i<size; i++){
#if  DEBUG == Debug_UART0
		while( R8_UART0_TFC == UART_FIFO_SIZE );
		R8_UART0_THR = *buf++;
#elif DEBUG == Debug_UART1 
		while( R8_UART1_TFC == UART_FIFO_SIZE );
		R8_UART1_THR = *buf++;
#elif DEBUG == Debug_UART2 
		while( R8_UART2_TFC == UART_FIFO_SIZE );
		R8_UART2_THR = *buf++;
#elif DEBUG == Debug_UART3  
		while( R8_UART3_TFC == UART_FIFO_SIZE );
		R8_UART3_THR = *buf++;		
#endif
	
	}
	
	return size;
}
#endif

