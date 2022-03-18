/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.1
* Date               : 2020/12/23
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56xusb30_lib.h"
#include "CH56x_usb30.h"
#include "CH56x_usb20.h"
/* Global define */
#define  FREQ_SYS       120000000
#define  UART1_BAUD     921600
/* Global Variable */

/* Function declaration */

/*******************************************************************************
 * @fn        DebugInit
 *
 * @brief     Initializes the UART1 peripheral.
 *
 * @param     baudrate: UART1 communication baud rate.
 *
 * @return    None
 */
void DebugInit(UINT32 baudrate)
{
	UINT32 x;
	UINT32 t = FREQ_SYS;
	x = 10 * t * 2 / 16 / baudrate;
	x = ( x + 5 ) / 10;
	R8_UART1_DIV = 1;
	R16_UART1_DL = x;
	R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
	R8_UART1_LCR = RB_LCR_WORD_SZ;
	R8_UART1_IER = RB_IER_TXD_EN;
	R32_PA_SMT |= (1<<8) |(1<<7);
	R32_PA_DIR |= (1<<8);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main()
{

    SystemInit(FREQ_SYS);
	Delay_Init(FREQ_SYS);

/* 配置串口调试 */
	DebugInit(UART1_BAUD);
	PRINT("CH56x USB3.0 & USB2.0 device test(120MHz) !\n");

//USB初始化
    R32_USB_CONTROL = 0;
    PFIC_EnableIRQ(USBSS_IRQn);
    PFIC_EnableIRQ(LINK_IRQn);

    PFIC_EnableIRQ(TMR0_IRQn);
    R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
    TMR0_TimerInit( 67000000 );   //约0.5秒

	USB30D_init(ENABLE);          //USB3.0初始化 初始化之前确保USB3.0两个中断使能

	while(1);
}

