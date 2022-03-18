/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#include "CH56x_common.h"
#include "usb30_porp.H"
#include "ov.h"
#include "dvp.h"

#define	UART1_BAUD	921600

void USBSS_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));
void LINK_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*******************************************************************************
 * @fn       DebugInit
 *
 * @brief    Initializes the UART1 peripheral.
 *
 * @param    baudrate: UART1 communication baud rate.
 *
 * @return   None
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
	PRINT("UVC Program(80MHz) !\n");

	/*初始化摄像头*/
	while(OV2640_Init())			//初始化OV2640
	{
		PRINT("摄像头型号 Err\r\n");
		mDelaymS(500);
	}

	JPEG_Mode_Init();
	mDelaymS(200);

	dvp_Init(); //DVP

	R32_USB_CONTROL = 0;
	USB30_init();

	PFIC_EnableIRQ(USBSS_IRQn);
	PFIC_EnableIRQ(LINK_IRQn);

	while(1);
}

/*******************************************************************************
 * @fn      LINK_IRQHandler
 *
 * @brief   USB3.0 Link Interrupt Handler.
 *
 * @return  None
 */
void LINK_IRQHandler(void) //USBSS link interrupt service
{
	USB30_linkIRQHandler();
}

/*******************************************************************************
 * @fn      USBSS_IRQHandler
 *
 * @brief   USB3.0 Interrupt Handler.
 *
 * @return  None

 */
void USBSS_IRQHandler(void) //USBSS interrupt service
{
	USB30_usbssIRQHandler();
}
