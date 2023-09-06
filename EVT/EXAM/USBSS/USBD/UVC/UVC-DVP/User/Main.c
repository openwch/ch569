/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 * @Note
 * This routine demonstrates that CH565 is simulated as a UVC device.
 * OV2640 obtains data through DVP interface and supports MJPEG and YUV2 video formats.
*/

#include "CH56x_common.h"
#include "ov.h"
#include "dvp.h"
#include "CH56x_usb30.h"
#include "UVCLIB.h"

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

	/* Configure uart debugging */
	DebugInit(115200);
	PRINT("UVC Program(80MHz) !\n");

	/*Initialize camera*/
	while(OV2640_Init())//Initialize OV2640
	{
		PRINT("Camera model Err\r\n");
		mDelaymS(500);
	}

	dvp_Init();//Initialize DVP
	mDelaymS(200);

	R32_USB_CONTROL = 0;
    PFIC_EnableIRQ(USBSS_IRQn);
    PFIC_EnableIRQ(LINK_IRQn);
    PFIC_EnableIRQ( TMR0_IRQn );

    TMR0_TimerInit( 67000000 );
    USB30D_init(ENABLE);

	while(1);
}


