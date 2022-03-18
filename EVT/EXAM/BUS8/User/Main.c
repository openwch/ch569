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

#define  FREQ_SYS   30000000

/*******************************************************************************
 * @fn      DebugInit
 *
 * @brief   Initializes the UART3 peripheral.
 *
 * @param   baudrate - UART3 communication baud rate.
 *
 * @return  None
 */
void DebugInit(UINT32 baudrate)
{
	UINT32 x;
	UINT32 t = FREQ_SYS;
	
	x = 10 * t * 2 / 16 / baudrate;
	x = ( x + 5 ) / 10;
	R8_UART3_DIV = 1;
	R16_UART3_DL = x;
	R8_UART3_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
	R8_UART3_LCR = RB_LCR_WORD_SZ;
	R8_UART3_IER = RB_IER_TXD_EN;
	R32_PB_SMT |= (1<<4) |(1<<3);
	R32_PB_DIR |= (1<<4);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
#define R8_xbus_cmd    (*((PUINT8V)0x80000001))
#define R8_xbus_dat    (*((PUINT8V)0x80000000))

int main()
{  
    SystemInit(FREQ_SYS);
    Delay_Init(FREQ_SYS);

	DebugInit(115200);
	PRINT("Start @ChipID=%02X\r\n", R8_CHIP_ID );
#if 1
	UINT8 ver;

	BUS8_Init(ADDR_6, WIDTH_16, HOLD_3, SETUP_3);

	R8_xbus_cmd = 0x01;
	ver = R8_xbus_dat;
	PRINT("%x\n",ver);

#endif

	while(1);
}





