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
 * This routine demonstrates that CH569 is the HSPI master sending data to the slave.
 * A flow control line needs to be connected:
 * CH569(HOST) GPIOB22 -> CH569(SLAVE) GPIOB23(Can be changed to other GPIO in HAL.h)
 * Overall process: CH569(A) -> HSPI -> CH569(B) -> USB upload
*/

#include "MAIN.h"
#include "CH56x_common.h"

/* Global define */
#define  UART1_BAUD     115200

/******************************************************************************/
UINT32V Dbg_Idle_TimeCount = 0x00;
UINT64V Dbg_HSPI_Tx_TLen = 0x00;
UINT64V Dbg_HSPI_Rx_TLen = 0x00;
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

	/* Configure serial port debugging */
	DebugInit(UART1_BAUD);
	PRINT("CH56x USB3.0 & USB2.0 device test(80MHz) !\n");
    HSPI_GPIO_Init( );                                                          /* GPIO initialization related to HSPI interface */
    GPIO_Init( );                                                               /* Hardware related GPIO initialization */
    DUG_PRINTF("CH56x HSPI Test@120MHZ_V1.%d\n",(UINT16)DEF_PROG_VERSION - 0x01 );
    DUG_PRINTF("Edit Date and Time is: "__DATE__"  " __TIME__"\n");

    /*****************************Timer initialization **************************/
    Timer1_Init( 240000 );                                                      /* 2mS */

    /*****************************HSPI*************************************/
    /* Configure PB24 as push-pull output, and the drive capacity is 16mA level*/
    R32_PB_DIR |= ( ( 1 << 24 ) );
    R32_PB_DRV |= ( ( 1 << 24 ) );
    R32_PB_PU &= ~( ( 1 << 24 ) );
    R32_PB_PD &= ~( ( 1 << 24 ) );

    Dbg_Idle_TimeCount = 0x00;
    Dbg_HSPI_Tx_TLen = 0x00;
    Dbg_HSPI_Rx_TLen = 0x00;

	while(1)
	{
	    BULKMode_HSPI_Test( );
	}
}


