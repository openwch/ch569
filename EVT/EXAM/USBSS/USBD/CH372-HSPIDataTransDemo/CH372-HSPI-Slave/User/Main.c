/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2022/11/18
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
*******************************************************************************/
#include "MAIN.h"

/*
 * @Note
 * This routine demonstrates that CH569 is the HSPI master sending data to the slave.
 * A flow control line needs to be connected:
 * CH569(HOST) GPIOB22 -> CH569(SLAVE) GPIOB23(Can be changed to other GPIO in HAL.h)
 * Overall process: CH569(A) -> HSPI -> CH569(B) -> USB upload
*/

/* Global define */
#define  UART1_BAUD     115200
/*******************************************************************************
 * @fn        DebugInit
 *
 * @brief     Initializes the UART1 peripheral.
 *
 * @return    None
 */
void DebugInit(UINT32 baudrate)//uart1
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
	DebugInit(UART1_BAUD);

    R32_USB_CONTROL = 0;
    PFIC_EnableIRQ(USBSS_IRQn);
    PFIC_EnableIRQ(LINK_IRQn);

    PFIC_EnableIRQ(TMR0_IRQn);
    R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
    TMR0_TimerInit( 67000000 );
	USB30D_init(ENABLE);

	HSPI_GPIO_Init( );                                                          /* GPIO initialization related to HSPI interface */
    GPIO_Init( );                                                               /* Hardware related GPIO initialization */


    /*****************************Timer initialization **************************/
    Timer1_Init( 160000 );                                                      /* 2ms */

#if( DEF_FUN_DEBUG_EN == 0x01 )
    DUG_PRINTF("Main\n");
#endif
    /*****************************HSPI test*************************************/
    /* Configure PB24 as push-pull output, and the drive capacity is 16mA level*/
    R32_PB_DIR |= ( ( 1 << 24 ) );
    R32_PB_DRV |= ( ( 1 << 24 ) );
    R32_PB_PU &= ~( ( 1 << 24 ) );
    R32_PB_PD &= ~( ( 1 << 24 ) );

    HSPI_DataTrans( );

	while(1);
}

