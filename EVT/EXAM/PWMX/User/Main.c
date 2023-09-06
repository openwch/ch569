/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : 
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 *PWMX routine
 * 
 */

#include "CH56x_common.h"

#define  FREQ_SYS   80000000

/*******************************************************************************
 * @fn        DebugInit
 *
 * @brief     Initializes the UART1 peripheral.
 *
 * @param     baudrate: UART1 communication baud rate.
 *
 * @return  None
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

    /*Configure serial debugging */
	DebugInit(115200);

	printf("Start PWM Test\n");
	//PRINT("Start @ChipID=%02X\r\n", R8_CHIP_ID );

	/*PWM1 pin configuration, PA4 */
	R32_PA_PD  &= ~(1<<4);
	R32_PA_DRV &= ~(1<<4);
	R32_PA_DIR |= 1<<4;

	/*PWM2 pin configuration, PB1*/
	R32_PB_PD  &= ~(1<<1);
	R32_PB_DRV &= ~(1<<1);
	R32_PB_DIR |= 1<<1;

	/*PWM3 pin configuration, PB2*/
	R32_PB_PD  &= ~(1<<2);
	R32_PB_DRV &= ~(1<<2);
	R32_PB_DIR |= 1<<2;

	/*PWM0 pin configuration, PB15 is multiplexed as an external reset pin, the reset function needs to be turned off to enable the PWM function*/
	R32_PB_PD  &= ~(1<<15);
	R32_PB_DRV &= ~(1<<15);
	R32_PB_DIR |= 1<<15;


    PWMX_CLKCfg( 4 );                    // cycle = 4/Fsys
    PWMX_CycleCfg( PWMX_Cycle_256 );     // cycle = 256*cycle

    PWMX_ACTOUT( CH_PWM1, 256/8, Low_Level, ENABLE);     //duty cycle
    PWMX_ACTOUT( CH_PWM2, 256/4, Low_Level, ENABLE);
    PWMX_ACTOUT( CH_PWM3, 256/8, Low_Level, ENABLE);
    PWMX_ACTOUT( CH_PWM0, 256/8, Low_Level, ENABLE);

    while(1);    
}





