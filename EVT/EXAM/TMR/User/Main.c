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

#define  FREQ_SYS   80000000


__attribute__ ((aligned(4))) UINT32 CapBuf[100];
volatile UINT8 capFlag = 0;


#define count 1    //开启计数功能
//#define pwm   1    //开启脉冲调制波输出功能
//#define capture 1    //开启捕获功能

void TMR0_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TMR2_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));
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
	UINT8 i;
	
	SystemInit(FREQ_SYS);
	Delay_Init(FREQ_SYS);

/* 配置串口调试 */
	DebugInit(115200);
	PRINT("Start @ChipID=%02X\r\n", R8_CHIP_ID );




/************************************************************
 * TMR0 counting
 *****/
#if count
	GPIOA_SetBits(GPIO_Pin_9);
	GPIOA_ModeCfg( GPIO_Pin_9 , GPIO_Slowascent_PP_8mA );
	TMR0_TimerInit(40000000);

	TMR0_ITCfg( ENABLE ,RB_TMR_IE_CYC_END);
	PFIC_EnableIRQ( TMR0_IRQn );

	while(1);

#endif

/************************************************************
 * TMR1定时器PWM输出
 *****/
#if pwm

	 GPIOPinRemap(ENABLE,RB_PIN_TMR1);  //使能TMR1重映射功能
	 GPIOB_ResetBits(GPIO_Pin_0);
	 GPIOB_ModeCfg(GPIO_Pin_0,GPIO_Slowascent_PP_8mA);

	 TMR1_PWMInit( high_on_low , PWM_Times_1 );
	 TMR1_PWMCycleCfg(40000);
	 TMR1_PWMActDataWidth(20000);

	 while(1);

#endif

/************************************************************
 * TMR2定时器DMA捕获
 *****/
#if capture
	 GPIOA_ResetBits(GPIO_Pin_4);
	 GPIOA_ModeCfg( GPIO_Pin_4, GPIO_ModeIN_PU_NSMT );

	 TMR2_CapInit( Edge_To_Edge );
	 TMR2_CAPTimeoutCfg( 67108863 );
	 TMR2_DMACfg( ENABLE, (UINT16)(UINT32)(&CapBuf[0]), (UINT16)(UINT32)(&CapBuf[99]), Mode_Single );
	 TMR2_ITCfg( ENABLE , RB_TMR_IE_DMA_END);
	 PFIC_EnableIRQ(TMR2_IRQn);


	 while( capFlag == 0 );
	 capFlag = 0;
	 for( i=0; i<100; i++ ) {
		PRINT("%08ld ", CapBuf[i]&0x1ffffff);      // bit26 最高位表示 高电平还是低电平
	 }PRINT("\n");

	 while(1);

#endif


}


/*******************************************************************************
 * @fn      TMR0_IRQHandler
 *
 * @brief   Interruption function
 *
 * @return  None
 */
void TMR0_IRQHandler(void)
{
	 if( TMR0_GetITFlag( RB_TMR_IE_CYC_END ) )
	 {
		PRINT("counting done!\r\n");
	    TMR0_ClearITFlag( RB_TMR_IE_CYC_END );      // 清除中断标志
	    GPIOA_InverseBits( GPIO_Pin_9 );
	 }
}

/*********************************************************************
 * @fn       TMR2_IRQHandler
 *
 * @brief    Interruption function
 *
 * @return   None
 */
void TMR2_IRQHandler(void)
{
	PRINT("in Capture!\r\n");
	if( R8_TMR2_INT_FLAG & RB_TMR_IF_DMA_END  )
	{
		R8_TMR2_INTER_EN &= ~RB_TMR_IE_DMA_END;       // 使用单次DMA功能+中断，注意完成后关闭此中断使能，否则会一直上报中断。
		R8_TMR2_INT_FLAG &= ~RB_TMR_IF_DMA_END;      // 清除中断标志
		capFlag = 1;
		PRINT("*");
	}
}
