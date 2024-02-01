/********************************** (C) COPYRIGHT *******************************
* File Name          : HAL.C
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/******************************************************************************/
#include "MAIN.h"

/*******************************************************************************
* Function Name  : Delay_uS
* Description    : Microsecond delay function
* Input          : delay
* Output         : None
* Return         : None
*******************************************************************************/
void Delay_uS( UINT16 delay )
{
	UINT16 i, j;

	for( i = delay; i != 0; i -- ) 
	{
		for( j = 6; j != 0; j -- )
		{
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
		}
	}
}

/*******************************************************************************
* Function Name  : Delay_mS
* Description    : Millisecond delay function
* Input          : delay
* Output         : None
* Return         : None
*******************************************************************************/
void Delay_mS( UINT16 delay )
{
    UINT16 i, j;

    for( i = delay; i != 0; i -- )
    {
        for( j = 200; j != 0; j -- )
        {
            Delay_uS( 5 );
        }
    }
}

/*******************************************************************************
* Function Name  : UART1_Init
* Description    : Serial port 1 initialization
* Input          : baudrate
* Output         : None
* Return         : None
*******************************************************************************/
void UART1_Init( UINT32 baudrate )
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
    R32_PA_SMT |= ( 1 << 8 ) | ( 1 << 7 );
    R32_PA_DIR |= ( 1 << 8 );
}

/*******************************************************************************
* Function Name  : Timer1_Init
* Description    : Timer 1 initialization (2mS)
* Input          : time
* Output         : None
* Return         : None
*******************************************************************************/
void Timer1_Init( UINT32 time )
{
    R32_TMR1_CNT_END = time;
    R8_TMR1_CTRL_MOD = RB_TMR_ALL_CLEAR;
    R8_TMR1_CTRL_MOD = RB_TMR_COUNT_EN | RB_TMR_CAP_COUNT;
    R8_TMR1_INTER_EN |= RB_TMR_IE_CYC_END;

    PFIC_EnableIRQ( TMR1_IRQn );
}

/*******************************************************************************
* Function Name  : TMR1_IRQHandler
* Description    : Interruption function
* Input          : None
* Return         : None
*******************************************************************************/
void TMR1_IRQHandler( void ) __attribute__((interrupt("WCH-Interrupt-fast")));
void TMR1_IRQHandler( void )
{
    static UINT8V count = 0;
    if( R8_TMR1_INT_FLAG & RB_TMR_IE_CYC_END )
    {
        /* Clear interrupt flag */
        R8_TMR1_INT_FLAG = RB_TMR_IE_CYC_END;

        /* Endpoint 1 data download idle timing */
        Endp1_Down_IdleCount++;


        /* Print idle timing */
#if( DEF_FUN_DEBUG_EN == 0x01 )
        Dbg_Idle_TimeCount++;
        if( Dbg_Idle_TimeCount >= 15 * 1000 )
        {
            Dbg_Idle_TimeCount = 0;

            /* Print without HSPI sending or receiving */
            printf("~~~~~~~~~~~~~~~~~~~~~~1\n");
            printf("Dbg_USB_Down_TLen: %x%x\n",(UINT32)( Dbg_USB_Down_TLen >> 32 ), (UINT32)( Dbg_USB_Down_TLen ) );
            printf("Dbg_HSPI_Tx_TLen:  %x%x\n",(UINT32)( Dbg_HSPI_Tx_TLen >> 32 ), (UINT32)( Dbg_HSPI_Tx_TLen ) );
            printf("Dbg_HSPI_Rx_TLen:  %x%x\n",(UINT32)( Dbg_HSPI_Rx_TLen >> 32 ), (UINT32)( Dbg_HSPI_Rx_TLen ) );
            printf("Dbg_USB_Up_TLen:   %x%x\n",(UINT32)( Dbg_USB_Up_TLen >> 32 ), (UINT32)( Dbg_USB_Up_TLen ) );
            printf("HSPI_Tx_Data_Len:  %x\n",(UINT32)HSPI_Tx_Data_RemainLen);
            printf("HSPI_Rx_Data_Len:  %x\n",(UINT32)HSPI_Rx_Data_RemainLen);
            printf("Endp1_Up_Status:   %x\n",(UINT32)Endp1_Up_Status);

            printf("~~~~~~~~~~~~~~~~~~~~~~2\n");
            printf("Dbg_USB_Down_TLen: %d\n",(UINT32)Dbg_USB_Down_TLen);
            printf("Dbg_HSPI_Tx_TLen:  %d\n",(UINT32)Dbg_HSPI_Tx_TLen);
            printf("Dbg_HSPI_Rx_TLen:  %d\n",(UINT32)Dbg_HSPI_Rx_TLen);
            printf("Dbg_USB_Up_TLen:   %d\n",(UINT32)Dbg_USB_Up_TLen);
            printf("HSPI_Tx_Data_Len:  %d\n",(UINT64)HSPI_Tx_Data_RemainLen);
            printf("HSPI_Rx_Data_Len:  %d\n",(UINT64)HSPI_Rx_Data_RemainLen);
            printf("Endp1_Up_Status:   %d\n",(UINT64)Endp1_Up_Status);
        }
#endif
    }
}

/*******************************************************************************
* Function Name  : GPIO_Init
* Description    : Hardware related GPIO initialization
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Init( void )
{
    UINT32 temp1;

    /* configure ACT(PB24)¡¢HRTS(PB23) Push-pull output,16mA */
    PIN_HRTS_HIGH( );
    R32_PB_DIR |= (  1 << 23  );
    R32_PB_DRV |= (  1 << 23  );
    R32_PB_PU &= ~(  1 << 23  );
    R32_PB_PD &= ~(  1 << 23  );

    /* configure HCTS(PB22) Pull up input */
    R32_PB_DIR &= ~(  1 << 22  );
    R32_PB_PU |= (  1 << 22  );
    R32_PB_PD &= ~(  1 << 22  );

}
