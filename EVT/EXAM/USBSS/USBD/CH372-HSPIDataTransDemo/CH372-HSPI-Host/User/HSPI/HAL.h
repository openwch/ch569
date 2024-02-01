/********************************** (C) COPYRIGHT *******************************
* File Name          : HAL.h
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __HAL_H__
#define __HAL_H__

#include "CH56xSFR.h"

/******************************************************************************/
#define MY_DEBUG_PRINTF             1

#if( MY_DEBUG_PRINTF == 1 )
#define DUG_PRINTF( format, arg... )    printf( format, ##arg )
#else
#define DUG_PRINTF( format, arg... )    do{ if( 0 )printf( format, ##arg ); }while( 0 );
#endif

#define PIN_HCTS_RD( )              ( R32_PB_PIN & ( 1 << 22 ) )                /* HCTS Level reading */

#define PIN_HRTS_LOW( )             ( R32_PB_CLR |= ( 1 << 23 ) )               /* HRTS Output low */
#define PIN_HRTS_HIGH( )            ( R32_PB_OUT |= ( 1 << 23 ) )               /* HRTS Output high */

/******************************************************************************/
extern void Delay_uS( UINT16 delay );
extern void Delay_mS( UINT16 delay );
extern void UART1_Init( UINT32 baudrate );
extern void Timer1_Init( UINT32 time );
extern void GPIO_Init( void );

#endif

/*********************************END OF FILE**********************************/






