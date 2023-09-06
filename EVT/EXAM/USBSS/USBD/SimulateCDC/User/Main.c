/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 * @Note
 *  Example routine to emulate a simulate USB-CDC Device,Use UART2(PA2/PA3).
*/

#include "CH56x_common.h"
#include "CH56x_usb30.h"
#include "cdc.h"

/* Global define */
#define UART1_BAUD  115200

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

/*******************************************************************************
 * @fn        main
 *
 * @brief     main program
 *
 * @return    None
 */
int main()
{
    SystemInit(FREQ_SYS);
    Delay_Init(FREQ_SYS);

    /* Configure serial port debugging */
    DebugInit(UART1_BAUD);
    PRINT("CH56x USB3.0 & USB2.0 device test(80MHz) !\n");

    /* USB initialization */
    TMR2_TimerInit1();
    R32_USB_CONTROL = 0;
    PFIC_EnableIRQ(USBSS_IRQn);
    PFIC_EnableIRQ(LINK_IRQn);
    PFIC_EnableIRQ(TMR0_IRQn);
    R8_TMR0_INTER_EN = 1;
    TMR0_TimerInit( 67000000 );
    USB30D_init(ENABLE);

    while(1)
    {
        CDC_Uart_Deal();
    }
}


