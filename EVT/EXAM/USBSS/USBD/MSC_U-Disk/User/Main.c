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
 * This routine is used for USB3.0 device.Simulated a CH372 device.
 * The host can burst 4 packets of data to endpoint 1 and then take 4 packets of data from endpoint 1.
 * The host can continuously send or receive data to endpoint 2-7 (to test the speed).
*/

#include "CH56x_common.h"
#include "CH56x_usb30.h"
#include "CH56x_usb20.h"
#include "SW_UDISK.h"

/* Global define */
#define  UART1_BAUD     115200

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
int main( void )
{
    UINT8   s;
    UINT16  i;

    SystemInit(FREQ_SYS);
	Delay_Init(FREQ_SYS);

	/* Configure serial port debugging */
	DebugInit(UART1_BAUD);
	PRINT("CH56x USB3.0 & USB2.0 device test(80MHz) !\n");

	/* EMMC initialization */
    PFIC_EnableIRQ(EMMC_IRQn);
    R32_EMMC_TRAN_MODE = 0;                              //ensure EMMC clock
    EMMCIO0Init();
    s = EMMCCardConfig( &TF_EMMCParam );
    PRINT("SDLinkSatus:%x\r\n",TF_EMMCParam.EMMCLinkSatus);
    PRINT("SDCardSatus:%x\r\n",TF_EMMCParam.EMMCCardSatus);
    PRINT("SDVoltageMode:%x\r\n",TF_EMMCParam.EMMCVoltageMode);
    PRINT("SDSecSize:%x\r\n",TF_EMMCParam.EMMCSecSize);
    PRINT("SDSecNum:%x\r\n",TF_EMMCParam.EMMCSecNum);
    PRINT("SDOpErr:%x\r\n",TF_EMMCParam.EMMCOpErr);

    if(s != OP_SUCCESS)
    {
        PRINT("Emmc Init Failed...\n");
        while(1);
    }
    else
    {
        PRINT("Emmc Init Success...\n");
    }

	/* USB initialization */
    R32_USB_CONTROL = 0;
    PFIC_EnableIRQ(USBSS_IRQn);
    PFIC_EnableIRQ(LINK_IRQn);

    PFIC_EnableIRQ(TMR0_IRQn);
    R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
    TMR0_TimerInit( 67000000 );
	USB30D_init(ENABLE);

    /* Enable Udisk */
	Udisk_Capability = TF_EMMCParam.EMMCSecNum;
    Udisk_Status |= DEF_UDISK_EN_FLAG;

	while(1)
	{
	    UDISK_onePack_Deal();
	}
}

