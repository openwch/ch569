/********************************** (C) COPYRIGHT *******************************
* File Name          : type.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56xSFR.h"

/*********************************************************************
 * @fn      usbhs_init
 *
 * @brief   usbhs_init
 *
 * @return  none
 */
void usbhs_init()   // USBSS device initial
{
    R8_USB_CTRL = RB_USB_CLR_ALL  | RB_USB_RESET_SIE;         //清空中断标志和FIFO，并软复位
    R8_USB_INT_EN = 0;
    R8_USB_CTRL = RB_USB_HOST_MODE | RB_USB_INT_BUSY | UCST_HS | RB_USB_DMA_EN;
    R8_USB_DEV_AD = 0x00;
    R8_UHOST_CTRL = 0x00;                              //主机模式下，正常工作状态

    R16_UH_RX_MAX_LEN = 512;                 //主机最大接收包的长度
    R8_UH_RX_CTRL = 0;
    R8_UH_TX_CTRL = 0;
    R8_UH_EP_MOD = RB_UH_RX_EN | RB_UH_TX_EN ;       //开启发送接送功能，单缓冲模式
}

/*********************************************************************
 * @fn      USBSS_IRQHandler
 *
 * @brief   USBSS_IRQHandler.
 *
 * @return  none
 */
void USBSS_IRQHandler (void)            //USBSSH interrupt severice
{

}
