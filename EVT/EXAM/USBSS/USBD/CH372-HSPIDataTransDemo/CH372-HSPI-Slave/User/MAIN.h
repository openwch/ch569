/********************************** (C) COPYRIGHT *******************************
* File Name          : MAIN.H
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
#include <stdio.h>
#include <string.h>													
#include "string.h"
#include "HAL.h"
#include "CH56x_common.h"
#include "HSPI.h"
#include "CH56x_usb30.h"
/*******************************************************************************/
#define DEF_PROG_VERSION           0x01

/*******************************************************************************/
/*Host and slave mode settings*/
/*******************************************************************************/
#define DEF_HSPI_HOST_MODE         0                                            /* HSPI Host */
#define DEF_HSPI_SLAVER_MODE       1                                            /* HSPI Slaver */
#define DEF_HSPI_MODE              DEF_HSPI_SLAVER_MODE                           /* **Select the corresponding macro to change the master and slave** */

/*******************************************************************************/
#define DEF_FUN_PACK_SN_EN         0x00                                         /* Package serial number function enable */

#define DEF_ENDP0_DMA_ADDR         ( 0x20020000 )                                   /* DMA address of USB endpoint 0 */
#define DEF_ENDP0_BUF_LEN          2048                                             /* USB endpoint 0 buffer size */
#define DEF_ENDP2_TX_DMA_ADDR      ( DEF_ENDP0_DMA_ADDR + DEF_ENDP0_BUF_LEN )       /* USB endpoint 2 sends DMA address */
#define DEF_ENDP2_TX_BUF_LEN       1024                                             /* USB endpoint 2 send buffer size */
#define DEF_ENDP2_RX_DMA_ADDR      ( DEF_ENDP2_TX_DMA_ADDR + DEF_ENDP2_TX_BUF_LEN ) /* USB endpoint 2 receives DMA address */
#define DEF_ENDP2_RX_BUF_LEN       1024                                             /* USB endpoint 2 receive buffer size */
#define DEF_ENDP3_TX_DMA_ADDR      ( DEF_ENDP2_RX_DMA_ADDR + DEF_ENDP2_RX_BUF_LEN ) /* USB endpoint 3 sends DMA address */
#define DEF_ENDP3_TX_BUF_LEN       1024                                             /* USB endpoint 3 send buffer size */
#define DEF_ENDP3_RX_DMA_ADDR      ( DEF_ENDP3_TX_DMA_ADDR + DEF_ENDP3_TX_BUF_LEN ) /* USB endpoint 3 receives DMA address */
#define DEF_ENDP3_RX_BUF_LEN       1024                                             /* USB endpoint 3 receive buffer size */
#define DEF_ENDP1_TX_DMA_ADDR      ( DEF_ENDP3_RX_DMA_ADDR + DEF_ENDP3_RX_BUF_LEN ) /* USB endpoint 1 sends DMA address */
#define DEF_ENDP1_TX_BUF_LEN_BULK  ( 48 * 1024 )                                    /* USB endpoint 1 send buffer size */
#define DEF_ENDP1_TX_BUF_LEN_UVC   ( 90 * 1024 )                                    /* USB endpoint 1 send buffer size */
#define DEF_ENDP1_RX_DMA_ADDR      ( DEF_ENDP1_TX_DMA_ADDR + DEF_ENDP1_TX_BUF_LEN_BULK ) /* USB endpoint 1 receives DMA address */
#define DEF_ENDP1_RX_BUF_LEN       ( 32 * 1024 )                                    /* USB endpoint 1 receive buffer size */

/* HSPI-related RAM address allocation */
#define DEF_HPSI_DMA_TX_ADDR0      DEF_ENDP1_RX_DMA_ADDR                            /* HSPI sends DMA0 address*/
#define DEF_HPSI_DMA_RX_ADDR0      DEF_ENDP1_TX_DMA_ADDR                            /* HSPI receive DMA0 address */

#define DEF_HPSI_TX_DMA_ADDR_MAX   ( DEF_HPSI_DMA_TX_ADDR0 + DEF_ENDP1_RX_BUF_LEN ) /* HSPI data transmission DMA maximum address */
#define DEF_HPSI_RX_DMA_ADDR_MAX   ( DEF_HPSI_DMA_RX_ADDR0 + DEF_ENDP1_TX_BUF_LEN_BULK ) /* HSPI data receiving DMA maximum address */

#define DEF_USB_DOWN_IDLE_TIMEOUT  500                                          /* USB Download idle time definition */


#ifdef __cplusplus
}
#endif

#endif

/*********************************END OF FILE**********************************/
