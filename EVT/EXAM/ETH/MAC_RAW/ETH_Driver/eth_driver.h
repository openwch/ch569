/********************************** (C) COPYRIGHT *******************************
* File Name          : eth_driver.h
* Author             : WCH
* Version            : V1.3.0
* Date               : 2023/03/03
* Description        : This file contains the headers of the ETH Driver.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __ETH_DRIVER__
#define __ETH_DRIVER__

#ifdef __cplusplus
 extern "C" {
#endif

#include "CH56xSFR.h"
#include "CH56x_common.h"

#define PHY_ADDRESS                             1
#define ETH_TXBUFNB                             2
#define ETH_RXBUFNB                             4
#define ETH_RX_BUF_SZE                          ETH_MAX_PACKET_SIZE
#define ETH_TX_BUF_SZE                          ETH_MAX_PACKET_SIZE

extern ETH_DMADESCTypeDef *DMATxDescToSet;
extern ETH_DMADESCTypeDef *DMARxDescToGet;
extern __attribute__ ((aligned(4))) uint8_t  MACTxBuf[ETH_TXBUFNB*ETH_TX_BUF_SZE];

void ETH_ETHIsr(void);
void ETH_PHYLink( void );
void ETH_Init( uint8_t *macAddr );
void ETH_GetMacAddr( uint8_t *p );
void ETH_Configuration( uint8_t *macAddr );
uint32_t MACRAW_Tx(uint8_t *buff, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
