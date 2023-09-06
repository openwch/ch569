/********************************** (C) COPYRIGHT *******************************
* File Name          : HSPI.H
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __HSPI_H__
#define __HSPI_H__


/******************************************************************************/
#if( DEF_HSPI_MODE == DEF_HSPI_HOST_MODE )
#define HPSI_DMA_TX_Addr0          0x20020000                                   /* HSPI send DMA0 address */
#define HPSI_DMA_TX_Addr1          HPSI_DMA_TX_Addr0 + DEF_HSPI_DMA_PACK_LEN    /* HSPI send DMA1 address */
#define HPSI_DMA_RX_Addr0          0x2002D000                                   /* HSPI recive DMA0 address */
#define HPSI_DMA_RX_Addr1          HPSI_DMA_RX_Addr0 + DEF_HSPI_DMA_PACK_LEN    /* HSPI recive DMA1 address */
#else
#define HPSI_DMA_TX_Addr0          0x20020000                                   /* HSPI send DMA0 address */
#define HPSI_DMA_TX_Addr1          HPSI_DMA_TX_Addr0 + DEF_HSPI_DMA_PACK_LEN    /* HSPI send DMA1 address */
#define HPSI_DMA_RX_Addr0          0x20021000                                   /* HSPI recive DMA0 address */
#define HPSI_DMA_RX_Addr1          HPSI_DMA_RX_Addr0 + DEF_HSPI_DMA_PACK_LEN    /* HSPI recive DMA1 address */
#endif

#define DEF_HSPI_DOUBLE_DIR_EN     1

#define DEF_HSPI_DATASIZE_8        0                                            /* 8 bit data */
#define DEF_HSPI_DATASIZE_16       1                                            /* 16 bit data */
#define DEF_HSPI_DATASIZE_32       2                                            /* 32 bit data */
#define DEF_HSPI_DATASIZE          DEF_HSPI_DATASIZE_32

#define DEF_HSPI_UDF0              0x3ABCDEF
#define DEF_HSPI_UDF1              0x3ABCDEF

#define DEF_HSPI_DMA_PACK_LEN      4096                                        /* DMA Packet length */

#define DEF_HSPI_BULK_BPACK_NUM    2                                           /* Definition of the number of burst mode packets in batch transmission mode */

#define DEF_HSPI_BULK_BPACK_LEN    ( DEF_HSPI_DMA_PACK_LEN * DEF_HSPI_BULK_BPACK_NUM )
#define DEF_HSPI_UVC_BPACK_LEN     ( DEF_HSPI_DMA_PACK_LEN * DEF_HSPI_UVC_BPACK_NUM )

/******************************************************************************/
extern UINT8V  HSPI_Tx_PackCnt;
extern UINT8V  HSPI_Tx_AddrTog;
extern UINT8V  HSPI_Rx_PackCnt;
extern UINT8V  HSPI_Rx_AddrTog;
extern UINT8V  HSPI_Tx_Status;
extern UINT32V HSPI_Tx_Data_LoadAddr;
extern UINT32V HSPI_Tx_Data_DealAddr;
extern UINT32V HSPI_Tx_Data_RemainLen;
extern UINT16V HSPI_Tx_LastPackLen;
extern UINT8V  HSPI_Int_En_Save;
extern UINT32V HSPI_Rx_Data_LoadAddr;
extern UINT32V HSPI_Rx_Data_DealAddr;
extern UINT32V HSPI_Rx_Data_RemainLen;
extern UINT8V  HSPI_Rx_Notice_Status;
extern UINT16V USB3_2_Switchover;

extern UINT32V Pack_Send_Num;
extern UINT32V Pack_Recv_Num;

/******************************************************************************/
extern void HSPI_GPIO_Init( void );
extern void HSPI_Init( void );
extern void HSPI_DataTrans( void );
extern void HSPI_Tx_Data_Deal( void );
#endif

/*********************************END OF FILE**********************************/






