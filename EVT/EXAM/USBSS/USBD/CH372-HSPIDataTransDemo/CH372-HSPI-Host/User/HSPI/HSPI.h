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
#ifndef	__HSPI_H__
#define __HSPI_H__


/******************************************************************************/
#define DEF_HSPI_HOST_MODE         0											/* HSPI Host */
#define DEF_HSPI_SLAVER_MODE       1											/* HSPI Slaver */
#define DEF_HSPI_MODE              DEF_HSPI_HOST_MODE


/* Definition of data bits */
#define DEF_HSPI_DATASIZE_8        0											/* 8 BIT */
#define DEF_HSPI_DATASIZE_16       1											/* 16 BIT */
#define DEF_HSPI_DATASIZE_32       2											/* 32 BIT */
#define DEF_HSPI_DATASIZE          DEF_HSPI_DATASIZE_32

/* Custom byte definition */
#define DEF_HSPI_UDF0              0x3ABCDEF
#define DEF_HSPI_UDF1              0x3ABCDEF

/* DMA Packet length definition */
#define DEF_HSPI_DMA_PACK_LEN      4096                                        /* DMA Packet length */

/* DMA Definition of the number of burst mode packets */
#define DEF_HSPI_BULK_BPACK_NUM    2

/* DMA Definition of total packet size in burst mode */
#define DEF_HSPI_BULK_BPACK_LEN    ( DEF_HSPI_DMA_PACK_LEN * DEF_HSPI_BULK_BPACK_NUM )
#define DEF_HSPI_UVC_BPACK_LEN     ( DEF_HSPI_DMA_PACK_LEN * DEF_HSPI_UVC_BPACK_NUM )

/******************************************************************************/
extern volatile UINT8  HSPI_Tx_PackCnt;
extern volatile UINT8  HSPI_Tx_AddrTog;
extern volatile UINT8  HSPI_Rx_PackCnt;
extern volatile UINT8  HSPI_Rx_AddrTog;

extern volatile UINT8  HSPI_Tx_Status;
extern volatile UINT32 Pack_Send_Num;
extern volatile UINT32 Pack_Recv_Num;

/******************************************************************************/
extern void HSPI_GPIO_Init( void );
extern void HSPI_Init( void );

extern void BULKMode_HSPI_Test( void );
extern void UVCMode_HSPI_Test( void );

#endif

/*********************************END OF FILE**********************************/






