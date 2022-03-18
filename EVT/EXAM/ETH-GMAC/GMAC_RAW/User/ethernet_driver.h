/********************************** (C) COPYRIGHT *******************************
* File Name          : ethernet_driver.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : CH565/569 千兆以太网链路层收发接口定义
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef __ethernet_driver__h__
#define __ethernet_driver__h__

#include "CH56x_common.h"


#define  RX_Des_Num        8                                    /* 接收描述符和缓冲区的数量 */
#define  TX_Des_Num        2                                    /* 发送描述符和缓冲区的数量 */
#define  RX_Buf_Size       1520                                 /* 接收缓冲区的大小*/
#define  TX_Buf_Size       1520                                 /* 发送缓冲区的大小*/

/* 是否使用PHY中断。评估板启用PHY中断需要禁用RST引脚的硬件复位功能并启用GPIO中断。 */
#define USE_ETH_PHY_INTERRUPT    1                              /* 默认不使用PHY中断 */

/* 启用光纤1000BASE-X模式。本例程在使用光纤模式时，需要强制速度为1000M 。如果注释下句，则物理层会工作在10BASE-T/100BASE-TX/1000BASE-T自适应的模式下*/
//#define FIBER_1000M

/* 启用内部125MHz时钟源。使用内部时钟源会更改USB部分的电源设置，请注意。如果注释下句，物理层会使用来自ETCKI引脚的125MHz */
//#define USE_USB125M

/* 物理层选择。例程支持RTL8211DN和FS两种型号的以太网物理层。每次使用只能且必须定义其中一种 */
#define USE_RTL8211FS
//#define USE_RTL8211DN

#ifdef USE_RTL8211FS
	#define PHY_ADDRESS     0x01			/* RTL8211FS  PHY芯片地址 */
#endif
#ifdef USE_RTL8211DN
	#define PHY_ADDRESS     0x00			/* RTL821DN的PHY芯片地址.用户需要根据实际值修改 */
#endif

#define Dscr_length  (16)                /* 描述符的长度，需要加括号 */

/* CH565/569的高速DMA是128位宽的，缓冲区需要16字节对齐。请勿修改此值 */
#define Desc_buf_align   16              /* 描述符和缓冲区对齐 */
//#define buff_base_addr   0x20020000

extern UINT8 MAC_QUEUE[];/* MAC使用的缓冲区位置，在main.c中向编译器申请，需要16字节对齐 */
//#define     ETH_queue_base           (volatile UINT8*)(buff_base_addr+20*1024)
//#define ETH_queue_base  (volatile UINT8*)(buff_base_addr)
#define ETH_queue_base  MAC_QUEUE

#if buff_base_addr%Desc_buf_align!=0
#error("Error：the base address of des and buff are not aligned by 16bytes!");
#endif

/* 发送描述符偏移 */
#if Dscr_length%Desc_buf_align
	#define TxDscrOffset (Dscr_length/16+1)*16
#else
	#define TxDscrOffset Dscr_length
#endif

/* 接收描述符偏移 */
#if Dscr_length%Desc_buf_align
	#define RxDscrOffset (Dscr_length/16+1)*16
#else
	#define RxDscrOffset Dscr_length
#endif

/* 发送缓冲区偏移 */
#define TxBufOffset TX_Des_Num*TX_Buf_Size

/* 发送描述符偏移 */
#define RxBufOffset RX_Des_Num*RX_Buf_Size

/* 发送描述符地址 */
#define pDMATxDscrTab (( ETH_DMADESCTypeDef*)ETH_queue_base)     /* 接收描述符 */
/* 接收描述符地址 */
#define pDMARxDscrTab (( ETH_DMADESCTypeDef*)(ETH_queue_base+TxDscrOffset*TX_Des_Num))     /* 发送描述符 */
/* 发送缓冲区地址 */
#define	pTx_Buff      (( UINT8 *)(ETH_queue_base+TxDscrOffset*TX_Des_Num+RxDscrOffset*RX_Des_Num))     /* 接收缓冲区 */
/* 接收缓冲区地址 */
#define	pRx_Buff      (( UINT8 *)(ETH_queue_base+TxDscrOffset*TX_Des_Num+RxDscrOffset*RX_Des_Num+TX_Des_Num*TX_Buf_Size))     /* 发送缓冲区 */


extern ETH_DMADESCTypeDef *DMARxDscrTab;			//以太网DMA接收描述符数据结构体指针
extern ETH_DMADESCTypeDef *DMATxDscrTab;			//以太网DMA发送描述符数据结构体指针 
extern UINT8 *Rx_Buff; 							//以太网底层驱动接收buffers指针
extern UINT8 *Tx_Buff; 							//以太网底层驱动发送buffers指针
extern ETH_DMADESCTypeDef  *DMATxDescToSet;			//DMA发送描述符追踪指针
extern ETH_DMADESCTypeDef  *DMARxDescToGet; 		//DMA接收描述符追踪指针 

typedef enum
{
	ETH_disconnect=0,
	ETH_10M       =1,
	ETH_100M      =2,
	ETH_Gigabit   =3,
} Link_Speed_t;
//void	mDelayuS( unsigned short n );
//void	mDelaymS(  unsigned short n );

void ETH_GPIO_Init(void);
void RGMII_TXC_Delay(UINT8 clock_polarity,UINT8 delay_time);
UINT8 ETH_Mem_Malloc(void);
Link_Speed_t rtl8211dn_Get_Speed(void);
UINT8 ETH_buf_init(void);
void ENABLE_ETH_WAKEUP(void);
UINT8 ETH_MACDMA_Config(void);
UINT8 ETH_MACDMA_reConfig(void);
UINT8 Mac_init(void);
void ETH_init(void);
void ENABLE_PMT_INT(void);
void Check_TxDes(void);
UINT8 ETH_Tx_Packet(unsigned short FrameLength);
UINT32 ETH_GetCurrentTxBuffer(void);

void ETH_StructInit(ETH_InitTypeDef* ETH_InitStruct);

UINT8 mac_send(UINT32 length);
void MAC_ReInit(ETH_InitTypeDef* ETH_InitStruct);
UINT16 mac_rece(UINT8 ** ptr/*,UINT16 *rece_lengeh*/);
void ETH_DMAClearITPendingBit(UINT32 ETH_DMA_IT);
void ETH_DMAITConfig(UINT32 ETH_DMA_IT, FunctionalState NewState);
void ETH_MACAddressConfig(UINT32 MacAddr, UINT8 *Addr);
void ETH_DMARxDescChainInit(ETH_DMADESCTypeDef *DMARxDescTab, UINT8 *RxBuff, UINT32 RxBuffCount);
void ETH_DMATxDescChainInit(ETH_DMADESCTypeDef *DMATxDescTab, UINT8* TxBuff, UINT32 TxBuffCount);
UINT32 ETH_Init(ETH_InitTypeDef* ETH_InitStruct, UINT16 PHYAddress);
void ETH_StructInit(ETH_InitTypeDef* ETH_InitStruct);
void ETH_SoftwareReset(void);
FlagStatus ETH_GetSoftwareResetStatus(void);
UINT16 ETH_ReadPHYRegister(UINT16 PHYAddress, UINT16 PHYReg);
UINT32 ETH_WritePHYRegister(UINT16 PHYAddress, UINT16 PHYReg, UINT16 PHYValue);

/* 全局接收状态，用来描述以太网驱动接收状态  */
typedef struct
{
	ETH_DMADESCTypeDef *LastRxDes; /* 指向最近一次处理完的接收描述符的下一个描述符的地址，就是待处理的接收描述符的地址 */
	UINT16 pengding_RxDes_cnt;     /* 待处理的接收描述符的个数，在接收中断中自增，在处理流程中自减 ，如果这个值大于接收队列深度，则已经产生丢包 */
#if 1
	UINT16 package_loss_cnt;       /* 表示由于软件优化不足和处理器速度问题，导致的丢包次数，这不是丢包的包数 */
#endif
}Globe_RxDes_status_t;


//#define if_any_frames_not_read()  (Globe_RxDes_status.pengding_RxDes_cnt)
#define get_txbuff_addr()         ((UINT8*)(DMATxDescToSet->Buffer1Addr))

#endif 

