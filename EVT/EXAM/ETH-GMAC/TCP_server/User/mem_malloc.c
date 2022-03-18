/********************************** (C) COPYRIGHT *******************************
* File Name          : mem_malloc.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : CH565/569 千兆以太网收发器收发队列及 TCP/IP协议栈内存申请函数
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include <CH569Net_lib.h>                         /* 库头文件 */
#include "ethernet_config.h"
#include "ethernet_driver.h"


/*-------------------------------MAC%DMA空间-----------------------------------------------------*/

/* MAC收发队列内存申请，这个空间用来存放收发队列和描述符 */
__attribute__ ((aligned(16)))
UINT8 MAC_QUEUE[TxDscrOffset*TX_Des_Num+                     /* 发送描述符需要的空间 */
				RxDscrOffset*RX_Des_Num+                     /* 接收描述符需要的空间 */
				TX_Des_Num/*1*/*TX_Buf_Size+                 /* 发送缓冲区需要的空间 */   //发送缓冲区个数1  by ltp
				RX_Des_Num*RX_Buf_Size]={0};                 /* 接收缓冲区需要的空间 */

/* 这两个全局指针用来指示软件当前使用的描述符，注意和CHRDR/CHTDR区分 */
 ETH_DMADESCTypeDef  *DMATxDescToSet;                         /* 当前发送描述符指针 */
 ETH_DMADESCTypeDef  *DMARxDescToGet;                         /* 当前接收描述符指针 */


/*-----------------------------协议栈缓冲空间--------------------------*/
__attribute__ ((aligned(4)))
SOCK_INF SocketInf[CH569NET_MAX_SOCKET_NUM];                  /* Socket信息表，4字节对齐，可以根据实际使用的socket减少 */

/*原来个数为8,现增加4组用内存池管理内存堆的每组个数在MemNum中定义 */
const UINT16 MemNum[12]  = {MemNum_content};                   /* 一些固定的参数 */
const UINT16 MemSize[12] = {MemSize_content};                  /* 一些固定的参数 */

/* 内存池空间 *//* 协议栈需要的池空间 */
__attribute__ ((aligned(4)))
UINT8 Memp_Memory[CH569NET_MEMP_SIZE]={0};


#if 0
/* 协议栈需要的堆空间 */
__attribute__ ((aligned(4)))//__attribute__((section(".dmadata")))
UINT8 Mem_Heap_Memory[CH569NET_RAM_HEAP_SIZE]={0};
#endif

/* 协议栈需要的ARP表空间 */
__attribute__ ((aligned(4)))
UINT8 Mem_ArpTable[CH569NET_RAM_ARP_TABLE_SIZE]={0};








