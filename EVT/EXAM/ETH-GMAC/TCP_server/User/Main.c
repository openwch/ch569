/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.1
* Date               : 2021/07/31
* Description 		 : CH565/569 TCP/IP服务器演示例程，使用以太网时，主频不得低于60MHz
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

/*-----------------------------------头文件包含--------------------------------------*/
#include "CH56x_common.h"    /* CH565基本头文件  */
#include "ethernet_driver.h" /* 以太网驱动  */
#include "ethernet_config.h" /* 以太网配置  */
#include <CH569Net_lib.h>    /* 协议栈库头文件 */
#include "ISPEM569.h"        /* FLASH库头文件  */
#include "timer.h"           /* 协议栈时基需要的定时器和其他定时器 */
#include "macros.h"          /* 调试时用到的宏  */

/*---------------------------------------宏------------------------------------------*/
#define KEEPLIVE_ENABLE    1  /* 是否启用KEEPLIVE机制 */
#define create_a_socket    1  /* 是否建立一个socket */

/*------------------------------------全局变量----------------------------------------*/

/* TCP/IP协议栈本地参数 */
UINT8 local_mac[6] = {
    0x84,
    0xc2,
    0xe4,
    0x01,
    0x02,
    0x03,
};                                         /* 本地MAC地址，会在后面填入实际的MAC地址  */
UINT8 CH569IPAddr[4] = {192, 168, 1, 200}; /* 目的IP地址 */
UINT8 GatewayIp[4] = {192, 168, 1, 1};     /* 网关地址  */
UINT8 SubMaskIp[4] = {255, 255, 225, 0};   /* 子网掩码   */
#if create_a_socket
UINT8 DESIP[4] = {192, 168, 1, 100}; /* 目标IP */
#endif

/* 如果使用DHCP的配置 */
#ifdef USE_DHCP
UINT8 dhcp_source_ip[4] = {0, 0, 0, 0};
  #define init_ip    dhcp_source_ip
#else
  #define init_ip    CH569IPAddr
#endif

UINT8 SocketId;                                             /* 保存socket索引，可以不用定义 */
UINT8 SocketRecvBuf[CH569NET_MAX_SOCKET_NUM][RECE_BUF_LEN]; /* socket接收缓冲区 */
UINT8 MyBuf[RECE_BUF_LEN];                                  /* 定义一个临时缓冲区 */

/*******************************************************************************
 * @fn       DebugInit
 *
 * @brief    Initializes the UART1 peripheral.
 *
 * @param    baudrate: UART1 communication baud rate.
 *
 * @return   None
 */
void DebugInit(UINT32 baudrate)
{
    UINT32 x;
    UINT32 t = FREQ_SYS;

    x = 10 * t * 2 / 16 / baudrate;
    x = (x + 5) / 10;
    R8_UART1_DIV = 1;
    R16_UART1_DL = x;
    R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART1_LCR = RB_LCR_WORD_SZ;
    R8_UART1_IER = RB_IER_TXD_EN;
    R32_PA_SMT |= (1 << 8) | (1 << 7);
    R32_PA_DIR |= (1 << 8);
}

/*******************************************************************************
 * @fn        net_initkeeplive
 *
 * @brief     keeplive初始化
 *
 * @return    None
 */
#ifdef KEEPLIVE_ENABLE
void net_initkeeplive(void)
{
    struct _KEEP_CFG klcfg;

    klcfg.KLIdle = 72000000; /* 空闲 */
    klcfg.KLIntvl = 75000;   /* 间隔 */
    klcfg.KLCount = 9;       /* 次数 */
    CH569NET_ConfigKeepLive(&klcfg);
}
#endif

/*******************************************************************************
 * @fn       CH563NET_CreatUpdSocket
 *
 * @brief    创建一个 UDPsocket
 *
 * @return   None
 */
void CH563NET_CreatUpdSocket(void)
{
    UINT8    i;
    SOCK_INF TmpSocketInf; /* 创建临时socket变量 */

    memset((void *)&TmpSocketInf, 0, sizeof(SOCK_INF));     /* 库内部会将此变量复制，所以最好将临时变量先全部清零 */
    memcpy((void *)TmpSocketInf.IPAddr, DESIP, 4);          /* 设置目的IP地址 */
    TmpSocketInf.DesPort = 1000;                            /* 设置目的端口 */
    TmpSocketInf.SourPort = 2000;                           /* 设置源端口 */
    TmpSocketInf.ProtoType = PROTO_TYPE_UDP;                /* 设置socekt类型 */
    TmpSocketInf.RecvStartPoint = (UINT32)SocketRecvBuf[0]; /* 设置接收缓冲区的接收缓冲区 */
    TmpSocketInf.RecvBufLen = RECE_BUF_LEN;                 /* 设置接收缓冲区的接收长度 */
    i = CH569NET_SocketCreat(&SocketId, &TmpSocketInf);     /* 创建socket，将返回的socket索引保存在SocketId中 */
    mStopIfError(i);                                        /* 检查错误 */
}

/*******************************************************************************
 * @fn            CH569NET_LibInit
 *
 * @brief         库初始化操作
 *
 * @param ip      ip - 地址指针
 *                gwip - 网关ip地址指针
 *                mask - 掩码指针
 *                macaddr - MAC地址指针
 *
 * @return        执行状态
 */
UINT8 CH569NET_LibInit(UINT8 *ip, UINT8 *gwip, UINT8 *mask, UINT8 *macaddr)
{
    UINT8             i;
    struct _CH569_CFG cfg;

    if(CH569NET_GetVer() != CH569NET_LIB_VER)
        return 0xfc;                /* 获取库的版本号，检查是否和头文件一致 */
    CH569NETConfig = LIB_CFG_VALUE; /* 将配置信息传递给库的配置变量 */
    cfg.RxBufSize = /*RX_BUF_SIZE*/ 1520;
    cfg.TCPMss = CH569NET_TCP_MSS;
    cfg.HeapSize = CH569_MEM_HEAP_SIZE;
    cfg.ARPTableNum = CH569NET_NUM_ARP_TABLE;
    cfg.MiscConfig0 = CH569NET_MISC_CONFIG0;

    CH569NET_ConfigLIB(&cfg);
    i = CH569NET_Init(ip, gwip, mask, macaddr);
#ifdef KEEPLIVE_ENABLE
    net_initkeeplive(); /* 设置自动义的保活定时器参数，需要在初始化完之后设定 */
#endif
    return (i); /* 库初始化 */
}

/*******************************************************************************
 * @fn        CH569NET_HandleSockInt
 *
 * @brief     Socket中断处理函数
 *
 * @param     sockeid - socket索引
 *            initstat - 中断状态
 *
 * @return     None
 */
void CH569NET_HandleSockInt(UINT8 sockeid, UINT8 initstat)
{
    UINT32 len;
    UINT32 totallen;
#if create_a_socket
    UINT8 *p = MyBuf, i;
    char   rc;
#endif

    printf("CH569NET_HandleSockInt:%02x.\n", initstat);
    if(initstat & SINT_STAT_RECV) /* 接收中断 */
    {
#if create_a_socket
        len = CH569NET_SocketRecvLen(sockeid, NULL); /* 查询长度*/
        totallen = len;
        CH569NET_SocketRecv(sockeid, MyBuf, &len); /* 将接收缓冲区的数据读到MyBuf中 */
  #if 1
        while(1)
        {
            i++;
            len = totallen;
            rc = CH569NET_SocketSend(sockeid, p, &len);
            if(rc != 0) /* 将MyBuf中的数据发送 */
            {
                printf("---------------Send error:%x.---------------------\n", rc);
                break;
            }
            totallen -= len; /* 将总长度减去以及发送完毕的长度 */
            p += len;        /* 将缓冲区指针偏移*/
            if(i >= 10)
                break;
            if(totallen)
                continue; /* 如果数据未发送完毕，则继续发送*/
            break;        /* 发送完毕，退出 */
        }
  #endif
#endif
    }
    if(initstat & SINT_STAT_CONNECT) /* TCP连接中断 */
    {                                /* 产生此中断表示TCP已经连接，可以进行收发数据 */
        printf("TCP Connect Success\n");

        CH569NET_ModifyRecvBuf(sockeid, (UINT32)SocketRecvBuf[sockeid], RECE_BUF_LEN);
#if KEEPLIVE_ENABLE
        CH569NET_SocketSetKeepLive(sockeid, 1);
#endif
    }
    if(initstat & SINT_STAT_DISCONNECT) /* TCP断开中断 */
    {                                   /* 产生此中断，CH569库内部会将此socket清除，置为关闭*/
        printf("TCP Disconnect\n");     /* 应用曾需可以重新创建连接 */
    }
    if(initstat & SINT_STAT_TIM_OUT) /* TCP超时中断 */
    {                                /* 产生此中断，CH569库内部会将此socket清除，置为关闭*/
        printf("TCP Timout\n");      /* 应用曾需可以重新创建连接 */
    }
}

/*******************************************************************************
 * @fn        CH569NET_HandleGloableInt
 *
 * @brief     全局中断处理函数
 *
 * @return    None
 */
void CH569NET_HandleGlobalInt(void)
{
    UINT8 initstat;
    UINT8 i;
    UINT8 socketinit;

    initstat = CH569NET_GetGlobalInt(); /* 读全局中断状态并清除 */
    if(initstat & GINT_STAT_UNREACH)    /* 不可达中断 */
    {
        printf("UnreachCode ：%d\n", CH569Inf.UnreachCode);   /* 查看不可达代码 */
        printf("UnreachProto ：%d\n", CH569Inf.UnreachProto); /* 查看不可达协议类型 */
        printf("UnreachPort ：%d\n", CH569Inf.UnreachPort);   /* 查询不可达端口 */
    }
    if(initstat & GINT_STAT_IP_CONFLI) /* IP冲突中断 */
    {
        printf("Error:IP conflict.\n");
    }
    if(initstat & GINT_STAT_SOCKET) /* Socket中断 */
    {
        for(i = 0; i < CH569NET_MAX_SOCKET_NUM; i++)
        {
            socketinit = CH569NET_GetSocketInt(i); /* 读socket中断并清零 */
            if(socketinit)
                CH569NET_HandleSockInt(i, socketinit); /* 如果有中断则清零 */
        }
    }
}

/*******************************************************************************
 * @fn       CH569NET_CreatTcpSocket
 *
 * @brief    创建TCP Server socket
 *
 * @return   None
 */
void CH569NET_CreatTcpSocket(void)
{
    UINT8    i;
    SOCK_INF TmpSocketInf; /* 创建临时socket变量 */

    memset((void *)&TmpSocketInf, 0, sizeof(SOCK_INF)); /* 库内部会将此变量复制，所以最好将临时变量先全部清零 */

    TmpSocketInf.SourPort = 2000;                        /* 设置源端口 */
    TmpSocketInf.ProtoType = PROTO_TYPE_TCP;             /* 设置socekt类型 */
    TmpSocketInf.RecvStartPoint = (UINT32)SocketRecvBuf; /* 设置接收缓冲区的接收缓冲区 */
    TmpSocketInf.RecvBufLen = RECE_BUF_LEN;              /* 设置接收缓冲区的接收长度 */
    i = CH569NET_SocketCreat(&SocketId, &TmpSocketInf);  /* 创建socket，将返回的socket索引保存在SocketId中 */
    mStopIfError(i);                                     /* 检查错误 */
    i = CH569NET_SocketListen(SocketId);
    mStopIfError(i); /* 检查错误 */
}

/*******************************************************************************
 * @fn        DHCP_CALLBACK
 *
 * @brief     DHCP回调函数
 *
 * @param     rc - 状态
 *            p - 参数指针
 *
 * @return    执行状态
 */
UINT8 DHCP_CALLBACK(UINT8 rc, void *p)
{
    UINT8 *ptr = p;
    UINT8  ip[4], mask[4], gateway[4];

    if(rc != 0)
    {
        printf("DHCP failed!\n");
        reset_net_para(CH569IPAddr, SubMaskIp, GatewayIp);
        return 1;
    }
    printf("DHCP success.\n");
    printf("ip:%d.%d.%d.%d\ngate:%d.%d.%d.%d\nmask:%d.%d.%d.%d\ndns:%d.%d.%d.%d\ndns2:%d.%d.%d.%d\n",
           *ptr++, *ptr++, *ptr++, *ptr++,
           *ptr++, *ptr++, *ptr++, *ptr++,
           *ptr++, *ptr++, *ptr++, *ptr++,
           *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr);
    memcpy(ip, ptr - 19, 4);
    memcpy(mask, ptr - 11, 4);
    memcpy(gateway, ptr - 15, 4);
    reset_net_para(ip, mask, gateway);
    return 0;
}

/*******************************************************************************
 * @fn       register_if_fn
 *
 * @brief    为协议栈注册以太网接口函数
 *
 * @return   None
 */
void register_if_fn(void)
{
    ethernet_if.get_send_ptr = Get_TxBuff_Addr;
    ethernet_if.rece_fn = mac_rece;
    ethernet_if.send_fn = mac_send;
}

/*******************************************************************************
 * @fn         tcp_ip_stack_mian_process
 *
 * @brief      协议栈进程，需要在主循环中不断调用
 *
 * @return     None
 */
void tcp_ip_stack_main_process(void)
{
    CH569NET_MainTask(); /* CH563NET库主任务函数，需要在主循环中不断调用 */
    if(CH569NET_QueryGlobalInt())
        CH569NET_HandleGlobalInt(); /* 查询中断，如果有中断，则调用全局中断处理函数 */
}

/*******************************************************************************
 * @fn       get_unique_mac
 *
 * @brief    获取唯一MAC地址的函数
 *
 * @param    存储MAC的6字节空间的指针
 *
 * @return   None
 */
void get_unique_mac(UINT8 *MAC_PTR)
{
    UINT8 rc, buf[8];

    rc = FLASH_ROMA_READ((0x7FFE4 - 0x8000), buf, 8);
    mStopIfError(rc);
    for(rc = 0; rc < 6; rc++)
        MAC_PTR[0 + rc] = buf[5 - rc];
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main()
{
    UINT8  rc; /* 状态 */
    UINT8 *read_ptr, **p = &read_ptr;
    UINT16 read_length, i;
    UINT32 DIFF = 0;

    /* 系统初始化 */
    SystemInit(FREQ_SYS);
    Delay_Init(FREQ_SYS);
    /* 配置串口调试 */
    DebugInit(921600);
    get_unique_mac(local_mac);
    printf("Start @ChipID=%02X\r\n", R8_CHIP_ID);
    printf("WCH TCP server demo.\n");
    printf("System frequency is :%d Hz.Compiled at %s,%s\n", FREQ_SYS, __TIME__, __DATE__);
    printf("CH569 MAC address is:%02x:%02x:%02x:%02x:%02x:%02x\n", local_mac[0], local_mac[1], local_mac[2], local_mac[3], local_mac[4], local_mac[5]);

    /* MAC初始化，需要保证网线/光纤已经插上 */
    ETH_init();
    /* 定时器初始化,每10毫秒进一次中断服务函数 */
    TMR0_init(FREQ_SYS / 100);
    tmr1_init();
#ifdef USE_RTL8211FS
    /* 配置发送时钟延迟，在接收侧，RGMII的时钟需要比数据延迟1/4周期。用户需要根据自己的PCB情况保证 */
    RGMII_TXC_Delay(0, 2); /* RGMII发送延迟和相位调整 */
#endif
#if USE_ETH_PHY_INTERRUPT
    phy_int_cheak_init();
#endif
    mDelaymS(2000);
#if 0 /* 在11月改版之前的芯片需要保留下一句  */
	Check_TxDes();                                                              /* 发送描述符检查  */
#endif
    register_if_fn();                                               /* 向库中注册MAC接口函数 */
    i = CH569NET_LibInit(init_ip, GatewayIp, SubMaskIp, local_mac); /* 库初始化 */
    mStopIfError(i);                                                /* 检查上步的错误  */

    Enabl_MAC_RECV(); /* 使能接收 */

#if create_a_socket
    CH569NET_CreatTcpSocket(); /* 创建TCP Socket */
#endif

    printf("enter main loop!\r\n");

    while(1)
    {
        tcp_ip_stack_main_process();
    }
}

/*******************************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   发送硬件错误
 *
 * @return  None
 */
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void)
{
    printf("mepc=%08x\r\n", read_csr(mepc));
    printf("mcause=%08x\r\n", read_csr(mcause));
    printf("mtval=%08x\r\n", read_csr(mtval));
    while(1);
}
