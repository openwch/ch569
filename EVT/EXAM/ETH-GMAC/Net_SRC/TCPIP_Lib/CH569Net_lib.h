/********************************** (C) COPYRIGHT *******************************
* File Name          : CH569Net_lib.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
*******************************************************************************/
/*
 * 本文件只能被用户程序包含
 *  */
#ifndef   _CH569NETLIB_H__
#define   _CH569NETLIB_H__

#ifndef UINT8
typedef unsigned char UINT8;
#endif

#ifndef UINT16
typedef unsigned short UINT16;
#endif

#ifndef UINT32
typedef unsigned long UINT32;
#endif

#define CH569NET_LIB_VER                      0x02                              /* 版本号：0x02 */

#ifndef CH569NET_MAX_SOCKET_NUM
#define CH569NET_MAX_SOCKET_NUM               4                                 /* Socket的个数，用户可以配置，默认为4个Socket,最大为32 */
#endif

#define CH569NET_TCP_MSS                      536

#ifndef CH569NET_PING_ENABLE
#define CH569NET_PING_ENABLE                  TRUE                              /* 默认PING开启 */
#endif                                                                          /* PING使能 */

#ifndef TCP_RETRY_COUNT
#define TCP_RETRY_COUNT                       20                                /* TCP重试次数，位宽为5位*/
#endif

#ifndef TCP_RETRY_PERIOD
#define TCP_RETRY_PERIOD                      2                                /* TCP重试周期，单位为50MS，位宽为5位 */
#endif

#ifndef CH569NETTIMEPERIOD
#define CH569NETTIMEPERIOD                    10                                /* 定时器周期，单位Ms,不得大于500 */
#endif

#ifndef SOCKET_SEND_RETRY
#define SOCKET_SEND_RETRY                     1                                 /* 默认发送重试 */
#endif

#define LIB_CFG_VALUE                         ((SOCKET_SEND_RETRY << 25) |\
                                              (/*MAC_INT_TYPE*/1 << 24) |\
                                              (TCP_RETRY_PERIOD << 19) |\
                                              (TCP_RETRY_COUNT << 14) |\
                                              (CH569NET_PING_ENABLE << 13) |\
                                              (/*TX_QUEUE_ENTRIES*/2 << 9) |\
                                              (/*RX_QUEUE_ENTRIES*/8 << 5) |\
                                              (CH569NET_MAX_SOCKET_NUM))
#ifndef MISCE_CFG0_TCP_SEND_COPY
#define MISCE_CFG0_TCP_SEND_COPY              1                                 /* TCP发送缓冲区复制 */
#endif

#ifndef MISCE_CFG0_TCP_RECV_COPY
#define MISCE_CFG0_TCP_RECV_COPY              1                                 /* TCP接收复制优化，内部调试使用 */
#endif

#ifndef MISCE_CFG0_TCP_OLD_DELETE
#define MISCE_CFG0_TCP_OLD_DELETE             0                                 /* 删除最早的TCP连接 */
#endif

/*关于内存分配 */
#ifndef CH569NET_NUM_IPRAW
#define CH569NET_NUM_IPRAW                    4                                 /* IPRAW连接的个数 */
#endif

#ifndef CH569NET_NUM_UDP
#define CH569NET_NUM_UDP                      4                                 /* UDP连接的个数 */
#endif

#ifndef CH569NET_NUM_TCP
#define CH569NET_NUM_TCP                      4                                 /* TCP连接的个数 */
#endif

#ifndef CH569NET_NUM_TCP_LISTEN
#define CH569NET_NUM_TCP_LISTEN               4                                 /* TCP监听的个数 */
#endif

#ifndef CH569NET_NUM_PBUF
#define CH569NET_NUM_PBUF                     16                                 /* PBUF结构的个数 */
#endif

#ifndef CH569NET_NUM_POOL_BUF
#define CH569NET_NUM_POOL_BUF                 0                                 /* POOL BUF的个数 */
#endif

#ifndef CH569NET_NUM_TCP_SEG
#define CH569NET_NUM_TCP_SEG                  16                                 /* tcp段的个数*/
#endif

#ifndef CH569NET_NUM_IP_REASSDATA
#define CH569NET_NUM_IP_REASSDATA             16                                 /* IP分段的长度 */
#endif

/*内存池管理内存堆*/
#ifndef CH569NET_NUM_POOL_256
#define CH569NET_NUM_POOL_256                 12                               /* 256字节的缓冲区个数 */
#endif

#ifndef CH569NET_NUM_POOL_512
#define CH569NET_NUM_POOL_512                 4                                 /* 512字节的缓冲区个数 */
#endif

#ifndef CH569NET_NUM_POOL_1024
#define CH569NET_NUM_POOL_1024                0                                 /* 1024字节的缓冲区个数 */
#endif

#ifndef CH569NET_NUM_POOL_1512
#define CH569NET_NUM_POOL_1512                0                                 /* 1512字节的缓冲区个数 */
#endif
/* end add by ltp */


#ifndef CH569NET_TCP_MSS
#define CH569NET_TCP_MSS                      1460                              /* tcp MSS的大小*/
#endif

#ifndef RECE_BUF_LEN
#define RECE_BUF_LEN                          (1460*1)                           /* 接收缓冲区的大小 */
#endif

#ifndef CH569_MEM_HEAP_SIZE
#define CH569_MEM_HEAP_SIZE                   4600                             /* 内存堆大小 */
#endif

#ifndef CH569NET_NUM_ARP_TABLE
#define CH569NET_NUM_ARP_TABLE                16                                /* ARP列表个数 */
#endif

#ifndef CH569NET_MEM_ALIGNMENT
#define CH569NET_MEM_ALIGNMENT                4                                 /* 4字节对齐 */
#endif

#ifndef CH569NET_IP_REASS_PBUFS
#if (CH569NET_NUM_POOL_BUF < 32)
#define CH569NET_IP_REASS_PBUFS               (CH569NET_NUM_POOL_BUF - 1)       /* IP分片的PBUF个数，最大为31 */
#else
#define CH569NET_IP_REASS_PBUFS               31
#endif
#endif

#define CH569NET_MISC_CONFIG0                 ((MISCE_CFG0_TCP_SEND_COPY << 0) |\
                                               (MISCE_CFG0_TCP_RECV_COPY << 1) |\
                                               (MISCE_CFG0_TCP_OLD_DELETE << 2)|\
                                               (CH569NET_IP_REASS_PBUFS)<<3)

#define  MemNum_content  CH569NET_NUM_IPRAW,\
                         CH569NET_NUM_UDP,\
                         CH569NET_NUM_TCP,\
                         CH569NET_NUM_TCP_LISTEN,\
                         CH569NET_NUM_TCP_SEG,\
                         CH569NET_NUM_IP_REASSDATA,\
                         CH569NET_NUM_PBUF,\
                         CH569NET_NUM_POOL_BUF,\
                         CH569NET_NUM_POOL_256,\
                         CH569NET_NUM_POOL_512,\
                         CH569NET_NUM_POOL_1024,\
                         CH569NET_NUM_POOL_1512

#define MemSize_content  CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_IPRAW_PCB),\
                         CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_UDP_PCB),\
                         CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_TCP_PCB),\
                         CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_TCP_PCB_LISTEN),\
                         CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_TCP_SEG),\
                         CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_IP_REASSDATA),\
                         CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_PBUF) + CH569NET_MEM_ALIGN_SIZE(0),\
                         CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_PBUF) + CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_POOL_BUF),\
                         CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_POOL_256),\
                         CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_POOL_512),\
                         CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_POOL_1024),\
                         CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_POOL_1512)



#ifndef CH569NET_MEM_ALIGNMENT
#define CH569NET_MEM_ALIGNMENT                4                                 /* 4字节对齐 */
#endif

#define CH569NET_MEM_ALIGN_SIZE(size)         (((size) + CH569NET_MEM_ALIGNMENT - 1) & ~(CH569NET_MEM_ALIGNMENT-1))/* 这个可以保证size一定能被16整除 */
#define CH569NET_SIZE_IPRAW_PCB               0x1C                              /* IPRAW PCB大小 */
#define CH569NET_SIZE_UDP_PCB                 0x20                              /* UDP PCB大小 */
#define CH569NET_SIZE_TCP_PCB                 0xAC                              /* TCP PCB大小 */
#define CH569NET_SIZE_TCP_PCB_LISTEN          0x20                              /* TCP LISTEN PCB大小 */
#define CH569NET_SIZE_IP_REASSDATA            0x20                              /* IP分片管理  */
#define CH569NET_SIZE_PBUF                    0x10                              /* Packet Buf */
#define CH569NET_SIZE_TCP_SEG                 0x14                              /* TCP SEG结构 */
#define CH569NET_SIZE_MEM                     0x06                              /* sizeof(struct mem) */
#define CH569NET_SIZE_ARP_TABLE               0x10                              /* sizeof arp table */

/* 增加内存池管理内存堆部分 */
#define CH569NET_SIZE_POOL_256                0x100
#define CH569NET_SIZE_POOL_512                0x200
#define CH569NET_SIZE_POOL_1024               0x400
#define CH569NET_SIZE_POOL_1512               0x600
/* end add by ltp */


#define CH569NET_SIZE_POOL_BUF                CH569NET_MEM_ALIGN_SIZE(CH569NET_TCP_MSS + 40 +14) /* pbuf池大小 */
#define CH569NET_MEMP_SIZE                    ((CH569NET_MEM_ALIGNMENT - 1) + \
                                              (CH569NET_NUM_IPRAW * CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_IPRAW_PCB)) + \
                                              (CH569NET_NUM_UDP * CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_UDP_PCB)) + \
                                              (CH569NET_NUM_TCP * CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_TCP_PCB)) + \
                                              (CH569NET_NUM_TCP_LISTEN * CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_TCP_PCB_LISTEN)) + \
                                              (CH569NET_NUM_TCP_SEG * CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_TCP_SEG)) + \
                                              (CH569NET_NUM_IP_REASSDATA * CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_IP_REASSDATA)) + \
                                              (CH569NET_NUM_PBUF * (CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_PBUF) + CH569NET_MEM_ALIGN_SIZE(0))) + \
                                              (CH569NET_NUM_POOL_BUF * (CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_PBUF) + CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_POOL_BUF)))) + \
                                              (CH569NET_NUM_POOL_256 *  CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_POOL_256)) + \
                                              (CH569NET_NUM_POOL_512 *  CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_POOL_512)) + \
                                              (CH569NET_NUM_POOL_1024 *  CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_POOL_1024)) + \
                                              (CH569NET_NUM_POOL_1512 *  CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_POOL_1512))


#define  HEAP_MEM_ALIGN_SIZE                  (CH569NET_MEM_ALIGN_SIZE(CH569NET_SIZE_MEM))
#define  CH569NET_RAM_HEAP_SIZE               (CH569_MEM_HEAP_SIZE + (2 * HEAP_MEM_ALIGN_SIZE) + CH569NET_MEM_ALIGNMENT)
#define  CH569NET_RAM_ARP_TABLE_SIZE          (CH569NET_SIZE_ARP_TABLE * CH569NET_NUM_ARP_TABLE)

/* Socket 工作模式定义,协议类型 */
#define PROTO_TYPE_IP_RAW                     0                                 /* IP层原始数据 */
#define PROTO_TYPE_UDP                        2                                 /* UDP协议类型 */
#define PROTO_TYPE_TCP                        3                                 /* TCP协议类型 */

/* 中断状态 */
/* 以下为GLOB_INT会产生的状态 */
#define GINT_STAT_UNREACH                     (1<<0)                            /* 不可达中断*/
#define GINT_STAT_IP_CONFLI                   (1<<1)                            /* IP冲突*/
//#define GINT_STAT_PHY_CHANGE                  (1<<2)                            /* PHY状态改变 */
#define GINT_STAT_SOCKET                      (1<<4)                            /* scoket 产生中断 */

/*以下为Sn_INT会产生的状态*/
#define SINT_STAT_RECV                        (1<<2)                            /* socket端口接收到数据或者接收缓冲区不为空 */
#define SINT_STAT_CONNECT                     (1<<3)                            /* 连接成功,TCP模式下产生此中断 */
#define SINT_STAT_DISCONNECT                  (1<<4)                            /* 连接断开,TCP模式下产生此中断 */
#define SINT_STAT_TIM_OUT                     (1<<6)                            /* ARP和TCP模式下会发生此中断 */

/* 错误码 */
#define CH569NET_ERR_SUCCESS                  0x00                              /* 命令操作成功 */
#define CH569NET_RET_ABORT                    0x5F                              /* 命令操作失败 */
#define CH569NET_ERR_BUSY                     0x10                              /* 忙状态，表示当前正在执行命令 */
#define CH569NET_ERR_MEM                      0x11                              /* 内存错误 */
#define CH569NET_ERR_BUF                      0x12                              /* 缓冲区错误 */
#define CH569NET_ERR_TIMEOUT                  0x13                              /* 超时 */
#define CH569NET_ERR_RTE                      0x14                              /* 路由错误*/
#define CH569NET_ERR_ABRT                     0x15                              /* 连接停止*/
#define CH569NET_ERR_RST                      0x16                              /* 连接复位 */
#define CH569NET_ERR_CLSD                     0x17                              /* 连接关闭/socket 在关闭状态*/
#define CH569NET_ERR_CONN                     0x18                              /* 无连接 */
#define CH569NET_ERR_VAL                      0x19                              /* 错误的值 */
#define CH569NET_ERR_ARG                      0x1a                              /* 错误的参数 */
#define CH569NET_ERR_USE                      0x1b                              /* 已经被使用 */
#define CH569NET_ERR_IF                       0x1c                              /* MAC错误  */
#define CH569NET_ERR_ISCONN                   0x1d                              /* 已连接 */
#define CH569NET_ERR_SOCKET_MEM               0X20                              /* Socket信息列表已满或者错误 */
#define CH569NET_ERR_UNSUPPORT_PROTO          0X21                              /* 不支持的协议类型 */
#define CH569NET_ERR_UNKNOW                   0xFA                              /* 未知错误 */

/* 不可达代码 */
#define UNREACH_CODE_HOST                     0                                 /* 主机不可达 */
#define UNREACH_CODE_NET                      1                                 /* 网络不可达 */
#define UNREACH_CODE_PROTOCOL                 2                                 /* 协议不可达 */
#define UNREACH_CODE_PROT                     3                                 /* 端口不可达 */
/*其他值请参考RFC792文档*/

/* TCP关闭参数 */
#define TCP_CLOSE_NORMAL                      0                                 /* 正常关闭，进行4此握手 */
#define TCP_CLOSE_RST                         1                                 /* 复位连接，并关闭  */
#define TCP_CLOSE_ABANDON                     2                                 /* CH569NET内部丢弃连接，不会发送任何终止报文 */

/* socket状态 */
#define  SOCK_STAT_CLOSED                     0X00                              /* socket关闭 */
#define  SOCK_STAT_OPEN                       0X05                              /* socket打开 */

/* TCP状态 */
#define TCP_CLOSED                            0                                 /* TCP连接 */
#define TCP_LISTEN                            1                                 /* TCP关闭 */
#define TCP_SYN_SENT                          2                                 /* SYN发送，连接请求 */
#define TCP_SYN_RCVD                          3                                 /* SYN接收，接收到连接请求 */
#define TCP_ESTABLISHED                       4                                 /* TCP连接建立 */
#define TCP_FIN_WAIT_1                        5                                 /* WAIT_1状态 */
#define TCP_FIN_WAIT_2                        6                                 /* WAIT_2状态 */
#define TCP_CLOSE_WAIT                        7                                 /* 等待关闭 */
#define TCP_CLOSING                           8                                 /* 正在关闭 */
#define TCP_LAST_ACK                          9                                 /* LAST_ACK*/
#define TCP_TIME_WAIT                         10                                /* 2MSL等待 */

#define mStopIfError(x)\
do{if(x!=0){printf("Error: %02x,@ line %d of \"%s\".\r\n", (uint16_t)x,__LINE__,__FILE__);while(1);}}while(0)


/* sokcet信息表 */
typedef struct _SCOK_INF
{
    UINT32 IntStatus;                                                           /* 中断状态 */
    UINT32 SockIndex;                                                           /* Socket索引值 */
    UINT32 RecvStartPoint;                                                      /* 接收缓冲区的开始指针 */
    UINT32 RecvBufLen;                                                          /* 接收缓冲区长度 */
    UINT32 RecvCurPoint;                                                        /* 接收缓冲区的当前指针 */
    UINT32 RecvReadPoint;                                                       /* 接收缓冲区的读指针 */
    UINT32 RecvRemLen;                                                          /* 接收缓冲区的剩余长度 */
    UINT32 ProtoType;                                                           /* 协议类型 */
    UINT32 ScokStatus;                                                          /* 低字节Socket状态，次低字节为TCP状态，仅TCP模式下有意义 */
    UINT32 DesPort;                                                             /* 目的端口 */
    UINT32 SourPort;                                                            /* 源端口在IPRAW模式下为协议类型 */
    UINT8  IPAddr[4];                                                           /* Socket目标IP地址 32bit*/
    void *Resv1;                                                                /* 保留，内部使用 */
    void *Resv2;                                                                /* 保留，内部使用，TCP Server使用 */
 // void (*RecvCallBack)(struct _SCOK_INF *socinf,UINT32 ipaddr,UINT16 port,UINT8 *buf,UINT32 len); /* 接收回调函数*/
    void (*AppCallBack)(struct _SCOK_INF *,UINT32 ,UINT16 ,UINT8 *,UINT32 ); /* 接收回调函数*/

}SOCK_INF;

/* CH569全局信息 */
struct _CH569_SYS
{
    UINT8  IPAddr[4];                                                           /* CH569IP地址 32bit */
    UINT8  GWIPAddr[4];                                                         /* CH569网关地址 32bit */
    UINT8  MASKAddr[4];                                                         /* CH569子网掩码 32bit */
    UINT8  MacAddr[8];                                                          /* CH569MAC地址 48bit */
    UINT8  UnreachIPAddr[4];                                                    /* 不可到达IP */
    UINT32 RetranCount;                                                         /* 重试次数 默认为10次 */
    UINT32 RetranPeriod;                                                        /* 重试周期,单位MS,默认200MS */
    UINT32 PHYStat;                                                             /* CH569PHY状态码 8bit */
    UINT32 CH569Stat;                                                           /* CH569的状态 ，包含是否打开等 */
    UINT32 MackFilt;                                                            /* CH569 MAC过滤，默认为接收广播，接收本机MAC 8bit */
    UINT32 GlobIntStatus;                                                       /* 全局中断 */
    UINT32 UnreachCode;                                                         /* 不可达 */
    UINT32 UnreachProto;                                                        /* 不可达协议 */
    UINT32 UnreachPort;                                                         /* 不可到达端口 */
    UINT32 SendFlag;
};

/* 内存以及杂项配置 */
struct _CH569_CFG
{
    UINT32 RxBufSize;                                                            /* MAC接收缓冲区大小 */
    UINT32 TCPMss;                                                               /* TCP MSS大小 */
    UINT32 HeapSize;                                                             /* 堆分配内存大小 */
    UINT32 ARPTableNum;                                                          /* ARP列表个数 */
    UINT32 MiscConfig0;                                                          /* 其他杂项配置 */
};

/* KEEP LIVE配置结构体 */
struct _KEEP_CFG
{
   UINT32 KLIdle;                                                               /* KEEPLIVE空闲时间 */
   UINT32 KLIntvl;                                                              /* KEEPLIVE周期 */
   UINT32 KLCount;                                                              /* KEEPLIVE次数 */
};

extern UINT8  CH569IPIntStatus;                                    /* 中断状态 */
extern struct _CH569_SYS   CH569Inf;
extern UINT32 CH569NETConfig;                                      /* 库配置，下行说明 */
/* 位0-4 Socket的个数,最大值为31 */
/* 位5-8 MAC 接收描述符的个数，最大值为15 */
/* 位13 PING使能，1为开启PING，0为关闭PING，默认为开启 */
/* 位14-18 TCP重试次数*/
/* 位19-23 TCP重试周期，单位为50毫秒*/
/* 位24 以太网中断类型 */
/* 位25 发送重试配置 */

struct ethernet_if_fn
{
    UINT8(*send_fn)(UINT32 sendlength);
    UINT8* (*get_send_ptr)(void);
    UINT16(*rece_fn)(UINT8**receptr);
};
extern struct ethernet_if_fn ethernet_if;

//#define USE_DHCP                               /* 是否使用DHCP,不使用请不要定义USE_DHCP宏  */

/*------------------------协议栈函数接口---------------------------*/
UINT8 CH569NET_Init( UINT8* ip, UINT8* gwip, UINT8* mask, UINT8* macaddr); /* 库初始化 */

UINT8 CH569NET_GetVer(void);                                                    /* 查询库的版本号 */
#if 1
UINT8 CH569NET_ConfigLIB(struct _CH569_CFG *cfg);                               /* 配置库*/
#endif
void  CH569NET_MainTask(void);                                                  /* 库主任务函数，需要一直不断调用 */

void CH569NET_TimeIsr(UINT16 timperiod);                                        /* 时钟中断服务函数，调用前请配置时钟周期 */

UINT8 CH569NET_QueryGlobalInt(void);                                             /* 查询全局中断 */

UINT8 CH569NET_GetGlobalInt (void);                                             /* 读全局中断并将全局中断清零 */

void CH569NET_OpenMac(void);                                                    /* 打开MAC */

void CH569NET_CloseMac(void);                                                   /* 关闭MAC */

UINT8 CH569NET_SocketCreat(UINT8 *socketid,SOCK_INF *socinf);                   /* 创建socket */

UINT8 CH569NET_SocketSend(UINT8 socketid,UINT8 *buf,UINT32 *len);               /* Socket发送数据 */

UINT8 CH569NET_SocketRecv(UINT8 socketid,UINT8 *buf,UINT32 *len);               /* Socket接收数据 */

UINT8 CH569NET_GetSocketInt(UINT8 sockedid);                                    /* 获取socket中断并清零 */

UINT32 CH569NET_SocketRecvLen(UINT8 socketid,UINT32 *bufaddr);                  /* 获取socket接收长度 */

UINT8 CH569NET_SocketConnect(UINT8 socketid);                                   /* TCP连接*/

UINT8 CH569NET_SocketListen(UINT8 socindex);                                    /* TCP监听 */

UINT8 CH569NET_SocketClose(UINT8 socindex,UINT8 flag);                          /* 关闭连接 */

void CH569NET_ModifyRecvBuf(UINT8 sockeid,UINT32 bufaddr,UINT32 bufsize);       /* 修改接收缓冲区 */

UINT8 CH569NET_SocketUdpSendTo(UINT8 socketid, UINT8 *buf, UINT32 *slen,UINT8 *sip,UINT16 port);/* 向指定的目的IP，端口发送UDP包 */

UINT8 CH569NET_Aton(const UINT8 *cp, UINT8 *addr);                              /* ASCII码地址转网络地址 */

UINT8 *CH569NET_Ntoa(UINT8 *ipaddr);                                            /* 网络地址转ASCII地址 */

UINT8 CH569NET_SetSocketTTL(UINT8 socketid, UINT8 ttl);                         /* 设置socket的TTL */

void CH569NET_RetrySendUnack(UINT8 socketid);                                   /* TCP重传 */

UINT8 CH569NET_QueryUnack(SOCK_INF  *sockinf,UINT32 *addrlist,UINT16 lislen)    /* 查询未发送成功的数据包 */;

UINT8 CH569NET_DHCPStart(UINT8(* usercall)(UINT8 status,void *));               /* DHCP启动 */

UINT8 CH569NET_DHCPStop(void);                                                  /* DHCP停止 */

void CH569NET_ConfigKeepLive(struct _KEEP_CFG *cfg);                            /* 配置库KEEP LIVE参数 */

UINT8 CH569NET_SocketSetKeepLive(UINT8 socindex,UINT8 cfg);                     /* 配置socket KEEP LIVE*/

void frame_input(void);

void reset_net_para(UINT8 *ip,UINT8 *mask,UINT8 *gateway);
#endif

