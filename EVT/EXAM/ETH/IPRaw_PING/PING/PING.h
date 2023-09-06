/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/06/11
* Description        : Definition for PING.c.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __PINC_H__
#define __PINC_H__
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "wchnet.h"

#define ICMP_SOKE_CON                0
#define ICMP_SEND_ERR                1
#define ICMP_SEND_SUC                2
#define ICMP_RECV_ERR                3
#define ICMP_RECV_SUC                4
#define ICMP_UNRECH                  5
#define ICMP_REPLY                   6
#define ICMP_REPLY_SUC               7
#define ICMP_KEEP_NO                 10


#define ICMP_HEAD_TYPE               8
#define ICMP_HEAD_REPLY              0
#define ICMP_HEAD_CODE               0
#define ICMP_HEAD_ID                 512
#define ICMP_HEAD_SEQ                100
#define ICMP_DATA_BYTES              32

#define PING_SEND_CNT                5

/*ICMP header field data structure*/
typedef struct _icmphdr 
{
    uint8_t   i_type;                 //ICMP message type
    uint8_t   i_code;                 //code number in the type
    uint16_t  i_cksum;                //checksum
    uint16_t  i_id;                   //identifier
    uint16_t  i_seq;                  //sequence
    uint8_t   i_data[32];             //data area
}IcmpHeader,*IcmpHead;

extern uint8_t DESIP[4];
extern uint8_t ICMPSuc;

extern void InitPING( void );

extern void InitParameter( void );

extern void WCHNET_PINGCmd( void );

extern void WCHNET_PINGInit( void );

extern void mStopIfError(uint8_t iError);

extern void Respond_PING( uint8_t *pDat );

extern void WCHNET_ICMPRecvData( uint32_t len, uint8_t *pDat );

extern void WCHNET_PINGSendData( uint8_t *PSend, uint32_t Len,uint8_t id );

extern void WCHNET_ProcessReceDat( char *recv_buff,uint8_t check_type,uint8_t socketid );

#endif
