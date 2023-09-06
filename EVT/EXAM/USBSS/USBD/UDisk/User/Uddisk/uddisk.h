/********************************** (C) COPYRIGHT *******************************
* File Name          : uddisk.h
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef USER_UDDISK_H_
#define USER_UDDISK_H_
#include "CH56x_common.h"

#define MAX_BUF_BLOCK 20
#define TIME_OVER_SET  6000000
extern __attribute__ ((aligned(16))) UINT8 CSW_PARAMETER[13] __attribute__((section(".DMADATA")));
extern EMMC_PARAMETER   TF_EMMCParam;
const uint8_t s03[18];
const uint8_t s1238[56];
const uint8_t s1a03[36];
const uint8_t INQUIRY1[64];
const uint8_t LONG_INQUIRY[255];
const uint8_t INQUIRY32[32];

/* CBW */
struct {
uint32_t  dCBWsignature;
uint32_t  dCBWTag ;                     //CBWCSW check
uint32_t  dCBWDataTransferLength ;      //CBW Data stage length
uint8_t   bCBWFLAGs ;                   //Highest bit:0-out,1-in
uint8_t   bCBWLUN ;                     //Lower 4 bits Destination logical unit number
uint8_t   bCBWCBlength ;                //CBWCB length
uint8_t   CBWCB[15];                    //Executed command
}CBW_PARAMETER;

struct {

UINT8  NULL1;
UINT8  NULL2;
UINT8  NULL3;
UINT8  Cap_List_Length;//The length of the capacity list is 8 bytes after this byte

UINT8   Block_Num_32_24;
UINT8   Block_Num_24_16;
UINT8   Block_Num_16_8;
UINT8   Block_Num_8_0;

UINT8   descriptor_code;

UINT8  blocksize24;
UINT8  blocksize16;
UINT8  blocksize8 ;

}FORMAT_CAPACITIES;

struct {//Structure type requested

UINT8  maxblock32;      //Last logical block address
UINT8  maxblock24 ;
UINT8  maxblock16;
UINT8  maxblock8;

UINT8  blocksize32;     //Logical block size
UINT8  blocksize24;
UINT8  blocksize16;
UINT8  blocksize8;

}READ_CAPACITY;

#define blocksize 512
#define USB_SEQ_MASK        0x1f<<21
#define EOB   1
#define NRDY_TP             0
#define ACK_TP              0x01
#define STALL_TP            0x02
#define INVALID             0x03

#define NUMP_0              0x00
#define NUMP_1              0x01
#define NUMP_2              0x02
#define NUMP_3              0x03
#define NUMP_4              0x04
#define NUMP_5              0x05
#define NUMP_6              0x06

void Uinfo_init( void );
void Ucsw(UINT8 sta);
void csw_err_pres(UINT16 num , UINT16 cnt , UINT8 dir);
void write_start( UINT16 pReqnum, UINT32 Lbaaddr,UINT8 cmden);
void read_start( UINT16 pReqnum, UINT32 Lbaaddr,UINT8 cmden);
void cmd12( void );

#endif /* USER_UDDISK_H_ */
