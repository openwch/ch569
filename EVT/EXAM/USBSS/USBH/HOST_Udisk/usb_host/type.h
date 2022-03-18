/********************************** (C) COPYRIGHT *******************************
* File Name          : type.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/


#ifndef __TYPE_H__
#define __TYPE_H__

#ifdef __cplusplus
extern "C" {
#endif


typedef unsigned char       ntfs_u8;
typedef unsigned short      ntfs_u16;
typedef unsigned long       ntfs_u32;
typedef  unsigned long long ntfs_u64;
typedef char                ntfs_s8;
typedef short               ntfs_s16;
typedef long                ntfs_s32;
#define  ntfs_s64	__int64_t

typedef ntfs_s32            ntfs_cluster_t;
typedef ntfs_s16            ntfs_char;


ntfs_u8  disk_read_nt( ntfs_u64 iLbaStart, ntfs_u16 iSectorCount, ntfs_u8 *mBufferPoint );
ntfs_u8  disk_read_ft( ntfs_u32 iLbaStart, ntfs_u16 iSectorCount, ntfs_u8 *mBufferPoint );


#define AA			( ( PUINT8 )0x20034000 )		//8K»º³åÇø
#define BB			( ( PUINT8 )0x20036000 )		//8K»º³åÇø
#define NTFS_ADDR	( ( PUINT8 )0x20030800 )		//1800

#ifdef __cplusplus
}
#endif

#endif

