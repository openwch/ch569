/********************************** (C) COPYRIGHT *******************************
* File Name          : fat_process.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#ifndef UDISK_LIB_FAT_PROCESS_H_
#define UDISK_LIB_FAT_PROCESS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define     MAX_LEN_MCU         32

typedef struct              //数据位对齐
{
    UINT8   Name[ 13 ];             // 短文件名 8+3格式(已经转换过)
    UINT8   attrib;                 // 是文件还是文件夹0--文件，1--文件夹
    UINT32  FileLen;                // 文件长度，文件夹为0
    UINT16  UpdateDate;             // 修改日期
    UINT16  UpdateTime;             // 修改时间
    UINT16  CreateDate;             // 创建日期
    UINT16  CreateTime;             // 创建时间
    UINT16  LongNameLen;            // 存放长文件名 有效长度等于: RecCmd.len - 25
} disk_2;


//#pragma pack(1)
typedef  union
    {
    unsigned char   buf[ MAX_LEN_MCU];  // 存放数据
    disk_2  FileDef;
} DiskData_1;
DiskData_1  DiskData;

extern  UINT8  Fat_Init( void );
extern  unsigned char Enum_Dir( void );
extern  unsigned char Manual_Send_Dir( void );
extern  UINT8 path_name[255],back_path_name[255];

#ifdef __cplusplus
}
#endif

#endif /* UDISK_LIB_FAT_PROCESS_H_ */
