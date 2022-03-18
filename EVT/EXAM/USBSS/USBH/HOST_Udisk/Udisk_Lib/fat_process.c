/********************************** (C) COPYRIGHT *******************************
* File Name          : fat_process.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#include "string.h"
#include "CH56xSFR.h"
#include "core_riscv.h"
#include "CHRV3UFI.h"
#include "fat_process.h"
#include "stdio.h"
#include "type.h"


#define CREAT              1
#define READ               0
#define WRITE              0
#define BYTEWRITE_READ     1
#define MODIFY             1
#define ERASE              0

__attribute__ ((aligned(16))) UINT8 buffer[512] __attribute__((section(".DMADATA"))); //Data sending/receiving buffer of endpoint1

/*******************************************************************************
 * @fn       Fat_Init
 *
 * @return   None
 */
 UINT8  Fat_Init( void )
 {
	UINT8  i,s,c,count;
	UINT8  *pd;
	UINT8  *pd1;
	UINT32 temp32;
	pd = AA;
	pd1 = BB;
	i = CHRV3LibInit();

	printf("LIB_Init=%02x\n",i);
	CHRV3DiskStatus = DISK_MOUNTED;	/* 磁盘已经初始化成功,但是尚未分析文件系统或者文件系统不支持 */
	CHRV3vSectorSize =  512;
	CHRV3vSectorSizeB = 0;
	temp32 = CHRV3vSectorSize;
	for( i = 0; i <= 12; i++ )
	{
		temp32 = temp32 >> 1;
		CHRV3vSectorSizeB++;
		if( ( temp32 & 0x01 )== 0x01 )
		{
			break;
		}
	}

#if CREAT

    strcpy( (char *)&mCmdParam.Create.mPathName,( const char * )"\\NEWFILE.TXT");  //创建236.txt文件
    i = CHRV3FileOpen();
    if((i == ERR_MISS_DIR)||(i == ERR_MISS_FILE))
    {
        printf( "Find No File And Create\r\n" );
        i = CHRV3FileCreate();
    }
    if( i!= ERR_SUCCESS )  return i;

    mCmdParam.Close.mUpdateLen=1;
    i = CHRV3FileClose();                                //修改文件信息完成
    if( i != ERR_SUCCESS )  return i;
#endif


#if WRITE
    printf("write\n");
    strcpy( (char *)&mCmdParam.Open.mPathName,( const char * )"\\NEWFILE.TXT");
    i = CHRV3FileOpen();
    if( i!= ERR_SUCCESS )return i;
    memset(buffer,0x42,512);

    mCmdParam.Write.mDataBuffer = buffer;
    mCmdParam.Write.mSectorCount = 1;
    i=CHRV3FileWrite( );                               //向NEWFILE.txt中写入数据

    if( i != ERR_SUCCESS )  printf("write_error\n");
    for(s=0;s!=5;s++)  printf("%02x ",buffer[s]);
    printf("\n");

    mCmdParam.Close.mUpdateLen = 1;
   i= CHRV3FileClose();                                 //向NEWFILE.txt中写入完成
#endif

#if READ
   printf("read\n");
    strcpy( (char *)&mCmdParam.Open.mPathName,( const char * )"\\NEWFILE.TXT");    //打开文件
    i = CHRV3FileOpen();
    if( i != ERR_SUCCESS ) return i;

    while(1){                                   //读取文件

      mCmdParam.Read.mDataBuffer = pd;
      mCmdParam.Read.mSectorCount = 16;
      i=CHRV3FileRead( );
      if( i != ERR_SUCCESS ) break;

      if( mCmdParam.Read.mSectorCount < 16 )break;

      mCmdParam.Read.mDataBuffer = pd1;
      mCmdParam.Read.mSectorCount = 16;
      i=CHRV3FileRead( );
      if( i != ERR_SUCCESS )break;

      if( mCmdParam.Read.mSectorCount < 16 )break;
      if( i != ERR_SUCCESS ) return i;
    }
    mCmdParam.Close.mUpdateLen = 1;
    i= CHRV3FileClose();
#endif

#if BYTEWRITE_READ
    printf( "ByteWrite\r\n" );
    strcpy( (char *)&mCmdParam.Open.mPathName,( const char * )"\\NEWFILE.TXT");    //打开文件
    i = CHRV3FileOpen();
    if( i != ERR_SUCCESS ) return i;

  //实际应该判断写数据长度和定义缓冲区长度是否相符 如果大于缓冲区长度则需要多次写入

    i = sprintf( (PCHAR)buffer,"Note: \xd\xa这个程序是以字节为单位进行U盘文件读写,简单演示功能。\xd\xa");  /*演示 */
    for(c=0; c<10; c++)
    {
        mCmdParam.ByteWrite.mByteCount = i;                          /* 指定本次写入的字节数 */
        mCmdParam.ByteWrite.mByteBuffer = buffer;                       /* 指向缓冲区 */
        s = CHRV3ByteWrite( );                                       /* 以字节为单位向文件写入数据 */
        if(s != ERR_SUCCESS )
        {
            printf("write error:%02x\n",s);
            return s;
        }
        printf("成功写入 %02X次\r\n",(UINT16)c);
    }
    ///////////二、读取文件前N字节/////////////////////////////////////////
    count = 10;                                                               //设置准备读取总长度100字节
    printf( "读出的前%d个字符是:\r\n",count );
    while ( count ) {                                                        //如果文件比较大,一次读不完,可以再调用CHRV3ByteRead继续读取,文件指针自动向后移动
        if ( count > (MAX_PATH_LEN-1) ) c = MAX_PATH_LEN-1;                          /* 剩余数据较多,限制单次读写的长度不能超过 sizeof( mCmdParam.Other.mBuffer ) */
        else c = count;                                                              /* 最后剩余的字节数 */
        mCmdParam.ByteRead.mByteCount = c;                                           /* 请求读出几十字节数据 */
        mCmdParam.ByteRead.mByteBuffer= &buffer[0];
        s = CHRV3ByteRead( );                                                                                    /* 以字节为单位读取数据块,单次读写的长度不能超过MAX_BYTE_IO,第二次调用时接着刚才的向后读 */
        count -= mCmdParam.ByteRead.mByteCount;                                                                 /* 计数,减去当前实际已经读出的字符数 */
        for ( i=0; i!=mCmdParam.ByteRead.mByteCount; i++ ) printf( "%c ", mCmdParam.ByteRead.mByteBuffer[i] );   /* 显示读出的字符 */
        if ( mCmdParam.ByteRead.mByteCount < c ) {                                                              /* 实际读出的字符数少于要求读出的字符数,说明已经到文件的结尾 */
            printf( "\r\n" );
            printf( "文件已经结束\r\n" );
            break;
        }
    }
    printf( "Close\r\n" );
    i = CHRV3FileClose( );
#endif

#if MODIFY

   strcpy( (char *)&mCmdParam.Open.mPathName,( const char * )"\\NEWFILE.TXT");
   i = CHRV3FileOpen();
   if( i!= ERR_SUCCESS )         return i;

   mCmdParam.Modify.mFileAttr = 0xff;
   mCmdParam.Modify.mFileTime = 0xffff;
   mCmdParam.Modify.mFileDate = MAKE_FILE_DATE(2021, 12, 20);
   mCmdParam.Modify.mFileSize = 0xffffffff;
   i = CHRV3FileModify();                                  //修改文件信息
   if( i!= ERR_SUCCESS ) return i;
   printf("modify=%02x\n",i);

   mCmdParam.Close.mUpdateLen = 1 ;
   i= CHRV3FileClose();                                //修改文件信息完成
   if( i!= ERR_SUCCESS )   return i;

#endif

#if ERASE

     strcpy( (char *)&mCmdParam.Erase.mPathName,( const char * )"\\NEWFILE.TXT");
     i = CHRV3FileErase();                                  //修改文件信息
     if( i!= ERR_SUCCESS )return i;
     printf("erase=%02x\n",i);
#endif
     return i;
 }




