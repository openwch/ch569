/********************************** (C) COPYRIGHT *******************************
* File Name          : SW_UDISK.C
* Author             : WCH
* Version            : V1.00
* Date               : 2021/08/01
* Description        : 模拟U盘(EMMC作为存储介质)部分程序
*******************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "SW_UDISK.h"
#include "CH56x_usb30.h"
#include "CH56x_usb20.h"
#include "CH56xusb30_LIB.h"
/******************************************************************************/
/* 常量、变量定义 */

void EMMC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
EMMC_PARAMETER  TF_EMMCParam;

/******************************************************************************/
/* INQUITY信息 */
uint8_t UDISK_Inquity_Tab[ ] =
{
    /* UDISK */
    0x00,                                                /* Peripheral Device Type：UDISK = 0x00 */
    0x80,                                                /* Removable */
    0x02,                                                /* ISO/ECMA */
    0x02,
    0x1F,                                                /* Additional Length */
    0x00,                                                /* Reserved */
    0x00,                                                /* Reserved */
    0x00,                                                /* Reserved */
    'E',                                                 /* Vendor Information */
    'M',
    'M',
    'C',
    ' ',
    ' ',
    ' ',
    ' ',
    'U',
    'S',
    'B',
    '3',
    '.',
    '0',
    ' ',
    ' ',
    ' ',
    ' ',
    ' ',
    ' ',
    'D',
    'i',
    's',
    'k',
    ' ',
    'W',
    'C',
    'H'
};

/******************************************************************************/
/* 格式化容量信息 */
uint8_t  const  UDISK_Rd_Format_Capacity[ ] =
{
    0x00,
    0x00,
    0x00,
    0x08,
    ( MY_UDISK_SIZE >> 24 ) & 0xFF,
    ( MY_UDISK_SIZE >> 16 ) & 0xFF,
    ( MY_UDISK_SIZE >> 8 ) & 0xFF,
    ( MY_UDISK_SIZE ) & 0xFF,
    0x02,
    ( DEF_CFG_DISK_SEC_SIZE >> 16 ) & 0xFF,             /* Number of Blocks */
    ( DEF_CFG_DISK_SEC_SIZE >> 8 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE ) & 0xFF,
};

/******************************************************************************/
/* 容量信息 */
uint8_t  const  UDISK_Rd_Capacity[ ] =
{
    ( ( MY_UDISK_SIZE - 1 ) >> 24 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 ) >> 16 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 ) >> 8 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 ) ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 24 ) & 0xFF,             /* Number of Blocks */
    ( DEF_CFG_DISK_SEC_SIZE >> 16 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 8 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE ) & 0xFF,
};

/******************************************************************************/
/* MODE_SENSE信息,对于CMD 0X1A */
uint8_t  const  UDISK_Mode_Sense_1A[ ] =
{
    0x0B,
    0x00,
    0x00,                                               /* 0x00:write-unprotected,0x80:write-protected */
    0x08,
    ( ( MY_UDISK_SIZE - 1 ) >> 24 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 )  >> 16 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 )  >> 8 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 )  ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 24 ) & 0xFF,             /* Number of Blocks */
    ( DEF_CFG_DISK_SEC_SIZE >> 16 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 8 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE ) & 0xFF,
};

/******************************************************************************/
/* MODE_SENSE信息,对于CMD 0X5A */
uint8_t  const  UDISK_Mode_Senese_5A[ ] =
{
    0x00,
    0x0E,
    0x00,
    0x00,                                              /* 0x00:write-unprotected,0x80:write-protected */
    0x00,
    0x00,
    0x00,
    0x08,
    ( ( MY_UDISK_SIZE - 1 ) >> 24 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 )  >> 16 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 )  >> 8 ) & 0xFF,
    ( ( MY_UDISK_SIZE - 1 )  ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 24 ) & 0xFF,            /* Number of Blocks */
    ( DEF_CFG_DISK_SEC_SIZE >> 16 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE >> 8 ) & 0xFF,
    ( DEF_CFG_DISK_SEC_SIZE ) & 0xFF,
};


volatile uint8_t  Udisk_Status = 0x00;                                                              /* U盘相关状态 */
volatile uint8_t  Udisk_Transfer_Status = 0x00;                                                     /* U盘传输相关状态 */
volatile uint32_t Udisk_Capability = 0x00;                                                          /* U盘格式化容量 */
volatile uint8_t  Udisk_CBW_Tag_Save[ 4 ];
volatile uint8_t  Udisk_Sense_Key = 0x00;
volatile uint8_t  Udisk_Sense_ASC = 0x00;
volatile uint8_t  Udisk_CSW_Status = 0x00;

volatile uint32_t UDISK_Transfer_DataLen = 0x00;
volatile uint32_t UDISK_Cur_Sec_Lba = 0x00;
volatile uint16_t UDISK_Sec_Pack_Count = 0x00;
volatile uint16_t UDISK_Pack_Size = DEF_UDISK_PACK_512;

volatile uint8_t UDISK_OutPackflag = 0;
volatile uint8_t UDISK_InPackflag = 0;

BULK_ONLY_CMD mBOC;
uint8_t   *pEndp2_Buf;


/*******************************************************************************
* Function Name  : UDISK_CMD_Deal_Status
* Description    : 当前U盘命令执行状态
* Input          : key------磁盘错误细节信息的主键
*                  asc------磁盘错误细节信息的次键
*                  status---当前命令执行结果状态
* Output         : None
* Return         : None
*******************************************************************************/
void UDISK_CMD_Deal_Status( uint8_t key, uint8_t asc, uint8_t status )
{
    Udisk_Sense_Key  = key;
    Udisk_Sense_ASC  = asc;
    Udisk_CSW_Status = status;
}

/*******************************************************************************
* Function Name  : UDISK_CMD_Deal_Fail
* Description    : 当前U盘命令执行失败
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UDISK_CMD_Deal_Fail( void )
{
    if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_UP_FLAG )
    {
        /* 端点2上传返回STALL */
        if( Link_Sta == LINK_STA_1 )
        {
            R8_UEP2_TX_CTRL = (R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_STALL;
        }
        else
        {
            USB30_IN_Set(ENDP_2, ENABLE, STALL, 0, 0);
        }
        Udisk_Transfer_Status &= ~DEF_UDISK_BLUCK_UP_FLAG;
    }
    if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_DOWN_FLAG )
    {
        /* 端点3下传返回STALL */
        if( Link_Sta == LINK_STA_1 )
        {
            R8_UEP3_RX_CTRL = (R8_UEP3_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_STALL;
        }
        else
        {
            USB30_OUT_Set(ENDP_3, STALL, 0);
        }
        Udisk_Transfer_Status &= ~DEF_UDISK_BLUCK_DOWN_FLAG;
    }
}

/*******************************************************************************
* Function Name  : CMD_RD_WR_Deal_Pre
* Description    : 读写扇区处理前的准备
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CMD_RD_WR_Deal_Pre( void )
{
    /* 保存当前要操作的扇区号 */
    UDISK_Cur_Sec_Lba = (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 2 ] << 24;
    UDISK_Cur_Sec_Lba = UDISK_Cur_Sec_Lba + ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 3 ] << 16 );
    UDISK_Cur_Sec_Lba = UDISK_Cur_Sec_Lba + ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 4 ] << 8 );
    UDISK_Cur_Sec_Lba = UDISK_Cur_Sec_Lba + ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 5 ] );
        
    /* 保存当前要操作的数据长度 */                    
    UDISK_Transfer_DataLen = ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 7 ] << 8 );
    UDISK_Transfer_DataLen = UDISK_Transfer_DataLen + ( (uint32_t)mBOC.mCBW.mCBW_CB_Buf[ 8 ] );
    UDISK_Transfer_DataLen = UDISK_Transfer_DataLen * DEF_UDISK_SECTOR_SIZE;         

    /* 清相关变量 */
    UDISK_Sec_Pack_Count = 0x00;
    UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
}

/*******************************************************************************
* Function Name  : UDISK_SCSI_CMD_Deal
* Description    : 
* Input          : 
* Output         : 
* Return         : None
*******************************************************************************/
void UDISK_SCSI_CMD_Deal( void )
{
    uint8_t i;

    if( ( mBOC.mCBW.mCBW_Sig[ 0 ] == 'U' ) && ( mBOC.mCBW.mCBW_Sig[ 1 ] == 'S' ) 
      &&( mBOC.mCBW.mCBW_Sig[ 2 ] == 'B' ) && ( mBOC.mCBW.mCBW_Sig[ 3 ] == 'C' ) )
    {
        Udisk_CBW_Tag_Save[ 0 ] = mBOC.mCBW.mCBW_Tag[ 0 ];
        Udisk_CBW_Tag_Save[ 1 ] = mBOC.mCBW.mCBW_Tag[ 1 ];
        Udisk_CBW_Tag_Save[ 2 ] = mBOC.mCBW.mCBW_Tag[ 2 ];
        Udisk_CBW_Tag_Save[ 3 ] = mBOC.mCBW.mCBW_Tag[ 3 ];

        UDISK_Transfer_DataLen = ( uint32_t )mBOC.mCBW.mCBW_DataLen[ 3 ] << 24;
        UDISK_Transfer_DataLen += ( ( uint32_t )mBOC.mCBW.mCBW_DataLen[ 2 ] << 16 );
        UDISK_Transfer_DataLen += ( ( uint32_t )mBOC.mCBW.mCBW_DataLen[ 1 ] << 8 );
        UDISK_Transfer_DataLen += ( ( uint32_t )mBOC.mCBW.mCBW_DataLen[ 0 ] );
        
        if( UDISK_Transfer_DataLen )                                     
        {
            if( mBOC.mCBW.mCBW_Flag & 0x80 )
            {
                Udisk_Transfer_Status |= DEF_UDISK_BLUCK_UP_FLAG;
            }    
            else
            {    
                Udisk_Transfer_Status |= DEF_UDISK_BLUCK_DOWN_FLAG;
            }
        }
        Udisk_Transfer_Status |= DEF_UDISK_CSW_UP_FLAG;

        /* U盘SCSI命令包处理 */ 
        switch( mBOC.mCBW.mCBW_CB_Buf[ 0 ] )
        {
            case  CMD_U_INQUIRY:                                                                    
                /* CMD: 0x12 */
                if( UDISK_Transfer_DataLen > 0x24 )
                {
                    UDISK_Transfer_DataLen = 0x24;
                }    

                /* UDISK模式 */
                UDISK_Inquity_Tab[ 0 ] = 0x00;
                pEndp2_Buf = (uint8_t *)UDISK_Inquity_Tab;
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;
    
            case  CMD_U_READ_FORMAT_CAPACITY:                                     
                /* CMD: 0x23 */
                if( ( Udisk_Status & DEF_UDISK_EN_FLAG ) )
                {    
                    if( UDISK_Transfer_DataLen > 0x0C )
                    {
                        UDISK_Transfer_DataLen = 0x0C; 
                    }    
                                    
                    for( i = 0x00; i < UDISK_Transfer_DataLen; i++ )
                    {
                        mBOC.buf[ i ] = UDISK_Rd_Format_Capacity[ i ];
                    } 
                    mBOC.buf[ 4 ] = ( ( Udisk_Capability >> 24 ) & 0xFF );
                    mBOC.buf[ 5 ] = ( ( Udisk_Capability >> 16 ) & 0xFF );
                    mBOC.buf[ 6 ] = ( ( Udisk_Capability >> 8  ) & 0xFF );
                    mBOC.buf[ 7 ] = ( ( Udisk_Capability       ) & 0xFF );
                    pEndp2_Buf = mBOC.buf;   
                    UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }    
                break;

            case  CMD_U_READ_CAPACITY:                                              
                /* CMD: 0x25 */
                if( ( Udisk_Status & DEF_UDISK_EN_FLAG ) )  
                {    
                    if( UDISK_Transfer_DataLen > 0x08 )
                    {
                        UDISK_Transfer_DataLen = 0x08;
                    }    
                                                
                    for( i = 0x00; i < UDISK_Transfer_DataLen; i++ )
                    {
                        mBOC.buf[ i ] = UDISK_Rd_Capacity[ i ];    
                    } 
                    mBOC.buf[ 0 ] = ( ( Udisk_Capability - 1 ) >> 24 ) & 0xFF;
                    mBOC.buf[ 1 ] = ( ( Udisk_Capability - 1 ) >> 16 ) & 0xFF;
                    mBOC.buf[ 2 ] = ( ( Udisk_Capability - 1 ) >> 8  ) & 0xFF;
                    mBOC.buf[ 3 ] = ( ( Udisk_Capability - 1 )       ) & 0xFF;

                    pEndp2_Buf = mBOC.buf;     
                    UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 ); 
                    UDISK_CMD_Deal_Fail( ); 
                }    
                break;

            case  CMD_U_READ10:
                /* CMD: 0x28 */
                if( ( Udisk_Status & DEF_UDISK_EN_FLAG ) )
                {                    
                    CMD_RD_WR_Deal_Pre( );
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }    
                break;
    
            case  CMD_U_WR_VERIFY10:                                             
                /* CMD: 0x2E */
            case  CMD_U_WRITE10:                                                
                /* CMD: 0x2A */
                if( Udisk_Status & DEF_UDISK_EN_FLAG )
                {
                    CMD_RD_WR_Deal_Pre( );
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }                
                break;
    
            case  CMD_U_MODE_SENSE:                                                 
                /* CMD: 0x1A */
                if( ( Udisk_Status & DEF_UDISK_EN_FLAG ) )
                {    
                    if( UDISK_Transfer_DataLen > 0x0C )
                    {
                        UDISK_Transfer_DataLen = 0x0C;
                    }
                    for( i = 0x00; i < UDISK_Transfer_DataLen; i++ )
                    {
                        mBOC.buf[ i ] = UDISK_Mode_Sense_1A[ i ];    
                    } 
                    mBOC.buf[ 4 ] = ( Udisk_Capability >> 24 ) & 0xFF;
                    mBOC.buf[ 5 ] = ( Udisk_Capability >> 16 ) & 0xFF;
                    mBOC.buf[ 6 ] = ( Udisk_Capability >> 8  ) & 0xFF;
                    mBOC.buf[ 7 ] = ( Udisk_Capability       ) & 0xFF;
                    pEndp2_Buf = mBOC.buf;                
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }
                break;

            case  CMD_U_MODE_SENSE2:                                             
                /* CMD: 0x5A */
                if( mBOC.mCBW.mCBW_CB_Buf[ 2 ] == 0x3F )
                {    
                    if( UDISK_Transfer_DataLen > 0x10 )
                    {
                        UDISK_Transfer_DataLen = 0x10;
                    }

                    for( i = 0x00; i < UDISK_Transfer_DataLen; i++ )
                    {
                        mBOC.buf[ i ] = UDISK_Mode_Senese_5A[ i ];    
                    } 
                    mBOC.buf[ 8 ]  = ( Udisk_Capability >> 24 ) & 0xFF;
                    mBOC.buf[ 9 ]  = ( Udisk_Capability >> 16 ) & 0xFF;
                    mBOC.buf[ 10 ] = ( Udisk_Capability >> 8  ) & 0xFF;
                    mBOC.buf[ 11 ] = ( Udisk_Capability       ) & 0xFF;
                    pEndp2_Buf = mBOC.buf;         
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x05, 0x20, 0x01 );
                    UDISK_CMD_Deal_Fail( );
                }
                break;
    
            case  CMD_U_REQUEST_SENSE:                                              
                /* CMD: 0x03 */
                mBOC.ReqSense.ErrorCode = 0x70;
                mBOC.ReqSense.Reserved1 = 0x00;
                mBOC.ReqSense.SenseKey  = Udisk_Sense_Key;
                mBOC.ReqSense.Information[ 0 ] = 0x00;
                mBOC.ReqSense.Information[ 1 ] = 0x00;
                mBOC.ReqSense.Information[ 2 ] = 0x00;
                mBOC.ReqSense.Information[ 3 ] = 0x00;
                mBOC.ReqSense.SenseLength = 0x0A;
                mBOC.ReqSense.Reserved2[ 0 ] = 0x00;
                mBOC.ReqSense.Reserved2[ 1 ] = 0x00;
                mBOC.ReqSense.Reserved2[ 2 ] = 0x00;
                mBOC.ReqSense.Reserved2[ 3 ] = 0x00;
                mBOC.ReqSense.SenseCode = Udisk_Sense_ASC;
                mBOC.ReqSense.SenseCodeQua = 0x00;
                mBOC.ReqSense.Reserved3[ 0 ] = 0x00;
                mBOC.ReqSense.Reserved3[ 1 ] = 0x00;
                mBOC.ReqSense.Reserved3[ 2 ] = 0x00;
                mBOC.ReqSense.Reserved3[ 3 ] = 0x00;
                pEndp2_Buf = mBOC.buf;
                Udisk_CSW_Status = 0x00;
                break;
            case  CMD_U_TEST_READY:       
                if( Udisk_Status & DEF_UDISK_EN_FLAG )
                {    
                    UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                }
                else
                {
                    UDISK_CMD_Deal_Status( 0x02, 0x3A, 0x01 ); 
                    Udisk_Transfer_Status |= DEF_UDISK_BLUCK_UP_FLAG;   
                    UDISK_CMD_Deal_Fail( ); 
                }
                break;
    
            case  CMD_U_PREVT_REMOVE:                                             
                /* CMD: 0x1E */
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;
    
            case  CMD_U_VERIFY10:                                                  
                /* CMD: 0x1F */
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;
                
            case  CMD_U_START_STOP:                                                  
                /* CMD: 0x1B */
                UDISK_CMD_Deal_Status( 0x00, 0x00, 0x00 );
                break;

            default:
                UDISK_CMD_Deal_Status( 0x05, 0x20, 0x01 );
                Udisk_Transfer_Status |= DEF_UDISK_BLUCK_UP_FLAG; 
                UDISK_CMD_Deal_Fail( );
                break;
        }
    }    
    else                                                                         
    {   /* CBW包的包标志出错 */
        UDISK_CMD_Deal_Status( 0x05, 0x20, 0x02 );
        Udisk_Transfer_Status |= DEF_UDISK_BLUCK_UP_FLAG;
        Udisk_Transfer_Status |= DEF_UDISK_BLUCK_DOWN_FLAG;
        UDISK_CMD_Deal_Fail(  );
    }
}

/*******************************************************************************
* Function Name  : UDISK_In_EP_Deal
* Description    : U盘上传端点处理
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UDISK_In_EP_Deal( void )
{        
    if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_UP_FLAG ) 
    {
        if( mBOC.mCBW.mCBW_CB_Buf[ 0 ] == CMD_U_READ10 )
        {
//            UDISK_Up_OnePack( );
            UDISK_InPackflag = 1;
        }
        else
        {
            UDISK_Bulk_UpData( );
        }
    }
    else if( Udisk_Transfer_Status & DEF_UDISK_CSW_UP_FLAG )
    {    
        UDISK_Up_CSW( );
    }
}

/*******************************************************************************
* Function Name  : UDISK_Out_EP_Deal
* Description    : U盘下传端点处理
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UDISK_Out_EP_Deal( uint8_t *pbuf, uint16_t packlen )
{
    uint32_t i;
    /* 进行端点2下传数据处理 */
    if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_DOWN_FLAG )
    {
//        /* 处理下传的USB数据包 */
//        UDISK_Down_OnePack( );
        UDISK_OutPackflag = 1;
    }
    else
    {                                
        if( packlen == 0x1F )
        {
            for( i = 0; i < packlen; i++ ) 
            {
                mBOC.buf[ i ] = *pbuf++;
            }

            /* 进行SCSI命令包的处理 */
            UDISK_SCSI_CMD_Deal( );
            if( ( Udisk_Transfer_Status & DEF_UDISK_BLUCK_DOWN_FLAG ) == 0x00 )
            {
                if( Udisk_Transfer_Status & DEF_UDISK_BLUCK_UP_FLAG )
                {
                    if( mBOC.mCBW.mCBW_CB_Buf[ 0 ] == CMD_U_READ10 )
                    {
//                        UDISK_Up_OnePack( );
                        UDISK_InPackflag = 1;
                    }
                    else
                    {
                        UDISK_Bulk_UpData( );
                    }
                }
                else if( Udisk_CSW_Status == 0x00 )
                {
                    /* 上传CSW包 */
                    UDISK_Up_CSW(  );                     
                }                        
            }
        }
    }
}

/*******************************************************************************
* Function Name  : UDISK_Bulk_UpData
* Description    : 批量端点端点2上传数据包
* Input          : Transfer_DataLen--- 传输的数据长度
*                  *pBuf---数据地址指针
* Output         : Transfer_DataLen--- 传输的数据长度
*                  *pBuf---数据地址指针
* Return         : None
*******************************************************************************/
void UDISK_Bulk_UpData( void )
{                                            
    uint32_t  len;

    if( UDISK_Transfer_DataLen > UDISK_Pack_Size )
    {
        len = UDISK_Pack_Size;
        UDISK_Transfer_DataLen -= UDISK_Pack_Size;
    }
    else
    {
        len = UDISK_Transfer_DataLen;
        UDISK_Transfer_DataLen = 0x00;
        Udisk_Transfer_Status &= ~DEF_UDISK_BLUCK_UP_FLAG;        
    }

    /* 将数据装载进上传缓冲区中,并启动上传 */
    if( Link_Sta == LINK_STA_1 )
    {
        memcpy(UDisk_In_Buf,pEndp2_Buf,len);
        R16_UEP2_T_LEN = len;
        R32_UEP2_TX_DMA = (UINT32)(UINT8 *)UDisk_In_Buf;
        R8_UEP2_TX_CTRL = (R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
    }
    else
    {

        memcpy(UDisk_In_Buf,pEndp2_Buf,len);
        USBSS->UEP2_TX_DMA = (UINT32)(UINT8 *)UDisk_In_Buf;
        USB30_IN_Set(ENDP_2, ENABLE, ACK, DEF_ENDP2_IN_BURST_LEVEL, len);
        USB30_Send_ERDY(ENDP_2 | IN, DEF_ENDP2_IN_BURST_LEVEL);
    }
}

/*******************************************************************************
* Function Name  : UDISK_Up_CSW
* Description    : 批量端点端点2上传CSW包
* Input          : CBW_Tag_Save---命令块标签
*                  CSW_Status---当前命令执行结果状态
* Output         : Bit_DISK_CSW---尚未上传过CSW包标志
*                  Bit_DISK_Bluck_Up---数据上传标志
*                  Bit_DISK_Bluck_Down---数据下传标志
* Return         : None
*******************************************************************************/
void UDISK_Up_CSW( void )
{
    Udisk_Transfer_Status = 0x00;

    mBOC.mCSW.mCSW_Sig[ 0 ] = 'U';
    mBOC.mCSW.mCSW_Sig[ 1 ] = 'S';
    mBOC.mCSW.mCSW_Sig[ 2 ] = 'B';
    mBOC.mCSW.mCSW_Sig[ 3 ] = 'S';
    mBOC.mCSW.mCSW_Tag[ 0 ] = Udisk_CBW_Tag_Save[ 0 ];
    mBOC.mCSW.mCSW_Tag[ 1 ] = Udisk_CBW_Tag_Save[ 1 ];
    mBOC.mCSW.mCSW_Tag[ 2 ] = Udisk_CBW_Tag_Save[ 2 ];
    mBOC.mCSW.mCSW_Tag[ 3 ] = Udisk_CBW_Tag_Save[ 3 ];
    mBOC.mCSW.mCSW_Residue[ 0 ] = 0x00;
    mBOC.mCSW.mCSW_Residue[ 1 ] = 0x00;
    mBOC.mCSW.mCSW_Residue[ 2 ] = 0x00;
    mBOC.mCSW.mCSW_Residue[ 3 ] = 0x00;
    mBOC.mCSW.mCSW_Status = Udisk_CSW_Status;

    /* 将数据装载进上传缓冲区中,并启动上传 */
    if( Link_Sta == LINK_STA_1 )
    {
        memcpy(UDisk_In_Buf,(UINT8 *)mBOC.buf,0x0D);
        R16_UEP2_T_LEN = 0x0D;
        R32_UEP2_TX_DMA = (UINT32)(UINT8 *)UDisk_In_Buf;
        R8_UEP2_TX_CTRL = (R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
    }
    else
    {
        memcpy(UDisk_In_Buf,(UINT8 *)mBOC.buf,0x0D);
        USBSS->UEP2_TX_DMA = (UINT32)(UINT8 *)UDisk_In_Buf;
        USB30_IN_Set(ENDP_2, ENABLE, ACK, DEF_ENDP2_IN_BURST_LEVEL, 0x0D);
        USB30_Send_ERDY(ENDP_2 | IN, DEF_ENDP2_IN_BURST_LEVEL);
    }
}

/*******************************************************************************
* Function Name  : UDISK_onePack_Deal
*
* Description    : 批量端点端点传输数据包处理-保证传输速度，关闭usb中断，emmc与usb同时传输处理
*
* Return         : None
*******************************************************************************/
void UDISK_onePack_Deal( void )
{
    if( UDISK_InPackflag == 1 )
    {
        UDISK_InPackflag = 0;
        UDISK_Up_OnePack();
    }

    if( UDISK_OutPackflag == 1 )
    {
        UDISK_OutPackflag = 0;
        UDISK_Down_OnePack();

        if( Link_Sta == LINK_STA_1 )
        {
            R8_UEP3_RX_CTRL = (R8_UEP3_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;
        }
        else
        {
            USB30_OUT_Set(ENDP_3, ACK, DEF_ENDP3_OUT_BURST_LEVEL);
            USB30_Send_ERDY(ENDP_3 | OUT, DEF_ENDP3_OUT_BURST_LEVEL);
        }
    }
}

/*******************************************************************************
* Function Name  : UDISK_Up_OnePack
* Description    : USB存储设备上传一包数据
*                  该函数在U盘读扇区命令时使用,每次上传一包数据后,根据需要在中断中在
*                  考虑是否继续上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UDISK_Up_OnePack( void )
{
    /* 判断是否需要发起读扇区命令 */
    UINT8 *pbuf = NULL;
    UINT16 preqnum = 0 ;
    UINT8 s;
    UINT32 cmd_arg_val;
    UINT16 cmd_set_val;
    UINT16 sdtran = 0, usbtran = 0;
    UINT8 sdstep=0, usbstep=0;
    UINT8 lock = 0, flag = 1;
    UINT8 uep0rxsave = R8_UEP0_RX_CTRL;
    UINT8 uep0txsave = R8_UEP0_TX_CTRL;

    /* USB上传本包数据 */
    if( Link_Sta == LINK_STA_1 )
    {
        PFIC_DisableIRQ(USBHS_IRQn);

        R8_UEP0_TX_CTRL = UEP_T_RES_NAK ;
        R8_UEP0_RX_CTRL = UEP_R_RES_NAK ;

        preqnum = UDISK_Transfer_DataLen/512;
        UDISK_Transfer_DataLen = 0;

        /* sec 1 */
        R32_EMMC_DMA_BEG1 = (UINT32)UDisk_In_Buf;                                   //data buffer address
        R32_EMMC_TRAN_MODE = (1<<4)|(1<<1);                                         //EMMC to controller
        R32_EMMC_BLOCK_CFG = (512)<<16 | preqnum;

        //cmd18
        cmd_arg_val = UDISK_Cur_Sec_Lba;
        cmd_set_val = RB_EMMC_CKIDX |
                      RB_EMMC_CKCRC |
                      RESP_TYPE_48  |
                      EMMC_CMD18;
        EMMCSendCmd(cmd_arg_val, cmd_set_val);

        while( 1 )
        {

            if( ( usbtran < (sdtran-1) ) &&  ( (R8_USB_INT_FG & RB_USB_IF_TRANSFER) || flag ) )
            {
                flag = 0;
                R8_UEP2_TX_CTRL = (R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_NAK;
                R32_UEP2_TX_DMA = (UINT32)(UINT8 *)( UDisk_In_Buf + usbstep * 512 );
                R16_UEP2_T_LEN = 512;
                R8_UEP2_TX_CTRL = (R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK ;

                R8_USB_INT_FG = RB_USB_IF_TRANSFER;

                usbtran++;
                usbstep++;
                if( usbstep == UDISKSIZE/512 )
                {
                    usbstep = 0;
                }
                if( lock )
                {
                    lock = 0;
                    R32_EMMC_TRAN_MODE = (UINT32)(1<<4);
                }
            }

            if( R8_USB_INT_FG & RB_USB_IF_SETUOACT )                   //SETUP interrupt
            {
                R8_USB_INT_FG = RB_USB_IF_SETUOACT;              // clear int flag
            }

            if( R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP )
            {
                R16_EMMC_INT_FG = RB_EMMC_IF_BKGAP;
                sdtran++;
                sdstep++;
                if( sdstep == UDISKSIZE/512 )
                {
                    sdstep = 0;
                }
                R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)(UDisk_In_Buf + sdstep*512);
                if( (sdtran-usbtran) < ((UDISKSIZE/512)-2) )
                {
                    R32_EMMC_TRAN_MODE = (UINT32)(1<<4);
                }
                else
                {
                    lock = 1;
                }
            }
            else if( R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE )
            {
                R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE | RB_EMMC_IF_CMDDONE;
                sdtran++;
                sdstep++;
                break;
            }
        }

        while( 1 )
        {
            if( ( R8_USB_INT_FG & RB_USB_IF_TRANSFER ) || flag  )
            {
                flag = 0;

                R8_UEP2_TX_CTRL = (R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_NAK;
                R32_UEP2_TX_DMA = (UINT32)(UINT8 *)( UDisk_In_Buf + usbstep * 512 );
                R16_UEP2_T_LEN = 512;
                R8_UEP2_TX_CTRL = (R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK ;

                R8_USB_INT_FG = RB_USB_IF_TRANSFER;

                usbtran++;
                usbstep++;
                if( usbstep == UDISKSIZE/512 )
                {
                    usbstep = 0;
                }

                if( usbtran == sdtran )
                {
                    while( !(R8_USB_INT_FG & RB_USB_IF_TRANSFER) );
                    R8_UEP2_TX_CTRL = (R8_UEP2_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_NAK;
                    R8_USB_INT_FG = RB_USB_IF_TRANSFER;
                    break;
                }

            }

            if( R8_USB_INT_FG & RB_USB_IF_SETUOACT )                   //SETUP interrupt
            {
                R8_USB_INT_FG = RB_USB_IF_SETUOACT;              // clear int flag
            }
        }

        R32_EMMC_TRAN_MODE = 0;
        //cmd12
        cmd_arg_val = 0;
        cmd_set_val = RB_EMMC_CKIDX |
                      RB_EMMC_CKCRC |
                      RESP_TYPE_R1b |
                      EMMC_CMD12;
        EMMCSendCmd(cmd_arg_val, cmd_set_val);
        while(1)
        {
            s = CheckCMDComp( &TF_EMMCParam );
            if( s != CMD_NULL ) break;
        }
        R16_EMMC_INT_FG = 0xffff;

        PFIC_EnableIRQ(USBHS_IRQn);
        R8_UEP0_TX_CTRL = uep0txsave ;
        R8_UEP0_RX_CTRL = uep0rxsave ;
    }
    else
    {
        PFIC_DisableIRQ(USBSS_IRQn);

        preqnum = UDISK_Transfer_DataLen/512;
        UDISK_Transfer_DataLen = 0;

        /* sec 1 */
        R32_EMMC_DMA_BEG1 = (UINT32)UDisk_In_Buf;                                   //data buffer address
        R32_EMMC_TRAN_MODE = (1<<4)|(1<<1);                                                     //EMMC to controller
        R32_EMMC_BLOCK_CFG = (512)<<16 | preqnum;

        //cmd18
        cmd_arg_val = UDISK_Cur_Sec_Lba;
        cmd_set_val = RB_EMMC_CKIDX |
                      RB_EMMC_CKCRC |
                      RESP_TYPE_48  |
                      EMMC_CMD18;
        EMMCSendCmd(cmd_arg_val, cmd_set_val);

        /* sec 2 */
        while(1)
        {
            if( ( usbtran < (sdtran-1) ) && ( ( USBSS->UEP2_TX_CTRL & (1<<31) ) || flag ) )
            {
                USB30_IN_ClearIT( ENDP_2);
                USBSS->UEP2_TX_DMA = (UINT32)(UINT8 *)( UDisk_In_Buf + usbstep * 512 );
                USB30_IN_Set(ENDP_2,ENABLE,ACK,1,1024);
                USB30_Send_ERDY( ENDP_2 | IN, 1 );
                usbtran+=2;
                usbstep+=2;
                if( usbstep == UDISKSIZE/512 )
                {
                    usbstep = 0;
                }
                flag = 0;
                if( lock )
                {
                    lock = 0;
                    R32_EMMC_TRAN_MODE = (UINT32)(1<<4);
                }
            }

            if( R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP )
            {  //Block transfer completed
                R16_EMMC_INT_FG = RB_EMMC_IF_BKGAP;

                sdtran++;
                sdstep++;
                if( sdstep == UDISKSIZE/512 )
                {
                    sdstep = 0;
                }

                R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)(UDisk_In_Buf + sdstep*512);
                if((sdtran-usbtran)<((UDISKSIZE/512)-2))
                {
                    R32_EMMC_TRAN_MODE = (UINT32)(1<<4);
                }
                else
                {
                    lock = 1;
                }
            }
            else if( R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE )
            {
                R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE | RB_EMMC_IF_CMDDONE;
                sdtran++;
                sdstep++;
                break;
            }
        }

        while( 1 )
        {
            if(( USBSS->UEP2_TX_CTRL & (1<<31) )|| flag )
            {
                flag = 0;
                USB30_IN_ClearIT( ENDP_2);
                if((sdtran-usbtran) >1)
                {
                    USBSS->UEP2_TX_DMA = (UINT32)(UINT8 *)( UDisk_In_Buf + usbstep * 512 );
                    USB30_IN_Set(ENDP_2,ENABLE,ACK,1,1024);
                    USB30_Send_ERDY( ENDP_2 | IN, 1 );
                    usbtran+=2;
                    usbstep+=2;
                }
                else
                {
                    USBSS->UEP2_TX_DMA = (UINT32)(UINT8 *)( UDisk_In_Buf + usbstep * 512 );
                    USB30_IN_Set(ENDP_2,ENABLE,ACK,1,512);
                    USB30_Send_ERDY( ENDP_2 | IN, 1 );
                    usbtran++;
                    usbstep++;
                }
                if( usbstep == UDISKSIZE/512 )
                {
                    usbstep = 0;
                }
                if( usbtran == sdtran )
                {
                    while(!(USBSS->UEP2_TX_CTRL & (1<<31)));
                    USB30_IN_ClearIT( ENDP_2);
                    break;
                }
            }
        }

        R32_EMMC_TRAN_MODE = 0;
        //cmd12
        cmd_arg_val = 0;
        cmd_set_val = RB_EMMC_CKIDX |
                      RB_EMMC_CKCRC |
                      RESP_TYPE_R1b |
                      EMMC_CMD12;
        EMMCSendCmd(cmd_arg_val, cmd_set_val);
        while(1)
        {
            s = CheckCMDComp( &TF_EMMCParam );
            if( s != CMD_NULL ) break;
        }
        R16_EMMC_INT_FG = 0xffff;

        PFIC_EnableIRQ(USBSS_IRQn);
    }

    if( UDISK_Transfer_DataLen == 0 )
    {
        UDISK_Up_CSW( );
    }
    UDISK_Cur_Sec_Lba += preqnum;

    /* 判断数据是否全部上传完毕 */    
     if( UDISK_Transfer_DataLen == 0x00 )
    {    
        Udisk_Transfer_Status &= ~DEF_UDISK_BLUCK_UP_FLAG;
    }
}

/*******************************************************************************
* Function Name  : UDISK_Down_OnePack
* Description    : USB存储设备处理一包下传数据
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UDISK_Down_OnePack( void )
{
    uint8_t s;
    uint8_t nump;
    uint16_t len;
    uint16_t preqnum;
    UINT32 cmd_arg_val;
    UINT16 cmd_set_val;
    UINT16  sdtran = 0, usbtran = 0;
    UINT8   sdstep=0, usbstep=0;
    UINT8  flag = 0 ,full = 0;
    UINT16 ret_len = 0;
    UINT8 uep0rxsave = R8_UEP0_RX_CTRL;
    UINT8 uep0txsave = R8_UEP0_TX_CTRL;

    if( Link_Sta == LINK_STA_1 )
    {
        PFIC_DisableIRQ(USBHS_IRQn);
        R8_UEP0_TX_CTRL = UEP_T_RES_NAK ;
        R8_UEP0_RX_CTRL = UEP_R_RES_NAK ;

        preqnum = UDISK_Transfer_DataLen/512;

        UDISK_Transfer_DataLen = 0;

        //cmd25
        cmd_arg_val = UDISK_Cur_Sec_Lba;
        cmd_set_val = RB_EMMC_CKIDX |
                      RB_EMMC_CKCRC |
                      RESP_TYPE_48  |
                      EMMC_CMD25;
        EMMCSendCmd(cmd_arg_val, cmd_set_val);
        while(1)
        {
            s = CheckCMDComp( &TF_EMMCParam );
            if( s != CMD_NULL ) break;
        }

        R32_EMMC_TRAN_MODE = RB_EMMC_DMA_DIR |(1<<6);
        R32_EMMC_DMA_BEG1 = (UINT32)UDisk_Out_Buf;
        R32_EMMC_BLOCK_CFG = 512<<16 | preqnum;

        usbtran += 1;
        usbstep += 1;

        R32_UEP3_RX_DMA = (UINT32)(UINT8 *)(UDisk_Out_Buf + usbstep*512);
        while(1)
        {
            if( R8_USB_INT_FG & RB_USB_IF_SETUOACT )                   //SETUP interrupt
            {
                R8_USB_INT_FG = RB_USB_IF_SETUOACT;              // clear int flag
            }

            if( R8_USB_INT_FG & RB_USB_IF_TRANSFER )
            {
                R8_UEP3_RX_CTRL = (R8_UEP3_RX_CTRL & ~RB_UEP_RRES_MASK) |UEP_R_RES_NAK;
                usbtran++;
                usbstep++;

                if( usbstep == UDISKSIZE/512 )
                {
                    usbstep = 0;
                }
                R32_UEP3_RX_DMA = (UINT32)(UINT8 *)( UDisk_Out_Buf + usbstep * 512 );
                if( ( usbtran - sdtran ) == UDISKSIZE/512 )
                {
                    R8_UEP3_RX_CTRL = (R8_UEP3_RX_CTRL & ~RB_UEP_RRES_MASK) |UEP_R_RES_NAK;
                }
                else
                {
                    R8_UEP3_RX_CTRL = (R8_UEP3_RX_CTRL & ~RB_UEP_RRES_MASK) |UEP_R_RES_ACK;
                }
                if( flag )
                {
                    R32_EMMC_DMA_BEG1 = (UINT32)(UINT8 *)( UDisk_Out_Buf + sdstep * 512 );
                    flag = 0;
                }
                R8_USB_INT_FG = RB_USB_IF_TRANSFER;
            }

            if(R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP)
            {
                R16_EMMC_INT_FG = RB_EMMC_IF_BKGAP;
                sdtran++;
                sdstep++;
                if( sdstep == UDISKSIZE/512 )
                {
                sdstep = 0;
                }
                if(sdtran < usbtran)
                {
                R32_EMMC_DMA_BEG1 = (UINT32)(UINT8 *)( UDisk_Out_Buf + sdstep * 512 );
                }
                else
                {
                flag = 1;
                }
                R8_UEP3_RX_CTRL = (R8_UEP3_RX_CTRL & ~RB_UEP_RRES_MASK) |UEP_R_RES_ACK;
            }
            else if(R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE)
            {
                R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE;

                R8_UEP3_RX_CTRL = (R8_UEP3_RX_CTRL & ~RB_UEP_RRES_MASK) |UEP_R_RES_NAK;
                break;
            }
        }

        R32_EMMC_TRAN_MODE = 0;
        //cmd12
        cmd_arg_val = 0;
        cmd_set_val = RB_EMMC_CKIDX |
                      RB_EMMC_CKCRC |
                      RESP_TYPE_R1b |
                      EMMC_CMD12;
        EMMCSendCmd(cmd_arg_val, cmd_set_val);
        while(1)
        {
            s = CheckCMDComp( &TF_EMMCParam );
            if( s != CMD_NULL ) break;
        }
        R16_EMMC_INT_FG = 0xffff;

        R32_UEP3_RX_DMA = (UINT32)(UINT8 *)UDisk_Out_Buf;
        PFIC_EnableIRQ(USBHS_IRQn);
        R8_UEP0_TX_CTRL = uep0txsave ;
        R8_UEP0_RX_CTRL = uep0rxsave ;

    }
    else
    {
        PFIC_DisableIRQ(USBSS_IRQn);

        preqnum = UDISK_Transfer_DataLen/512;

        UDISK_Transfer_DataLen = 0;

        //cmd25
        cmd_arg_val = UDISK_Cur_Sec_Lba;
        cmd_set_val = RB_EMMC_CKIDX |
                      RB_EMMC_CKCRC |
                      RESP_TYPE_48  |
                      EMMC_CMD25;
        EMMCSendCmd(cmd_arg_val, cmd_set_val);
        while(1)
        {
            s = CheckCMDComp( &TF_EMMCParam );
            if( s != CMD_NULL ) break;
        }

        R32_EMMC_TRAN_MODE = RB_EMMC_DMA_DIR |(1<<6);
        R32_EMMC_DMA_BEG1 = (UINT32)UDisk_Out_Buf;
        R32_EMMC_BLOCK_CFG = 512<<16 | preqnum;


        usbtran += 2;
        usbstep += 2;

        if( usbtran < preqnum )
        {
            USBSS->UEP3_RX_DMA = (UINT32)(UINT8 *)( UDisk_Out_Buf + (usbstep*512)  );
            USB30_OUT_Set(ENDP_3, ACK, DEF_ENDP3_OUT_BURST_LEVEL);
            USB30_Send_ERDY(ENDP_3 | OUT, DEF_ENDP3_OUT_BURST_LEVEL);
        }
        else
        {
            USB30_OUT_Set( ENDP_3 , NRDY , 0 );
        }

        while(1)
        {
            if( USB30_OUT_ITflag( ENDP_3 ) ){ //USB Transfer completed
                USB30_OUT_ClearIT(ENDP_3);
                USB30_OUT_Set( ENDP_3 , NRDY , 0 );
                usbtran +=2;
                usbstep +=2;
                if( usbstep == UDISKSIZE/512 ){   usbstep = 0;    }
                USBSS->UEP3_RX_DMA = (UINT32)(UINT8 *)( UDisk_Out_Buf + (usbstep*512)  );
                if( ( usbtran - sdtran ) >= ((UDISKSIZE/512)-2) )
                { //full
                    full = 1;
                }
                else
                {
                    if( usbtran < preqnum )
                    {
                        USB30_OUT_Set( ENDP_3 , ACK , 1 );
                        USB30_Send_ERDY( ENDP_3 | OUT, 1 );
                    }
                }

                if( flag )
                {
                    R32_EMMC_DMA_BEG1 = (UINT32)(UINT8 *)( UDisk_Out_Buf + sdstep * 512 ); //start
                    flag = 0;
                }
            }
            if(R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP)
            {  //emmc Block transfer completed
                R16_EMMC_INT_FG = RB_EMMC_IF_BKGAP;  //Block cleaning completion interrupt

                sdtran++;
                sdstep++;
                if( sdstep == UDISKSIZE/512 ){   sdstep = 0;   }
                if(sdtran < usbtran)
                {
                    R32_EMMC_DMA_BEG1 = (UINT32)(UINT8 *)( UDisk_Out_Buf + sdstep * 512 ); //Write next sector
                }
                else{        //If the annulus is empty, the transfer will be stopped and the next USB transfer will be completed
                    flag = 1;
                }
                if(((usbtran-sdtran)<=((UDISKSIZE/512)-2)) && full )
                {
                    full = 0;
                    if( usbtran < preqnum )
                    {
                        USB30_OUT_Set( ENDP_3 , ACK , 1 );
                        USB30_Send_ERDY( ENDP_3 | OUT, 1 );
                    }
                }
            }
            else if(R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE)
            {
                R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE;
                break;
            }
        }

        R32_EMMC_TRAN_MODE = 0;
        //cmd12
        cmd_arg_val = 0;
        cmd_set_val = RB_EMMC_CKIDX |
                      RB_EMMC_CKCRC |
                      RESP_TYPE_R1b |
                      EMMC_CMD12;
        EMMCSendCmd(cmd_arg_val, cmd_set_val);
        while(1)
        {
            s = CheckCMDComp( &TF_EMMCParam );
            if( s != CMD_NULL ) break;
        }
        R16_EMMC_INT_FG = 0xffff;

        USB30_OUT_ClearIT(ENDP_3);
        USBSS->UEP3_RX_DMA = (UINT32)(UINT8 *)UDisk_Out_Buf;

        PFIC_EnableIRQ(USBSS_IRQn);
    }

    if( UDISK_Transfer_DataLen == 0 )
    {
        Udisk_Transfer_Status &= ~DEF_UDISK_BLUCK_DOWN_FLAG;
        UDISK_Up_CSW();
    }
}

/*********************************************************************
 * @fn      EMMC_IRQHandler
 *
 * @brief   This function handles EMMC exception.
 *
 * @return  none
 */
void EMMC_IRQHandler(void)
{
    if(R16_EMMC_INT_FG)                      //Error interuption
    {
        PRINT("e:%04x\n", R16_EMMC_INT_FG);
    }
}
