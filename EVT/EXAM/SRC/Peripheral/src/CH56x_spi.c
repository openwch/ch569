/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_spi.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#include "CH56x_common.h"

/*******************************************************************************
 * @fn     SPI0_MasterDefInit
 *
 * @brief  主机模式默认初始化
 *
 * @return   None
 */
void SPI0_MasterDefInit( void )
{
    R8_SPI0_CLOCK_DIV = 4;                                   //主频时钟4分频
    R8_SPI0_CTRL_MOD = RB_SPI_ALL_CLEAR;                     //FIFO/计数器/中断标志寄存器清0，写1强制清空或清零
    R8_SPI0_CTRL_MOD = RB_SPI_MOSI_OE | RB_SPI_SCK_OE ;      //MOSI引脚和SCK引脚输出使能
    R8_SPI0_CTRL_CFG |= RB_SPI_AUTO_IF;                      //使能访问BUFFER/FIFO自动清除标志位
    R8_SPI0_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;                  //不启动DMA方式
}

/*******************************************************************************
 * @fn     SPI0_DataMode
 *
 * @brief  设置数据流模式
 *
 * @param  m - 数据流模式
 *
 * @return   None
 */
void SPI0_DataMode( ModeBitOrderTypeDef m )
{
    switch( m )
    {
        case Mode0_LowBitINFront:                       //模式0，低位在前
            R8_SPI0_CTRL_MOD &= ~RB_SPI_MST_SCK_MOD;
            R8_SPI0_CTRL_CFG |= RB_SPI_BIT_ORDER;
            break;
        case Mode0_HighBitINFront:                      //模式0，高位在前
            R8_SPI0_CTRL_MOD &= ~RB_SPI_MST_SCK_MOD;
            R8_SPI0_CTRL_CFG &= ~RB_SPI_BIT_ORDER;
            break;
        case Mode3_LowBitINFront:                       //模式3，低位在前
            R8_SPI0_CTRL_MOD |= RB_SPI_MST_SCK_MOD;
            R8_SPI0_CTRL_CFG |= RB_SPI_BIT_ORDER;
            break;
        case Mode3_HighBitINFront:                      //模式3，高位在前
            R8_SPI0_CTRL_MOD |= RB_SPI_MST_SCK_MOD;
            R8_SPI0_CTRL_CFG &= ~RB_SPI_BIT_ORDER;
            break;
        default:
            break;
    }
}

/*******************************************************************************
 * @fn     SPI0_MasterSendByte
 *
 * @brief  发送单字节(buffer)
 *
 * @param  d - 发送字节
 *
 * @return  None
 */
void SPI0_MasterSendByte( UINT8 d )
{
    R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R8_SPI0_BUFFER = d;
    while( !(R8_SPI0_INT_FLAG & RB_SPI_FREE) );
}

/*******************************************************************************
 * @fn     SPI0_MasterRecvByte
 *
 * @brief  接收单字节 (buffer)
 *
 * @return   接收到的字节
 */
UINT8 SPI0_MasterRecvByte( void )
{
    R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R8_SPI0_BUFFER = 0xFF;                                //启动传输
    while( !(R8_SPI0_INT_FLAG & RB_SPI_FREE) );
    return ( R8_SPI0_BUFFER );
}

/*******************************************************************************
 * @fn     SPI0_MasterTrans
 *
 * @brief  使用FIFO连续发送多字节
 *
 * @param  pbuf:待发送的数据内容首地址
 *
 * @return   None
 */
void SPI0_MasterTrans( UINT8 *pbuf, UINT16 len )
{
    UINT16 sendlen;

    sendlen = len;
    R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;        //设置数据方向为输出
    R16_SPI0_TOTAL_CNT = sendlen;                //设置要发送的数据长度
    R8_SPI0_INT_FLAG = RB_SPI_IF_CNT_END;
    while( sendlen )
    {
        if( R8_SPI0_FIFO_COUNT < SPI_FIFO_SIZE )
        {
            R8_SPI0_FIFO = *pbuf;
            pbuf++;
            sendlen--;
        }
    }
    while( R8_SPI0_FIFO_COUNT != 0 );             //等待FIFO中的数据全部发送完成
}

/*******************************************************************************
 * @fn     SPI0_MasterRecv
 *
 * @brief  使用FIFO连续接收多字节
 *
 * @param  pbuf: 待发送的数据内容首地址
 *
 * @return   None
 **/
void SPI0_MasterRecv( UINT8 *pbuf, UINT16 len )
{
    UINT16  readlen;

    readlen = len;
    R8_SPI0_CTRL_MOD |= RB_SPI_FIFO_DIR;          //设置数据方向为输入
    R16_SPI0_TOTAL_CNT = len;                     //设置需要接收的数据长度，FIFO方向为输入长度不为0则会启动传输
    R8_SPI0_INT_FLAG = RB_SPI_IF_CNT_END;
    while( readlen )
    {
        if( R8_SPI0_FIFO_COUNT )
        {
            *pbuf = R8_SPI0_FIFO;
            pbuf++;
            readlen--;
        }
    }
}

/*******************************************************************************
 * @fn     SPI0_MasterDMATrans
 *
 * @brief  DMA方式连续发送数据
 *
 * @param  pbuf: 待发送数据起始地址
 *
 * @return   None
 */
void SPI0_MasterDMATrans( PUINT8 pbuf, UINT16 len)
{
    R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R32_SPI0_DMA_BEG = (UINT32)pbuf;
    R32_SPI0_DMA_END = (UINT32)(pbuf + len);
    R16_SPI0_TOTAL_CNT = len;
    R8_SPI0_INT_FLAG = RB_SPI_IF_CNT_END|RB_SPI_IF_DMA_END;
    R8_SPI0_CTRL_CFG |= RB_SPI_DMA_ENABLE;
    while(!(R8_SPI0_INT_FLAG & RB_SPI_IF_CNT_END));
    R8_SPI0_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;
}

/*******************************************************************************
 * @fn     SPI0_MasterDMARecv
 *
 * @brief  DMA方式连续接收数据
 *
 * @param  pbuf: 待接收数据存放起始地址
 *
 * @return   None
 **/
void SPI0_MasterDMARecv( PUINT8 pbuf, UINT16 len)
{
    R8_SPI0_CTRL_MOD |= RB_SPI_FIFO_DIR;
    R32_SPI0_DMA_BEG = (UINT32)pbuf;
    R32_SPI0_DMA_END = (UINT32)(pbuf + len);
    R16_SPI0_TOTAL_CNT = len;
    R8_SPI0_INT_FLAG = RB_SPI_IF_CNT_END|RB_SPI_IF_DMA_END;
    R8_SPI0_CTRL_CFG |= RB_SPI_DMA_ENABLE;
    while(!(R8_SPI0_INT_FLAG & RB_SPI_IF_CNT_END));
    R8_SPI0_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;
}

/*******************************************************************************
 * @fn     SPI0_SlaveInit
 *
 * @brief  设备模式默认初始化
 * 
 * @return   None
 */
void SPI0_SlaveInit( void )
{
    R8_SPI0_CTRL_MOD = RB_SPI_ALL_CLEAR;             //FIFO/计数器/中断标志寄存器清0，写1强制清空或清零
    R8_SPI0_CTRL_MOD = RB_SPI_MISO_OE | RB_SPI_MODE_SLAVE;
    R8_SPI0_CTRL_CFG |= RB_SPI_AUTO_IF;              //使能访问BUFFER/FIFO自动清除标志位
}

/*******************************************************************************
 * @fn     SPI0_SlaveRecvByte
 *
 * @brief  从机模式，接收一字节数据
 * 
 * @return   接收到数据
 */
UINT8 SPI0_SlaveRecvByte( void )
{
    R8_SPI0_CTRL_MOD |= RB_SPI_FIFO_DIR;              //设置为输入模式，接收数据
    while( R8_SPI0_FIFO_COUNT == 0 );
    return R8_SPI0_FIFO;
}

/*******************************************************************************
 * @fn     SPI0_SlaveRecvByte
 *
 * @brief  从机模式，发送一字节数据
 * 
 * @return   接收到数据
 **/
void SPI0_SlaveSendByte( UINT8 d )
{
    R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;              //设置数据方向为输出
    R8_SPI0_FIFO = d;
    while( R8_SPI0_FIFO_COUNT != 0 );                  //等待发送完成
}

/*******************************************************************************
 * @fn     SPI0_SlaveRecv
 *
 * @brief  从机模式，接收多字节数据
 *
 * @param  pbuf: 接收数据存放起始地址
 *
 * @return   None
 **/
void SPI0_SlaveRecv( PUINT8 pbuf, UINT16 len )
{
    UINT16 revlen;

    revlen = len;
    R8_SPI0_CTRL_MOD |= RB_SPI_FIFO_DIR;                //设置为输入模式，接收数据
    R16_SPI0_TOTAL_CNT = revlen;                        //给SPI收发数据总长度寄存器进行赋值
    R8_SPI0_INT_FLAG = RB_SPI_IF_CNT_END;               //SPI中断标志寄存器  全部字节传输完成标志，写1清0
    while( revlen )
    {
        if( R8_SPI0_FIFO_COUNT )                        //当前FIFO中字节计数
        {
            *pbuf = R8_SPI0_FIFO;
            pbuf++;
            revlen--;
        }
    }
}

/*******************************************************************************
 * @fn     SPI0_SlaveTrans
 *
 * @brief  从机模式，发送多字节数据
 *
 * @param  pbuf: 待发送的数据内容首地址
 *
 * @return   None
 */
void SPI0_SlaveTrans( UINT8 *pbuf, UINT16 len )
{
    UINT16 sendlen;

    sendlen = len;
    R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;              //设置数据方向为输出
    R16_SPI0_TOTAL_CNT = sendlen;                      //设置要发送的数据长度
    R8_SPI0_INT_FLAG = RB_SPI_IF_CNT_END;              //SPI中断标志寄存器  全部字节传输完成标志，写1清0
    while( sendlen )
    {
        if( R8_SPI0_FIFO_COUNT < SPI_FIFO_SIZE )       //比较当前FIFO中字节计数大小
        {
            R8_SPI0_FIFO = *pbuf;
            pbuf++;
            sendlen--;
        }
    }
    while( R8_SPI0_FIFO_COUNT != 0 );                  //等待FIFO中的数据全部发送完成
}


/*******************************************************************************
 * @fn     SPI1_MasterDefInit
 *
 * @brief  主机模式默认初始化
 *
 * @return   None
 */
void SPI1_MasterDefInit( void )
{
    R8_SPI1_CLOCK_DIV = 4;                                   //主频时钟4分频
    R8_SPI1_CTRL_MOD = RB_SPI_ALL_CLEAR;
    R8_SPI1_CTRL_MOD = RB_SPI_MOSI_OE | RB_SPI_SCK_OE ;
    R8_SPI1_CTRL_CFG |= RB_SPI_AUTO_IF;
    R8_SPI1_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;                  //不启动DMA方式
    //R8_SPI1_CTRL_CFG |= RB_SPI_DMA_ENABLE;                   //启动DMA方式
}

/*******************************************************************************
 * @fn     SPI1_DataMode
 *
 * @brief  设置数据流模式
 *
 * @param  m: 数据流模式
 *
 * @return   None
 ***/
void SPI1_DataMode( ModeBitOrderTypeDef m )
{
    switch( m )
    {
        case Mode0_LowBitINFront:
            R8_SPI1_CTRL_MOD &= ~RB_SPI_MST_SCK_MOD;
            R8_SPI1_CTRL_CFG |= RB_SPI_BIT_ORDER;
            break;
        case Mode0_HighBitINFront:
            R8_SPI1_CTRL_MOD &= ~RB_SPI_MST_SCK_MOD;
            R8_SPI1_CTRL_CFG &= ~RB_SPI_BIT_ORDER;
            break;
        case Mode3_LowBitINFront:
            R8_SPI1_CTRL_MOD |= RB_SPI_MST_SCK_MOD;
            R8_SPI1_CTRL_CFG |= RB_SPI_BIT_ORDER;
            break;
        case Mode3_HighBitINFront:
            R8_SPI1_CTRL_MOD |= RB_SPI_MST_SCK_MOD;
            R8_SPI1_CTRL_CFG &= ~RB_SPI_BIT_ORDER;
            break;
        default:
            break;
    }
}

/*******************************************************************************
 * @fn     SPI1_MasterSendByte
 *
 * @brief  发送单字节(buffer)
 *
 * @param  d - 发送字节
 *
 * @return   None
 */
void SPI1_MasterSendByte( UINT8 d )
{
    R8_SPI1_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R8_SPI1_BUFFER = d;
    while( !(R8_SPI1_INT_FLAG & RB_SPI_FREE) );
}

/*******************************************************************************
 * @fn     SPI1_MasterRecvByte
 *
 * @brief  接收单字节 (buffer)
 *
 * @return   接收到的字节
 */
UINT8 SPI1_MasterRecvByte( void )
{
    R8_SPI1_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R8_SPI1_BUFFER = 0xFF;                     //启动传输
    while( !(R8_SPI1_INT_FLAG & RB_SPI_FREE) );
    return ( R8_SPI1_BUFFER );
}

/*******************************************************************************
 * @fn     SPI1_MasterTrans
 *
 * @brief  使用FIFO连续发送多字节
 *
 * @param  pbuf - 待发送的数据内容首地址
 *         len - 请求发送的数据长度，最大4095
 * @return   None
 */
void SPI1_MasterTrans( UINT8 *pbuf, UINT16 len )
{
    UINT16 sendlen;

    sendlen = len;
    R8_SPI1_CTRL_MOD &= ~RB_SPI_FIFO_DIR;        //设置数据方向为输出
    R16_SPI1_TOTAL_CNT = sendlen;                //设置要发送的数据长度
    R8_SPI1_INT_FLAG = RB_SPI_IF_CNT_END;
    while( sendlen )
    {
        if( R8_SPI1_FIFO_COUNT < SPI_FIFO_SIZE )
        {
            R8_SPI1_FIFO = *pbuf;
            pbuf++;
            sendlen--;
        }
    }
    while( R8_SPI1_FIFO_COUNT != 0 );             //等待FIFO中的数据全部发送完成
}

/*******************************************************************************
 * @fn     SPI1_MasterRecv
 *
 * @brief  使用FIFO连续接收多字节
 *
 * @param  pbuf - 待发送的数据内容首地址
 *         len - 请求发送的数据长度，最大4095
 * @return   None
 **/
void SPI1_MasterRecv( UINT8 *pbuf, UINT16 len )
{
    UINT16  readlen;

    readlen = len;
    R8_SPI1_CTRL_MOD |= RB_SPI_FIFO_DIR;         //设置数据方向为输入
    R16_SPI1_TOTAL_CNT = len;                    //设置需要接收的数据长度，FIFO方向为输入长度不为0则会启动传输
    R8_SPI1_INT_FLAG = RB_SPI_IF_CNT_END;
    while( readlen )
    {
        if( R8_SPI1_FIFO_COUNT )
        {
            *pbuf = R8_SPI1_FIFO;
            pbuf++;
            readlen--;
        }
    }
}

/*******************************************************************************
 * @fn     SPI1_MasterDMATrans
 *
 * @brief  DMA方式连续发送数据
 *
 * @param  pbuf - 待发送数据起始地址
 *         len - 带发送数据长度
 * @return   None
 */
void SPI1_MasterDMATrans( PUINT8 pbuf, UINT16 len)
{
    R8_SPI1_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R32_SPI1_DMA_BEG = (UINT32)pbuf;
    R32_SPI1_DMA_END = (UINT32)(pbuf + len);
    R16_SPI1_TOTAL_CNT = len;
    R8_SPI1_INT_FLAG = RB_SPI_IF_CNT_END|RB_SPI_IF_DMA_END;
    R8_SPI1_CTRL_CFG |= RB_SPI_DMA_ENABLE;
    while(!(R8_SPI1_INT_FLAG & RB_SPI_IF_CNT_END));
    R8_SPI1_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;
}

/*******************************************************************************
 * @fn     SPI1_MasterDMARecv
 *
 * @brief  DMA方式连续接收数据
 *
 * @param  pbuf - 待接收数据存放起始地址
 *         len - 待接收数据长度
 *
 * @return   None
 */
void SPI1_MasterDMARecv( PUINT8 pbuf, UINT16 len)
{
    R8_SPI1_CTRL_MOD |= RB_SPI_FIFO_DIR;
    R32_SPI1_DMA_BEG = (UINT32)pbuf;
    R32_SPI1_DMA_END = (UINT32)(pbuf + len);
    R16_SPI1_TOTAL_CNT = len;
    R8_SPI1_INT_FLAG = RB_SPI_IF_CNT_END|RB_SPI_IF_DMA_END;
    R8_SPI1_CTRL_CFG |= RB_SPI_DMA_ENABLE;
    while(!(R8_SPI0_INT_FLAG & RB_SPI_IF_CNT_END));
    R8_SPI1_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;
}

/*******************************************************************************
 * @fn     SPI1_SlaveInit
 *
 * @brief  设备模式默认初始化
 *
 * @return   None
 */
void SPI1_SlaveInit( void )
{
    R8_SPI1_CTRL_MOD = RB_SPI_ALL_CLEAR;
    R8_SPI1_CTRL_MOD = RB_SPI_MISO_OE | RB_SPI_MODE_SLAVE;
    R8_SPI1_CTRL_CFG |= RB_SPI_AUTO_IF;
}

/*******************************************************************************
 * @fn     SPI1_SlaveRecvByte
 *
 * @brief  从机模式，接收一字节数据
 * 
 * @return   接收到数据
 */
UINT8 SPI1_SlaveRecvByte( void )
{
    R8_SPI1_CTRL_MOD |= RB_SPI_FIFO_DIR;
    while( R8_SPI1_FIFO_COUNT == 0 );
    return R8_SPI1_FIFO;
}

/*******************************************************************************
 * @fn     SPI1_SlaveRecvByte
 *
 * @brief  从机模式，发送一字节数据
 * 
 * @return   接收到数据
 */
void SPI1_SlaveSendByte( UINT8 d )
{
    R8_SPI1_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R8_SPI1_FIFO = d;
    while( R8_SPI1_FIFO_COUNT != 0 );        //等待发送完成
}

/*******************************************************************************
 * @fn     SPI1_SlaveRecv
 * @brief  从机模式，接收多字节数据
 * @param  pbuf - 接收数据存放起始地址
 *         len - 请求接收数据长度
 * @return   None
 */
void SPI1_SlaveRecv( PUINT8 pbuf, UINT16 len )
{
    UINT16 revlen;

    revlen = len;
    R8_SPI1_CTRL_MOD |= RB_SPI_FIFO_DIR;
    R16_SPI1_TOTAL_CNT = revlen;
    R8_SPI1_INT_FLAG = RB_SPI_IF_CNT_END;
    while( revlen )
    {
        if( R8_SPI1_FIFO_COUNT )
        {
            *pbuf = R8_SPI1_FIFO;
            pbuf++;
            revlen--;
        }
    }
}

/*******************************************************************************
 * @fn     SPI1_SlaveTrans
 *
 * @brief  从机模式，发送多字节数据
 *
 * @param  pbuf - 待发送的数据内容首地址
 *         len - 请求发送的数据长度，最大4095
 * @return   None
 */
void SPI1_SlaveTrans( UINT8 *pbuf, UINT16 len )
{
    UINT16 sendlen;

    sendlen = len;
    R8_SPI1_CTRL_MOD &= ~RB_SPI_FIFO_DIR;              //设置数据方向为输出
    R16_SPI1_TOTAL_CNT = sendlen;                      //设置要发送的数据长度
    R8_SPI1_INT_FLAG = RB_SPI_IF_CNT_END;
    while( sendlen )
    {
        if( R8_SPI1_FIFO_COUNT < SPI_FIFO_SIZE )
        {
            R8_SPI1_FIFO = *pbuf;
            pbuf++;
            sendlen--;
        }
    }
    while( R8_SPI1_FIFO_COUNT != 0 );                   //等待FIFO中的数据全部发送完成
}







