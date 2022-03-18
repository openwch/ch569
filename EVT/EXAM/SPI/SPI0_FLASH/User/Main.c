/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#include "CH56x_common.h"

#define  FREQ_SYS   80000000

#define  CMD_STATUS1         0x05
#define  CMD_WR_ENABLE       0x06
#define  CMD_ERASE_4KBYTE    0x20
#define  CMD_ERASE_32KBYTE   0x52
#define  CMD_READ_DATA       0x03
#define  CMD_PAGE_PROG       0x02
#define  CMD_FAST_READ       0x0B
#define  CMD_DEVICE_ID       0x90

/********************************* 引脚定义 ************************************
*    PA12  <===========>  SCS0
*    PA13  <===========>  SCK0
*    PA14  <===========>  MOSI0
*    PA15  <===========>  MISO0
*******************************************************************************/
#define  SPI0_CS_LOW()        R32_PA_CLR |= 1<<12
#define  SPI0_CS_HIGH()       R32_PA_OUT |= 1<<12

/*******************************************************************************
 * @fn        DebugInit
 *
 * @brief     Initializes the UART1 peripheral.
 *
 * @param     baudrate - UART1 communication baud rate.
 *
 * @return    None
 */
void DebugInit(UINT32 baudrate)
{
	UINT32 x;
	UINT32 t = FREQ_SYS;
	
	x = 10 * t * 2 / 16 / baudrate;
	x = ( x + 5 ) / 10;
	R8_UART1_DIV = 1;
	R16_UART1_DL = x;
	R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
	R8_UART1_LCR = RB_LCR_WORD_SZ;
	R8_UART1_IER = RB_IER_TXD_EN;
	R32_PA_SMT |= (1<<8) |(1<<7);
	R32_PA_DIR |= (1<<8);
}

/*******************************************************************************
 * @fn      SPI_MASTER_INIT
 *
 * @brief   SPI0主机模式初始化
 *
 * @return  None
 */
void SPI_MASTER_INIT(void)
{
  R8_SPI0_CTRL_MOD = RB_SPI_MOSI_OE|RB_SPI_SCK_OE;                              /* MOSI,SCK输出使能，主机模式，方式0 */
  R8_SPI0_CLOCK_DIV = 0x0a;                                                     /* 10分频，100/10=10M */
  R32_PA_DIR |= (1<<14 | 1<<13 | 1<<12);                                        /* MOSI(PA14),SCK0(PA13),SCS(PA12)为输出*/
  R32_PA_PU  |=  1<<12 ;
  R8_SPI0_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;
}

/*******************************************************************************
 * @fn   SPI0_Trans
 *
 * @brief 发送一字节数据
 *
 * @param  data -要发送的数据
 *
 * @return  None
 */
void SPI0_Trans(UINT8 data)
{

//    R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
//    R8_SPI0_BUFFER = data;
//    while( !(R8_SPI0_INT_FLAG & RB_SPI_FREE) );

    R32_SPI0_FIFO = data;
    R16_SPI0_TOTAL_CNT = 0x01;
    while( R8_SPI0_FIFO_COUNT != 0 );                                           /* 等待数据发送完毕 */
}

/*******************************************************************************
 * @fn      SPI0_Recv
 *
 * @brief   接收一字节数据
 *
 * @return  None
 */
UINT8 SPI0_Recv(void)
{
//    R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
//    R8_SPI0_BUFFER = 0xFF;                                //启动传输
//    while( !(R8_SPI0_INT_FLAG & RB_SPI_FREE) );
//    return ( R8_SPI0_BUFFER );

    UINT8 data;
    R32_SPI0_FIFO = 0xff;
    R16_SPI0_TOTAL_CNT = 0x01;
    while( R8_SPI0_FIFO_COUNT != 0 );                                           /* 等待数据回来 */
    data = R8_SPI0_BUFFER;
    return data;
}

/*******************************************************************************
 * @fn       SPI0_RecvS
 *
 * @brief    使用FIFO连续接收多字节
 *
 * @param    pbuf - 待发送的数据内容首地址
             len - 请求发送的数据长度，最大4095

 * @return   None
 */
void SPI0_RecvS(UINT8 *pbuf, UINT16 len)
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
 * @fn      ReadExternalFlashStatusReg_SPI
 *
 * @brief   用来读取状态寄存器，并返回状态寄存器的值
 *
 * @return  ExFlashRegStatus
 */
UINT8 ReadExternalFlashStatusReg_SPI(void)
{
    UINT8 ExFlashRegStatus;


    SPI0_CS_LOW();
    SPI0_Trans( CMD_STATUS1 );                                          //发送读状态寄存器的命令
    ExFlashRegStatus = SPI0_Recv();                                     //读取状态寄存器
    SPI0_CS_HIGH();

    return ExFlashRegStatus;
}

/*******************************************************************************
 * @fn      WaitExternalFlashIfBusy
 *
 * @brief   等待芯片空闲(在执行Byte-Program, Sector-Erase, Block-Erase, Chip-Erase操作后)
 *
 * @return  None
 */
void WaitExternalFlashIfBusy(void)
{
    while ((ReadExternalFlashStatusReg_SPI())&0x01 == 0x01 )
    {
        ;    //等待直到Flash空闲
    }
}

/*******************************************************************************
 * @fn       WriteExternalFlashEnable_SPI
 *
 * @brief    写使能，同样可以用于使能写状态寄存器
 *
 * @return   None
 */
void WriteExternalFlashEnable_SPI(void)
{
    SPI0_CS_LOW();
    SPI0_Trans( CMD_WR_ENABLE );                                        //发送写使能命令
    SPI0_CS_HIGH();
}

/*******************************************************************************
 * @fn       EraseExternal4KFlash_SPI
 *
 * @brief    擦除4K Flash  擦除一个扇区
 *
 * @param    Dst_Addr 0-1 ffff ffff ,清除任意地址所在的扇区
 *
 * @return   None
 */
void EraseExternal4KFlash_SPI(UINT32 Dst_Addr)
{
    WriteExternalFlashEnable_SPI();
    WaitExternalFlashIfBusy();

    SPI0_CS_LOW();
    SPI0_Trans(CMD_ERASE_4KBYTE);                                      //扇区擦除命令
    SPI0_Trans(((Dst_Addr & 0xFFFFFF) >> 16));                         //发送3字节地址
    SPI0_Trans(((Dst_Addr & 0xFFFF) >> 8));
    SPI0_Trans(Dst_Addr & 0xFF);
    SPI0_CS_HIGH();

    WaitExternalFlashIfBusy();
}

/*******************************************************************************
 * @fn         EraseExternalFlash_SPI
 *
 * @brief      擦除32K Flash  擦除一个扇区
 *
 * @param      Dst_Addr 0-1 ffff ffff ,清除任意地址所在的扇区
 *
 * @return     None
 */
void EraseExternal32KFlash_SPI(UINT32 Dst_Addr)
{
    WriteExternalFlashEnable_SPI();
    WaitExternalFlashIfBusy();

    SPI0_CS_LOW();
    SPI0_Trans(CMD_ERASE_32KBYTE);                                    //32K擦除命令
    SPI0_Trans(((Dst_Addr & 0xFFFFFF) >> 16));                        //发送3字节地址
    SPI0_Trans(((Dst_Addr & 0xFFFF) >> 8));
    SPI0_Trans(Dst_Addr & 0xFF);
    SPI0_CS_HIGH();

    WaitExternalFlashIfBusy();
}

/*******************************************************************************
 * @fn           PageWriteExternalFlash_SPI
 *
 * @brief        页写，SPI在一页内写入少于256个字节的数据
 *
 * @param        RcvBuffer - 数据存储区
 *               StarAddr - 开始写入的地址
 *               Len - 要写入的字节数(最大256),该数不应该超过该页剩余的字节数
 *
 * @returnNone
 */
void PageWriteExternalFlash_SPI(UINT32 StarAddr, UINT16 Len, PUINT8 RcvBuffer)
{
    UINT16 i;

    WriteExternalFlashEnable_SPI();                                   //SET WEL

    SPI0_CS_LOW();
    SPI0_Trans(CMD_PAGE_PROG);                                        //发送写页命令
    SPI0_Trans(((StarAddr & 0xFFFFFF) >> 16));                        //发送24bit地址
    SPI0_Trans(((StarAddr & 0xFFFF) >> 8));
    SPI0_Trans(StarAddr & 0xFF);
    for(i=0; i!=Len; i++){
    	SPI0_Trans(RcvBuffer[i]);    //循环写数
    }
    SPI0_CS_HIGH();

    WaitExternalFlashIfBusy();                                        //等待写入结束
}

/*******************************************************************************
 * @fn       BlukWriteExternalFlash_SPI
 *
 * @brief    无检验写SPI FLASH
 *               必须确保所写地址范围内的数据全部为0XFF，否则在非0XFF处写入的数据将失败；
 *
 * @param    SendBuffer - 数据存储区
 *           StarAddr - 开始写入的地址
 *           Len - 要写入的字节数(最大65535)
 *
 * @return   None
 */
void BlukWriteExternalFlash_SPI(UINT32 StarAddr, UINT16 Len, PUINT8 SendBuffer)
{
    UINT16  pageremain;

    pageremain = 256-StarAddr%256;                                     //单页剩余的字节数
    if(Len<=pageremain)
    {
        pageremain=Len;                                                //不大于256个字节
    }
    while(1)
    {
        PageWriteExternalFlash_SPI(StarAddr,pageremain,SendBuffer);
        if(Len==pageremain)
        {
            break;                                                     //写入结束了
        }
        else
        {
            SendBuffer+=pageremain;
            StarAddr+=pageremain;
            Len-=pageremain;                                           //减去已经写入的字节数
            if(Len>256)
            {
                pageremain=256;                                        //一次可以写入256个字节
            }
            else
            {
                pageremain=Len;                                        //不够256个字节
            }
        }
    }
}

/*******************************************************************************
 * @fn  ReadExternalFlash_SPI
 *
 * @brief 读取地址的数据
 *
 * @param     StarAddr
 *            Len 读取数据长度
 *            RcvBuffer 接收缓冲区起始地址
 *
 * @return None
 */
void ReadExternalFlash_SPI(UINT32 StarAddr, UINT16 Len, PUINT8 RcvBuffer)
{
    SPI0_CS_LOW();
    SPI0_Trans(CMD_READ_DATA);                                         //读命令
    SPI0_Trans(((StarAddr & 0xFFFFFF) >> 16));                         //发送3字节地址
    SPI0_Trans(((StarAddr & 0xFFFF) >> 8));
    SPI0_Trans(StarAddr & 0xFF);
    SPI0_RecvS( RcvBuffer, Len );
    SPI0_CS_HIGH();
}

/*******************************************************************************
 * @fn        BlukReadExternalFlash_SPI
 *
 * @brief     读取起始地址内多个字节的数据，存入缓冲区中
 *
 * @param     StarAddr -Destination Address 000000H - 1FFFFFH
              Len - 读取数据长度
              RcvBuffer - 接收缓冲区起始地址

 * @return None
 */
void BlukReadExternalFlash_SPI(UINT32 StarAddr, UINT16 Len, PUINT8 RcvBuffer)
{
    SPI0_CS_LOW();
    SPI0_Trans(CMD_FAST_READ);                                         //高速度
    SPI0_Trans(((StarAddr & 0xFFFFFF) >> 16));                         //发送3字节地址
    SPI0_Trans(((StarAddr & 0xFFFF) >> 8));
    SPI0_Trans(StarAddr & 0xFF);
    SPI0_Trans(0x00);
    SPI0_RecvS( RcvBuffer, Len );
    SPI0_CS_HIGH();
}

/*******************************************************************************
 * @fn        SPIFlash_ReadID
 *
 * @brief     SPI Flash读取芯片ID
 *
 * @return    0XEF13 - 表示芯片型号为W25Q80
 *            0XEF14 - 表示芯片型号为W25Q16
 *            0XEF15 - 表示芯片型号为W25Q32
 *            0XEF16 - 表示芯片型号为W25Q64
 *            0XEF17 - 表示芯片型号为W25Q128
 */
UINT16 SPIFlash_ReadID(void)
{
    UINT16  temp = 0;

    R32_PA_CLR |=  1<<12 ;

    SPI0_Trans(0x90);                    //读取ID命令
    SPI0_Trans(0x00);
    SPI0_Trans(0x00);
    SPI0_Trans(0x00);
    temp = SPI0_Recv();
    temp = temp<<8;
    temp |= SPI0_Recv();

    R32_PA_OUT |=  1<<12 ;

    return temp;
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

	UINT8 buf[1024];
	UINT8 i;

	SystemInit(FREQ_SYS);
    Delay_Init(FREQ_SYS);


    /*配置串口调试 */
	DebugInit(115200);
	printf("Start @ChipID=%02X\r\n", R8_CHIP_ID );

	SPI_MASTER_INIT ( );                                                       /* SPI0主机模式初始化 */

    printf("START SPI FLASH\n");

    printf("id:0x%04x\n", SPIFlash_ReadID() );                                  /*读取芯片ID */

    for(i=0; i!=255; i++){
        buf[i] = i;
    }


    EraseExternal4KFlash_SPI(0);
    BlukWriteExternalFlash_SPI(0,255,buf);
    BlukReadExternalFlash_SPI( 0,255,buf );

    for(i=0; i!=255; i++){
    	printf("%d ",(UINT16)buf[i]);
    }
    printf("done\n");


    while(1);    
}



