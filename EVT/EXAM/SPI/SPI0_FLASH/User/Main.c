/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : 
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 *SPI0_FLASH routine
 * SPI0 operation external FLASH
 */

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

/********************************* Pin Definitions ************************************
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
 * @brief   SPI0 master mode initialization
 *
 * @return  None
 */
void SPI_MASTER_INIT(void)
{
  R8_SPI0_CTRL_MOD = RB_SPI_MOSI_OE|RB_SPI_SCK_OE;                              /* MOSI, SCK output enable, host mode, mode 0 */
  R8_SPI0_CLOCK_DIV = 0x0a;                                                     /* 10 frequency division, 100/10=10M */
  R32_PA_DIR |= (1<<14 | 1<<13 | 1<<12);                                        /* MOSI(PA14), SCK0(PA13), SCS(PA12) are the output*/
  R32_PA_PU  |=  1<<12 ;
  R8_SPI0_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;
}

/*******************************************************************************
 * @fn   SPI0_Trans
 *
 * @brief send a byte of data
 *
 * @param  data - data to send
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
    while( R8_SPI0_FIFO_COUNT != 0 );                                           /* Wait for the data to be sent */
}

/*******************************************************************************
 * @fn      SPI0_Recv
 *
 * @brief   Receive a byte of data
 *
 * @return  None
 */
UINT8 SPI0_Recv(void)
{
//    R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
//    R8_SPI0_BUFFER = 0xFF;                                //start transfer
//    while( !(R8_SPI0_INT_FLAG & RB_SPI_FREE) );
//    return ( R8_SPI0_BUFFER );

    UINT8 data;
    R32_SPI0_FIFO = 0xff;
    R16_SPI0_TOTAL_CNT = 0x01;
    while( R8_SPI0_FIFO_COUNT != 0 );                                           /* wait for data to come back */
    data = R8_SPI0_BUFFER;
    return data;
}

/*******************************************************************************
 * @fn       SPI0_RecvS
 *
 * @brief    Receive multiple bytes continuously using FIFO
 *
 * @param    pbuf - The first address of the data content to be sent
             len  - The length of the data sent by the request, the maximum is 4095

 * @return   None
 */
void SPI0_RecvS(UINT8 *pbuf, UINT16 len)
{
    UINT16  readlen;

    readlen = len;
    R8_SPI0_CTRL_MOD |= RB_SPI_FIFO_DIR;          //Set data direction to input
    R16_SPI0_TOTAL_CNT = len;                     //Set the length of the data to be received, the FIFO direction will start the transmission if the input length is not 0
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
 * @brief   Used to read the status register and return the value of the status register
 *
 * @return  ExFlashRegStatus
 */
UINT8 ReadExternalFlashStatusReg_SPI(void)
{
    UINT8 ExFlashRegStatus;


    SPI0_CS_LOW();
    SPI0_Trans( CMD_STATUS1 );                                          //Send a command to read the status register
    ExFlashRegStatus = SPI0_Recv();                                     //read status register
    SPI0_CS_HIGH();

    return ExFlashRegStatus;
}

/*******************************************************************************
 * @fn      WaitExternalFlashIfBusy
 *
 * @brief   Wait for the chip to be free (after performing Byte-Program, Sector-Erase, Block-Erase, Chip-Erase operations)
 *
 * @return  None
 */
void WaitExternalFlashIfBusy(void)
{
    while ((ReadExternalFlashStatusReg_SPI())&0x01 == 0x01 )
    {
        ;    //Waiting for Flash to be idle
    }
}

/*******************************************************************************
 * @fn       WriteExternalFlashEnable_SPI
 *
 * @brief    Write enable, also can be used to enable write status register
 *
 * @return   None
 */
void WriteExternalFlashEnable_SPI(void)
{
    SPI0_CS_LOW();
    SPI0_Trans( CMD_WR_ENABLE );                                        //Send write enable command
    SPI0_CS_HIGH();
}

/*******************************************************************************
 * @fn       EraseExternal4KFlash_SPI
 *
 * @brief    Erase 4K Flash Erase a sector
 *
 * @param    Dst_Addr 0-1 ffff ffff, Clear the sector where any address is located
 *
 * @return   None
 */
void EraseExternal4KFlash_SPI(UINT32 Dst_Addr)
{
    WriteExternalFlashEnable_SPI();
    WaitExternalFlashIfBusy();

    SPI0_CS_LOW();
    SPI0_Trans(CMD_ERASE_4KBYTE);                                      //sector erase command
    SPI0_Trans(((Dst_Addr & 0xFFFFFF) >> 16));                         //Send 3 byte address
    SPI0_Trans(((Dst_Addr & 0xFFFF) >> 8));
    SPI0_Trans(Dst_Addr & 0xFF);
    SPI0_CS_HIGH();

    WaitExternalFlashIfBusy();
}

/*******************************************************************************
 * @fn         EraseExternalFlash_SPI
 *
 * @brief      Erase 32K Flash Erase a sector
 *
 * @param      Dst_Addr 0-1 ffff ffff, Clear the sector where any address is located
 *
 * @return     None
 */
void EraseExternal32KFlash_SPI(UINT32 Dst_Addr)
{
    WriteExternalFlashEnable_SPI();
    WaitExternalFlashIfBusy();

    SPI0_CS_LOW();
    SPI0_Trans(CMD_ERASE_32KBYTE);                                    //32K erase command
    SPI0_Trans(((Dst_Addr & 0xFFFFFF) >> 16));                        //Send 3 byte address
    SPI0_Trans(((Dst_Addr & 0xFFFF) >> 8));
    SPI0_Trans(Dst_Addr & 0xFF);
    SPI0_CS_HIGH();

    WaitExternalFlashIfBusy();
}

/*******************************************************************************
 * @fn           PageWriteExternalFlash_SPI
 *
 * @brief        Page write, SPI writes less than 256 bytes of data in one page
 *
 * @param        RcvBuffer - data storage area
 *               StarAddr - address to start writing
 *               Len - The number of bytes to write (up to 256), which should not exceed the number of bytes remaining on the page
 *
 * @returnNone
 */
void PageWriteExternalFlash_SPI(UINT32 StarAddr, UINT16 Len, PUINT8 RcvBuffer)
{
    UINT16 i;

    WriteExternalFlashEnable_SPI();                                   //SET WEL

    SPI0_CS_LOW();
    SPI0_Trans(CMD_PAGE_PROG);                                        //send write page command
    SPI0_Trans(((StarAddr & 0xFFFFFF) >> 16));                        //Send 24bit address
    SPI0_Trans(((StarAddr & 0xFFFF) >> 8));
    SPI0_Trans(StarAddr & 0xFF);
    for(i=0; i!=Len; i++){
    	SPI0_Trans(RcvBuffer[i]);    //cycle write
    }
    SPI0_CS_HIGH();

    WaitExternalFlashIfBusy();                                        //Wait for write to end
}

/*******************************************************************************
 * @fn       BlukWriteExternalFlash_SPI
 *
 * @brief    Write SPI FLASH without verification
 *               It must be ensured that the data in the address range to be written is all 0XFF, otherwise the data written at non-0XFF will fail
 *
 * @param    SendBuffer - data storage area
 *           StarAddr - address to start writing
 *           Len - The number of bytes to write (max 65535)
 *
 * @return   None
 */
void BlukWriteExternalFlash_SPI(UINT32 StarAddr, UINT16 Len, PUINT8 SendBuffer)
{
    UINT16  pageremain;

    pageremain = 256-StarAddr%256;                                     //The remaining bytes of a single page
    if(Len<=pageremain)
    {
        pageremain=Len;                                                //No more than 256 bytes
    }
    while(1)
    {
        PageWriteExternalFlash_SPI(StarAddr,pageremain,SendBuffer);
        if(Len==pageremain)
        {
            break;                                                     //end of writing
        }
        else
        {
            SendBuffer+=pageremain;
            StarAddr+=pageremain;
            Len-=pageremain;                                           //Subtract the number of bytes already written
            if(Len>256)
            {
                pageremain=256;                                        //256 bytes can be written at a time
            }
            else
            {
                pageremain=Len;                                        //Not enough 256 bytes
            }
        }
    }
}

/*******************************************************************************
 * @fn  ReadExternalFlash_SPI
 *
 * @brief read data from address
 *
 * @param     StarAddr
 *            Len read data length
 *            RcvBuffer Receive buffer start address
 *
 * @return None
 */
void ReadExternalFlash_SPI(UINT32 StarAddr, UINT16 Len, PUINT8 RcvBuffer)
{
    SPI0_CS_LOW();
    SPI0_Trans(CMD_READ_DATA);                                         //read command
    SPI0_Trans(((StarAddr & 0xFFFFFF) >> 16));                         //Send 3 byte address
    SPI0_Trans(((StarAddr & 0xFFFF) >> 8));
    SPI0_Trans(StarAddr & 0xFF);
    SPI0_RecvS( RcvBuffer, Len );
    SPI0_CS_HIGH();
}

/*******************************************************************************
 * @fn        BlukReadExternalFlash_SPI
 *
 * @brief     Read the data of multiple bytes in the starting address and store it in the buffer
 *
 * @param     StarAddr -Destination Address 000000H - 1FFFFFH
              Len - read data length
              RcvBuffer - Receive buffer start address

 * @return None
 */
void BlukReadExternalFlash_SPI(UINT32 StarAddr, UINT16 Len, PUINT8 RcvBuffer)
{
    SPI0_CS_LOW();
    SPI0_Trans(CMD_FAST_READ);                                         //high speed
    SPI0_Trans(((StarAddr & 0xFFFFFF) >> 16));                         //Send 3 byte address
    SPI0_Trans(((StarAddr & 0xFFFF) >> 8));
    SPI0_Trans(StarAddr & 0xFF);
    SPI0_Trans(0x00);
    SPI0_RecvS( RcvBuffer, Len );
    SPI0_CS_HIGH();
}

/*******************************************************************************
 * @fn        SPIFlash_ReadID
 *
 * @brief     SPI Flash read chip ID
 *
 * @return    0XEF13 - Indicates that the chip model is W25Q80
 *            0XEF14 - Indicates that the chip model is W25Q16
 *            0XEF15 - Indicates that the chip model is W25Q32
 *            0XEF16 - Indicates that the chip model is W25Q64
 *            0XEF17 - Indicates that the chip model is W25Q128
 */
UINT16 SPIFlash_ReadID(void)
{
    UINT16  temp = 0;

    R32_PA_CLR |=  1<<12 ;

    SPI0_Trans(0x90);                    //read ID command
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


    /*Configure serial debugging */
	DebugInit(115200);
	printf("Start @ChipID=%02X\r\n", R8_CHIP_ID );

	SPI_MASTER_INIT ( );                                                       /* SPI0 master mode initialization */

    printf("START SPI FLASH\n");

    printf("id:0x%04x\n", SPIFlash_ReadID() );                                  /*Read chip ID */

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



