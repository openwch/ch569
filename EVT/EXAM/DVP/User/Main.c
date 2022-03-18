/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : 该例程操作 OV2640摄像头
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#include "CH56x_common.h"
#include "ov.h"

#define FREQ_SYS         80000000

//DVP Work Mode
#define RGB565_MODE      0
#define JPEG_MODE        1
//DVP Work Mode Selection
#define DVP_Work_Mode    JPEG_MODE
//#define DVP_Work_Mode    RGB565_MODE

UINT32 JPEG_DVPDMAaddr0 = 0x20020000;
UINT32 JPEG_DVPDMAaddr1 = 0x20020000 + OV2640_JPEG_WIDTH;

UINT32 RGB565_DVPDMAaddr0 = 0x20020000;
UINT32 RGB565_DVPDMAaddr1 = 0x20020000 + RGB565_COL_NUM;

volatile UINT32 frame_cnt = 0; //帧计数
volatile UINT32 addr_cnt = 0;
volatile UINT32 href_cnt = 0; //行数

void DVP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*******************************************************************************
 * @fn       DebugInit
 *
 * @brief    Initializes the UART1 peripheral.
 *
 * @param    baudrate - UART1 communication baud rate.
 *
 * @return   None
 */
void DebugInit(UINT32 baudrate)
{
    UINT32 x;
    UINT32 t = FREQ_SYS;

    x = 10 * t * 2 / 16 / baudrate;
    x = (x + 5) / 10;
    R8_UART1_DIV = 1;
    R16_UART1_DL = x;
    R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART1_LCR = RB_LCR_WORD_SZ;
    R8_UART1_IER = RB_IER_TXD_EN;
    R32_PA_SMT |= (1 << 8) | (1 << 7);
    R32_PA_DIR |= (1 << 8);
}

/*********************************************************************
 * @fn      UART1_Send_Byte
 *
 * @brief   UART1 send one byte data.
 *
 * @param   Data - UART send Data.
 *
 * @return  none
 */

void UART1_Send_Byte(UINT8 Data)
{
    while(R8_UART1_TFC == UART_FIFO_SIZE);
    R8_UART1_THR = Data;
}

/*********************************************************************
 * @fn      DVP_Init
 *
 * @brief   Init DVP
 *
 * @return  none
 */

void DVP_Init(void)
{
    R8_DVP_CR0 &= ~RB_DVP_MSK_DAT_MOD;

#if(DVP_Work_Mode == RGB565_MODE)
    //RGB565: VSYNC高电平有效, HSYNC高电平有效
    R8_DVP_CR0 |= RB_DVP_D8_MOD | RB_DVP_V_POLAR;
    R8_DVP_CR1 &= ~((RB_DVP_ALL_CLR) | RB_DVP_RCV_CLR);
    R16_DVP_ROW_NUM = RGB565_ROW_NUM; // rows
    R16_DVP_COL_NUM = RGB565_COL_NUM; // cols

    R32_DVP_DMA_BUF0 = RGB565_DVPDMAaddr0; //DMA addr0
    R32_DVP_DMA_BUF1 = RGB565_DVPDMAaddr1; //DMA addr1

#endif

#if(DVP_Work_Mode == JPEG_MODE)
    //JPEG:VSYNC高电平有效, HSYNC高电平有效
    R8_DVP_CR0 |= RB_DVP_D8_MOD | RB_DVP_V_POLAR | RB_DVP_JPEG;
    R8_DVP_CR1 &= ~(RB_DVP_ALL_CLR | RB_DVP_RCV_CLR);

    //JPEG模式下：cols: DMA长度  rows:无意义
    R16_DVP_COL_NUM = OV2640_JPEG_WIDTH;

    R32_DVP_DMA_BUF0 = JPEG_DVPDMAaddr0; //DMA addr0
    R32_DVP_DMA_BUF1 = JPEG_DVPDMAaddr1; //DMA addr1
#endif

    //Interupt Enable
    R8_DVP_INT_EN |= RB_DVP_IE_STP_FRM;
    R8_DVP_INT_EN |= RB_DVP_IE_FIFO_OV;
    R8_DVP_INT_EN |= RB_DVP_IE_FRM_DONE;
    R8_DVP_INT_EN |= RB_DVP_IE_ROW_DONE;
    R8_DVP_INT_EN |= RB_DVP_IE_STR_FRM;

    PFIC_EnableIRQ(DVP_IRQn); //enable DVP interrupt

    R8_DVP_CR1 |= RB_DVP_DMA_EN; //enable DMA
    R8_DVP_CR0 |= RB_DVP_ENABLE; //enable DVP
}

/*******************************************************************************
 * @fn DVP_IRQHandler
 *
 * @brief RGB565 - 取数据为 帧起始到帧接收完成 R16_DVP_ROW_NUM * R16_DVP_COL_NUM。
 *                  JPEG：取数据为  帧起始 到 帧结束  一帧数据中 取  0xFF，0xD8开头； 0xFF ,0xD9结尾
 *
 * @return None
 */
void DVP_IRQHandler(void)
{
    if(R8_DVP_INT_FLAG & RB_DVP_IF_ROW_DONE) //行结束中断
    {
        R8_DVP_INT_FLAG = RB_DVP_IF_ROW_DONE; //clear Interrupt

#if(DVP_Work_Mode == JPEG_MODE)
        href_cnt++;

        if(addr_cnt % 2) //buf1 done
        {
            addr_cnt++;
            R32_DVP_DMA_BUF1 += OV2640_JPEG_WIDTH * 2;
        }
        else //buf0 done
        {
            addr_cnt++;
            R32_DVP_DMA_BUF0 += OV2640_JPEG_WIDTH * 2;
        }

#endif

#if(DVP_Work_Mode == RGB565_MODE)
        if(addr_cnt % 2) //buf1 done
        {
            addr_cnt++;
            R32_DVP_DMA_BUF1 += RGB565_COL_NUM * 2;

            //Send USB数据
        }
        else //buf0 done
        {
            addr_cnt++;
            R32_DVP_DMA_BUF0 += RGB565_COL_NUM * 2;

            //Send USB数据
        }

        href_cnt++; //RGB565 接收行数

#endif
    }

    if(R8_DVP_INT_FLAG & RB_DVP_IF_FRM_DONE) //帧接收完成中断
    {
        R8_DVP_INT_FLAG = RB_DVP_IF_FRM_DONE; //clear Interrupt

#if(DVP_Work_Mode == JPEG_MODE)
        R8_DVP_CR0 &= ~RB_DVP_ENABLE; //disable DVP

        //使用串口摄像头，实时显示 JPEG 图片
        {
            UINT32 i;
            UINT8  val;

            href_cnt = href_cnt * OV2640_JPEG_WIDTH;

            for(i = 0; i < href_cnt; i++)
            {
                val = *(UINT8 *)(0x20020000 + i);
                UART1_Send_Byte(val);
            }
        }

        R8_DVP_CR0 |= RB_DVP_ENABLE; //enable DVP

        R32_DVP_DMA_BUF0 = JPEG_DVPDMAaddr0; //DMA addr0
        R32_DVP_DMA_BUF1 = JPEG_DVPDMAaddr1; //DMA addr1
        href_cnt = 0;

        addr_cnt = 0;

#endif

#if(DVP_Work_Mode == RGB565_MODE)

        addr_cnt = 0;
        href_cnt = 0;

#endif
    }

    if(R8_DVP_INT_FLAG & RB_DVP_IF_STR_FRM) //帧开始中断，
    {
        R8_DVP_INT_FLAG = RB_DVP_IF_STR_FRM; //clear Interrupt

        frame_cnt++;
    }

    if(R8_DVP_INT_FLAG & RB_DVP_IF_STP_FRM) //帧结束中断，
    {
        R8_DVP_INT_FLAG = RB_DVP_IF_STP_FRM; //clear Interrupt
    }

    if(R8_DVP_INT_FLAG & RB_DVP_IF_FIFO_OV) //FIFO 溢出中断
    {
        R8_DVP_INT_FLAG = RB_DVP_IF_FIFO_OV; //clear Interrupt

        PRINT("FIFO OV\r\n");
    }
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
    SystemInit(FREQ_SYS);
    Delay_Init(FREQ_SYS);

    /* 配置串口调试 */
    DebugInit(921600);
    PRINT("Start @ChipID=%02X\r\n", R8_CHIP_ID);

    while(OV2640_Init())
    {
        PRINT("Camera Model Err\r\n");
        mDelaymS(500);
    }

    mDelaymS(1000);

    RGB565_Mode_Init();
    mDelaymS(1000);

#if(DVP_Work_Mode == JPEG_MODE)
    JPEG_Mode_Init();
    mDelaymS(1000);

#endif

    DVP_Init();

    while(1);
}
