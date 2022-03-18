/********************************** (C) COPYRIGHT *******************************
* File Name          : dvp.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/08/6
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#ifndef DVP_DVP_H_
#define DVP_DVP_H_
#include "CH56x_common.h"
#include "ov.h"

#define JPEG_MODE     1

#define MODE_Vedio    JPEG_MODE

extern __attribute__ ((aligned(16))) UINT8	JPEG_DVPDMAaddr0[512] __attribute__((section(".DMADATA")));
extern __attribute__ ((aligned(16))) UINT8	JPEG_DVPDMAaddr1[512] __attribute__((section(".DMADATA")));
//IIC
#define IIC_SCL_OUT    {R32_PA_DIR |= 1<<21;}   //direction: 0=in, 1=out
#define IIC_SDA_OUT    {R32_PA_DIR |= 1<<22;}
#define IIC_SDA_IN     {R32_PA_DIR &= ~(1<<22);}

#define IIC_SCL_SET    {R32_PA_OUT |= 1<<21;}  //输出高
#define IIC_SCL_CLR    {R32_PA_CLR |= 1<<21;}  //输出低
#define IIC_SDA_SET    {R32_PA_OUT |= 1<<22;}  //输出高
#define IIC_SDA_CLR    {R32_PA_CLR |= 1<<22;}  //输出低

//SDA输入电平状态
#define SDA_IN_R	   		(R32_PA_PIN & (1<<22))

//器件ID
#define SCCB_ID   			0X60  			//OV2640的ID


//320*240
#define RGB565_ROW_NUM   320
#define RGB565_COL_NUM   480   //RGB 565 和 列*2

//JPEG分辨率
#define OV2640_JPEG_WIDTH	1024		//JPEG拍照的宽度
#define OV2640_JPEG_HEIGHT	768		//JPEG拍照的高度



///////////////////////////////////////////
void SCCB_Init(void);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_No_Ack(void);
UINT8 SCCB_WR_Byte(UINT8 dat);
UINT8 SCCB_RD_Byte(void);
UINT8 SCCB_WR_Reg(UINT8 reg,UINT8 data);
UINT8 SCCB_RD_Reg(UINT8 reg);

void dvp_Init(void);

#endif /* DVP_DVP_H_ */
