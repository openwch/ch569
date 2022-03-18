/********************************** (C) COPYRIGHT *******************************
* File Name          : ov.H
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : OV2640 摄像头 配置函数
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef __OV_H
#define __OV_H


//RGB565 PIXEL 320*240
#define RGB565_ROW_NUM   320
#define RGB565_COL_NUM   480   //列*2
#define OV2640_RGB565_HEIGHT   320
#define OV2640_RGB565_WIDTH	   240

//JPEG PIXEL 1024 * 768
#define OV2640_JPEG_HEIGHT	768
#define OV2640_JPEG_WIDTH	1024

//PA17:RESET  PA23:PWDN
#define OV_RESET_SET    {R32_PA_OUT |= 1<<17;}  //输出高
#define OV_RESET_CLR    {R32_PA_CLR |= 1<<17;}  //输出低
#define OV_PWDN_SET     {R32_PA_OUT |= 1<<23;}  //输出高
#define OV_PWDN_CLR     {R32_PA_CLR |= 1<<23;}  //输出低

//OV2640的ID
#define SCCB_ID   			    0X60
#define OV2640_MID				0X7FA2
#define OV2640_PID				0X2642

//SCCB Ctrl GPIO
#define IIC_SCL_OUT    {R32_PA_DIR |= 1<<21;}
#define IIC_SDA_OUT    {R32_PA_DIR |= 1<<22;}
#define IIC_SDA_IN     {R32_PA_DIR &= ~(1<<22);}

#define IIC_SCL_SET    {R32_PA_OUT |= 1<<21;}  //输出高
#define IIC_SCL_CLR    {R32_PA_CLR |= 1<<21;}  //输出低
#define IIC_SDA_SET    {R32_PA_OUT |= 1<<22;}  //输出高
#define IIC_SDA_CLR    {R32_PA_CLR |= 1<<22;}  //输出低

//SDA In
#define SDA_IN_R	   (R32_PA_PIN & (1<<22))

void SCCB_GPIO_Init(void);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_No_Ack(void);
UINT8 SCCB_WR_Byte(UINT8 data);
UINT8 SCCB_RD_Byte(void);
UINT8 SCCB_WR_Reg(UINT8 Reg_Adr,UINT8 Reg_Val);
UINT8 SCCB_RD_Reg(UINT8 Reg_Adr);

void RGB565_Mode_Init(void);
void JPEG_Mode_Init(void);
UINT8 OV2640_Init(void);
void OV2640_JPEG_Mode(void);
void OV2640_RGB565_Mode(void);
UINT8 OV2640_OutSize_Set(UINT16 Image_width,UINT16 Image_height);
void OV2640_Speed_Set(UINT8 Pclk_Div, UINT8 Xclk_Div);



#endif

