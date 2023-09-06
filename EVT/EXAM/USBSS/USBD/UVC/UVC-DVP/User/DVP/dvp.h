/********************************** (C) COPYRIGHT *******************************
* File Name          : dvp.h
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef DVP_DVP_H_
#define DVP_DVP_H_

#include "CH56x_common.h"
#include "ov.h"

/* Global define */
/*IIC*/
#define IIC_SCL_OUT    {R32_PA_DIR |= 1<<21;}  //direction: 0=in, 1=out
#define IIC_SDA_OUT    {R32_PA_DIR |= 1<<22;}
#define IIC_SDA_IN     {R32_PA_DIR &= ~(1<<22);}
#define IIC_SCL_SET    {R32_PA_OUT |= 1<<21;}  //Output high
#define IIC_SCL_CLR    {R32_PA_CLR |= 1<<21;}  //Output low
#define IIC_SDA_SET    {R32_PA_OUT |= 1<<22;}  //Output high
#define IIC_SDA_CLR    {R32_PA_CLR |= 1<<22;}  //Output low
/*SDA Input level status*/
#define SDA_IN_R	(R32_PA_PIN & (1<<22))
/*ID*/
#define SCCB_ID   	 0X60  	//OV2640 ID

#define UVC_DMA_SIZE 1024*80

/* Global Variable */
extern __attribute__ ((aligned(16))) UINT8  UVC_DMABuffer[UVC_DMA_SIZE] __attribute__((section(".DMADATA")));

/* Function declaration */
void SCCB_Init(void);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_No_Ack(void);
UINT8 SCCB_WR_Byte(UINT8 dat);
UINT8 SCCB_RD_Byte(void);
UINT8 SCCB_WR_Reg(UINT8 reg,UINT8 data);
UINT8 SCCB_RD_Reg(UINT8 reg);
void dvp_Init(void);
void Dvp_Row_Col_Set( UINT16 res_width , UINT16 res_height );
#endif /* DVP_DVP_H_ */
