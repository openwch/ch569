/********************************** (C) COPYRIGHT *******************************
* File Name          : dvp.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/08/6
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#include "dvp.h"
#include "UVCLIB.H"
void DVP_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));
__attribute__ ((aligned(16))) UINT8	JPEG_DVPDMAaddr0[512] __attribute__((section(".DMADATA")));
__attribute__ ((aligned(16))) UINT8	JPEG_DVPDMAaddr1[512] __attribute__((section(".DMADATA")));


/*******************************************************************************
 * @fn     DVP_IRQHandler
 *
 * @brief  RGB565 - 取数据为 帧起始到帧接收完成 R16_DVP_ROW_NUM * R16_DVP_COL_NUM。
 *         JPEG - 取数据为  帧起始 到 帧结束  一帧数据中 取  0xFF，0xD8开头； 0xFF ,0xD9结尾
 *
 * @return   None
 */
void DVP_IRQHandler(void)
{
	DVP_Hander();
}
/*******************************************************************************
 * @fn     dvp_Init
 *
 * @brief
 *
 * @return   None
 */
void dvp_Init(void)
{
	R8_DVP_CR0 &= ~RB_DVP_MSK_DAT_MOD;
	//JPEG - VSYNC高电平有效, HSYNC高电平有效
	R8_DVP_CR0 |= RB_DVP_D8_MOD | RB_DVP_V_POLAR | RB_DVP_JPEG;
	R8_DVP_CR1 &= ~(RB_DVP_ALL_CLR| RB_DVP_RCV_CLR);  
	R16_DVP_COL_NUM = 512;			

	R32_DVP_DMA_BUF0 = (UINT32)JPEG_DVPDMAaddr0;		//DMA addr0
	R32_DVP_DMA_BUF1 =  (UINT32)JPEG_DVPDMAaddr1;		//DMA addr1

    //Interupt Enable
	R8_DVP_INT_EN |= RB_DVP_IE_STP_FRM;
	R8_DVP_INT_EN |= RB_DVP_IE_FRM_DONE;
	R8_DVP_INT_EN |= RB_DVP_IE_ROW_DONE;
	R8_DVP_INT_EN |= RB_DVP_IE_STR_FRM;

	PFIC_EnableIRQ(DVP_IRQn);		//enable DVP interrupt
	R8_DVP_CR1 |= RB_DVP_DMA_EN;  //enable DMA
	R8_DVP_CR0 |= RB_DVP_ENABLE;  //enable DVP
}
/*******************************************************************************
 * @fn     SCCB_Init
 *
 * @brief  初始化SCCB接口
 *
 * @return   None
 */
void SCCB_Init(void)
{
	IIC_SCL_OUT;  
	IIC_SDA_OUT; 

	IIC_SCL_SET; 
	IIC_SDA_SET;  
}


/*******************************************************************************
 * @fn       SCCB_Start
 *
 * @brief    Start Signal
 *
 * @return   None
 */

void SCCB_Start(void)
{
	IIC_SDA_SET; 
	IIC_SCL_SET;  
	mDelayuS(50);
	IIC_SDA_CLR;  
	mDelayuS(50);
	IIC_SCL_CLR; 
}

/*******************************************************************************
 * @fn        SCCB_Stop
 *
 * @brief     Stop Signal
 *
 * @return    None
 */

void SCCB_Stop(void)
{
	IIC_SDA_CLR;  
	mDelayuS(50);
	IIC_SCL_SET;  
	mDelayuS(50);
	IIC_SDA_SET;  
	mDelayuS(50);
}

/*******************************************************************************
 * @fn      SCCB_No_Ack
 *
 * @brief   NAK Signal
 *
 * @return  None
 */

void SCCB_No_Ack(void)
{
	mDelayuS(50);
	IIC_SDA_SET;  
	IIC_SCL_SET;  
	mDelayuS(50);
	IIC_SCL_CLR; 
	mDelayuS(50);
	IIC_SDA_CLR;  
	mDelayuS(50);
}

/*******************************************************************************
 * @fn        SCCB_WR_Byte
 * @brief     Write One Byte
 * @param     data
 * @return    0 - 成功
 *            其他 - 失败
 */

UINT8 SCCB_WR_Byte(UINT8 dat)
{
	UINT8 j,res;

	for(j=0;j<8;j++) 
	{
		if(dat&0x80){
			IIC_SDA_SET;
		}
		else{
		    IIC_SDA_CLR;
		}

		dat<<=1;
		mDelayuS(50);
		IIC_SCL_SET;
		mDelayuS(50);
		IIC_SCL_CLR;
	}

	IIC_SDA_IN;		

	mDelayuS(50);
	IIC_SCL_SET;			
	mDelayuS(50);
	if(SDA_IN_R)res=1;  
	else res=0;         
	IIC_SCL_CLR;

	IIC_SDA_OUT;
	return res;

}

/*******************************************************************************
 * @fn        SCCB_RD_Byte
 *
 * @brief     Read One Byte
 *
 * @return    Read one byte data
 */

UINT8 SCCB_RD_Byte(void)
{
	UINT8 temp=0,j;

	IIC_SDA_IN;		

	for(j=8;j>0;j--) 
	{
		mDelayuS(50);
		IIC_SCL_SET;
		temp=temp<<1;
		if(SDA_IN_R)temp++;
		mDelayuS(50);
		IIC_SCL_CLR;
	}

	IIC_SDA_OUT;	

	return temp;
}

/*******************************************************************************
 * @fn        SCCB_WR_Reg
 *
 * @brief     Write camera Register
 *
 * @param     Reg_Adr - Register address
 *            Reg_Val - Register value
 *
 * @return    0 - 成功
 *            其他 - 失败
 */

UINT8 SCCB_WR_Reg(UINT8 reg,UINT8 data)
{
	UINT8 res=0;

	SCCB_Start(); 				
	if(SCCB_WR_Byte(SCCB_ID))res=1;	
	mDelayuS(100);
  	if(SCCB_WR_Byte(reg))res=1;		
  	mDelayuS(100);
  	if(SCCB_WR_Byte(data))res=1; 	
  	SCCB_Stop();
  	return	res;
}

/*******************************************************************************
 * @fn         SCCB_RD_Reg
 *
 * @brief      Read camera Register
 *
 * @return     Camera Register value
 */

UINT8 SCCB_RD_Reg(UINT8 reg)
{
	UINT8 val=0;
	SCCB_Start(); 			
	SCCB_WR_Byte(SCCB_ID);		
	mDelayuS(100);
  	SCCB_WR_Byte(reg);		
  	mDelayuS(100);
	SCCB_Stop();
	mDelayuS(100);

	SCCB_Start();
	SCCB_WR_Byte(SCCB_ID|0X01);	
	mDelayuS(100);
  	val=SCCB_RD_Byte();		 	
  	SCCB_No_Ack();
  	SCCB_Stop();
  	return val;
}


