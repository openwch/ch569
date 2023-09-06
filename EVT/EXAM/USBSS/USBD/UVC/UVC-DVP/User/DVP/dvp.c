/********************************** (C) COPYRIGHT *******************************
* File Name          : dvp.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "dvp.h"
#include "UVCLIB.H"

void DVP_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));
__attribute__ ((aligned(16))) UINT8	UVC_DMABuffer[UVC_DMA_SIZE] __attribute__((section(".DMADATA")));

/*******************************************************************************
 * @fn     DVP_IRQHandler
 *
 * @brief  RGB565 - Take the data as: frame start to frame reception completion R16_DVP_ROW_NUM * R16_DVP_COL_NUM.
 *         JPEG - The data taken is: 0xff and 0xd8 are taken from the data of one frame from the start of the frame
 *                to the end of the frame; 0xff, 0xd9 end
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
    R8_DVP_CR0 |= RB_DVP_D8_MOD | RB_DVP_V_POLAR /*| RB_DVP_RAW_CM*/ /*| RB_DVP_JPEG*/;
    R8_DVP_CR1 &= ~((RB_DVP_ALL_CLR) | RB_DVP_RCV_CLR);

    R32_DVP_DMA_BUF0 = (UINT32)(UINT8 *)Dvp_Recaddr;		//DMA addr0
	R32_DVP_DMA_BUF1 = (UINT32)(UINT8 *)Dvp_Recaddr+Dvp_DataSize;		//DMA addr1

    /*Interupt Enable*/
	R8_DVP_INT_EN |= RB_DVP_IE_STP_FRM;                 //Frame end interrupt enable
	R8_DVP_INT_EN |= RB_DVP_IE_FRM_DONE;                //Frame receiving completion interrupt enable
	R8_DVP_INT_EN |= RB_DVP_IE_ROW_DONE;                //Line end interrupt enable
	R8_DVP_INT_EN |= RB_DVP_IE_STR_FRM;                 //New frame start interrupt enable
    R8_DVP_INT_EN |= RB_DVP_IE_FIFO_OV;

	R8_DVP_CR1 |= RB_DVP_DMA_EN;  //enable DMA
	R8_DVP_CR0 |= RB_DVP_ENABLE;  //enable DVP

}

/*******************************************************************************
 * @fn       DVP_ROW_COL_Set
 *
 * @brief    Set the size of one row and one column
 *           res_width: row
 *           res_height col
 *
 * @return   None
 */
void Dvp_Row_Col_Set( UINT16 res_width , UINT16 res_height )
{
    switch(Formatchange_flag)
    {
        case FORMAT_MJPEG:
            R16_DVP_ROW_NUM = 0;
            R16_DVP_COL_NUM = 512;
            Dvp_DataSize    = 512;
            break;
        case FORMAT_YUV2:
            R16_DVP_ROW_NUM = res_height;
            R16_DVP_COL_NUM = res_width *2;
            Dvp_DataSize    = res_width *2;
            break;
    }
}

/*******************************************************************************
 * @fn     SCCB_Init
 *
 * @brief  Initialize SCCB
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
 * @return    0 - success
 *            other - fail
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
 * @return    0 - success
 *            other - fail
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


