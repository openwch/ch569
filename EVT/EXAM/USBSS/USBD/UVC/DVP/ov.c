/********************************** (C) COPYRIGHT *******************************
* File Name          : ov.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : OV2640 摄像头 配置函数
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#include "dvp.h"
#include "ov.h"
#include "ov2640cfg.h"
#include "CH56xSFR.h"


/*********************************************************************
 * @fn      OV2640_Init
 *
 * @brief   Init OV2640
 *
 * @return  0 - Success
 *          1 - Err
 */
UINT8 OV2640_Init(void)
{
	UINT16 i=0;
	UINT16 reg;

	//PA17:RESET  PA23:PWDN
	R32_PA_DIR |= (1<<17)|(1<<23);

	OV_PWDN_CLR;        //POWER ON
	mDelaymS(10);
 	OV_RESET_CLR;				//复位OV2640
 	mDelaymS(10);
	OV_RESET_SET;				//结束复位

  	SCCB_Init();        		//初始化SCCB 的IO口
	SCCB_WR_Reg(OV2640_DSP_RA_DLMT, 0x01);	//操作sensor寄存器
 	SCCB_WR_Reg(OV2640_SENSOR_COM7, 0x80);	//软复位OV2640

 	mDelaymS(50);
	reg=SCCB_RD_Reg(OV2640_SENSOR_MIDH);	//读取厂家ID 高八位
	reg<<=8;
	reg|=SCCB_RD_Reg(OV2640_SENSOR_MIDL);	//读取厂家ID 低八位

	if(reg!=OV2640_MID)
	{
		printf("MID:%d\r\n",reg);
		return 1;
	}

	reg=SCCB_RD_Reg(OV2640_SENSOR_PIDH);	//读取厂家ID 高八位
	reg<<=8;
	reg|=SCCB_RD_Reg(OV2640_SENSOR_PIDL);	//读取厂家ID 低八位

	if(reg!=OV2640_PID)
	{
		printf("HID:%d\r\n",reg);
		//return 2;
	}
 	//初始化 OV2640,采用SXGA分辨率(1600*1200)
	for(i=0;i<sizeof(ov2640_uxga_init_reg_tbl)/2;i++)
	{
	   	SCCB_WR_Reg(ov2640_uxga_init_reg_tbl[i][0],ov2640_uxga_init_reg_tbl[i][1]);
 	}

  	return 0x00; 	//ok
}


/*******************************************************************************
 * @fn     JPEG_Mode_Init
 *
 * @brief   以 0xFF，0xD8开头； 0xFF ,0xD9结尾
 *
 * @return   None
 */
void JPEG_Mode_Init(void)
{
	UINT8 res=0;
	UINT32 bwr;
	UINT32 i=0;
	UINT32 jpeglen=0;
	UINT8* pbuf;

	OV2640_JPEG_Mode();							//切换为JPEG模式
  	OV2640_OutSize_Set(OV2640_JPEG_WIDTH,OV2640_JPEG_HEIGHT);

//	ov2640_speed_ctrl(30,1);  //输出速度，时钟来自谁
 	ov2640_speed_ctrl(36,0);  //输出速度，时钟来自谁
}

/*******************************************************************************
 * @fn     OV2640_JPEG_Mode
 * @brief   OV2640切换为JPEG模式
 * @param   None
 * @return   None
 */
void OV2640_JPEG_Mode(void)
{
	UINT16 i=0;
	//设置:YUV422格式
	for(i=0;i<(sizeof(ov2640_yuv422_reg_tbl)/2);i++)
	{
		SCCB_WR_Reg(ov2640_yuv422_reg_tbl[i][0],ov2640_yuv422_reg_tbl[i][1]);
	}
	//设置:输出JPEG数据
	for(i=0;i<(sizeof(ov2640_jpeg_reg_tbl)/2);i++)
	{
		SCCB_WR_Reg(ov2640_jpeg_reg_tbl[i][0],ov2640_jpeg_reg_tbl[i][1]);
	}
}

/* Start Camera list of initialization configuration registers */
const static UINT8 OV2640_AUTOEXPOSURE_LEVEL[5][8]=
{
	{
		0xFF,0x01,
		0x24,0x20,
		0x25,0x18,
		0x26,0x60,
	},
	{
		0xFF,0x01,
		0x24,0x34,
		0x25,0x1c,
		0x26,0x00,
	},
	{
		0xFF,0x01,
		0x24,0x3e,
		0x25,0x38,
		0x26,0x81,
	},
	{
		0xFF,0x01,
		0x24,0x48,
		0x25,0x40,
		0x26,0x81,
	},
	{
		0xFF,0x01,
		0x24,0x58,
		0x25,0x50,
		0x26,0x92,
	},
};


/*******************************************************************************
 * @fn       OV2640_Auto_Exposure
 *
 * @brief    自动曝光等级设置
 *
 * @param    level - 0~4.
 *
 * @return   None
 */
void OV2640_Auto_Exposure(UINT8 level)
{
	UINT8 i;
	UINT8 *p=(UINT8*)OV2640_AUTOEXPOSURE_LEVEL[level];
	for(i=0;i<4;i++)
	{
		SCCB_WR_Reg(p[i*2],p[i*2+1]);
	}
}


/*******************************************************************************
 * @fn       OV2640_Light_Mode
 *
 * @brief    白平衡设置
 *
 * @param    mode -
 *             0 - 自动
 *             1 - 太阳sunny
 *             2 - 阴天cloudy
 *             3 - 办公室office
 *             4 - 家里home
 *
 * @return   None
 */
void OV2640_Light_Mode(UINT8 mode)
{
	UINT8 regccval=0X5E;//Sunny
	UINT8 regcdval=0X41;
	UINT8 regceval=0X54;
	switch(mode)
	{
		case 0://auto
			SCCB_WR_Reg(0XFF,0X00);
			SCCB_WR_Reg(0XC7,0X00);//AWB ON
			return;
		case 2://cloudy
			regccval=0X65;
			regcdval=0X41;
			regceval=0X4F;
			break;
		case 3://office
			regccval=0X52;
			regcdval=0X41;
			regceval=0X66;
			break;
		case 4://home
			regccval=0X42;
			regcdval=0X3F;
			regceval=0X71;
			break;
	}
	SCCB_WR_Reg(0XFF,0X00);
	SCCB_WR_Reg(0XC7,0X40);	//AWB OFF
	SCCB_WR_Reg(0XCC,regccval);
	SCCB_WR_Reg(0XCD,regcdval);
	SCCB_WR_Reg(0XCE,regceval);
}


/*******************************************************************************
 * @fn       OV2640_Color_Saturation
 *
 * @brief    色度设置
 *
 * @param    sat-
 *             0 - -2
 *             1 - -1
 *             2 - 0
 *             3 - +1
 *             4 - +2
 *
 * @return   None
 */
void OV2640_Color_Saturation(UINT8 sat)
{
	UINT8 reg7dval=((sat+2)<<4)|0X08;
	SCCB_WR_Reg(0XFF,0X00);
	SCCB_WR_Reg(0X7C,0X00);
	SCCB_WR_Reg(0X7D,0X02);
	SCCB_WR_Reg(0X7C,0X03);
	SCCB_WR_Reg(0X7D,reg7dval);
	SCCB_WR_Reg(0X7D,reg7dval);
}


/*******************************************************************************
 * @fn       OV2640_Brightness
 *
 * @brief    亮度设置
 *
 * @param    bright -
 *             0 - (0X00)-2
 *             1 - (0X10)-1
 *             2 - (0X20) 0
 *             3 - (0X30)+1
 *             4 - (0X40)+2
 *
 * @return   None
 */
void OV2640_Brightness(UINT8 bright)
{
  SCCB_WR_Reg(0xff, 0x00);
  SCCB_WR_Reg(0x7c, 0x00);
  SCCB_WR_Reg(0x7d, 0x04);
  SCCB_WR_Reg(0x7c, 0x09);
  SCCB_WR_Reg(0x7d, bright<<4);
  SCCB_WR_Reg(0x7d, 0x00);
}


/*******************************************************************************
 * @fn       OV2640_Contrast
 *
 * @brief    对比度设置
 *
 * @param    contrast - 
 *             0 - -2
 *             1 - -1
 *             2 - 0
 *             3 - +1
 *             4 - +2
 *
 * @return   None
 */
void OV2640_Contrast(UINT8 contrast)
{
	UINT8 reg7d0val=0X20;//默认为普通模式
	UINT8 reg7d1val=0X20;
  	switch(contrast)
	{
		case 0://-2
			reg7d0val=0X18;
			reg7d1val=0X34;
			break;
		case 1://-1
			reg7d0val=0X1C;
			reg7d1val=0X2A;
			break;
		case 3://1
			reg7d0val=0X24;
			reg7d1val=0X16;
			break;
		case 4://2
			reg7d0val=0X28;
			reg7d1val=0X0C;
			break;
	}
	SCCB_WR_Reg(0xff,0x00);
	SCCB_WR_Reg(0x7c,0x00);
	SCCB_WR_Reg(0x7d,0x04);
	SCCB_WR_Reg(0x7c,0x07);
	SCCB_WR_Reg(0x7d,0x20);
	SCCB_WR_Reg(0x7d,reg7d0val);
	SCCB_WR_Reg(0x7d,reg7d1val);
	SCCB_WR_Reg(0x7d,0x06);
}


/*******************************************************************************
 * @fn       OV2640_Special_Effects
 *
 * @brief    特效设置
 *
 * @param    eft -
 *             0 - 普通模式
 *             1 - 负片
 *             2 - 黑白
 *             3 - 偏红色
 *             4 - 偏绿色
 *             5 - 偏蓝色
 *             6 - 复古
 *
 * @return   None
 */
void OV2640_Special_Effects(UINT8 eft)
{
	UINT8 reg7d0val=0X00;//默认为普通模式
	UINT8 reg7d1val=0X80;
	UINT8 reg7d2val=0X80;
	switch(eft)
	{
		case 1://负片
			reg7d0val=0X40;
			break;
		case 2://黑白
			reg7d0val=0X18;
			break;
		case 3://偏红色
			reg7d0val=0X18;
			reg7d1val=0X40;
			reg7d2val=0XC0;
			break;
		case 4://偏绿色
			reg7d0val=0X18;
			reg7d1val=0X40;
			reg7d2val=0X40;
			break;
		case 5://偏蓝色
			reg7d0val=0X18;
			reg7d1val=0XA0;
			reg7d2val=0X40;
			break;
		case 6://复古
			reg7d0val=0X18;
			reg7d1val=0X40;
			reg7d2val=0XA6;
			break;
	}
	SCCB_WR_Reg(0xff,0x00);
	SCCB_WR_Reg(0x7c,0x00);
	SCCB_WR_Reg(0x7d,reg7d0val);
	SCCB_WR_Reg(0x7c,0x05);
	SCCB_WR_Reg(0x7d,reg7d1val);
	SCCB_WR_Reg(0x7d,reg7d2val);
}


/*******************************************************************************
 * @fn       OV2640_Color_Bar
 *
 * @brief    彩条测试
 *
 * @param    sw -
 *            0 - 关闭彩条
 *            1 - 开启彩条
 *
 * @return   None
 */
void OV2640_Color_Bar(UINT8 sw)
{
	UINT8 reg;
	SCCB_WR_Reg(0XFF,0X01);
	reg=SCCB_RD_Reg(0X12);
	reg&=~(1<<1);
	if(sw)reg|=1<<1;
	SCCB_WR_Reg(0X12,reg);
}

/*******************************************************************************
 * @fn       OV2640_Window_Set
 *
 * @brief    设置传感器输出窗口
 *
 * @param    sx,sy - 起始地址
 *           width,height - 宽度(对应:horizontal)和高度(对应:vertical)
 *
 * @return   None
 */
void OV2640_Window_Set(UINT16 sx,UINT16 sy,UINT16 width,UINT16 height)
{
	UINT16 endx;
	UINT16 endy;
	UINT8 temp;
	endx=sx+width/2;	//V*2
 	endy=sy+height/2;

	SCCB_WR_Reg(0XFF,0X01);
	temp=SCCB_RD_Reg(0X03);				//读取Vref之前的值
	temp&=0XF0;
	temp|=((endy&0X03)<<2)|(sy&0X03);
	SCCB_WR_Reg(0X03,temp);				//设置Vref的start和end的最低2位
	SCCB_WR_Reg(0X19,sy>>2);			//设置Vref的start高8位
	SCCB_WR_Reg(0X1A,endy>>2);			//设置Vref的end的高8位

	temp=SCCB_RD_Reg(0X32);				//读取Href之前的值
	temp&=0XC0;
	temp|=((endx&0X07)<<3)|(sx&0X07);
	SCCB_WR_Reg(0X32,temp);				//设置Href的start和end的最低3位
	SCCB_WR_Reg(0X17,sx>>3);			//设置Href的start高8位
	SCCB_WR_Reg(0X18,endx>>3);			//设置Href的end的高8位
}

/*******************************************************************************
 * @fn       设置图像输出大小
 *
 * @brief    OV2640输出图像的大小(分辨率)
 *
 * @param    width,height - 宽度(horizontal)和高度(vertical),width和height必须是4的倍数         
 *
 * @return   0 - Success
 *           1 - Error
 */
UINT8 OV2640_OutSize_Set(UINT16 width,UINT16 height)
{
	UINT16 outh;
	UINT16 outw;
	UINT8 temp;
	if(width%4)return 1;
	if(height%4)return 2;
	outw=width/4;
	outh=height/4;
	SCCB_WR_Reg(0XFF,0X00);
	SCCB_WR_Reg(0XE0,0X04);
	SCCB_WR_Reg(0X5A,outw&0XFF);		//设置OUTW的低八位
	SCCB_WR_Reg(0X5B,outh&0XFF);		//设置OUTH的低八位
	temp=(outw>>8)&0X03;
	temp|=(outh>>6)&0X04;
	SCCB_WR_Reg(0X5C,temp);				//设置OUTH/OUTW的高位
	SCCB_WR_Reg(0XE0,0X00);
	return 0;
}

/*******************************************************************************
 * @fn       OV2640_ImageWin_Set
 *
 * @brief    设置图像开窗大小
 *
 * @param    offx,offy - 起始地址
 *           width,height - 宽度(对应:horizontal)和高度(对应:vertical)
 *
 * @return   None
 */
UINT8 OV2640_ImageWin_Set(UINT16 offx,UINT16 offy,UINT16 width,UINT16 height)
{
	UINT16 hsize;
	UINT16 vsize;
	UINT8 temp;
	if(width%4)return 1;
	if(height%4)return 2;
	hsize=width/4;
	vsize=height/4;
	SCCB_WR_Reg(0XFF,0X00);
	SCCB_WR_Reg(0XE0,0X04);
	SCCB_WR_Reg(0X51,hsize&0XFF);		//设置H_SIZE的低八位
	SCCB_WR_Reg(0X52,vsize&0XFF);		//设置V_SIZE的低八位
	SCCB_WR_Reg(0X53,offx&0XFF);		//设置offx的低八位
	SCCB_WR_Reg(0X54,offy&0XFF);		//设置offy的低八位
	temp=(vsize>>1)&0X80;
	temp|=(offy>>4)&0X70;
	temp|=(hsize>>5)&0X08;
	temp|=(offx>>8)&0X07;
	SCCB_WR_Reg(0X55,temp);				//设置H_SIZE/V_SIZE/OFFX,OFFY的高位
	SCCB_WR_Reg(0X57,(hsize>>2)&0X80);	//设置H_SIZE/V_SIZE/OFFX,OFFY的高位
	SCCB_WR_Reg(0XE0,0X00);
	return 0;
}

/*******************************************************************************
 * @fn       OV2640_ImageSize_Set
 *
 * @brief    Image Resolution
 *
 * @param    Image_width - 图像宽度 (4的倍数)
 *           Image_height - 图像高度
 *
 * @return   0 - Success
 *           1 - Error
 */
UINT8 OV2640_ImageSize_Set(UINT16 width,UINT16 height)
{
	UINT8 temp;
	SCCB_WR_Reg(0XFF,0X00);
	SCCB_WR_Reg(0XE0,0X04);
	SCCB_WR_Reg(0XC0,(width)>>3&0XFF);		//设置HSIZE的10:3位
	SCCB_WR_Reg(0XC1,(height)>>3&0XFF);		//设置VSIZE的10:3位
	temp=(width&0X07)<<3;
	temp|=height&0X07;
	temp|=(width>>4)&0X80;
	SCCB_WR_Reg(0X8C,temp);
	SCCB_WR_Reg(0XE0,0X00);
	return 0;
}

/*******************************************************************************
 * @fn        ov2640_speed_ctrl
 *
 * @brief     Set DVP PCLK
 *
 * @param     Pclk_Div: DVP output speed ctrl
 *            Xclk_Div: 晶振输入分频
 *
 * @return    none
 */

void ov2640_speed_ctrl(UINT8 pclkdiv, UINT8 clkdiv)
{
	SCCB_WR_Reg(0XFF,0X00);
	SCCB_WR_Reg(0XD3,pclkdiv);	//设置PCLK分频
	SCCB_WR_Reg(0XFF,0X01);
	SCCB_WR_Reg(0X11,clkdiv);	//设置CLK分频
}

