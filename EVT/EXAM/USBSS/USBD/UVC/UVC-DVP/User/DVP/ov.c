/********************************** (C) COPYRIGHT *******************************
* File Name          : ov.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "dvp.h"
#include "ov.h"
#include "ov2640cfg.h"
#include "UVCLIB.h"

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

	/*PA17:RESET  PA23:PWDN*/
	R32_PA_DIR |= (1<<17)|(1<<23);

	OV_PWDN_CLR;                            //POWER ON
	mDelaymS(10);
 	OV_RESET_CLR;				            //Reset OV2640
 	mDelaymS(10);
	OV_RESET_SET;				            //End reset

  	SCCB_Init();        		            //initialization SCCB
	SCCB_WR_Reg(OV2640_DSP_RA_DLMT, 0x01);	//Operation sensor register
 	SCCB_WR_Reg(OV2640_SENSOR_COM7, 0x80);	//Soft reset OV2640

 	mDelaymS(50);
	reg=SCCB_RD_Reg(OV2640_SENSOR_MIDH);	//read ID high
	reg<<=8;
	reg|=SCCB_RD_Reg(OV2640_SENSOR_MIDL);	//read ID low

	if(reg!=OV2640_MID)
	{
		printf("MID:%d\r\n",reg);
		return 1;
	}

	reg=SCCB_RD_Reg(OV2640_SENSOR_PIDH);	//read ID high
	reg<<=8;
	reg|=SCCB_RD_Reg(OV2640_SENSOR_PIDL);	//read ID low

	if(reg!=OV2640_PID)
	{
		printf("HID:%d\r\n",reg);
	}
  	return 0x00;
}

/*******************************************************************************
 * @fn      OV2640_Format_Mode
 * @brief   Ov2640 Select frame format
 * @param   None
 * @return  None
 */
void OV2640_Format_Mode( UINT8 choice )
{
    UINT16 i = 0;

    SCCB_WR_Reg(0xff, 0x01);
    SCCB_WR_Reg(0x12, 0x80);
    mDelaymS(5);

    switch( choice ){
        case FORMAT_MJPEG:
            /*initialization OV2640,uxga resolution(1600*1200) 15fps */
            for(i=0;i<sizeof(ov2640_uxga_init_reg_tbl)/2;i++)
            {
                SCCB_WR_Reg(ov2640_uxga_init_reg_tbl[i][0],ov2640_uxga_init_reg_tbl[i][1]);
            }
            for(i=0;i<(sizeof(ov2640_jpeg_reg_tbl)/2);i++)
            {
                SCCB_WR_Reg(ov2640_jpeg_reg_tbl[i][0],ov2640_jpeg_reg_tbl[i][1]);
            }
            break;
        case FORMAT_YUV2:
            for(i=0;i<sizeof(yuv_uxga)/2;i++)
            {
                SCCB_WR_Reg(yuv_uxga[i][0],yuv_uxga[i][1]);
            }
            break;
    }
}

/*******************************************************************************
 * @fn      OV2640_Speed_Mode
 *
 * @brief   switch clk speed
 *
 * @return  None
 */
void OV2640_Speed_Mode( UINT8 format )
{
    switch( format ){
        case FORMAT_MJPEG:
            ov2640_speed_ctrl(36,0);
            break;
        case FORMAT_YUV2:
            ov2640_speed_ctrl(1,0);
            break;
    }
}

/*******************************************************************************
 * @fn     OV2640_Change_Resolution
 *
 * @brief   Ov2640 Select resolution and frame rate
 *
 * @return  None
 */
void OV2640_Change_Resolution( UINT8 formatindex,UINT8 frameindex )
{
    switch( formatindex ){
        case FORMAT_MJPEG:
            if( Link_sta == 3)
            {
                Resolution_width  = ov2640_JPEGframe_resolution_USB30[frameindex-1][0];
                Resolution_height = ov2640_JPEGframe_resolution_USB30[frameindex-1][1];
            }
            else
            {
                Resolution_width  = ov2640_JPEGframe_resolution_USB20[frameindex-1][0];
                Resolution_height = ov2640_JPEGframe_resolution_USB20[frameindex-1][1];
            }
            break;
        case FORMAT_YUV2:
            if( Link_sta == 3)
            {
                Resolution_width  = ov2640_YUVframe_resolution_USB30[frameindex-1][0];
                Resolution_height = ov2640_YUVframe_resolution_USB30[frameindex-1][1];
            }
            else
            {
                Resolution_width  = ov2640_YUVframe_resolution_USB20[frameindex-1][0];
                Resolution_height = ov2640_YUVframe_resolution_USB20[frameindex-1][1];
            }
            break;
    }
    Dvp_Row_Col_Set(Resolution_width , Resolution_height);
    OV2640_OutSize_Set(Resolution_width,Resolution_height);
    OV2640_Speed_Mode(Formatchange_flag);
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
 * @brief    Auto exposure level setting
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
 * @brief    White balance settings
 *
 * @param    mode -
 *             0 - automatic
 *             1 - sunny
 *             2 - cloudy
 *             3 - office
 *             4 - home
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
 * @brief    Chroma settings
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
 * @brief    Brightness setting
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
 * @brief    Contrast setting
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
	UINT8 reg7d0val=0X20;//The default is normal mode
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
 * @brief    Special effect settings
 *
 * @param    eft -
 *             0 - Normal mode
 *             1 - negative film
 *             2 - black and white
 *             3 - Reddish
 *             4 - Greenish
 *             5 - Bluish
 *             6 - Retro
 *
 * @return   None
 */
void OV2640_Special_Effects(UINT8 eft)
{
	UINT8 reg7d0val=0X00;//The default is normal mode
	UINT8 reg7d1val=0X80;
	UINT8 reg7d2val=0X80;
	switch(eft)
	{
		case 1://negative film
			reg7d0val=0X40;
			break;
		case 2://black and white
			reg7d0val=0X18;
			break;
		case 3://Reddish
			reg7d0val=0X18;
			reg7d1val=0X40;
			reg7d2val=0XC0;
			break;
		case 4://Greenish
			reg7d0val=0X18;
			reg7d1val=0X40;
			reg7d2val=0X40;
			break;
		case 5://Bluish
			reg7d0val=0X18;
			reg7d1val=0XA0;
			reg7d2val=0X40;
			break;
		case 6://Retro
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
 * @brief    Color bar test
 *
 * @param    sw -
 *            0 - Turn off color bar
 *            1 - Open color bar
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
 * @brief    Set sensor output window
 *
 * @param    sx,sy - Starting address
 *           width,height - horizontal,vertical
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
	temp=SCCB_RD_Reg(0X03);
	temp&=0XF0;
	temp|=((endy&0X03)<<2)|(sy&0X03);
	SCCB_WR_Reg(0X03,temp);
	SCCB_WR_Reg(0X19,sy>>2);
	SCCB_WR_Reg(0X1A,endy>>2);

	temp=SCCB_RD_Reg(0X32);
	temp&=0XC0;
	temp|=((endx&0X07)<<3)|(sx&0X07);
	SCCB_WR_Reg(0X32,temp);
	SCCB_WR_Reg(0X17,sx>>3);
	SCCB_WR_Reg(0X18,endx>>3);
}

/*******************************************************************************
 * @fn       OV2640_OutSize_Set
 *
 * @brief    OV2640 Size (resolution) of output image
 *
 * @param    width,height - horizontal,vertical ,Width and height must be multiples of 4
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
	SCCB_WR_Reg(0X5A,outw&0XFF);
	SCCB_WR_Reg(0X5B,outh&0XFF);
	temp=(outw>>8)&0X03;
	temp|=(outh>>6)&0X04;
	SCCB_WR_Reg(0X5C,temp);
	SCCB_WR_Reg(0XE0,0X00);
	return 0;
}

/*******************************************************************************
 * @fn       OV2640_ImageWin_Set
 *
 * @brief    Set image window size
 *
 * @param    offx,offy - start address
 *           width,height - horizontal,vertical
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
	SCCB_WR_Reg(0X51,hsize&0XFF);
	SCCB_WR_Reg(0X52,vsize&0XFF);
	SCCB_WR_Reg(0X53,offx&0XFF);
	SCCB_WR_Reg(0X54,offy&0XFF);
	temp=(vsize>>1)&0X80;
	temp|=(offy>>4)&0X70;
	temp|=(hsize>>5)&0X08;
	temp|=(offx>>8)&0X07;
	SCCB_WR_Reg(0X55,temp);
	SCCB_WR_Reg(0X57,(hsize>>2)&0X80);
	SCCB_WR_Reg(0XE0,0X00);
	return 0;
}

/*******************************************************************************
 * @fn       OV2640_ImageSize_Set
 *
 * @brief    Image Resolution
 *
 * @param    Image_width
 *           Image_height
 *
 * @return   0 - Success
 *           1 - Error
 */
UINT8 OV2640_ImageSize_Set(UINT16 width,UINT16 height)
{
	UINT8 temp;
	SCCB_WR_Reg(0XFF,0X00);
	SCCB_WR_Reg(0XE0,0X04);
	SCCB_WR_Reg(0XC0,(width)>>3&0XFF);
	SCCB_WR_Reg(0XC1,(height)>>3&0XFF);
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
 *            Xclk_Div: Crystal oscillator input frequency division
 *
 * @return    none
 */

void ov2640_speed_ctrl(UINT8 pclkdiv, UINT8 clkdiv)
{
	SCCB_WR_Reg(0XFF,0X00);
	SCCB_WR_Reg(0XD3,pclkdiv);	//Set PCLK frequency division
	SCCB_WR_Reg(0XFF,0X01);
	SCCB_WR_Reg(0X11,clkdiv);	//Set CLK frequency division
}

