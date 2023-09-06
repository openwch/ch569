/********************************** (C) COPYRIGHT *******************************
* File Name          : uvc.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "UVCLIB.H"

/* Global define */


/* Global Variable */
__attribute__ ((aligned(16))) UINT8 YUV2_addr[1024 * 16] __attribute__((section(".DMADATA")));

static UINT32V class=0;
static UINT32V UVC_DVP;
static UINT8V  Totalcnt=0;
static UINT16V packnum;
static UINT8V lastFrame = 0;
static UINT32V UVC_MBuf_PackCount = 0x00;                                              /* UVC Data Microbuffer Packet Count*/
static UINT8V  UVC_MFrame_PackCount = 0x00;                                            /* UVC Microframe packet count */
static UINT32V UVC_MFrame_PackTotalNum = 0x00;                                         /* UVC Total number of current micro frames */
static UINT32V UVC_MFrame_LastPackLen = 0x00;                                          /* UVC The packet length at the end of the current microframe */
static UINT32V UVC_MFrame_ValidFlag = 0x00;                                            /* UVC Valid flag of microframe data(0:invalid; 1£ºvalid;) */
UINT16V Resolution_width = 0;
UINT16V Resolution_height = 0;

/*UVC Frame header structure definition*/
typedef struct __attribute__((packed))
 {
    UINT8 len;
    UINT8 tog;
    UINT32 Frequeny;
    UINT32 Clock;
    UINT16 count;
    UINT32 reallen;
 }Picture;
Picture MJPEG,YUV2;

typedef struct __attribute__((packed))
{
    UINT32 Length[ 2 ];                                           /* UVC Buffer effective data length */
    UINT8  FullFlag[ 2 ];                                         /* UVC Buffer full flag: 0-full 1-not full  */
    UINT8  LoadNum;                                               /* UVC Buffer Load Number*/
    UINT8  DealNum;                                               /* UVC Buffer processing number */
    UINT8  RemainCount;                                           /* UVC Buffer Remaining Count */
}UVC_BUF_INFO;
volatile UVC_BUF_INFO UVC_BufInfo;

/*
 * Video Streaming Interface Control Request Descriptor.
 * Before you turn on the camera, the host will issue the setting descriptor.
 */
UINT8V Get_Curr[26]=
{
    0x00, 0x00,
    0x01,
    0x01,
    0x0A, 0x8b, 0x02, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x48, 0x3f, 0x00,
    0x00, 0x00, 0x00, 0x00
};

/*
 * Resolution settings supported by USB3.0 and 2.0 UVC. If you want to change the resolution,
 *  you also need to change the corresponding resolution in the USB configuration descriptor.
 */
const UINT16 frame_resolution[5][2]=
{
    0x320,0x258,    //800*600
    0x140,0x0f0,    //320*240
    0x280,0x1e0,    //640*480
    0x500,0x2D0,    //1280*720
    0x640,0x4b0,    //1600*1200
};

/*
 * This is a very important parameter that determines the frame rate displayed.
 * When you change the resolution, you also need to change these variables.
 *
 * 8000 - Number of microframes in one second.
 * 8000/(frame rate) = YUV_Totalcount. You need to ensure that the calculated number must be an integer(YUV_Totalcount).
 */
const UINT8 YUV_Totalcount[5]=
{
    120,   // 66.6fps    8000/120
    10,    // 800fps     8000/10
    30,    // 266.6fps   8000/30
    120,   // 66.6fps    8000/120
    120    // 66.6fps    8000/120
};

/*******************************************************************************
 * @fn      Switch_Resolution
 *
 * @brief   Select resolution and frame rate
 *
 * @return  None
 */
void Switch_Resolution( UINT8 frameindex )
{
    Resolution_width  = frame_resolution[frameindex-1][0];
    Resolution_height = frame_resolution[frameindex-1][1];
}

/*******************************************************************************
 * @fn      FillYUVdata
 *
 * @brief   Fill in data
 *
 * @return  None
 */
void FillYUVdata( void )
{
    UINT16 i;
    for ( i = 0; i < 1024 * 16; i += 4 ) //yuv data
    {
        YUV2_addr[i] =   0x23;
        YUV2_addr[i+1] = 0xd4;
        YUV2_addr[i+2] = 0x23;
        YUV2_addr[i+3] = 0x72;
    }
}

/*******************************************************************************
* Function Name  : UVC_SourceClock
* Description    : UVC Source Clock enable/disable
* Input          : sta     enable/disable
* Output         : None
* Return         : None
*******************************************************************************/
void UVC_SourceClock(FunctionalState sta)
{
  if(sta)
  {
   SysTick->CMP=0xffffffff;
   SysTick->CTLR=(1<<8)|(1<<2)|(1<<0);

  }
  else
  {
      /*systick close*/
      SysTick->CTLR=0;
  }
}

/*******************************************************************************
 * @fn      ctrlCamera
 *
 * @brief   USB3.0 set interface processing function
 *
 * @return  None
 */
void CtrlCamera(){
    class = 1;
    if(UsbSetupBuf->wValueL > 0)
    {
        /* Fill frame header data */
        MJPEG.Clock = 0x0136E5;
        MJPEG.Frequeny = ~((*(UINT32V *)(0xe000f004)));
        MJPEG.count=0x04DD;
        MJPEG.len=0x0c;
        MJPEG.tog = 0x8c;
        MJPEG.reallen=0x0C;

        YUV2.Clock = 0x0;
        YUV2.len=0x0c;
        YUV2.tog = 0x8c;
        YUV2.reallen=0x0C;


        /* Calculate related variables */
        UVC_MFrame_PackTotalNum = 0;
        UVC_MFrame_PackCount = UVC_MFrame_PackTotalNum;
        UVC_MBuf_PackCount = 0x00;
        Totalcnt = 0;

        UVC_DVP = 0x01;
        memcpy(endp1RTbuff,  (uint8_t *)&MJPEG,12 );
        USB30_IN_ClearIT(endp_1);
        USB30_IN_Set(endp_1,ENABLE,ACK_TP,NUMP_1,MJPEG.reallen);

        USB30_ITP_Enable(ENABLE);
    }
    else
    {
        UVC_DVP = 0x00;
        USB30_ITP_Enable(DISABLE);
    }
}
/*******************************************************************************
 * @fn      ctrlCamera_hs
 *
 * @brief   USB2.0 set interface processing function
 *
 * @return  None
 */
void CtrlCamera_Hs(){
    class = 1;
    if( UsbSetupBuf->wValueL > 0)
    {
        UVC_SourceClock(1);
        MJPEG.Clock = 0;
        MJPEG.Frequeny = ~((*(UINT32V *)(0xe000f004)));
        MJPEG.count = 0;
        MJPEG.len = 0x0c;
        MJPEG.tog = 0x8c;
        MJPEG.reallen=0x0C;

        YUV2.Clock = 0;
        YUV2.len=0x0c;
        YUV2.tog = 0x8c;
        YUV2.reallen=0x0C;

        UVC_DVP = 0x01;
        memcpy(endp1RTbuff,  (uint8_t *)&MJPEG,12 );
        R16_UEP1_T_LEN =  12;
        R8_UEP1_TX_CTRL &= ~(3<<3);
    }
    else
    {
        UVC_DVP = 0x00;
        UVC_SourceClock(0);
    }
}

/*******************************************************************************
 * @fn      endp1_ITPHander
 *
 * @brief   USB3.0 endpoint 1 ITP processing function
 *
 * @return  None
 */
void Endp1_ITPHander(void)
{
    /* Here, preset the amount of data to be sent for this microframe,
     * and then at Endp1_ Hander transfers the remaining data of this microframe.
    */
    UINT8 seq;

    UVC_MFrame_PackCount = 0x00;
    UVC_MBuf_PackCount = 0x00;
    UVC_MFrame_ValidFlag = 0x01;
    YUV2.tog &= ~(1<<1);
    if((lastFrame==1))
    {
        YUV2.tog ^= 0x01;
        YUV2.tog &= ~(1<<1);
        lastFrame=0;
    }
    if( UVC_BufInfo.FullFlag[ UVC_BufInfo.DealNum ] == 0x00 )
    {
        UVC_MFrame_PackTotalNum = UVC_BufInfo.Length[ UVC_BufInfo.DealNum ] / 1024;
        UVC_MFrame_LastPackLen =  UVC_BufInfo.Length[ UVC_BufInfo.DealNum ] % 1024;

        if( UVC_MFrame_LastPackLen )
        {
            UVC_MFrame_PackTotalNum++;
        }
        else if( UVC_MFrame_LastPackLen == 0 )
        {
            UVC_MFrame_LastPackLen = 1024;
        }

        UVC_BufInfo.FullFlag[0] = 0x00;
        UVC_BufInfo.Length[0] = (Resolution_width * Resolution_height * 2 / YUV_Totalcount[Get_Curr[3] - 1] ) +12;//The amount of data transmitted by the current microframe
        UVC_BufInfo.FullFlag[1] = 0x00;
        UVC_BufInfo.Length[1] = (Resolution_width * Resolution_height * 2 / YUV_Totalcount[Get_Curr[3] - 1] ) +12;

        USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)YUV2_addr;
        memcpy( (UINT8 *)( YUV2_addr ),  (uint8_t *)&YUV2, 12 );//Copy 12 byte packet header
        seq = UVC_MFrame_PackCount;
        if( (UVC_MFrame_PackTotalNum >= BURSTMAXSIZE))
        {
            USB30_IN_ClearIT(1);
            USB30_Set_endp_seqnumber(0x81,&seq);
            USB30_IN_Set(1 , 0 , NRDY , BURSTMAXSIZE , 1024  );
        }
        else
        {
            USB30_IN_ClearIT(1);
            USB30_Set_endp_seqnumber(0x81,&seq);
            USB30_IN_Set( 1 , 1 , NRDY , UVC_MFrame_PackTotalNum , UVC_MFrame_LastPackLen );
        }
    }
    else  //End of frame
    {
        YUV2.tog |= 0x02;
        lastFrame=1;
        UVC_MFrame_PackTotalNum = UVC_BufInfo.Length[ UVC_BufInfo.DealNum ] / 1024;
        UVC_MFrame_LastPackLen = UVC_BufInfo.Length[ UVC_BufInfo.DealNum ] % 1024;

        if( UVC_MFrame_LastPackLen )
        {
            UVC_MFrame_PackTotalNum++;
        }
        else if( UVC_MFrame_LastPackLen == 0 )
        {
            UVC_MFrame_LastPackLen = 1024;
        }

        USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)YUV2_addr;
        memcpy( (UINT8 *)( YUV2_addr ),  (uint8_t *)&YUV2, 12 );
        seq = UVC_MFrame_PackCount;
        if( (UVC_MFrame_PackTotalNum >= BURSTMAXSIZE) )
        {
            USB30_IN_ClearIT(1);
            USB30_Set_endp_seqnumber(0x81,&seq);
            USB30_IN_Set( 1 , 0 , NRDY , BURSTMAXSIZE , 1024  );
        }
        else
        {
            USB30_IN_ClearIT(1);
            USB30_Set_endp_seqnumber(0x81,&seq);
            USB30_IN_Set( 1 , 1 , NRDY , UVC_MFrame_PackTotalNum , UVC_MFrame_LastPackLen  );
        }
    }
}

/*******************************************************************************
 * @fn      endp1_Hander
 *
 * @brief   USB3.0 endpoint 1 processing function
 *
 * @return  None
 */
void Endp1_Hander(void)
{
    /*
     * Check for incomplete data in ITP interrupts.
    */

     UINT8 nump;
     UINT8 seq;
     USB30_IN_ClearIT(1);
     nump = USB30_IN_Nump( 1 );
     if( UVC_MFrame_ValidFlag )
     {
         UVC_MFrame_ValidFlag = 0x00;

         UVC_MBuf_PackCount += nump;
         if( UVC_MBuf_PackCount >= UVC_MFrame_PackTotalNum )
         {
             UVC_MBuf_PackCount -= UVC_MFrame_PackTotalNum;

             UVC_BufInfo.DealNum++;
             if( UVC_BufInfo.DealNum >= 2 )
             {
                 UVC_BufInfo.DealNum = 0x00;
             }
             Totalcnt++;
             if(Totalcnt == YUV_Totalcount[Get_Curr[3] - 1] - 1)
             {
                 UVC_BufInfo.FullFlag[UVC_BufInfo.DealNum] = 0x01;
             }
             if(Totalcnt == YUV_Totalcount[Get_Curr[3] - 1])
             {
                 Totalcnt=0;
             }
         }

         UVC_MFrame_PackCount += nump;
         if( UVC_MFrame_PackCount < UVC_MFrame_PackTotalNum )
         {
             USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)(YUV2_addr+12);
             packnum = UVC_MFrame_PackTotalNum - UVC_MFrame_PackCount;
             seq = UVC_MFrame_PackCount;
             if( packnum >= BURSTMAXSIZE )
             {
                 USB30_IN_ClearIT(1);
                 USB30_Set_endp_seqnumber(0x81,&seq);
                 USB30_IN_Set( 1 , 0 , NRDY , BURSTMAXSIZE , 1024  );
             }
             else
             {
                 USB30_IN_ClearIT(1);
                 USB30_Set_endp_seqnumber(0x81,&seq);
                 USB30_IN_Set( 1 , 1 , NRDY , packnum , UVC_MFrame_LastPackLen  );
             }
             UVC_MFrame_ValidFlag = 0x01;
         }
    }
}

/*******************************************************************************
 * @fn      endp1_ISOHander_hs
 *
 * @brief   USB2.0 endpoint 1 ISO processing function
 *
 * @return  None
 */
void Endp1_ISOHander_Hs(void)
{
    static UINT32 send_num = 0;
    static UINT8 togGG = 0;

    if(send_num >= PACKSIZE_3)
    {
        if(togGG == 2)
        {
            togGG = 1;
            R32_UEP1_TX_DMA = (UINT32)(UINT8 *)(YUV2_addr+1024);
            R16_UEP1_T_LEN = 1024;
        }
        else if(togGG == 1)
        {
            togGG = 0 ;
            R32_UEP1_TX_DMA = (UINT32)(UINT8 *)(YUV2_addr+1024);
            R16_UEP1_T_LEN = 1024;

            send_num -= 1024*3-12;
        }
        else if(togGG == 0)
        {
            memcpy( (UINT8 *)( YUV2_addr ),  (uint8_t *)&YUV2, 12 );
            YUV2.tog &= ~(1<<1);
            togGG = 2;
            R32_UEP1_TX_DMA = (UINT32)(UINT8 *)YUV2_addr;
            R16_UEP1_T_LEN = 1024;
        }
    }
    else if(send_num >= PACKSIZE_2 && send_num < PACKSIZE_3)//3 pack
    {
        if(togGG == 2)
        {
            togGG = 1;
            R32_UEP1_TX_DMA = (UINT32)(UINT8 *)(YUV2_addr+1024);
            R16_UEP1_T_LEN = 1024;
        }
        else if(togGG == 1)
        {
            togGG = 0 ;
            R32_UEP1_TX_DMA = (UINT32)(UINT8 *)(YUV2_addr+1024);
            R16_UEP1_T_LEN = send_num - 1024*2 +12;

            send_num = Resolution_width * Resolution_height * 2;
        }
        else if(togGG == 0)
        {
            YUV2.tog |= 0x02;
            YUV2.tog ^= 0x01;
            memcpy( (UINT8 *)( YUV2_addr ),  (uint8_t *)&YUV2, 12 );
            YUV2.tog &= ~(1<<1);
            togGG = 2;
            R32_UEP1_TX_DMA = (UINT32)(UINT8 *)YUV2_addr;
            R16_UEP1_T_LEN = 1024;
        }
    }
    else if(send_num >= PACKSIZE_1 && send_num < PACKSIZE_2)//2 pack
    {
        if(togGG == 1)
        {
            togGG = 0 ;
            R32_UEP1_TX_DMA = (UINT32)(UINT8 *)(YUV2_addr+1024);
            R16_UEP1_T_LEN = send_num - 1024*1 + 12;

            send_num = Resolution_width * Resolution_height * 2;
        }
        else if(togGG == 0)
        {
            YUV2.tog |= 0x02;
            YUV2.tog ^= 0x01;
            memcpy( (UINT8 *)( YUV2_addr ),  (uint8_t *)&YUV2, 12 );
            YUV2.tog &= ~(1<<1);

            togGG = 1;
            R32_UEP1_TX_DMA = (UINT32)(UINT8 *)YUV2_addr;
            R16_UEP1_T_LEN = 1024;
        }
    }
    else
    {
        YUV2.tog |= 0x02;
        YUV2.tog ^= 0x01;
        memcpy( (UINT8 *)( YUV2_addr ),  (uint8_t *)&YUV2, 12 );
        YUV2.tog &= ~(1<<1);

        togGG = 0;
        R32_UEP1_TX_DMA = (UINT32)(UINT8 *)YUV2_addr;
        R16_UEP1_T_LEN = send_num+12;

        send_num = Resolution_width * Resolution_height * 2;
    }
    R8_UEP1_TX_CTRL &= ~(3<<3);
    R8_UEP1_TX_CTRL |= (togGG<<3);

}

/*******************************************************************************
 * @fn      endp1_Hander_hs
 *
 * @brief   USB2.0 endpoint 1 processing function
 *
 * @return  None
 */
void Endp1_Hander_Hs(void)
{

}


