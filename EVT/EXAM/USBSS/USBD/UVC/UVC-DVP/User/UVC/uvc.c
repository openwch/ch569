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

#define FULLPACKSIZE    1024
#define PACKSIZE_3      FULLPACKSIZE*3
#define PACKSIZE_2      FULLPACKSIZE*2
#define PACKSIZE_1      FULLPACKSIZE*1
#define DVP_NUM         24

/* Global Variable */
static UINT32V Dvp_Class=0;
static UINT32V Dvp_Addrcount;                    //Double buffered address switch count
static UINT32V Dvp_Start;                        //Frame start flag
static UINT32V Dvp_End;                          //Frame end flag
static UINT32V UVC_DvpStart;                     //UVC Start transmitting data
static UINT32V Dvp_Reclength = 0;                //Length of data to be uploaded
static UINT32V DVP_Recaddpoint = 0;              //Storage address
PUINT8 Dvp_Recaddr = UVC_DMABuffer + UVC_HEADERSIZE; //Storage address
UINT16V Dvp_DataSize = 0;                        //Size of each package of DVP

static UINT8V USB_Uploadflg = 0;                 //USB is uploading flag
static UINT32V USB_MframeUploadLength = 0;       //Number of bytes to upload for a microframe
static UINT32V USB_LastMframeUploadLength = 0;   //Number of bytes to upload for the previous microframe
static UINT32V USB_Uploadcount = 0;              //Number of packages to upload for a microframe
static UINT8 USB_ISOSeqnumber = 0;               //USB3.0 seqnumber count
static UINT32V USB_Uploadpoint = 0;              //Point to the current USB upload address
static PUINT8 USBUploadaddr = UVC_DMABuffer;     //USB Upload address

UINT8V  Formatchange_flag = 1;                   // Video format switching : 1-mjpeg   2-yuv
UINT16V Resolution_width = 0;                    //Resolution width
UINT16V Resolution_height = 0;                   //Resolution length


/*
 * Resolution settings supported by USB3.0 and 2.0 UVC. If you want to change the resolution,
 * you also need to change the corresponding resolution in the USB configuration descriptor.
 */
const UINT16 ov2640_JPEGframe_resolution_USB30[5][2]=
{
    0x320,0x258,    //800*600
    0x160,0x120,    //352*288
    0x280,0x1e0,    //640*480
    0x400,0x300,    //1024*768
    0x640,0x4b0,    //1600*1200
};

const UINT16 ov2640_JPEGframe_resolution_USB20[5][2]=
{
    0x320,0x258,    //800*600
    0x160,0x120,    //352*288
    0x280,0x1e0,    //640*480
    0x400,0x300,    //1024*768
    0x640,0x4b0,    //1600*1200
};

const UINT16 ov2640_YUVframe_resolution_USB30[5][2]=
{
    0x280,0x1e0,    //640*480
    0x640,0x4b0,    //1600*1200
    0x500,0x3C0,    //1280*960
    0x400,0x300,    //1024*768
    0x160,0x120,    //352*288

};

const UINT16 ov2640_YUVframe_resolution_USB20[5][2]=
{
    0x280,0x1e0,    //640*480      VGA
    0x320,0x258,    //800*600
    0xb0,0x90,      //176*144
    0x140,0xf0,     //320*240
    0x160,0x120,    //352*288      CIF
};

/*UVC Frame header structure definition*/
typedef struct __attribute__((packed))
 {
    UINT8 len;
    UINT8 tog;
    UINT32 Frequeny;
    UINT32 Clock;
    UINT16 count;
    UINT32 alignment;
    UINT32 reallen;
 }Picture;
 Picture UVC;

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
 * @fn      SS_CtrlCamera
 *
 * @brief   USB3.0 set interface processing function
 *
 * @return  None
 */
void SS_CtrlCamera()
{
    Dvp_Class = 1;
    UINT16 i = 0;
    if(UsbSetupBuf->wValueL > 0)
    {
        /* Fill frame header data */
        UVC.Clock = 0x0136E5;
        UVC.Frequeny = ~((*(UINT32V *)(0xe000f004)));
        UVC.count=0x04DD;
        UVC.len = UVC_HEADERSIZE;
        UVC.tog = 0x8c;
        UVC.reallen = UVC_HEADERSIZE;
        UVC.alignment = 0x00;

        /*initialize variable*/
        UVC_DvpStart = 0x01;
        Dvp_Addrcount = 0;
        Dvp_Start = 0;
        Dvp_End = 0;
        Dvp_Reclength = 0;                //Length of data to be uploaded
        DVP_Recaddpoint = 0;              //Storage address
        USB_Uploadflg = 0;                //USB is uploading flag
        USB_MframeUploadLength = 0;
        USB_Uploadcount = 0;
        USB_ISOSeqnumber = 0;
        USB_Uploadpoint = 0;
        Dvp_Recaddr = UVC_DMABuffer + UVC_HEADERSIZE;    //Storage address
        USBUploadaddr = UVC_DMABuffer;                   //USB Upload address

        memcpy(endp1RTbuff,  (uint8_t *)&UVC,UVC.reallen );
        USB30_IN_ClearIT(endp_1);
        USB30_IN_Set(endp_1,ENABLE,ACK_TP,NUMP_1,UVC.reallen);
        PFIC_EnableIRQ(DVP_IRQn);
    }
    else
    {
        UVC_DvpStart = 0x00;
        PFIC_DisableIRQ(DVP_IRQn);
    }
}

/*******************************************************************************
 * @fn      HS_CtrlCamera
 *
 * @brief   USB2.0 set interface processing function
 *
 * @return  None
 */
void HS_CtrlCamera()
{
    Dvp_Class = 1;
    if( UsbSetupBuf->wValueL > 0)
    {
        UVC_SourceClock(1);
        UVC.Clock = 0;
        UVC.Frequeny = ~((*(UINT32V *)(0xe000f004)));
        UVC.count = 0;
        UVC.len = UVC_HEADERSIZE;
        UVC.tog = 0x8c;
        UVC.reallen = UVC_HEADERSIZE;
        UVC.alignment = 0x00;
        UVC_DvpStart = 0x01;

        /*initialize variable*/
        Dvp_Addrcount = 0;
        Dvp_Start = 0;
        Dvp_End = 0;
        Dvp_Reclength = 0;                //Length of data to be uploaded
        DVP_Recaddpoint = 0;              //Storage address
        USB_Uploadflg = 0;                //USB is uploading flag
        USB_MframeUploadLength = 0;
        USB_Uploadcount = 0;
        USB_ISOSeqnumber = 0;
        USB_Uploadpoint = 0;
        Dvp_Recaddr = UVC_DMABuffer + UVC_HEADERSIZE; //Storage address
        USBUploadaddr = UVC_DMABuffer;                   //USB Upload address

        memcpy(endp1RTbuff,  (uint8_t *)&UVC,UVC.reallen );
        R16_UEP1_T_LEN =  UVC.reallen;
        R8_UEP1_TX_CTRL &= ~(3<<3);
        PFIC_EnableIRQ(DVP_IRQn);
    }
    else
    {
        UVC_DvpStart = 0x00;
        UVC_SourceClock(0);
        PFIC_DisableIRQ(DVP_IRQn);
    }
}

/*******************************************************************************
 * @fn      HS_Endp1_Hander
 *
 * @brief   USB2.0 endpoint 1 processing function
 *
 * @return  None
 */
void HS_Endp1_Hander(void)
{
    if(USB_MframeUploadLength == 0)
    {
        USB_Uploadflg = 0;      //Current microframe data transmission completed
    }
    memcpy( (UINT8 *)USBUploadaddr,  (uint8_t *)&UVC, UVC_HEADERSIZE );//copy UVC frame Header-16byte
    UVC.tog &= 0xFD;
    UVC.reallen = UVC_HEADERSIZE;
}

/*******************************************************************************
 * @fn      HS_Endp1_ISOHander
 *
 * @brief   USB2.0 endpoint 1 ISO processing function
 *
 * @return  None
 */
void HS_Endp1_ISOHander()
{
    static UINT8 togGG = 0;//Used to set DATA0 1 2

    Dvp_Class++;
    UVC.Clock = (UINT32)SOURCECLOCK;
    /*Write structure every 8, add one for each IPT , 125us*/
    if(Dvp_Class%8==0)
    {
        UVC.count =(++UVC.count)&0x07ff;
    }

    if(Dvp_Reclength >= DVP_NUM * Dvp_DataSize)//Overflow clear
    {
        Dvp_Reclength = 0;
        DVP_Recaddpoint = 0;
        USB_Uploadpoint = 0;
    }

    if(Dvp_Reclength && USB_Uploadflg == 0)//Determine if there is data and if the current microframe data has been sent successfully
    {
        USB_Uploadflg = 1;                 //DATA0 1 2 There is a gap between packages,DVP interrupts may occur between intervals, resulting in Dvp_ Reclenth change
                                           //Ensure that the current microframe only determines the transmission length once USB_MframeUploadLength¡£

        USBUploadaddr = UVC_DMABuffer+USB_Uploadpoint;//Move to this sending address

        if( Dvp_Reclength + USB_Uploadpoint >= DVP_NUM * Dvp_DataSize )//If the data of DVP plus the USB pointer position is greater than or equal to the end of the buffer,
                                                                       //so this microframe is only sent to the end of the buffer
        {
            USB_MframeUploadLength = DVP_NUM * Dvp_DataSize - USB_Uploadpoint + UVC_HEADERSIZE;
        }
        else//Did not exceed the end of the buffer
        {
            USB_MframeUploadLength = Dvp_Reclength + UVC_HEADERSIZE;
        }

        if( USB_MframeUploadLength >= PACKSIZE_3 )//USB2.0 Up to one upload 3KB
        {
            USB_MframeUploadLength = PACKSIZE_3;
        }

        USB_Uploadpoint += (USB_MframeUploadLength - UVC_HEADERSIZE);  //Move USB buffer pointer
        if( USB_Uploadpoint >= DVP_NUM * Dvp_DataSize )//USB buffer pointer needs to be turned back when it reaches the end of the buffer
        {
            USB_Uploadpoint = 0;
        }

        Dvp_Reclength -= (USB_MframeUploadLength - UVC_HEADERSIZE);//Calculate remaining unsent data for DVP
    }

    if( USB_MframeUploadLength )
    {
        if( USB_MframeUploadLength > PACKSIZE_2 && USB_MframeUploadLength <= PACKSIZE_3 )
        {

            if(togGG == 2)//DATA1 - second package
            {
                togGG = 1;
                R32_UEP1_TX_DMA = (UINT32)(UINT8 *)USBUploadaddr+FULLPACKSIZE;
                UVC.reallen = FULLPACKSIZE;
            }
            else if(togGG == 1)//DATA0 - third package
            {
                togGG = 0 ;
                R32_UEP1_TX_DMA = (UINT32)(UINT8 *)USBUploadaddr+FULLPACKSIZE*2;
                UVC.reallen = USB_MframeUploadLength-FULLPACKSIZE*2;
                USB_MframeUploadLength = 0;
            }
            else if(togGG == 0)//DATA2 - first package
            {
                togGG = 2;
                R32_UEP1_TX_DMA = (UINT32)(UINT8 *)USBUploadaddr;
                UVC.reallen = FULLPACKSIZE;
            }
        }
        else if( USB_MframeUploadLength> PACKSIZE_1 && USB_MframeUploadLength <= PACKSIZE_2 )
        {
            if(togGG == 1)//DATA0 - second package
            {
                togGG = 0 ;
                R32_UEP1_TX_DMA = (UINT32)(UINT8 *)USBUploadaddr+FULLPACKSIZE;
                UVC.reallen = USB_MframeUploadLength-FULLPACKSIZE;
                USB_MframeUploadLength  = 0;
            }
            else if(togGG == 0)//DATA1 - first package
            {
                togGG = 1;
                R32_UEP1_TX_DMA = (UINT32)(UINT8 *)USBUploadaddr;
                UVC.reallen = FULLPACKSIZE;
            }
        }
        else if( USB_MframeUploadLength <= PACKSIZE_1 )//DATA0 - first package
        {
            R32_UEP1_TX_DMA = (UINT32)(UINT8 *)USBUploadaddr;
            UVC.reallen = USB_MframeUploadLength;
            togGG = 0;
            USB_MframeUploadLength  = 0;
        }

        R8_UEP1_TX_CTRL &= ~(3<<3);
        R8_UEP1_TX_CTRL |= (togGG<<3);
    }
    else//No data, only 16 byte frame header sent
    {
        if( Dvp_End == 1 )//End of frame
        {
            UVC.tog |= 0x02;
            Dvp_End = 0;
        }

        UVC.reallen = UVC_HEADERSIZE;
        R32_UEP1_TX_DMA = (UINT32)(UINT8 *)USBUploadaddr;
        R8_UEP1_TX_CTRL &= ~(3<<3);
        R8_UEP1_TX_CTRL |= (0<<3);
    }

    R16_UEP1_T_LEN =  UVC.reallen;//Set transmission length
}

/*******************************************************************************
 * @fn      SS_Endp1_Hander
 *
 * @brief   USB3.0 endpoint 1 processing function
 *
 * @return  None
 */
void SS_Endp1_Hander(void)
{
    UINT8 nump;
    static UINT32 currpoint = 0;

    nump = USB30_IN_Nump(endp_1);
    if( nump < USB_Uploadcount )
    {
        USB_MframeUploadLength -= nump * FULLPACKSIZE;//The remaining number of bytes to be transmitted in this microframe
        USB_Uploadcount -= nump;                      //The remaining number of packets to be transmitted in this interrupt
        USB_ISOSeqnumber += nump;                     //seqnum flag Count

        currpoint += nump*FULLPACKSIZE;          //The sending address that should be directed

        USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)(USBUploadaddr+currpoint);//Reset DMA address
        USB30_IN_ClearIT(endp_1);
        USB30_Set_endp_seqnumber(0x81,&USB_ISOSeqnumber);//set seqnum
        /*Determine if the package is full*/
        if( USB_MframeUploadLength % FULLPACKSIZE )//Not full package
        {
            USB30_IN_Set(endp_1,ENABLE,NRDY,USB_MframeUploadLength/FULLPACKSIZE +1 ,USB_MframeUploadLength % FULLPACKSIZE);
        }
        else//full package
        {
            USB30_IN_Set(endp_1,ENABLE,NRDY,USB_MframeUploadLength/FULLPACKSIZE , FULLPACKSIZE);
        }
    }
    else
    {
        Dvp_Class++;
        UVC.Clock = (UINT32)SOURCECLOCK;
        /*Write structure every 8, add one for each IPT , 125us*/
        if(Dvp_Class%8==0)
        {
            UVC.count =(++UVC.count)&0x07ff;
        }

        if(Dvp_Reclength >= DVP_NUM * Dvp_DataSize)
        {
            Dvp_Reclength = 0;
            DVP_Recaddpoint = 0;
            USB_Uploadpoint = 0;
        }

        if( Dvp_Reclength && USB_Uploadflg == 0)
        {
            USBUploadaddr = UVC_DMABuffer+USB_Uploadpoint;        //USB upload DMA address switching
            if( (Dvp_Reclength + USB_Uploadpoint) >= (DVP_NUM * Dvp_DataSize) )//If the data of DVP plus the USB pointer position is greater than or equal to the end of the buffer,
                                                                               //so this microframe is only sent to the end of the buffer
            {
                UVC.reallen = DVP_NUM * Dvp_DataSize - USB_Uploadpoint + UVC_HEADERSIZE;
            }
            else//Did not exceed the end of the buffer
            {
                UVC.reallen = Dvp_Reclength + UVC_HEADERSIZE;
            }

            USB_Uploadpoint += (UVC.reallen - UVC_HEADERSIZE);//Move to the next sending address
            if( USB_Uploadpoint >= DVP_NUM * Dvp_DataSize )//USB buffer pointer needs to be turned back when it reaches the end of the buffer
            {
                USB_Uploadpoint = 0;
            }

            USB_MframeUploadLength = UVC.reallen;//Used to complete interrupts to determine the total amount of data transmission

            USB_LastMframeUploadLength = USB_MframeUploadLength;
        }
        else//No data, only 16 byte frame header sent
        {
            if( Dvp_End == 1 )
            {
                UVC.tog |= 0x02;
                Dvp_End=0;
            }
            UVC.reallen = UVC_HEADERSIZE;//16 Byte
        }

        USB_ISOSeqnumber = 0;
        USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)USBUploadaddr;//set DMA address
        USB30_Set_endp_seqnumber(0x81,&USB_ISOSeqnumber);//set seqnum
        USB30_IN_ClearIT(endp_1);
        /*Determine if the package is full*/
        if( UVC.reallen % 1024 )//Not full package
        {
            USB30_IN_Set(endp_1,ENABLE,NRDY,((UVC.reallen/FULLPACKSIZE +1) > 31 ? 31 : (UVC.reallen/FULLPACKSIZE +1) ) ,UVC.reallen % FULLPACKSIZE);
            USB_Uploadcount = UVC.reallen/FULLPACKSIZE +1;  //Used to determine the number of packets sent by a microneedle upon completion of an interrupt
        }
        else//full package
        {
            USB30_IN_Set(endp_1,ENABLE,NRDY, ((UVC.reallen/FULLPACKSIZE) > 31 ? 31 : (UVC.reallen/FULLPACKSIZE) ), FULLPACKSIZE);
            USB_Uploadcount = UVC.reallen/FULLPACKSIZE;     //Used to determine the number of packets sent by a microneedle upon completion of an interrupt
        }

        if( UVC.reallen > UVC_HEADERSIZE )
        {
            USB_Uploadflg = 1;
        }
        else
        {
            if( USB_LastMframeUploadLength )//Reduce the length to be sent
            {
                Dvp_Reclength -= (USB_LastMframeUploadLength - UVC_HEADERSIZE);//Reduce the length to be sent
                USB_LastMframeUploadLength = 0;
            }
            USB_Uploadflg = 0;
        }

        UVC.reallen = UVC_HEADERSIZE;   //Clear sending length
        UVC.tog &= 0xFD;                //Clear frame header flag
        currpoint = 0;
    }
    memcpy( USBUploadaddr,  (uint8_t *)&UVC,UVC_HEADERSIZE );//Assign UVC frame header
}

/*******************************************************************************
 * @fn      DVP_Hander
 *
 * @brief   DVP Interrupt processing function
 *
 * @return  None
 */
void DVP_Hander(void){

    /* End of line interrupt */
    if (R8_DVP_INT_FLAG & RB_DVP_IF_ROW_DONE)
    {
        R8_DVP_INT_FLAG = RB_DVP_IF_ROW_DONE;

        Dvp_Reclength += Dvp_DataSize;//Whether in MJPEG or YUV format,the packet length is the same for each transmission of data

        DVP_Recaddpoint += Dvp_DataSize;//DVP receive pointer movement
        if( DVP_Recaddpoint >= DVP_NUM* Dvp_DataSize)//Reaching the end of the buffer
        {
            DVP_Recaddpoint = 0;
        }

        if ( Dvp_Addrcount%2 )//Two buffers used to determine the switching of dual buffering mode
        {
            if( DVP_Recaddpoint+Dvp_DataSize >= DVP_NUM * Dvp_DataSize) //Pointer turns back to prevent crossing boundaries
            {
                R32_DVP_DMA_BUF1 = (UINT32)(UINT8 *)Dvp_Recaddr;
            }
            else
            {
                R32_DVP_DMA_BUF1 = (UINT32)(UINT8 *)Dvp_Recaddr+DVP_Recaddpoint+Dvp_DataSize;//Preset the next received data for the next time
            }
        }
        else
        {
            if( DVP_Recaddpoint+Dvp_DataSize >= DVP_NUM * Dvp_DataSize)//Pointer turns back to prevent crossing boundaries
            {
                R32_DVP_DMA_BUF0 = (UINT32)(UINT8 *)Dvp_Recaddr;
            }
            else
            {
                R32_DVP_DMA_BUF0 = (UINT32)(UINT8 *)Dvp_Recaddr+DVP_Recaddpoint+Dvp_DataSize;//Preset the next received data for the next time
            }
        }
        Dvp_Addrcount++;

    }
    /* Frame reception completion interrupt */
    if (R8_DVP_INT_FLAG & RB_DVP_IF_FRM_DONE)
    {
        R8_DVP_INT_FLAG = RB_DVP_IF_FRM_DONE;
    }
    /* Frame start interrupt */
    if (R8_DVP_INT_FLAG & RB_DVP_IF_STR_FRM)
    {
        R8_DVP_INT_FLAG = RB_DVP_IF_STR_FRM;
        if((UVC_DvpStart & 0x01)==1)
        {
            UVC.Frequeny = ~((*(UINT32V *)(0xe000f004)));
            Dvp_Addrcount = 0;
            UVC.tog ^= 0x01;

            if( Formatchange_flag == FORMAT_YUV2 )
            {
                Dvp_Reclength = 0;
                DVP_Recaddpoint = 0;
                USB_Uploadpoint = 0;
            }
        }
    }
    /* Frame end interrupt */
    if (R8_DVP_INT_FLAG & RB_DVP_IF_STP_FRM)
    {
        R8_DVP_INT_FLAG = RB_DVP_IF_STP_FRM;
        Dvp_End=1;
    }
    /* FIFO Overflow interrupt */
    if (R8_DVP_INT_FLAG & RB_DVP_IF_FIFO_OV)
    {
        R8_DVP_INT_FLAG = RB_DVP_IF_FIFO_OV;
    }
}

