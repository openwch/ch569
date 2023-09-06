/********************************** (C) COPYRIGHT *******************************
* File Name          : uddisk.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "../Uddisk/uddisk.h"

#include "CH56xusb30_LIB.h"
#include "CH56x_usb30.h"

/* Global Variable */
__attribute__ ((aligned(16))) UINT8 CSW_PARAMETER[13] __attribute__((section(".DMADATA")));
const uint8_t s03[]={0x70,0x00,0x06,0x00,0x00,0x00,0x00,0x0A,0x00,0x00,0x00,0x00,0x28,0x00,0x00,0x00,0x00,0x00};
const uint8_t s1238[]={0x00,0x80,0x06,0x02,0x39,0x00,0x00,0x00,0x4B,0x69,0x6E,0x67,0x73,0x74,0x6F,0x6E,0x44,0x61,0x74,0x61,0x54,0x72,0x61,0x76,0x65,0x6C,0x65,0x72,0x20,0x33,0x2E,0x30,0x50,0x4D,0x41,0x50,0x50,0x4D,0x41,0x50,0x31,0x32,0x33,0x34,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
const uint8_t s1a03[]={0x23,0x00,0x00,0x00,0x05,0x1E,0xF0,0x00,0xFF,0x20,0x02,0x00,0x1D,0x69,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
const uint8_t INQUIRY1[64]=    {
    0x00,                                                                       /* Peripheral Device Type£ºDISK */
    0x80,                                                                       /* Can be removed */
    0x05,                                                                       /* ISO/ECMA */
    0x12,
    0x1F,                                                                       /* Additional Length */
    0x00,                                                                       /* Reserved */
    0x00,                                                                       /* Reserved */
    0x00,                                                                       /* Reserved */
    'E',  //vendor identification                                                                       /* Vendor Information */
    'M',
    'M',
    'C',
    ' ',
    ' ',
    ' ',
    ' ',
    'U',  //product identification                                                                      /* Product Identification */
    'S',
    'B',
    ' ',
    'S',
    'p',
    'e',
    'c',
    'i',
    'a',
    'l',
    ' ',
    'D',
    'i',
    's',
    'k',
    '3',                                                                       /* Product Revision Level */
    '.',
    '0',
    '0',
    'n',  //36vendor_specific
    'a',
    'n',
    'j',
    'i',
    'n',
    'g',
    'q',
    'i',
    'n',
    'h',
    'e',
    'n',
    'g',
    ' ',
    ' ',
    ' ',
    ' ',
    ' ',
    ' ',//retain
    ' ',
    ' ',
    ' ',
    ' ',
    ' ',
    ' ',
    ' ',
    ' '
};
const uint8_t LONG_INQUIRY[]={0X00,0X80,0X06,0XFB,0X30,0X42,0X30,0X45,0X31,0X35,0X39,0X31,0X45,0X42,0X30,0X37,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X04,0X60,0X04,0XC0,0X00,0X00,0XDC,0XF2,0X9E,0XEE,0XE6,0X3D,0X7E,0XE9,0X7F,0XF7,0XDF,0XE4,0XF6,0X7F,0XE0,0XF1,0X5F,0XD9,0X7A,0X04,0X0D,0XAF,0XEA,0X2B,0XEC,0XA9,0XFA,0XA1,0X47,0X2F,0XEB,0X57,0X5D,0XAF,0XBB,0X36,0X22,0XA9,0X6E,0X7F,0X47,0X67,0XFB,0XA0,0X5F,0X9B,0X83,0XFF,0XD3,0XD1,0XED,0X30,0XE3,0X49,0XE5,0X7B,0X50,0X66,0XDF,0XD9,0XAA,0X6A,0XB2,0X27,0XFF,0X3B,0XDB,0X71,0X99,0X7F,0X15,0XCB,0X7D,0X96,0X51,0XBA,0X57,0X4C,0XF7,0XD7,0XE5,0XEB,0XFE,0XF2,0X7E,0XC9,0X52,0XC5,0X77,0X9E,0X77,0X27,0X30,0XB7,0X63,0X7E,0X6E,0XF5,0X7F,0XEF,0X81,0X27,0X74,0XFB,0X7F,0XBA,0X7A,0X25,0XDF,0X7C,0XFB,0XFF,0XBF,0XCC,0XFF,0X8B,0XFB,0XFF,0X87,0X9C,0X73,0XFD,0X05,0XF5,0XBE,0XF5,0X34,0X7F,0XF4,0XEB,0XFF,0X6F,0X6C,0X93,0XF1,0X27,0X6E,0XA9,0XCE,0XA6,0XF6,0XC8,0X22,0X1F,0XF2,0XFF,0XE9,0X58,0XEE,0XDE,0XEA,0XAC,0XFF,0XD9,0X96,0XEE,0XBA,0XEF,0XEF,0X41,0X67,0XE3,0XDF,0XBF,0X8D,0X8E,0X6F,0X34,0XEC,0XFA,0XFD,0XD8,0XFB,0XC6,0X9F,0XF5,0XE7,0XF6,0X26,0XBF,0X8C,0XFD,0X0D,0XFD,0XB1,0XEE,0XF3,0X73,0X28,0XA5,0XD7};
const uint8_t INQUIRY32[]={ 0x00 ,0x80 ,0x00 ,0x1C ,0x30 ,0x45 ,0x34 ,0x35 ,0x39 ,0x34 ,0x32 ,0x30 ,0x32 ,0x44 ,0x36 ,0x39 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 };


/*******************************************************************************
 * @fn      Uinfo_init
 *
 * @brief   None
 *
 * @return  None
 */
void Uinfo_init( void ){

    CSW_PARAMETER[0] = 0x55;   //CSW HEADER
    CSW_PARAMETER[1] = 0x53;
    CSW_PARAMETER[2] = 0x42;
    CSW_PARAMETER[3] = 0x53;

    FORMAT_CAPACITIES.NULL1=0;
    FORMAT_CAPACITIES.NULL2=0;
    FORMAT_CAPACITIES.NULL3=0;
    FORMAT_CAPACITIES.Cap_List_Length=0x08;//Capacity list length
    FORMAT_CAPACITIES.Block_Num_32_24=0x00;
    FORMAT_CAPACITIES.Block_Num_24_16=0x00;
    FORMAT_CAPACITIES.Block_Num_16_8=0x10;
    FORMAT_CAPACITIES.Block_Num_8_0=0x00;
    FORMAT_CAPACITIES.descriptor_code=0x03;
    FORMAT_CAPACITIES.blocksize24 = (UINT8)(TF_EMMCParam.EMMCSecSize>>16);
    FORMAT_CAPACITIES.blocksize16 = (UINT8)(TF_EMMCParam.EMMCSecSize>>8);
    FORMAT_CAPACITIES.blocksize8  = (UINT8)(TF_EMMCParam.EMMCSecSize);

    READ_CAPACITY.maxblock32 =(UINT8)( (TF_EMMCParam.EMMCSecNum-1)>>24  );  //Last logical block address
    READ_CAPACITY.maxblock24 = (UINT8)( (TF_EMMCParam.EMMCSecNum-1)>>16  );
    READ_CAPACITY.maxblock16 = (UINT8)( (TF_EMMCParam.EMMCSecNum-1)>>8  );
    READ_CAPACITY.maxblock8 = (UINT8)( (TF_EMMCParam.EMMCSecNum-1)  );

    READ_CAPACITY.blocksize32 = (UINT8)(TF_EMMCParam.EMMCSecSize>>24);      //Logical block size
    READ_CAPACITY.blocksize24 = (UINT8)(TF_EMMCParam.EMMCSecSize>>16);
    READ_CAPACITY.blocksize16 = (UINT8)(TF_EMMCParam.EMMCSecSize>>8);
    READ_CAPACITY.blocksize8  = (UINT8)(TF_EMMCParam.EMMCSecSize);


}

/*******************************************************************************
 * @fn      Ucsw
 *
 * @brief   None
 *
 * @return  None
 */
void Ucsw(UINT8 sta){
    CSW_PARAMETER[12] = sta;

    USB30_IN_ClearIT( ENDP_1 );
    USBSS->UEP1_TX_DMA  = (UINT32)(UINT8 *)( CSW_PARAMETER );
    USB30_IN_Set( ENDP_1 ,ENABLE, ACK , 1 , 13 );
    USB30_Send_ERDY( ENDP_1 | IN , 1 );

}

/*******************************************************************************
 * @fn      csw_err_pres
 *
 * @brief   None
 *
 * @return  None
 */
void csw_err_pres(UINT16 num , UINT16 cnt , UINT8 dir)
{
    ;

}

/*******************************************************************************
 * @fn      write_start
 *
 * @brief   None
 *
 * @return  None
 */
void write_start( UINT16 pReqnum, UINT32 Lbaaddr,UINT8 cmden){
    UINT16  sdtran = 0, usbtran = 0;
    UINT8   sdstep=0, usbstep=0;
    UINT8  flag = 0 ,full = 0;
    UINT32 timecnt = 0;             //Timeout flag
    while(!(R32_EMMC_STATUS & (1<<17))){
        timecnt++;
        if(timecnt>TIME_OVER_SET)   //Timeout exit
            break;
    }
    if(cmden){
        EMMCSendCmd((UINT32)Lbaaddr,(UINT16)(RB_EMMC_CKIDX|RB_EMMC_CKCRC|RESP_TYPE_48|EMMC_CMD25));//Send continuous write command
        R32_EMMC_TRAN_MODE = RB_EMMC_DMA_DIR |(1<<6);        //Set transmission direction
        R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)endp2RTbuff;     //Set dma address
        USBSS->UEP2_RX_DMA = (UINT32)(UINT8 *)endp2RTbuff;   //USB initial address
        USB30_OUT_Set( ENDP_2 , ACK , 1 );                   //Need to receive the first packet of data
        USB30_Send_ERDY( ENDP_2 | OUT, 1 );
        timecnt=0;
        while(!(R16_EMMC_INT_FG & RB_EMMC_IF_CMDDONE)){      //Wait for the command to complete
            timecnt++;
            if(timecnt>TIME_OVER_SET){                       //Timeout reset EMMC
                R16_EMMC_CLK_DIV=0;
                R32_EMMC_BLOCK_CFG = 0;
                EMMCIO0Init();
                EMMCCardConfig( &TF_EMMCParam );
                EMMCSendCmd((UINT32)Lbaaddr,(UINT16)(RB_EMMC_CKIDX|RB_EMMC_CKCRC|RESP_TYPE_48|EMMC_CMD25));//Send continuous write command
                R32_EMMC_TRAN_MODE = RB_EMMC_DMA_DIR;        //Start emmc transmission
                R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)endp2RTbuff;
            }
        }
        R16_EMMC_INT_FG = RB_EMMC_IF_CMDDONE;                //Clear command completion interrupt
        while( !USB30_OUT_ITflag( ENDP_2 ) );                //Wait for the first packet data transmission to complete
        R32_EMMC_BLOCK_CFG = 512<<16 | pReqnum  ;            //EMMC can be transferred after USB transfer
    }
    else{
        USBSS->UEP2_RX_DMA = (UINT32)(UINT8 *)endp2RTbuff;   //USB Initial address
        USB30_OUT_Set( ENDP_2 , ACK , 1 );
        USB30_Send_ERDY( ENDP_2 | OUT, 1 );


        R32_EMMC_TRAN_MODE = RB_EMMC_DMA_DIR|(1<<6);         //Set transmission direction
        R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)endp2RTbuff;
        while( !USB30_OUT_ITflag( ENDP_2 ) );
        R32_EMMC_BLOCK_CFG = 512<<16 | pReqnum  ;
    }
    while(1){
        if( USB30_OUT_ITflag( ENDP_2 ) ){ //USB Transfer completed
            USB30_OUT_ClearIT(ENDP_2);
            usbtran +=2;
            usbstep +=2;
            if( usbstep == MAX_BUF_BLOCK ){   usbstep = 0;    }
            USBSS->UEP2_RX_DMA = (UINT32)(UINT8 *)( endp2RTbuff + (usbstep*512)  );
            if( ( usbtran - sdtran ) >= (MAX_BUF_BLOCK-2) ){ //full
                full = 1;
            }
            else{
                USB30_OUT_Set( ENDP_2 , ACK , 1 );
                USB30_Send_ERDY( ENDP_2 | OUT, 1 );
            }
            if( flag ){
                R32_EMMC_DMA_BEG1 = (UINT32)(UINT8 *)( endp2RTbuff + sdstep * 512 ); //start
                flag = 0;
            }
            timecnt = 0;
        }
        if(R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP){  //emmc Block transfer completed
            R16_EMMC_INT_FG = RB_EMMC_IF_BKGAP;  //Block cleaning completion interrupt
            sdtran++;
            sdstep++;
            if( sdstep == MAX_BUF_BLOCK ){   sdstep = 0;   }
            if(sdtran < usbtran){
                R32_EMMC_DMA_BEG1 = (UINT32)(UINT8 *)( endp2RTbuff + sdstep * 512 ); //Write next sector
            }
            else{        //If the annulus is empty, the transfer will be stopped and the next USB transfer will be completed
                flag = 1;
            }
            if(((usbtran-sdtran)<=(MAX_BUF_BLOCK-2)) && full ){
                full = 0;
                USB30_OUT_Set( ENDP_2 , ACK , 1 );
                USB30_Send_ERDY( ENDP_2 | OUT, 1 );
            }
            timecnt = 0;
        }
        else if(R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE)
        {
            R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE;
            break;
        }
        timecnt++;
        if(timecnt>TIME_OVER_SET){ //timeout handler
            printf("TO\n");
            timecnt = 0;
            if(full == 1){
                R16_EMMC_CLK_DIV=0;
                R32_EMMC_BLOCK_CFG = 0;
                EMMCIO0Init();
                EMMCCardConfig( &TF_EMMCParam ); //Reset EMMC
                EMMCSendCmd((UINT32)Lbaaddr+sdtran,(UINT16)(RB_EMMC_CKIDX|RB_EMMC_CKCRC|RESP_TYPE_48|EMMC_CMD25));
                R32_EMMC_TRAN_MODE = RB_EMMC_DMA_DIR|(1<<6);
                R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)( endp2RTbuff + sdstep * 512 );
                timecnt=0;
                while(!(R16_EMMC_INT_FG & RB_EMMC_IF_CMDDONE)){
                    timecnt++;
                    if(timecnt>TIME_OVER_SET){
                        timecnt=0;
                       break;
                    }
                }
                R16_EMMC_INT_FG = RB_EMMC_IF_CMDDONE;
                R32_EMMC_BLOCK_CFG = 512<<16 | (pReqnum-sdtran);
            }
        }
    }
    USB30_OUT_ClearIT(ENDP_2);
    USBSS->UEP2_RX_DMA = (UINT32)(UINT8 *)( endp2RTbuff );
}

/*******************************************************************************
 * @fn      read_start
 *
 * @brief   None
 *
 * @return  None
 */
void read_start( UINT16 pReqnum, UINT32 Lbaaddr,UINT8 cmden){
        UINT16V  sdtran = 0, usbtran = 0;
        UINT8V   sdstep=0, usbstep=0;
        UINT8V  lock = 0, flag = 1;
        UINT32 timecnt = 0;    //Timeout flag
        R32_EMMC_DMA_BEG1 = (UINT32)(UINT8 *)endp1RTbuff;
        if(cmden){
            R32_EMMC_TRAN_MODE = (UINT32)((1<<4)|(1<<1));
            R32_EMMC_BLOCK_CFG = (512<<16) | pReqnum;      //Write the number of blocks transferred and start
            EMMCSendCmd((UINT32)Lbaaddr,(UINT16)(RB_EMMC_CKIDX|RB_EMMC_CKCRC|RESP_TYPE_48|EMMC_CMD18));    //Continuous reading
        }
        else{  //If do not need to send the CMD command, clear the stop bit directly to continue the transmission
            R32_EMMC_BLOCK_CFG = (512<<16) | pReqnum;      //Write the number of blocks transferred and start
            R32_EMMC_TRAN_MODE = (UINT32)((1<<4));
            R32_EMMC_TRAN_MODE = (UINT32)((1<<4)|(1<<1));
        }
        while( 1 )
        {
            if( ( usbtran < (sdtran-1) ) && ( ( USBSS->UEP1_TX_CTRL & (1<<31) ) || flag ) ){
                USB30_IN_ClearIT( ENDP_1);
                USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)( endp1RTbuff + usbstep * 512 );
                USB30_IN_Set(ENDP_1,ENABLE,ACK,1,1024);
                USB30_Send_ERDY( ENDP_1 | IN, 1 );
                usbtran+=2;
                usbstep+=2;
                if( usbstep == MAX_BUF_BLOCK ){  usbstep = 0;  }
                flag = 0;
                if( lock )
                {
                    lock = 0;
                    R32_EMMC_TRAN_MODE = (UINT32)(1<<4);    //Block transfer mode
                }
                timecnt = 0;
            }

            if( R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP ){  //Block transfer completed
                R16_EMMC_INT_FG = RB_EMMC_IF_BKGAP;
                sdtran++;
                sdstep++;
                if( sdstep == MAX_BUF_BLOCK )
                {
                    sdstep = 0;
                }
                R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)(endp1RTbuff + sdstep*512);
                if((sdtran-usbtran)<(MAX_BUF_BLOCK-2))
                {
                    R32_EMMC_TRAN_MODE = (UINT32)(1<<4);
                }
                else   { lock = 1;}
                timecnt = 0;
            }
            else if( R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE ){
                R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE | RB_EMMC_IF_CMDDONE;
                sdtran++;
                sdstep++;
                break;
            }
            timecnt++;
            if(timecnt>TIME_OVER_SET){      //timeout
                R16_EMMC_CLK_DIV=0;
                R32_EMMC_BLOCK_CFG = 0;
                EMMCIO0Init();
                EMMCCardConfig( &TF_EMMCParam ); //reset EMMC
                R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)(endp1RTbuff + sdstep*512);
                R32_EMMC_TRAN_MODE = (UINT32)((1<<4)|(1<<1));
                R32_EMMC_BLOCK_CFG = (512<<16) | (pReqnum-sdtran);
                EMMCSendCmd((UINT32)Lbaaddr+sdtran,(UINT16)(RB_EMMC_CKIDX|RB_EMMC_CKCRC|RESP_TYPE_48|EMMC_CMD18));
                timecnt=0;
            }
        }
        while( 1 )
        {
            if(( USBSS->UEP1_TX_CTRL & (1<<31) )|| flag )
            {
                flag = 0;
                USB30_IN_ClearIT( ENDP_1);
                if((sdtran-usbtran) >1){
                    USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)( endp1RTbuff + usbstep * 512 );
                    USB30_IN_Set(ENDP_1,ENABLE,ACK,1,1024);
                    USB30_Send_ERDY( ENDP_1 | IN, 1 );
                    usbtran+=2;
                    usbstep+=2;
                }
                else{
                    USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)( endp1RTbuff + usbstep * 512 );
                    USB30_IN_Set(ENDP_1,ENABLE,ACK,1,512);
                    USB30_Send_ERDY( ENDP_1 | IN, 1 );
                    usbtran++;
                    usbstep++;
                }
                if( usbstep == MAX_BUF_BLOCK ){
                    usbstep = 0;
                }
                if( usbtran == sdtran ){
                    while(!(USBSS->UEP1_TX_CTRL & (1<<31)));
                    USB30_IN_ClearIT( ENDP_1);
                    break;
                }
            }
        }
}

/*******************************************************************************
 * @fn      cmd12
 *
 * @brief   None
 *
 * @return  None
 */
void cmd12( void ){
    UINT32 timecnt=0;

    R32_EMMC_TRAN_MODE = 0;
    R32_EMMC_ARGUMENT = 0;
    R16_EMMC_CMD_SET = (UINT16)(
                            RB_EMMC_CKIDX
                            |RB_EMMC_CKCRC
                            |RESP_TYPE_R1b
                            |EMMC_CMD12
                            );
    while(!(R16_EMMC_INT_FG & RB_EMMC_IF_CMDDONE)){
        timecnt++;
        if(timecnt>TIME_OVER_SET){
            printf("cmd12ov\n");
            R32_EMMC_ARGUMENT = 0;
            R16_EMMC_CMD_SET = (UINT16)(RB_EMMC_CKIDX|RB_EMMC_CKCRC|RESP_TYPE_R1b|EMMC_CMD12);
        }
    }
    R16_EMMC_INT_FG = RB_EMMC_IF_CMDDONE;
}




