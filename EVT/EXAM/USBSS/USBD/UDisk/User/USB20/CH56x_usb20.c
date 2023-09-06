/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb20.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_usb20.h"
#include "CH56x_usb30.h"
#include "CH56xusb30_LIB.h"
#include "../Uddisk/uddisk.h"

/* Global Define */
#define pSetupreq    ((PUSB_SETUP)endp0RTbuff)
#define DevEP0SIZE  0x40

/* Global Variable */
volatile BOOL Ttog=1;
UINT8 set_address_status =0;
UINT8 get_config_status =0;
UINT8 device_addr;
UINT8V restlen=0;
UINT8V RWsta_U2 = 0xff;
UINT8V gcsw=0;
UINT32V emmcrst_gbf=0;
UINT32V curWaddr_U2 = 0,curRaddr_U2 = 0;
extern UINT8V link_sta;

void USBHS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


const uint8_t config_descriptor[] =
{

    0X09,0X02,0X20,0X00,0X01,0X01,0X00,0X80,0X3f,

    0X09,0X04,0X00,0X00,0X02,0X08,0X06,0X50,0X00,

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x81,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00,   // polling interval in milliseconds (1 for iso)

    0x07,   // length of this endpoint descriptor
    0x05,   // ENDPOINT (5)
    0x02,   // endpoint direction (80 is in) and address
    0x02,   // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
    0x00,   // max packet size - 512 bytes
    0x02,   // max packet size - high
    0x00    // polling interval in milliseconds (1 for iso)
};

const uint8_t string_descriptor0[] =
{
    0x04,   // this descriptor length
    0x03,   // descriptor type
    0x09,   // Language ID 0 low byte
    0x04    // Language ID 0 high byte
};

const uint8_t string_descriptor1[] =
{
    0x08,   // length of this descriptor
    0x03,
    'W',
    0x00,
    'C',
    0x00,
    'H',
    0x00
};

const uint8_t string_descriptor2[] =
{
    0x1e,   // length of this descriptor
    0x03,
    'U',
    0x00,
    'S',
    0x00,
    'B',
    0x00,
    '3',
    0x00,
    '.',
    0x00,
    '0',
    0x00,
    ' ',
    0x00,
    'D',
    0x00,
    'E',
    0x00,
    'V',
    0x00,
    'I',
    0x00,
    'C',
    0x00,
    'E',
    0x00,
    ' ',
    0x00
};

const uint8_t bos_descriptor[] =
{
    0x05,   // length of this descriptor
    0x0f,   // CONFIGURATION (2)
    0x16,   // total length includes endpoint descriptors (should be 1 more than last address)
    0x00,   // total length high byte
    0x02,       // number of device cap

    0x07,
    0x10,   // DEVICE CAPABILITY type
    0x02,   // USB2.0 EXTENSION
    0x02,
    0x00,
    0x00,
    0x00,

    0x0a,   // length of this descriptor
    0x10,   // DEVICE CAPABILITY type
    0x03,   // superspeed usb device capability
    0x00,   //
    0x0e,   // ss/hs/fs
    0x00,
    0x01,   // the lowest speed is full speed
    0x0a,   // u1 exit latency is 10us
    0xff,   // u1 exit latency is 8us
    0x07
};

const uint8_t MyDevDescr_CH372[ ] = {
        0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,

        0x86,   // vendor id-0x1A86(qinheng)
        0x1A,
        0x10,   // product id
        0xfe,
        0x00, 0x01, 0x00, 0x0, 0x00, 0x01
};

const uint8_t My_FS_CfgDescr_CH372[ ] = {
        0X09,0X02,0X20,0X00,0X01,0X01,0X00,0X80,0X3f,
        0X09,0X04,0X00,0X00,0X02,0XFF,0XFF,0XFF,0X00,
        0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
        0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
};

const uint8_t My_HS_CfgDescr_CH372[ ] = {

                0X09,0X02,0X20,0X00,0X01,0X01,0X00,0X80,0X96,
              0X09,0X04,0X00,0X00,0X02,0X08,0X06,0X50,0X00,
              0x07, 0x05, 0x81, 0x02, 0x00, 0x02, 0x00,
              0x07, 0x05, 0x02, 0x02, 0x00, 0x02, 0x00,

};
const uint8_t qulify_descriptor[]={ 0x0a,0x06,0x10,0x02,0x00,0x00,0x00,0x40,0x01,0x00};

const uint8_t  MyDevDescr[] = {
    0x12, 0x01, 0x10, 0x01,0xFF, 0x80, 0x55, DevEP0SIZE,
    0x48, 0x43, 0x37, 0x55,
    0x00, 0x01, 0x01, 0x02, 0x00, 0x01,
};

const uint8_t  MyCfgDescr[] = {
    0x09, 0x02, 0x4A, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
    0x09, 0x04, 0x00, 0x00, 0x08, 0xFF, 0x80, 0x55, 0x00,
    0x07, 0x05, 0x84, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x04, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x03, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00,
};


/*******************************************************************************
 * @fn      USB20_endp_init
 *
 * @brief   USB2.0 endp initialization
 *
 * @return  None
 */
void USB20_endp_init () // USBHS device endpoint initial
{
    R8_UEP2_3_MOD = RB_UEP2_RX_EN;
    R8_UEP4_1_MOD = RB_UEP1_TX_EN;

    R16_UEP0_MAX_LEN = 64;
    R16_UEP1_MAX_LEN = 0x600;
    R16_UEP2_MAX_LEN = 0x600;


    R32_UEP0_RT_DMA = (UINT32)(UINT8 *)endp0RTbuff;
    R32_UEP1_TX_DMA = (UINT32)(UINT8 *)endp1RTbuff;
    R32_UEP2_RX_DMA = (UINT32)(UINT8 *)endp2RTbuff;

    R8_UEP0_TX_CTRL = UEP_T_RES_NAK;
    R8_UEP0_RX_CTRL = 0;
    R8_UEP1_TX_CTRL = 0;
    R8_UEP2_RX_CTRL = 0;
    R8_UEP2_TX_CTRL = 0;
}

/*******************************************************************************
 * @fn      USB20_dev_init
 *
 * @brief   USB2.0 device initialization
 *
 * @return  None
 */
void USB20_dev_init ( FunctionalState sta )
{
    if(sta){

        R8_USB_CTRL = RB_USB_CLR_ALL;
        R8_USB_CTRL = UCST_HS | RB_DEV_PU_EN | RB_USB_INT_BUSY | RB_USB_DMA_EN;
        R8_USB_INT_EN = RB_USB_IE_SUSPEND | RB_USB_IE_SETUPACT | RB_USB_IE_TRANS | RB_USB_IE_DETECT | RB_USB_IE_BUSRST;// |RB_USB_IE_FIFOOV;

        USB20_endp_init();
    }
    else{
        R8_USB_CTRL =RB_USB_CLR_ALL|RB_USB_RESET_SIE;
    }
}

/*******************************************************************************
 * @fn      USB20_set_address
 *
 * @brief   USB2.0 set address
 *
 * @return  None
 */
void USB20_set_address( UINT32 address )
{
    USBHS->USB_CONTROL &= 0x00ffffff;
    USBHS->USB_CONTROL |= address<<24; // SET ADDRESS

}

/*******************************************************************************
 * @fn      csw_U2
 *
 * @brief   None
 *
 * @return  None
 */
void csw_U2(void){
    USBHS->UEP1_TX_DMA = (UINT32)(UINT8 *)CSW_PARAMETER;
    USBHS->UEP1_CTRL = EP_T_RES_ACK | 13|(Ttog?(EP_T_TOG_0):(EP_T_TOG_1));
}

/*******************************************************************************
 * @fn      DevEP1_OUT_Deal
 *
 * @brief   None
 *
 * @return  None
 */
void DevEP1_OUT_Deal( UINT32 l )
{

}

/*******************************************************************************
 * @fn      write_start_U2
 *
 * @brief   emmc write function
 *           pReqnum : sector number
 *           Lbaaddr : Start address
 *           cmden   : Command update flag
 *
 * @return  None
 */
UINT8   write_start_U2( UINT16V pReqnum, UINT32 Lbaaddr,UINT8 cmden){
    UINT16 sdtran = 0, usbtran = 0;
    UINT8 sdstep=0, usbstep=0, flag = 0;
    UINT8 sta;
    UINT8 errflag=0;
    UINT32 timecnt=0;

    while(!(R32_EMMC_STATUS & (1<<17))){     //Must wait for EMMC to be idle
        timecnt++;
        if(timecnt>TIME_OVER_SET)            //timeout
            break;
    }

    if(cmden){
        R8_UEP2_RX_CTRL = UEP_R_RES_ACK;
        EMMCSendCmd((UINT32)Lbaaddr,(UINT16)(RB_EMMC_CKIDX|RB_EMMC_CKCRC|RESP_TYPE_48|EMMC_CMD25));
        R32_EMMC_TRAN_MODE = RB_EMMC_DMA_DIR;
        R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)endp2RTbuff;
        while(!(R8_USB_INT_FG & RB_USB_IF_TRANSFER));
        R8_UEP2_RX_CTRL = UEP_R_RES_NAK;

        timecnt=0;
        while(!(R16_EMMC_INT_FG & RB_EMMC_IF_CMDDONE)){
            timecnt++;
            if(timecnt>TIME_OVER_SET){
                R16_EMMC_CLK_DIV=0;
                R32_EMMC_BLOCK_CFG = 0;
                EMMCIO0Init();
                EMMCCardConfig( &TF_EMMCParam );
                EMMCSendCmd((UINT32)Lbaaddr,(UINT16)(RB_EMMC_CKIDX|RB_EMMC_CKCRC|RESP_TYPE_48|EMMC_CMD25));
                R32_EMMC_TRAN_MODE = RB_EMMC_DMA_DIR;
                R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)endp2RTbuff;
            }
        }
        R16_EMMC_INT_FG = RB_EMMC_IF_CMDDONE;
        R32_EMMC_BLOCK_CFG = (512<<16) | pReqnum;
    }
    else{
        R8_UEP2_RX_CTRL = UEP_R_RES_ACK;

        R32_EMMC_TRAN_MODE = RB_EMMC_DMA_DIR;
        R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)endp2RTbuff;
        while(!(R8_USB_INT_FG & RB_USB_IF_TRANSFER));
        R8_UEP2_RX_CTRL = UEP_R_RES_NAK;

        R32_EMMC_BLOCK_CFG = (512<<16) | pReqnum;
    }
    timecnt=0;
    while(1){
        if( R8_USB_INT_FG & RB_USB_IF_TRANSFER ){
            timecnt = 0;
            R8_UEP2_RX_CTRL = UEP_R_RES_NAK;
            usbtran++;
            usbstep++;

            if( usbstep == MAX_BUF_BLOCK ){
                usbstep = 0;
            }
            R32_UEP2_RX_DMA = (UINT32)(UINT8 *)( endp2RTbuff + usbstep * 512 );
            if( ( usbtran - sdtran ) == MAX_BUF_BLOCK ){
                R8_UEP2_RX_CTRL = UEP_R_RES_NAK;
            }
            else{
                R8_UEP2_RX_CTRL = UEP_R_RES_ACK;
            }
            if( flag ){
                R32_EMMC_DMA_BEG1 = (UINT32)(UINT8 *)( endp2RTbuff + sdstep * 512 );
                flag = 0;
            }
            R8_USB_INT_FG = RB_USB_IF_TRANSFER;
        }

      if(R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP)
      {
          timecnt = 0;
          R16_EMMC_INT_FG = RB_EMMC_IF_BKGAP;
          sdtran++;
          sdstep++;
          if( sdstep == MAX_BUF_BLOCK ){
              sdstep = 0;
          }
          if(sdtran < usbtran){
              R32_EMMC_DMA_BEG1 = (UINT32)(UINT8 *)( endp2RTbuff + sdstep * 512 );
          }
          else{
              flag = 1;
          }
          R8_UEP2_RX_CTRL = UEP_R_RES_ACK;
      }
      else if(R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE)
      {
          R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE;

          R8_UEP2_RX_CTRL = UEP_R_RES_NAK;
          break;
      }

      timecnt++;
        if(timecnt>TIME_OVER_SET){
            printf("WTO\n");
#if 1
        {
            emmcrst_gbf=1;
            R32_UEP2_RX_DMA=(UINT32)(UINT8 *)( endp2RTbuff  );
            return (1); //need host reset
        }
#else
        {
            timecnt = 0;
            if(( usbtran - sdtran ) == MAX_BUF_BLOCK){
                R16_EMMC_CLK_DIV=0;
                R32_EMMC_BLOCK_CFG = 0;
//                EMMCIO0Init();
//                EMMCCardConfig( &TF_EMMCParam ); //reset EMMC
                emmc_init();
                EMMCSendCmd((UINT32)Lbaaddr+sdtran,(UINT16)(RB_EMMC_CKIDX|RB_EMMC_CKCRC|RESP_TYPE_48|EMMC_CMD25));
                R32_EMMC_TRAN_MODE = RB_EMMC_DMA_DIR|(1<<6);
                R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)( endp2RTbuff + sdstep * 512 );
                timecnt = 0;
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
#endif
        }
    }
    R32_UEP2_RX_DMA=(UINT32)(UINT8 *)( endp2RTbuff  );
    return (0);
}


/*******************************************************************************
 * @fn      read_start_U2
 *
 * @brief   emmc write function
 *           pReqnum : sector number
 *           Lbaaddr : Start address
 *           cmden   : Command update flag
 *
 * @return  None
 */
UINT8 read_start_U2( UINT16 pReqnum, UINT32 Lbaaddr,UINT8 cmden){
        UINT16V  sdtran = 0, usbtran = 0;
        UINT8V   sdstep=0, usbstep=0;
        UINT8V  lock = 0, flag = 1;
        UINT32 timecnt = 0;

        R32_EMMC_DMA_BEG1 = (UINT32)(UINT8 *)endp1RTbuff;
        if(cmden){
              printf("cmd18\n");
            R32_EMMC_TRAN_MODE = (UINT32)((1<<4)|(1<<1));
            R32_EMMC_BLOCK_CFG = (512<<16) | pReqnum;
            EMMCSendCmd((UINT32)Lbaaddr,(UINT16)(RB_EMMC_CKIDX|RB_EMMC_CKCRC|RESP_TYPE_48|EMMC_CMD18));
        }
        else{
            R32_EMMC_BLOCK_CFG = (512<<16) | pReqnum;
            R32_EMMC_TRAN_MODE = (UINT32)((1<<4));
            R32_EMMC_TRAN_MODE = (UINT32)((1<<4)|(1<<1));
        }
        while( 1 )
        {
            if( ( usbtran < (sdtran-1) ) &&  ( (R8_USB_INT_FG & RB_USB_IF_TRANSFER) || flag ) ){

                flag = 0;
                USBHS->UEP1_CTRL = EP_T_RES_NAK;
                USBHS->UEP1_TX_DMA = (UINT32)(UINT8 *)( endp1RTbuff + usbstep * 512 );
                USBHS->UEP1_CTRL = EP_T_RES_ACK | 512 | (Ttog?(EP_T_TOG_0):(EP_T_TOG_1));

                R8_USB_INT_FG = RB_USB_IF_TRANSFER;

                Ttog^=1;
                usbtran+=1;
                usbstep+=1;
                if( usbstep == MAX_BUF_BLOCK )
                {
                    usbstep = 0;
                }
                if( lock ){
                    lock = 0;
                    R32_EMMC_TRAN_MODE = (UINT32)(1<<4);
                }
                timecnt=0;
            }

            if( R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP ){
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
                else
                {
                    lock = 1;
                }
                timecnt=0;
            }
            else if( R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE ){
                R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE | RB_EMMC_IF_CMDDONE;
                sdtran++;
                sdstep++;
                break;
            }

            timecnt++;
            if( timecnt > TIME_OVER_SET ){      //timeout
                printf("RTo\n");
#if 1
                {
                emmcrst_gbf=1;
//                emmc_init();
                return(1);  //need host reset
                }
#else
                {
                R16_EMMC_CLK_DIV=0;
                R32_EMMC_BLOCK_CFG = 0;
//                EMMCIO0Init();
//                EMMCCardConfig( &TF_EMMCParam ); //reset EMMC
                emmc_init();

                R32_EMMC_DMA_BEG1 =(UINT32)(UINT8 *)(endp1RTbuff + sdstep*512);
                R32_EMMC_TRAN_MODE = (UINT32)((1<<4)|(1<<1));
                R32_EMMC_BLOCK_CFG = (512<<16) | (pReqnum-sdtran);
                EMMCSendCmd((UINT32)Lbaaddr+sdtran,(UINT16)(RB_EMMC_CKIDX|RB_EMMC_CKCRC|RESP_TYPE_48|EMMC_CMD18));
                timecnt=0;
                }
#endif
            }
        }

        while( 1 )
        {
            if(( R8_USB_INT_FG & RB_USB_IF_TRANSFER)|| flag  )
            {
                flag = 0;
                USBHS->UEP1_CTRL = EP_T_RES_NAK;
                USBHS->UEP1_TX_DMA = (UINT32)(UINT8 *)( endp1RTbuff + usbstep * 512 );
                USBHS->UEP1_CTRL = EP_T_RES_ACK | 512 | (Ttog?(EP_T_TOG_0):(EP_T_TOG_1));

                R8_USB_INT_FG = RB_USB_IF_TRANSFER;

                Ttog^=1;
                usbtran+=1;
                usbstep+=1;

                if( usbstep == MAX_BUF_BLOCK ){
                    usbstep = 0;
                }
                if( usbtran == sdtran ){
                    while(!(R8_USB_INT_FG & RB_USB_IF_TRANSFER));
                    USBHS->UEP1_CTRL = EP_T_RES_NAK;
                    R8_USB_INT_FG = RB_USB_IF_TRANSFER;
                    break;
                }
            }
        }
        return(0);
}

/*******************************************************************************
 * @fn      DevEP2_OUT_Deal
 *
 * @brief   Deal device Endpoint 2 OUT.
 *
 * @return  None
 */
void DevEP2_OUT_Deal(   )
{
    UINT8 err_flag=0;
    UINT16  i;
    UINT8 *p;
    UINT16 len;
    UINT8 s;
    UINT32 LBAaddr_U2;
    UINT16 writeblocknum_U2 = 0;
    UINT16 reqblocknum_U2 = 0;

    if(*(UINT32 *)endp2RTbuff==0x43425355){//CBW
        *(UINT32 *)&CSW_PARAMETER[4]  = *(UINT32 *)&endp2RTbuff[4];
        printf("CBW:%02x\n",endp2RTbuff[0xf]);

        switch(endp2RTbuff[0xf]){
            case 0x28:   //read
                LBAaddr_U2=(endp2RTbuff[0xf+2]<<24)|(endp2RTbuff[0xf+3]<<16)|(endp2RTbuff[0xf+4]<<8)|(endp2RTbuff[0xf+5]);
                reqblocknum_U2=(endp2RTbuff[0xf+7]<<8)|(endp2RTbuff[0xf+8]);
                printf("Raddr:%08X\n",LBAaddr_U2);

                if((RWsta_U2 != 1) || (curRaddr_U2 != LBAaddr_U2)){
                    curRaddr_U2 = LBAaddr_U2;
                    if(RWsta_U2 != 0xff){
                        cmd12( );
                    }
                    err_flag=read_start_U2(reqblocknum_U2, LBAaddr_U2,1);
                }
                else{
                    curRaddr_U2 = LBAaddr_U2;
                    err_flag=read_start_U2(reqblocknum_U2, LBAaddr_U2,0);
                }
                PFIC_ClearPendingIRQ(USBHS_IRQn);

                len=0;
                curRaddr_U2 += reqblocknum_U2;
                RWsta_U2 = 1;
                if(err_flag){
                    printf("Rer\n");
                    return;
                }
                gcsw=1;
                break;
            case 0x2a:  //write
                  LBAaddr_U2=(endp2RTbuff[0xf+2]<<24)|(endp2RTbuff[0xf+3]<<16)|(endp2RTbuff[0xf+4]<<8)|(endp2RTbuff[0xf+5]);
                  writeblocknum_U2=(endp2RTbuff[0xf+7]<<8)|(endp2RTbuff[0xf+8]);
                  printf("Waddr:%08X\n",LBAaddr_U2);

                  if((RWsta_U2 != 2) || (curWaddr_U2 != LBAaddr_U2)){
                      curWaddr_U2 = LBAaddr_U2;
                      if(RWsta_U2 != 0xff){
                          cmd12( );
                      }
                      err_flag=write_start_U2(writeblocknum_U2, LBAaddr_U2,1);
                  }
                  else{
                      err_flag=write_start_U2(writeblocknum_U2, LBAaddr_U2,0);
                  }
                  PFIC_ClearPendingIRQ(USBHS_IRQn);
                  len=0;
                  RWsta_U2 = 2;
                  curWaddr_U2 += writeblocknum_U2;
                  if(err_flag){
                      return;
                  }
                  gcsw=1;
                  break;
            case 0x12:
                if(endp2RTbuff[0xf+4]==0x24){
                    memcpy(endp1RTbuff, (UINT8 *)INQUIRY1,36 );
                    len=36;
                    restlen=1;
                }
                else if(endp2RTbuff[0xf+4]==0x38){
                    memcpy(endp1RTbuff, (UINT8 *)s1238,56 );
                    len=56;
                    restlen=1;
                }
                else if(endp2RTbuff[0xf+4]==0x2f){
                    memcpy(endp1RTbuff, (UINT8 *)LONG_INQUIRY,0x2f );
                    len=0x2f;
                    restlen=1;
                }
                else if(endp2RTbuff[0xf+4]==0x40){
                    memcpy(endp1RTbuff, (UINT8 *)INQUIRY1,0x40 );
                    len=0x40;
                    restlen=1;
                }
                else if(endp2RTbuff[0xf+1]==0x1){
                    memcpy(endp1RTbuff, (UINT8 *)INQUIRY32,32 );
                    len=32;
                    restlen=1;
                }
                else{
                    memcpy(endp1RTbuff, (UINT8 *)LONG_INQUIRY,255 );
                    len=255;
                    restlen=1;
                }
                break;
            case 0x23:
                memcpy(endp1RTbuff, (UINT8 *)&FORMAT_CAPACITIES.NULL1,12 );
                len=12;
                restlen=1;
                break;
            case 0x25:   //read(10)
                memcpy(endp1RTbuff, (UINT8 *)&READ_CAPACITY.maxblock32,8);
                len=8;
                restlen=1;
                break;
            case 0x9e:   //read(16)
                memset(endp1RTbuff,0,32);
                memcpy(&endp1RTbuff[4], (UINT8 *)&READ_CAPACITY.maxblock32,8);
                restlen=1;
                len=32;
                break;
            case 0x1a:
                if(endp2RTbuff[0xf+2]==0x3f){
                    memcpy(endp1RTbuff, (UINT8 *)s1a03,36);
                    len=36;
                    restlen=1;
                }
                else{
                    *(UINT32 *)endp1RTbuff= 0x00000003;
                    len=4;
                    restlen=1;
                }
                break;
            case 0x00:  //test unit ready
                csw_U2();
                USBHS->UEP2_CTRL = (USBHS->UEP2_CTRL & ~EP_R_RES_MASK) | EP_R_RES_ACK;  // NAK
                len=0;
                break;
            case 0x35:
                csw_U2();
                len=0;
                break;
            case 0x1E:
                csw_U2();
                len=0;
                break;
            case 0x03:
                memcpy(endp1RTbuff, (UINT8 *)s03,18);
                len=18;
                restlen=1;
                break;
            default:
                USBHS->UEP1_CTRL = EP_T_RES_STALL | (Ttog?(EP_T_TOG_0):(EP_T_TOG_1));
                Ttog^=1;
                restlen=1;
                CSW_PARAMETER[12]=1;    //stall until need return CSW
                return;
        }
        CSW_PARAMETER[12]=0;   //csw no error
        if(len){
             USBHS->UEP1_TX_DMA =(UINT32)(UINT8 *)endp1RTbuff;
             USBHS->UEP1_CTRL = EP_T_RES_ACK | len | (Ttog?(EP_T_TOG_0):(EP_T_TOG_1));
        }
    }
    else{
        printf("errrr\n");
        for(i=0;i<512;i++)
            printf("%02X\n",endp2RTbuff[i]);
    }
}

/*******************************************************************************
 * @fn      USBHS_IRQHandler
 *
 * @brief   USB2.0 Interrupt Handler.
 *
 * @return  None
 */
void USBHS_IRQHandler (void){
	UINT32 end_num;
	UINT32 rx_token;
	UINT32 rx_len;
	UINT8  *p8;

	UINT32 i,temp,temp1;
	UINT8  send_data_len =0;
	UINT8  bmRequestType;
	UINT8  bRequest;
	UINT8  wValue_l;
	UINT8  wValue_h;
	UINT16  wLength;
	UINT8  config_len;

	if( R8_USB_INT_FG & RB_USB_IF_TRANSFER )
	{
	    R8_UEP2_RX_CTRL = UEP_R_RES_NAK;
	    R8_UEP1_TX_CTRL = UEP_T_RES_NAK;

	    R8_USB_INT_FG |= RB_USB_IF_TRANSFER;

		end_num =  ((USBHS->USB_STATUS)>>24) & 0xf;  //EPn
		rx_token = ((USBHS->USB_STATUS)>>28) & 0x3; // 00: OUT, 01:SOF, 10:IN, 11:SETUP

		if( end_num == 2 ) //EP2
        {
            if( rx_token == PID_OUT ) // IN - DATA - ACK
            {
                DevEP2_OUT_Deal();
                return;
            }
            else// OUT - DATA - ACK
            {
            }
        }
        else if( end_num == 1 )  //EP1
        {
            if( rx_token == PID_IN ) // IN - DATA - ACK
            {
                 Ttog^=1;
                 if(restlen == 1){
                     restlen = 0 ;
                     csw_U2();
                 }
                 else{
                     USBHS->UEP2_CTRL = (USBHS->UEP2_CTRL & ~EP_R_RES_MASK) | EP_R_RES_ACK;  //ACK
                 }
            }
            else
            {
            }
        }
		else if( end_num == 0 )   //EP0
		{
		    R8_UEP2_RX_CTRL = UEP_R_RES_ACK;
			if( rx_token == PID_IN ) // IN
			{
				if( set_address_status )
				{
				    USB20_set_address( device_addr );// SET ADDRESS
					set_address_status = 0;
					USBHS->UEP0_CTRL = 0;
				}
				else if( get_config_status )
				{
				    memcpy(endp0RTbuff, (UINT8 *)config_descriptor+ 64,config_len );
				    USBHS->UEP0_CTRL = EP_T_RES_ACK | EP_T_TOG_0 | config_len; // DATA stage (IN -DATA1-ACK)
					get_config_status = 0;
				}
				else
				{
					USBHS->UEP0_CTRL = EP_R_RES_ACK | EP_R_TOG_1;
				}
			}
			else if( rx_token == PID_OUT ) //OUT
			{
				USBHS->UEP0_CTRL = EP_T_RES_ACK | EP_T_TOG_1;
			}
		}

	 }
	 else if(R8_USB_INT_FG & RB_USB_IF_SETUOACT)
     {
         temp = endp0RTbuff[0]|(endp0RTbuff[1]<<8)|(endp0RTbuff[2]<<16)|(endp0RTbuff[3]<<24);
         temp1 =  endp0RTbuff[4]|(endp0RTbuff[5]<<8)|(endp0RTbuff[6]<<16)|(endp0RTbuff[7]<<24);
         bmRequestType =endp0RTbuff[0];
         bRequest = endp0RTbuff[1];
         wValue_l = endp0RTbuff[2];
         wValue_h = endp0RTbuff[3];

         if( bmRequestType == 0 ) // without data stage   SETUP
         {
             if( bRequest == 0x5 ) // set address
             {
                 set_address_status = 1;
                 device_addr = wValue_l;
             }
             else if( bRequest == 0x9 ) // set configuration
             {
                 R8_UEP2_RX_CTRL = UEP_R_RES_ACK;
             }
             USBHS->UEP0_CTRL = EP_T_RES_ACK | EP_T_TOG_1; // IN -DATA1-ACK( len =0 )
         }
         else if( bmRequestType == 0x80 ) // data stage is IN   SETUP
         {
             if( bRequest == 0x06 && wValue_h == 0x01 ) // dev desc
             {
                 memcpy(endp0RTbuff, (UINT8 *)MyDevDescr_CH372,18);
                 send_data_len = MyDevDescr_CH372[0];

             }
             else if( bRequest == 0x06 && wValue_h == 0x06 ) // qulify desc
             {
                memcpy(endp0RTbuff, (UINT8 *)qulify_descriptor,qulify_descriptor[0]);
                send_data_len = qulify_descriptor[0];
             }
             else if( pSetupreq->bRequest == 0x06 && wValue_h == 0x02 ) // config desc
             {
                 send_data_len =  ( 0x20 <= pSetupreq->wLength ) ? 0x20 : pSetupreq->wLength;
                 memcpy(endp0RTbuff, (UINT8 *)My_HS_CfgDescr_CH372,0x20);
             }

             else if( bRequest == 0x06 && wValue_h == 0x03  )// string0
             {
                 send_data_len = (string_descriptor0[0] <= pSetupreq->wLength) ? string_descriptor0[0] : pSetupreq->wLength;
                 memcpy(endp0RTbuff, (UINT8 *)string_descriptor0, string_descriptor0[0]);
             }
             USBHS->UEP0_CTRL = EP_T_RES_ACK | EP_T_TOG_1 | send_data_len; // DATA stage  (IN -DATA1-ACK) Send data EP0
         }
         else if( (temp & 0xffff) == 0xfea1 )
         {
             *( UINT8*) endp0RTbuff = 0;
             USBHS->UEP0_CTRL = EP_T_RES_ACK | EP_T_TOG_1 | 1;
         }
         else if( ( bRequest == USB_CLEAR_FEATURE) && ( bmRequestType == 0x02 ) ) // clear feature to end point
         {
             if(endp0RTbuff[4]==0x81){
                 Ttog=1;
                 if(restlen==1){
                     restlen=0;
                     gcsw=1;
                 }
             }
             else if(endp0RTbuff[4]==0x02){

             }
             USBHS->UEP0_CTRL = EP_T_RES_ACK | EP_T_TOG_1; // IN -DATA1-ACK( len =0 )
         }
         else
         {
             USBHS->UEP0_CTRL = EP_T_RES_STALL | EP_R_RES_STALL; // IN - STALL / OUT - DATA - STALL
         }

         R8_USB_INT_FG |= RB_USB_IF_SETUOACT;// clear int flag
     }
	 else if(R8_USB_INT_FG & RB_USB_IF_BUSRST )
	{
	     printf(" bus reset!:%d \n",link_sta);
	     Ttog=1;
	    USB20_dev_init ( ENABLE) ;
		device_addr = 0;
        USB20_set_address( device_addr );
        {
		    if( link_sta == 1 ){
                PFIC_EnableIRQ(USBSS_IRQn);
                PFIC_EnableIRQ(LINK_IRQn);
                PFIC_EnableIRQ(TMR0_IRQn);
                R8_TMR0_INTER_EN = 1;
                TMR0_TimerInit( 67000000 );
                USB30D_init(ENABLE);
		    }
        }
        R8_USB_INT_FG |= RB_USB_IF_BUSRST;
	}
	 else if(R8_USB_INT_FG & RB_USB_IF_SUSPEND )
	{
		printf(" bus suspend! \n");
		R8_USB_INT_FG |= RB_USB_IF_SUSPEND;
	}
}

/*******************************************************************************
 * @fn      u2_setup_packet
 *
 * @brief   None
 *
 * @return  None
 */
void u2_setup_packet( uint32_t tx_len )
{
    USBHS->UEP3_CTRL = tx_len;
    USBHS->UEP2_CTRL = 0xd<<4; // setup

    while( !(USBHS->USB_STATUS & USB2_ACT_FLAG));
    USBHS->UEP2_CTRL = 0;
    USBHS->USB_STATUS = USB2_ACT_FLAG;
}

/*******************************************************************************
 * @fn      u2_get_data
 *
 * @brief   None
 *
 * @return  None
 */
void u2_get_data( uint32_t endp_num )
{
    uint8_t tx_success;

    tx_success = 0;
    USBHS->UEP2_CTRL = (0x9<<4) | endp_num; // in(0x69)
    while( !tx_success )
    {
        if( USBHS->USB_STATUS & USB2_ACT_FLAG )
        {
            if( ((USBHS->USB_STATUS >>24) & 0xf) != 0xa )// !NAK(0x5A)
            {
                tx_success = 1;
                USBHS->UEP2_CTRL = 0;
            }
            USBHS->USB_STATUS = USB2_ACT_FLAG;
        }
    }
}

/*******************************************************************************
 * @fn      u2_send_data
 *
 * @brief   None
 *
 * @return  None
 */
void u2_send_data( uint32_t endp_num, uint32_t tx_len, uint32_t toggle )
{
    uint8_t tx_success;
    uint8_t ping_status;

    tx_success = 0;
    ping_status = 0;

    USBHS->UEP3_CTRL = tx_len | toggle;
    USBHS->UEP2_CTRL = (0xE1<<4) | endp_num; // out(0xE1)
    while( !tx_success )
    {
        if( USBHS->USB_STATUS & USB2_ACT_FLAG )
        {
            if( (((USBHS->USB_STATUS >>24) & 0xf) != 0xa)  & !ping_status )// !NAK(0x5A)
            {
                tx_success = 1;
                USBHS->UEP2_CTRL = 0;
            }
            else if( (((USBHS->USB_STATUS >>24) & 0xf) == 0xa)  & !ping_status )
            {
                ping_status = 1;
                USBHS->UEP2_CTRL = (0xB4<<4) | endp_num;
            }
            else if( (((USBHS->USB_STATUS >>24) & 0xf) != 0xa)  & ping_status  )
            {
                ping_status = 0;
                USBHS->UEP2_CTRL = (0xE1<<4) | endp_num; // out(0xE1)
            }
            else if( (((USBHS->USB_STATUS >>24) & 0xf) == 0xa)  & ping_status  )
            {
                USBHS->USB_STATUS = USB2_ACT_FLAG;
            }

            USBHS->USB_STATUS = USB2_ACT_FLAG;
        }
    }
}

/*******************************************************************************
 * @fn      u2_set_address
 *
 * @brief   set address
 *
 * @return  None
 */
void u2_set_address( uint32_t address )
{
    USBHS->USB_CONTROL &= 0x00ffffff;
    USBHS->USB_CONTROL |= address<<24; // SET ADDRESS
}
