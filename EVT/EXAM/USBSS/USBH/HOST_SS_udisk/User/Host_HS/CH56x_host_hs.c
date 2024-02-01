/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_host_hs.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_host_hs.h"
#include "CH56x_usb30h.h"
#include "CH56xusb30h_LIB.h"
#include "CH56X_UDISK.h"

__attribute__ ((aligned(4))) const UINT8  GetDevDescrptor[]={USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof( USB_DEV_DESCR ), 0x00};
__attribute__ ((aligned(4))) const UINT8  GetConfigDescrptor[]= {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00};
__attribute__ ((aligned(4))) const UINT8  SetAddress[]={USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const UINT8  SetConfig[]={USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const UINT8  Clear_EndpStall[]={USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*global define */
#define pSetupReq    ((PUSB_SETUP_REQ)endpTXbuff)

/*global variable */
DEV_INFO_Typedef g_U20DevInfo;
UINT8 U20_Endp0MaxSize = 0;
USB_SETUP_REQ *Ctl_Setup;
UINT8 UsbDevEndp0Size = 8;

/*******************************************************************************
 * @fn        user2mem_copy
 *
 * @brief     copy the contents of the buffer to another address.
 *
 * @param     usrbuf - buffer address
 *            addr - target address
 *            bytes - length
 *
 * @return    None
 */
void CopySetupReqPkg( const UINT8 *pReqPkt )
{
    UINT8 i;

    for ( i = 0; i != sizeof( USB_SETUP_REQ ); i ++ )
    {
        ((PUINT8)pSetupReq)[ i ] = *pReqPkt;
        pReqPkt++;
    }
}

/*******************************************************************************
 * @fn        SetBusReset
 *
 * @brief     send bus reset.
 *
 * @return    None
 */
void USB20HOST_SetBusReset( void )
{
    UINT16 i;

    R8_UHOST_CTRL = RB_UH_BUS_RESET;
    for(i = 0;i<150;i++)
    {
        DelayUs(100);
        if( R8_USB_SPD_TYPE & UST_HS)  break;
    }
    R8_UHOST_CTRL &= 0 ;
    DelayMs(1);
    R8_UHOST_CTRL = RB_UH_AUTOSOF_EN;
    if( R8_USB_SPD_TYPE & UST_HS )       g_U20DevInfo.DeviceSpeed = 1;
}

/*******************************************************************************
 * @fn        USBHS_Host_Init
 *
 * @brief     USB host initialized.
 *
 * @param     sta - 1-enable 0-disable
 *
 * @return    None
 */
void USB20Host_Init(FunctionalState sta)
{
    if(sta)
    {
        R8_USB_CTRL = RB_USB_CLR_ALL| RB_USB_RESET_SIE;
        R8_USB_INT_EN = 0;
        R8_USB_CTRL = RB_USB_HOST_MODE |RB_USB_INT_BUSY| UCST_HS  | RB_USB_DMA_EN;
        R8_USB_DEV_AD = 0;
        R8_UHOST_CTRL = 0;

        R32_UH_TX_DMA = (UINT32)(UINT8 *)endpTXbuff;
        R32_UH_RX_DMA = (UINT32)(UINT8 *)endpRXbuff;

        R16_UH_RX_MAX_LEN = U20_MAXPACKET_LEN;
        R8_UH_EP_MOD = RB_UH_TX_EN | RB_UH_RX_EN ;
        R8_UH_TX_CTRL = 0;
        R8_UH_RX_CTRL = 0;
    }
    else
    {
        R8_USB_CTRL = RB_USB_CLR_ALL | RB_USB_RESET_SIE ;
    }
}

/*******************************************************************************
 * @fn        USB20HOST_Transact
 *
 * @brief     USB transact
 *
 * @param     endp_pid - bit7~bit4 current PID  of USB transact. bit3~bit0 target endpoint number
 *            toggle - sync trigger bit
 *            timeout - value of timeout
 *
 * @return    ERR_SUCCESS
 *            ERR_USB_TRANSFER
 *            ERR_USB_UNKNOWN
 *            ERR_USB_DISCON
 *            ERR_USB_CONNECT
 */
UINT8 USB20HOST_Transact( UINT8 endp_pid, UINT8 toggle,UINT32 timeout)
{
    UINT8 TransRetry=0;
    UINT8 r;
    UINT16  i;

    do
    {
        R8_USB_INT_FG = RB_USB_IF_TRANSFER;
        R8_UH_TX_CTRL = (toggle<<3) ;
        R8_UH_RX_CTRL = (toggle<<3) ;
        R16_UH_EP_PID = endp_pid;
        for ( i = 2000; i != 0 && (R8_USB_INT_FG&(RB_USB_IF_TRANSFER|RB_USB_IF_DETECT)) == 0; i -- )
        {
            mDelayuS(1);
        }
        R16_UH_EP_PID = 0;
        if ( (R8_USB_INT_FG&(RB_USB_IF_TRANSFER|RB_USB_IF_DETECT)) == 0 )     {return( ERR_USB_UNKNOWN );}

        if( R8_USB_INT_FG & RB_USB_IF_DETECT )
        {
            mDelayuS(200);
            R8_USB_INT_FG = RB_USB_IF_DETECT;
            if( R8_USB_MIS_ST & RB_USB_ATTACH )
            {
                if(R8_UHOST_CTRL & RB_UH_AUTOSOF_EN)
                {
                    return ( ERR_USB_CONNECT );
                }
            }
            else    return ( ERR_USB_DISCON );
        }

        if( R8_USB_INT_FG & RB_USB_IF_TRANSFER )
        {
            if ( R8_USB_INT_ST & RB_USB_ST_TOGOK )        {return( ERR_SUCCESS1 );}
            r = R8_USB_INT_ST & RB_HOST_RES_MASK;
            if ( r == USB_PID_STALL )                   {return( r | ERR_USB_TRANSFER );}
            if ( r == USB_PID_NAK )
            {
                if ( timeout == 0 )                     {return( r | ERR_USB_TRANSFER );}
                if ( timeout < 0xFFFFFFFF ) timeout --;
                -- TransRetry;
            }
            else{
                switch ( endp_pid >> 4 )
                {
                case USB_PID_SETUP:
                case USB_PID_OUT:
                    if ( r ) {return( r | ERR_USB_TRANSFER );}
                    break;

                case USB_PID_IN:
                    if ( r == USB_PID_DATA0 || r == USB_PID_DATA1 ) { }
                    else if ( r ) {return( r | ERR_USB_TRANSFER );}
                    break;

                default:
                    return( ERR_USB_UNKNOWN );
                    break;
                }
            }
        }
        else        R8_USB_INT_FG = 0xFF;
      //  mDelayuS(15);
    }while(++ TransRetry < 10);
    return (ERR_USB_TRANSFER);
}


/*******************************************************************************
* Function Name  : U20HOST_GetDeviceDescr
* Description    : USB Host gets device descriptor
* Input          :
                   *buf------data buffer
                   *len------data length
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HOST_GetDeviceDescr( UINT8 *buf ,UINT16 *len )
{
    UINT8 l;
    UINT8  status;
    UINT8 setup_buf[8];
    l = *len;
    setup_buf[0] = 0x80;
    setup_buf[1] = 0x06;
    setup_buf[2]  = 0x00;
    setup_buf[3]  = 0x01;
    setup_buf[4]  = 0x00;
    setup_buf[5]  = 0x00;
    setup_buf[6] = (UINT8)l;
    setup_buf[7] = 0x00;

    status = USB20HOST_CtrlTransfer( setup_buf,buf,&l );
    if( status == USB_INT_SUCCESS )
    {
        *len = l;
        UsbDevEndp0Size = *(buf+7);
    }
    return status;
}

/*******************************************************************************
* Function Name  : U20HOST_GetConfigDescr
* Description    : USB Host gets device descriptor
* Input          :
                   *buf------data buffer
                   *len------data length
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HOST_GetConfigDescr( UINT8 *buf ,UINT16 *len )
{
    UINT8 l;
    UINT8  status;
    UINT8 setup_buf[8];
    setup_buf[0] = 0x80;
    setup_buf[1] = 0x06;
    setup_buf[2]  = 0x00;
    setup_buf[3]  = 0x02;
    setup_buf[4]  = 0x00;
    setup_buf[5]  = 0x00;
    setup_buf[6] = 0x40;
    setup_buf[7] = 0x00;
    l = 0x40;
    status = USB20HOST_CtrlTransfer( setup_buf,buf,&l );
    if( status == USB_INT_SUCCESS )
    {
        setup_buf[6] = (( (PUSB_CFG_DESCR)buf ) -> wTotalLength)&0xff;
        setup_buf[7] = (( (PUSB_CFG_DESCR)buf ) -> wTotalLength)>>8;

        l = Ctl_Setup->wLength;
        status = USB20HOST_CtrlTransfer( setup_buf,buf,&l );
        if( status == USB_INT_SUCCESS )
        {
            *len = l;
        }
        else
        {
            *len = 0x00;
        }
    }
    return status;
}

/*******************************************************************************
* Function Name  : U20HOST_SetAddress
* Description    : USB Host setting address
* Input          :
                   addr------setting address
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HOST_SetAddress( UINT8 addr )
{
    UINT8  status;

    UINT8 setup_buf[8];
    setup_buf[0] = 0X00;
    setup_buf[1] = 0x05;
    setup_buf[2]  = addr;
    setup_buf[3]  = 0x00;
    setup_buf[4]  = 0x00;
    setup_buf[5]  = 0x00;
    setup_buf[6] = 0x00;
    setup_buf[7] = 0x00;
    status = USB20HOST_CtrlTransfer( setup_buf,NULL,NULL );
    if( status == USB_INT_SUCCESS )
    {
        R8_USB_DEV_AD = addr;
        mDelaymS( 10 );
    }
    mDelaymS( 5 );
    return( status );
}

/*******************************************************************************
* Function Name  : USBHOST_SetConfig
* Description    : USB Host setting configuration value
* Input          :
                   addr------ Configuration value to set
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HOST_SetConfig( UINT8 cfg )
{
    UINT8  status;
    UINT8 setup_buf[8];
    setup_buf[0] = 0x00;
    setup_buf[1] = 0x09;
    setup_buf[2]  = cfg;
    setup_buf[3]  = 0x00;
    setup_buf[4]  = 0x00;
    setup_buf[5]  = 0x00;
    setup_buf[6] = 0x00;
    setup_buf[7] = 0x00;
    status = USB20HOST_CtrlTransfer( setup_buf,NULL,NULL );
    return( status );
}

/*******************************************************************************
* Function Name  : U20HOST_CofDescrAnalyse
* Description    : USB Host Analysis Configuration Descriptor
* Input          : *pbuf---Descriptor buffer
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 U20HOST_CofDescrAnalyse( UINT8 *pbuf )
{
    static UINT8 gDeviceClassType;
    gDeviceClassType = ( (PUSB_CFG_DESCR_LONG)pbuf ) -> itf_descr.bInterfaceClass;
    if( ( gDeviceClassType <= 0x09 ) || ( gDeviceClassType == 0xFF ) )
    {
        return( USB_INT_SUCCESS );
    }
    else
    {
        gDeviceClassType = USB_DEV_CLASS_RESERVED;
        return( ERR_USB_UNKNOWN );
    }
}

/*******************************************************************************
 * @fn        USB20Host_Enum
 *
 * @brief     enumerate device.
 *
 * @param     Databuf - receive buffer
 *
 * @return    ERR_SUCCESS
 *            ERR_USB_TRANSFER
 */
UINT8 USB20Host_Enum( UINT8 *Databuf )
{
       UINT8 status;
       UINT8 cfg,i;
       UINT32 timeout;
       UINT16 len;
       USB20HOST_SetBusReset();
       timeout = 0;
       while(1){
           R8_USB_INT_FG = RB_USB_IF_DETECT;
           if( R8_USB_MIS_ST&RB_USB_ATTACH )
           {
               printf("USB2.0 DEVICE ATTACH1111 !\n");
               break;
           }
           else
           {
               printf("USB2.0 DEVICE DEATTACH2222 !\n");
               return USB_INT_DISCONNECT;
           }
           timeout++;
           if( timeout>=0x10000 )return USB_CH56XUSBTIMEOUT;
       }
       R8_USB_DEV_AD = 0;
       len = 7;
       status = U20HOST_GetDeviceDescr( Databuf,&len );
       if( status != USB_INT_SUCCESS )
       {
           printf("GetDev_ERROR=%02x\n",status);
           return USB_OPERATE_ERROR;
       }
       UsbDevEndp0Size = *(Databuf+4);

       status = U20HOST_SetAddress( 0x08 );
       if( status != USB_INT_SUCCESS )
       {
           printf("SetAddr_ERROR=%02x\n",status);
           return USB_OPERATE_ERROR;
       }
       len = 0x12;
       status = U20HOST_GetDeviceDescr( Databuf,&len );
       if( status != USB_INT_SUCCESS )
       {
           printf("GetDev_ERROR=%02x\n",status);
           return USB_OPERATE_ERROR;
       }
       status = U20HOST_GetConfigDescr( Databuf,&len );
       if( status != USB_INT_SUCCESS )
       {
           printf("GetDev_ERROR=%02x\n",status);
           return USB_OPERATE_ERROR;
       }
       cfg = ( (PUSB_CFG_DESCR)Databuf ) -> bConfigurationValue;
       printf("cfg=%02x\n",cfg);
       printf("ConfigDescr:");
       for( i=0;i!=len;i++ )
       {
           printf("%02x ",*(Databuf+i));
       }
       printf("\n");
       status=U20HOST_CofDescrAnalyse(Databuf);
       if( status != USB_INT_SUCCESS )
       {
           printf( "Current Device is Not USB Mass Storage Device\n" );
           return( USB_OPERATE_ERROR );
       }
       status = MS_U20HOST_CofDescrAnalyse( Databuf );

       status = U20HOST_SetConfig( cfg );
       if ( status != USB_INT_SUCCESS )
       {
           printf( "SetConfig_ERROR = %02X\n", (UINT16)status );
           return( USB_OPERATE_ERROR );
       }
       return status;
}


/*******************************************************************************
 * @fn        USB20HOST_CtrlTransfer
 *
 * @brief     enumerate device.
 *
 * @param     ReqBuf -- input parameter
 *            DataBuf-- read data
 *            RetLen--Enter data length or return data length
 *
 * @return    ERR_SUCCESS
 *            ERR_USB_TRANSFER
 */
UINT8 USB20HOST_CtrlTransfer(UINT8 *ReqBuf, UINT8 *DataBuf, UINT8 *RetLen )
{
    UINT16  RemLen  = 0;
    UINT8   s, RxLen;
    UINT8   tog;
    PUINT8  pBuf;
    PUINT8  pLen;
    UINT8 *p;

    pBuf = DataBuf;
    pLen = RetLen;
    R32_UH_TX_DMA = (UINT32)endpTXbuff;
    R32_UH_RX_DMA = (UINT32)endpRXbuff;
    if ( pLen )*pLen = 0;
    p = (UINT8 *)endpTXbuff;
    memcpy( p,ReqBuf,8 );


    R16_UH_TX_LEN = 8;
    s = USB20HOST_Transact( USB_PID_SETUP << 4 | 0x00, 0x00, 0x1ffffff );
    if ( s != ERR_SUCCESS1 )             return( s );
    R8_UH_RX_CTRL = UEP_DATA1;
    R8_UH_TX_CTRL = UEP_DATA1;
    R16_UH_TX_LEN = 0x01;
    RemLen = *( ReqBuf + 6 );
    if ( RemLen && pBuf )
    {
        tog = 1;
        if ( *ReqBuf & 0x80 )
        {
            while ( RemLen )
            {

                s = USB20HOST_Transact( USB_PID_IN << 4 | 0x00, tog, 0x1ffffff );
                if ( s != ERR_SUCCESS1 ) return( s );
                tog = tog ^ 1;
                RxLen = R16_USB_RX_LEN < RemLen ? R16_USB_RX_LEN : RemLen;
                RemLen -= RxLen;
                if ( pLen )*pLen += RxLen;
                p = (UINT8 *)endpRXbuff;
                memcpy( DataBuf,p,RxLen );

                DataBuf += *pLen;

                if ( R16_USB_RX_LEN == 0 || ( R16_USB_RX_LEN & ( UsbDevEndp0Size - 1 ) ) ) break;
            }
            R16_UH_TX_LEN = 0x00;
        }
        else
        {
            while ( RemLen )
            {
                R32_UH_TX_DMA = (UINT32)pBuf + *pLen;
                R16_UH_TX_LEN = RemLen >= UsbDevEndp0Size ? UsbDevEndp0Size : RemLen;

                s = USB20HOST_Transact( USB_PID_OUT << 4 | 0x00, tog, 0x1ffffff );
                if ( s != ERR_SUCCESS1 )         return( s );
                tog = tog ^ 1;
                RemLen -= R16_UH_TX_LEN;
                if ( pLen )*pLen += R16_UH_TX_LEN;
            }
            R16_UH_TX_LEN = 0x01;
        }
    }
    mDelayuS( 200 );
    s = USB20HOST_Transact( ( R16_UH_TX_LEN ? (USB_PID_IN << 4 | 0x00): (USB_PID_OUT << 4 | 0x00) ), 1, 100000 );
    if ( s != ERR_SUCCESS1 )         return( s );
    if ( R16_UH_TX_LEN == 0 )       return( ERR_SUCCESS1 );
    if ( R16_USB_RX_LEN == 0 )      return( ERR_SUCCESS1 );
    return( ERR_USB_BUF_OVER );
}

/*******************************************************************************
* Function Name  : USBHSHOST_ClearEndpStall
* Description    : USB Host Purge Endpoint
* Input          : *Device---device operated
                   endp------clear endpoint number
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 USB20HOST_ClearEndpStall( UINT8 endp )
{
    UINT8  status = 0;
    UINT8  setup_buf[8];
    setup_buf[0] = 0x02;
    setup_buf[1] = 0x01;
    setup_buf[2] = 0X00;
    setup_buf[3] = 0x00;
    setup_buf[4] = endp;
    setup_buf[5] = 0x00;
    setup_buf[6] = 0x00;
    setup_buf[7] = 0x00;
    status = USB20HOST_CtrlTransfer( setup_buf, NULL, NULL  );
    return( status );
}

