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
#include "hub.h"


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
UINT8 gDiskInterfNumber = 0;
UINT16 U20_ENDP_SIZE;
UINT16 Hid_Report_Len;
UINT8 test_flag = 0;
UINT8 gEndp_Num = 0;
UINT8 Global_Index = 0;
UINT16 EndpnMaxSize = 0;
#define     PID_SSPLIT      (0<<0)
#define     PID_CSPLIT      (1<<0)
#define     WAIT_USB_TOUT_200US     300000
#define     RB_UIS_TOG_OK       0x40      // RO, indicate current USB transfer toggle is OK
#define     MASK_UIS_H_RES      0x0F      // RO, bit mask of current transfer handshake response for USB host mode: 0000=no response, time out from device, others=handshake response PID received
#define     RB_UH_T_NODATA      0x40
#define     RB_UMS_SPLIT_CAN    0x01      // RO,
#define     RB_UH_R_NODATA      0x40

/*******************************************************************************
 * @fn        CopySetupReqPkg
 *
 * @briefI    copy the contents of the buffer to another address.
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
 * @briefI    send bus reset.
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
 * @briefI    USB host initialized.
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
 * @briefI    USB transact
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
            if( R8_USB_MIS_ST & RB_USB_ATTACH ){
                if(R8_UHOST_CTRL & RB_UH_AUTOSOF_EN){
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
                switch ( endp_pid >> 4 ){
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
//        mDelayuS(15);
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
    if( status == USB_INT_SUCCESS ){
        *len = l;
        UsbDevEndp0Size = *(buf+7);
    }
    return status;
}

/*******************************************************************************
* Function Name  : U20HOST_GetDeviceDescr
* Description    : USB Host gets device descriptor
* Input          :
                   portn------HUB Structure Data
                   pdesc------data buffer
                   l    ------desc length
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
void HubAnalysis_Descr(PHUB_Port_Info portn,PUINT8 pdesc, UINT16 l)
{
    UINT16 i;

    UINT8 endp_num = 0;
    for( i=0; i<l; i++ )                                                //Analysis Descriptor
    {
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x02))
         {
                printf("bNumInterfaces:%02x \n",pdesc[i+4]);            //Number of interfaces in configuration descriptor -5th byte
         }
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x04))
         {
              printf("device_type:%02x \n",pdesc[i+5]);                 //interface type
              portn->devicetype = pdesc[i+5];
         }
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x21))                       //Report descriptor length
         {
              Hid_Report_Len = ((UINT16)pdesc[i+8]<<8)|pdesc[i+7];
              printf("Hid_Report_Len:%02x \n",Hid_Report_Len);                 //interface type
         }
         if((pdesc[i]==0x07)&&(pdesc[i+1]==0x05))
         {
            if((pdesc[i+2])&0x80)
            {
                 printf("endpIN:%02x \n",pdesc[i+2]&0x0f);              //Take in endpoint number
                 portn->portEndp[ endp_num ].num = pdesc[i+2];
                 portn->portEndp[ endp_num ].endptype = pdesc[i+3];
                 if(((pdesc[i+5])&0x18) && (portn->speed == 0x01) )     //High speed and high bandwidth endpoints
                 {
                     EndpnMaxSize = ((UINT16)(pdesc[i+5]&0x07)<<8)|pdesc[i+4]; //Take endpoint size
                     portn->portEndp[ endp_num ].HighTransNum = ((pdesc[i+5]&0x18)>>3); //  Number of transactions within a microframe
                 }
                 else {
                     EndpnMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     //Take endpoint size
                     portn->portEndp[ endp_num ].HighTransNum = 0;
                 }
                 portn->portEndp[ endp_num ].endp_size = EndpnMaxSize;
                 portn->portEndp[ endp_num ].tog = 0;
                 endp_num++;
                 portn->endpnum = endp_num;
            }
            else
            {
                printf("endpOUT:%02x \n",pdesc[i+2]&0x0f);              //Take out endpoint number
                EndpnMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     //Take endpoint size
                portn->portEndp[ endp_num ].num = pdesc[i+2];
                portn->portEndp[ endp_num ].endptype = pdesc[i+3];
                if(((pdesc[i+5])&0x18) && (portn->speed == 0x01) )     //High speed and high bandwidth endpoints
                {
                    EndpnMaxSize = ((UINT16)(pdesc[i+5]&0x07)<<8)|pdesc[i+4]; //Take endpoint size
                }
                else {
                    EndpnMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     //Take endpoint size
                }
                portn->portEndp[ endp_num ].endp_size = EndpnMaxSize;
                portn->portEndp[ endp_num ].tog = 0;
                endp_num++;
                portn->endpnum = endp_num;
                printf("Out_endpmaxsize:%02x \n",EndpnMaxSize);
            }
        }
  }
}

/*********************************************************************
 * @fn      USBHS_Analysis_Descr
 *
 * @brief   Descriptor analysis.
 *
 * @param   pusbdev - device information variable.
 *          pdesc - descriptor buffer to analyze
 *          l - length
 *
 * @return  none
 */
void USBHS_Analysis_Descr(pDEV_INFO_Typedef pusbdev,PUINT8 pdesc, UINT16 l)
{
    UINT16 i;
    for( i=0; i<l; i++ )                                                //Analysis Descriptor
    {
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x02))
         {
                printf("bNumInterfaces:%02x \n",pdesc[i+4]);            //Number of interfaces in configuration descriptor -5th byte
         }
         if((pdesc[i]==0x09)&&(pdesc[i+1]==0x04))
         {
             printf("device_type:%02x \n",pdesc[i+5]);                  //interface type
             pusbdev->DeviceType = pdesc[i+5];
         }
         if((pdesc[i]==0x07)&&(pdesc[i+1]==0x05))
         {
            if((pdesc[i+2])&0x80)
            {
                 printf("endpIN:%02x \n",pdesc[i+2]&0x0f);              //Take in endpoint number
                 pusbdev->DevEndp.InEndpNum = pdesc[i+2]&0x0f;
                 pusbdev->DevEndp.InEndpCount++;
                 EndpnMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     //Take endpoint size
                 pusbdev->DevEndp.InEndpMaxSize = EndpnMaxSize;
                 printf("In_endpmaxsize:%02x \n",EndpnMaxSize);
            }
            else
            {
                printf("endpOUT:%02x \n",pdesc[i+2]&0x0f);              //Take out endpoint number
                pusbdev->DevEndp.OutEndpNum = pdesc[i+2]&0x0f;
                pusbdev->DevEndp.OutEndpCount++;
                EndpnMaxSize =((UINT16)pdesc[i+5]<<8)|pdesc[i+4];       //Take endpoint size
                pusbdev->DevEndp.OutEndpMaxSize = EndpnMaxSize;
                printf("Out_endpmaxsize:%02x \n",EndpnMaxSize);
            }
        }
  }
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
UINT8 U20HOST_GetConfigDescr( PHUB_Port_Info phub,UINT8 *buf ,UINT16 *len )
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
    if( status == USB_INT_SUCCESS ){
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

    thisUsbDev.DeviceCongValue = ( (PUSB_CFG_DESCR)buf )-> bConfigurationValue;
    printf("thisUsbDev.DeviceType=%02x\n",thisUsbDev.DeviceType);
    if( thisUsbDev.DeviceType == 0x09 ){
        //HUB processing
        HubAnalysis_Descr( phub,(UINT8 *)buf, pSetupReq->wLength );
    }
    else{
        //Other equipment processing
        USBHS_Analysis_Descr( &thisUsbDev, (UINT8 *)buf, pSetupReq->wLength );
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
* Description    : USBHost Analysis Configuration Descriptor
* Input          : *pbuf---Descriptor buffer
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 U20HOST_CofDescrAnalyse( UINT8 depth,UINT8 *pbuf, UINT8 port)
{
    gDeviceClassType = ( (PUSB_CFG_DESCR_LONG)pbuf ) -> itf_descr.bInterfaceClass;
    if( ( gDeviceClassType <= 0x09 ) || ( gDeviceClassType == 0xFF ) )
    {
        if( gDeviceClassType == 0x09 )
        {         //HUB
            if( port == 0 ){
                hs_hub_info[depth].device_type = gDeviceClassType;
            }
            else{
                hs_hub_info[depth].portD[port-1].devicetype = gDeviceClassType;
            }
        }
        else if( gDeviceClassType == 0x08 )
        {    //U-DISK
            if( port == 0 )
            {
                hs_hub_info[depth].device_type = gDeviceClassType;
            }
            else
            {
                hs_hub_info[depth].portD[port-1].devicetype = gDeviceClassType;
            }
        }
        else
        {
            if( port == 0 )
            {
                hs_hub_info[depth].device_type = gDeviceClassType;
            }
            else
            {
                hs_hub_info[depth].portD[port-1].devicetype = gDeviceClassType;
            }
        }
        return USB_INT_SUCCESS;
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
 * @briefI    enumerate device.
 *
 * @param     Databuf - receive buffer
 *
 * @return    ERR_SUCCESS
 *            ERR_USB_TRANSFER
 */
UINT8 USB20Host_Enum( UINT8 depth,UINT8 *Databuf )
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
    if( status != USB_INT_SUCCESS ){
        printf("GetDev_ERROR=%02x\n",status);
        return USB_OPERATE_ERROR;
    }
    UsbDevEndp0Size = *(Databuf+4);

    status = U20HOST_SetAddress( 0x08 );
    if( status != USB_INT_SUCCESS ){
        printf("SetAddr_ERROR=%02x\n",status);
        return USB_OPERATE_ERROR;
    }
    len = 0x12;
    status = U20HOST_GetDeviceDescr( Databuf,&len );
    if( status != USB_INT_SUCCESS ){
        printf("GetDev_ERROR=%02x\n",status);
        return USB_OPERATE_ERROR;
    }
    status = U20HOST_GetConfigDescr( NULL,Databuf,&len );
    if( status != USB_INT_SUCCESS ){
        printf("GetDev_ERROR=%02x\n",status);
        return USB_OPERATE_ERROR;
    }
    cfg = ( (PUSB_CFG_DESCR)Databuf ) -> bConfigurationValue;
    printf("cfg=%02x\n",cfg);
    printf("ConfigDescr:");
    for( i=0;i!=len;i++ ){
        printf("%02x ",*(Databuf+i));
    }
    printf("\n");
    status=U20HOST_CofDescrAnalyse( depth,Databuf,0);
    if( status != USB_INT_SUCCESS )
    {
        printf( "Current Device is Not USB Mass Storage Device\n" );
        return( USB_OPERATE_ERROR );
    }
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
 * @briefI    enumerate device.
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

/*******************************************************************************
* Function Name  : SetBusReset
* Description    : SetBusReset
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SetBusReset( void )
{
    UINT16 i;

    R8_UHOST_CTRL = RB_UH_BUS_RESET;          //Send bus reset signal
    for( i=0; i<150; i++ )
    {
        mDelayuS( 100 );
        if(R8_USB_SPD_TYPE & UST_HS)break;
    }
    R8_UHOST_CTRL = 0x00;                     // End reset
    mDelaymS( 1 );
    R8_UHOST_CTRL = RB_UH_AUTOSOF_EN;
    mDelaymS( 200 );
}

/*******************************************************************************
* Function Name  : U20HOST_GetHUBDevDescr
* Description    : USB Host gets HUB descriptor
* Input          : *buf------data buffer
                   *len------data length
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HOST_GetHUBDevDescr( UINT8 *buf, UINT16 *len )
{
    UINT8 l;
    UINT8  status;
    UINT8 setup_buf[8];
    l = *len;

    setup_buf[0] = 0xA0;
    setup_buf[1] = 0x06;
    setup_buf[2]  = 0x00;
    setup_buf[3]  = 0x29;
    setup_buf[4]  = 0x00;
    setup_buf[5]  = 0x00;
    setup_buf[6] =  UsbDevEndp0Size;
    setup_buf[7] = 0x00;
    l = UsbDevEndp0Size;
    status = USB20HOST_CtrlTransfer( setup_buf,buf,&l );
    if( status == USB_INT_SUCCESS )
    {
        *len = l;
    }
    l = setup_buf[6] =  *buf;
    status = USB20HOST_CtrlTransfer( setup_buf,buf,&l );
    if( status == USB_INT_SUCCESS )
    {
        *len = l;
    }

    return status;
}

/*******************************************************************************
* Function Name  : U20HOST_SetPortFeatrue
* Description    : USB Host Set Port Properties
* Input          :
                   port number
                   NULL
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HOST_SetPortFeatrue( UINT8 port ,UINT8 port_status )
{
    UINT8  status;
    UINT8 setup_buf[8];
    setup_buf[0] = 0x23;
    setup_buf[1] = 0x03;
    setup_buf[2]  = port_status;
    setup_buf[3]  = 0x00;
    setup_buf[4]  = port;
    setup_buf[5]  = 0x00;
    setup_buf[6] = 0x00;
    setup_buf[7] = 0x00;
    status = USB20HOST_CtrlTransfer( setup_buf,NULL,NULL );
    if( status == USB_INT_SUCCESS )
    {
    }
    return status;
}

/*******************************************************************************
* Function Name  : U20USBHostTransact
* Description    : USB Host Set Port Properties
* Input          : None
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT32 timeout )
{
    UINT8 TransRetry=0;
    UINT8  r;
    UINT16  i;

    do
    {
        R8_USB_INT_FG = RB_USB_IF_TRANSFER;
        R8_UH_TX_CTRL = (tog<<3) ;
        R8_UH_RX_CTRL = (tog<<3) ;
        R16_UH_EP_PID = endp_pid;
        for ( i = 2000; i != 0 && (R8_USB_INT_FG&(RB_USB_IF_TRANSFER|RB_USB_IF_DETECT)) == 0; i -- );
        R16_UH_EP_PID = 0;
        if ( (R8_USB_INT_FG&(RB_USB_IF_TRANSFER|RB_USB_IF_DETECT)) == 0 )     {return( ERR_USB_UNKNOWN );}

        if( R8_USB_INT_FG & RB_USB_IF_DETECT )
        {
            mDelayuS(200);
            R8_USB_INT_FG = RB_USB_IF_DETECT;
            if( R8_USB_MIS_ST & RB_USB_ATTACH ){
                if(R8_UHOST_CTRL & RB_UH_AUTOSOF_EN){
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
                switch ( endp_pid >> 4 ){
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
* Function Name  : U20OST_GetPortStstus
* Description    : USB Host gets port status
* Input          : port number
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HOST_GetPortStstus( UINT8 depth,UINT8 port )
{
    UINT8 l;
    UINT16  Port_Status_Bits,Port_Change_Field;
    UINT8  status;
    UINT8 setup_buf[8];
    UINT8 buf[4];
    setup_buf[0] = 0xA3;
    setup_buf[1] = 0x00;
    setup_buf[2] = 0x00;
    setup_buf[3] = 0x00;
    setup_buf[4] = port;
    setup_buf[5] = 0x00;
    setup_buf[6] = 0x04;
    setup_buf[7] = 0x00;
    l = 4;
    status = USB20HOST_CtrlTransfer( setup_buf,buf,&l );
    if( status == USB_INT_SUCCESS )
    {
        Port_Status_Bits = buf[0];
        Port_Status_Bits |= ((UINT16)buf[1]<<8);
        Port_Change_Field = buf[2];
        Port_Change_Field |= ((UINT16)buf[3]<<8);

        if( Port_Status_Bits&PORT_CONNECTION )
        {
            if(hs_hub_info[depth].portD[port-1].status<PORT_CONNECTION)
            {
                hs_hub_info[depth].portD[port-1].status = PORT_CONNECTION;
            }
            if( Port_Status_Bits & 0x200 )
            {
                hs_hub_info[depth].portD[port-1].speed = PORT_LOW_SPEED;
            }
            else if( Port_Status_Bits & 0x400 )
            {
                hs_hub_info[depth].portD[port-1].speed = PORT_HIGH_SPEED;
            }
            else
            {
                hs_hub_info[depth].portD[port-1].speed = PORT_FULL_SPEED;
            }
        }
        else
        {
            hs_hub_info[depth].portD[port-1].status = PORT_DISCONNECT;
        }
        hs_hub_info[depth].portD[port-1].portpchangefield = Port_Change_Field;

    }
    return status;
}

/*******************************************************************************
* Function Name  : U20HOST_Hub_IntConnect_Process
* Description    :
* Input          :
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8  U20HOST_Hub_IntConnect_Process( UINT8 depth,UINT8 *pbuf )
{
    UINT8 status;
    UINT8 *p;
    UINT16 len;
    p = (UINT8 *)endpTXbuff;
    R32_UH_TX_DMA = (UINT32)(UINT8 *)endpTXbuff;
    R32_UH_RX_DMA = (UINT32)(UINT8 *)endpRXbuff;
    status = U20USBHostTransact( USB_PID_IN << 4 | hs_hub_info[depth].rootEndp[0].num, hs_hub_info[depth].rootEndp[0].tog, 1 );
    if( status == 0x14 ){
        len = R16_USB_RX_LEN;
        p = (UINT8 *)endpRXbuff;
        memcpy( pbuf,p,len );
        hs_hub_info[depth].rootEndp[0].tog^=0x01;
    }
    return status;
}

/*******************************************************************************
* Function Name  : U20HOST_ClearPortFeatrue
* Description    : Clear Port Properties
* Input          :
                   port
                   port_status
                   NULL
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HOST_ClearPortFeatrue( UINT8 port ,UINT8 port_status )
{
    UINT8  status;
    UINT8 setup_buf[8];
    setup_buf[0] = 0x23;
    setup_buf[1] = 0x01;
    setup_buf[2]  = port_status;
    setup_buf[3]  = 0x00;
    setup_buf[4]  = port;
    setup_buf[5]  = 0x00;
    setup_buf[6] = 0x00;
    setup_buf[7] = 0x00;
    status = USB20HOST_CtrlTransfer( setup_buf,NULL,NULL );
    if( status == USB_INT_SUCCESS ){
    }
    return status;
}

/*******************************************************************************
* Function Name  : U20HOST_ClearPortFeatrue_Process
* Description    :
* Input          :
                   port
                   NULL
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HOST_ClearPortFeatrue_Process( UINT8 depth,UINT8 port )
{
    UINT8 status;
    if( hs_hub_info[depth].portD[port-1].portpchangefield&0x01 ){
        status = U20HOST_ClearPortFeatrue( port,C_PORT_CONNECTION );
        if( status == USB_INT_SUCCESS )return status;
    }
    if( hs_hub_info[depth].portD[port-1].portpchangefield&0x02 ){
        status = U20HOST_ClearPortFeatrue( port,C_PORT_ENABLE );
        if( status == USB_INT_SUCCESS )return status;
    }
    if( hs_hub_info[depth].portD[port-1].portpchangefield&0x04 ){
        status = U20HOST_ClearPortFeatrue( port,C_PORT_SUSPEND );
        if( status == USB_INT_SUCCESS )return status;
    }
    if( hs_hub_info[depth].portD[port-1].portpchangefield&0x08 ){
        status = U20HOST_ClearPortFeatrue( port,C_PORT_OVER_CURRENT );
        if( status == USB_INT_SUCCESS )return status;
    }
    if( hs_hub_info[depth].portD[port-1].portpchangefield&0x10 ){
        status = U20HOST_ClearPortFeatrue( port,C_PORT_RESET );
        if( status == USB_INT_SUCCESS )return status;
    }
}

/*******************************************************************************
* Function Name  : SSPLITPacket
* Description    : Start separation processing
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 SSPLITPacket( PHUB_Port_Info portn, UINT8 *pbuf, UINT16 *plen, UINT8 endp_pid, UINT8 tog ,UINT8 hubaddr)
{
    uint32_t  i;
    uint16_t   r, type;

    if((endp_pid&0x0f)) type = portn->portEndp[gEndp_Num].endptype;//portn->portEndp[gEndp_Num].EndpType;
    else  type = 0;

    R16_UH_SPLIT_DATA =  (type<<10)|(portn->port_num<<1)|( portn->speed ? 0x100 : 0 );
    R8_USB_DEV_AD = hubaddr;//HubInfo.DevAddr;

    R8_USB_INT_FG = RB_USB_IF_TRANSFER;

    for ( i = 500000; i != 0; i -- ){
        r = ( R16_USB_FRAME_NO>>13 );
        if( r == 3 )    break;
    }
    if ( i == 0 )    return( ERR_USB_TRANSFER );

    R16_UH_EP_PID = (USB_PID_SPLIT<<4)|(endp_pid&0x0F);      // SSPLIT
    for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_USB_IF_TRANSFER) == 0 ; i -- );//  mDelayuS( 1 );
    R16_UH_EP_PID = 0x00;
    if ( (R8_USB_INT_FG & RB_USB_IF_TRANSFER) == 0 )    return( ERR_USB_TRANSFER );

    if( (endp_pid>>4) != USB_PID_IN ){
        R8_UH_TX_CTRL = (tog<<3);
        memcpy( endpTXbuff,pbuf,*plen );
        R32_UH_TX_DMA = (UINT32)endpTXbuff;

        R16_UH_TX_LEN = *plen;
    }
    else{                                                    // IN
        R8_UH_RX_CTRL = RB_UH_R_NODATA | (tog<<3) ;     //no data
    }
    R8_USB_DEV_AD = portn->addr;//portn->Addr;
    R16_UH_EP_PID = endp_pid;                                // SETUP/OUT/IN
    R8_USB_INT_FG = RB_USB_IF_TRANSFER;
    for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_USB_IF_TRANSFER) == 0 ; i -- )  mDelayuS( 1 );
    R16_UH_EP_PID = 0x00;
    if ( R8_USB_INT_FG & RB_USB_IF_TRANSFER ){
        r = R8_USB_INT_ST & MASK_UIS_H_RES;
        if ( r == USB_PID_ACK )                    return( ERR_SUCCESS );       // hub no space
        if( type == 3 && r == USB_PID_NULL )       return( ERR_SUCCESS );
    }
    return( ERR_USB_TRANSFER );
}

/*******************************************************************************
* Function Name  : CSPLITPacket
* Description    : none
* Return         : Returns the current command execution status
*******************************************************************************/
uint8_t CSPLITPacket( PHUB_Port_Info portn, uint8_t *pbuf, uint16_t *plen, uint8_t endp_pid, uint8_t tog ,UINT8 hubaddr)
{
    uint32_t  i;
    uint16_t   r, type;
    uint8_t   TransRetry=0;
    if((endp_pid&0x0f)) type = portn->portEndp[gEndp_Num].endptype;//portn->portEndp[gEndp_Num].EndpType;
    else  type = 0;
    do{
        R16_UH_SPLIT_DATA = (type<<10)|(portn->port_num<<1)|( portn->speed ? 0x101 : 0x001 );
        R8_USB_DEV_AD = hubaddr;//HubInfo.DevAddr;

        R8_USB_INT_FG = RB_USB_IF_TRANSFER;
        while(!(R8_USB_MIS_ST & RB_UMS_SPLIT_CAN));
        R16_UH_EP_PID = (USB_PID_SPLIT<<4)|(endp_pid&0x0F);      // SSPLIT
        for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_USB_IF_TRANSFER) == 0 ; i -- ) ;// mDelayuS( 1 );
        R16_UH_EP_PID = 0x00;
        if ( (R8_USB_INT_FG & RB_USB_IF_TRANSFER) == 0 )          return( ERR_USB_TRANSFER );

        if((endp_pid>>4) == USB_PID_IN){
            R8_UH_RX_CTRL = (tog<<3);
            R32_UH_RX_DMA = (UINT32)endpRXbuff;;
        }
        else{
            R8_UH_TX_CTRL = (tog<<3)|RB_UH_T_NODATA;
        }
        R8_USB_DEV_AD = portn->addr;//portn->Addr;
        R16_UH_EP_PID = endp_pid;                                // SETUP/OUT/IN
        R8_USB_INT_FG = RB_USB_IF_TRANSFER;
        for ( i = WAIT_USB_TOUT_200US; i != 0 && (R8_USB_INT_FG & RB_USB_IF_TRANSFER) == 0 ; i -- )  mDelayuS( 1 );
        R16_UH_EP_PID = 0x00;
        if ( R8_USB_INT_FG & RB_USB_IF_TRANSFER ){
            r = R8_USB_INT_ST & MASK_UIS_H_RES;
            if( (R8_USB_INT_ST&RB_UIS_TOG_OK) && ((endp_pid>>4)==USB_PID_IN)){
                *plen = R16_USB_RX_LEN;//R16_USBHS_RX_LEN;
                memcpy( pbuf,endpRXbuff,*plen );
                return( ERR_SUCCESS );
            }
            if ( r == USB_PID_ACK )                    return( ERR_SUCCESS );
            else if ( r == USB_PID_NYET )              ;
            else                                       return( r | ERR_USB_TRANSFER );
        }
        mDelayuS( 20 );
    }while( ++TransRetry<100 );
    return( ERR_USB_TRANSFER );
}

/*******************************************************************************
* Function Name  : HubCtrlTransfer
* Description    : Execute control transmission
* Input          : NULL
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 HubCtrlTransfer( PHUB_Port_Info phub,UINT8 *ReqBuf, uint8_t *pDataBuf, uint16_t *pRetLen ,UINT8 hubaddr)
{
    uint16_t  RemLen, len,i;
    uint8_t   s, ctrltog;
    uint8_t   *pBuf;
    uint16_t  *pLen;
    pBuf = pDataBuf;
    pLen = pRetLen;
    if ( pLen ) *pLen = 0;

    len = 8;
    s = SSPLITPacket(phub, (uint8_t *)ReqBuf, &len, USB_PID_SETUP << 4 | 0x00, 0 ,hubaddr);
    if( s != ERR_SUCCESS )  return s;
    s = CSPLITPacket(phub, NULL, NULL, USB_PID_SETUP << 4 | 0x00, 0,hubaddr);
    if( s != ERR_SUCCESS )  return s;
    RemLen = ((PUSB_SETUP_REQ)ReqBuf) -> wLength;
    ctrltog = 1;
    if ( RemLen && pDataBuf ){
        if ( ((PUSB_SETUP_REQ)ReqBuf)-> bRequestType & USB_REQ_TYP_IN ){
            while ( RemLen ){
                mDelayuS( 100 );
                s = SSPLITPacket(phub, pBuf, &len, USB_PID_IN << 4 | 0x00, ctrltog ,hubaddr);
                if( s != ERR_SUCCESS )  {printf("12=%02x\n",s);return s;}
                s = CSPLITPacket(phub, pBuf, &len, USB_PID_IN << 4 | 0x00, ctrltog ,hubaddr);
                if( s == HUB_CS_NAK )continue;
                if( s != ERR_SUCCESS )  {printf("bbb=%02x,%02x\n",len,s);return s;}
                ctrltog ^= 1;
                RemLen -= len;
                pBuf += len;
                if ( pLen ) *pLen += len;
                if ( (len == 0) || ( len & ( UsbDevEndp0Size - 1 ) ) ){
                    break;
                }
            }
            ctrltog = 0;
        }
        else{
            while ( RemLen ){
                len = RemLen >= UsbDevEndp0Size ? UsbDevEndp0Size : RemLen;
                s = SSPLITPacket(phub, pBuf, &len, USB_PID_OUT << 4 | 0x00, ctrltog ,hubaddr);
                if( s != ERR_SUCCESS )  return s;
                s = CSPLITPacket(phub, pBuf, &len, USB_PID_OUT << 4 | 0x00, ctrltog ,hubaddr);
                if( s == HUB_CS_NAK )continue;
                if( s != ERR_SUCCESS )  return s;
                ctrltog ^= 1;
                RemLen -= len;
                pBuf += len;
                if ( pLen ) *pLen += len;
            }
            ctrltog = 1;
        }
    }
    mDelayuS( 200 );
    len = 0;
    for( i=0;i!=100;i++ ){
        s = SSPLITPacket(phub, pBuf, &len, (ctrltog ? USB_PID_IN : USB_PID_OUT) << 4 | 0x00, 1 ,hubaddr);
        if( s != ERR_SUCCESS )  return s;
        s = CSPLITPacket(phub, pBuf, &len, (ctrltog ? USB_PID_IN : USB_PID_OUT) << 4 | 0x00, 1 ,hubaddr);
        if( s == HUB_CS_NAK )continue;
        if( s != ERR_SUCCESS )  return s;
        if( len != 0 )    return( ERR_USB_BUF_OVER );
        return( ERR_SUCCESS );
    }
}

/*******************************************************************************
* Function Name  : U20HubUSBGetDevDescr
* Description    : Get the device descriptor under the full low-speed device
* Input          : None
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HubUSBGetDevDescr( PHUB_Port_Info phub,UINT8 *Databuf,UINT8 *len ,UINT8 hubaddr)
{
    UINT16 l;
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

    status = HubCtrlTransfer(  phub,setup_buf,Databuf,&l, hubaddr );
    if( status == ERR_SUCCESS ){
        for( l=0;l!=*len;l++ ){
            printf("%02x ",*(Databuf+l));
        }
        printf("\n");
    }
    return status;
}

/*******************************************************************************
* Function Name  : U20HubUSBSetAddr
* Description    : Get the setting address of the low-speed device under HUB
* Input          : None
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HubUSBSetAddr( PHUB_Port_Info phub,UINT8 addr ,UINT8 hubaddr)
{
    UINT16 l;
    UINT8  status;
    UINT8 setup_buf[8];

    setup_buf[0] = 0x00;
    setup_buf[1] = 0x05;
    setup_buf[2] = addr;
    setup_buf[3] = 0x00;
    setup_buf[4] = 0x00;
    setup_buf[5] = 0x00;
    setup_buf[6] = 0x00;
    setup_buf[7] = 0x00;

    status = HubCtrlTransfer( phub,setup_buf,NULL,&l , hubaddr);
    if( status == ERR_SUCCESS )
    {
        R8_USB_DEV_AD = addr;
        phub->addr = addr;
    }
    return status;
}

/*******************************************************************************
* Function Name  : U20HubUSBGetConfigDescr
* Description    : Get the device descriptor under the full low-speed device
* Input          : None
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HubUSBGetConfigDescr( PHUB_Port_Info phub,UINT8 *Databuf,UINT8 *len ,UINT8 hubaddr)
{
    UINT16 l;
    UINT8  status;
    UINT8 setup_buf[8];
    l = *len;

    setup_buf[0] = 0x80;
    setup_buf[1] = 0x06;
    setup_buf[2]  = 0x00;
    setup_buf[3]  = 0x02;
    setup_buf[4]  = 0x00;
    setup_buf[5]  = 0x00;
    setup_buf[6]  = (UINT8)l;
    setup_buf[7]  = 0x00;

    status = HubCtrlTransfer(  phub,setup_buf,Databuf,&l , hubaddr);
    if( status == ERR_SUCCESS )
    {
       setup_buf[6] = (( (PUSB_CFG_DESCR)Databuf ) -> wTotalLength)&0xff;
       setup_buf[7] = (( (PUSB_CFG_DESCR)Databuf ) -> wTotalLength)>>8;
       l = setup_buf[6];
        status = HubCtrlTransfer(  phub,setup_buf,Databuf,&l , hubaddr);
        if( status == ERR_SUCCESS )
        {
            *len = l;
            for( l=0;l!=*len;l++ )
            {
                printf("%02x ",*(Databuf+l));
            }
            printf("\n");
        }
        else
        {
            *len = 0;
        }
    }

    thisUsbDev.DeviceCongValue = ( (PUSB_CFG_DESCR)Databuf )-> bConfigurationValue;
    printf("thisUsbDev.DeviceType=%02x\n",thisUsbDev.DeviceType);
    if( thisUsbDev.DeviceType == 0x09 )
    {        //HUB
        HubAnalysis_Descr( phub,(UINT8 *)Databuf, pSetupReq->wLength );
    }
    else
    {       //Other device types
        USBHS_Analysis_Descr( &thisUsbDev, (UINT8 *)Databuf, pSetupReq->wLength );
    }

    return status;
}

/*******************************************************************************
* Function Name  : U20HubUSBSetConfig
* Description    : USB20HubUSBSetConfig
* Input          : None
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U20HubUSBSetConfig( PHUB_Port_Info phub, UINT8 cfg ,UINT8 hubaddr)
{
    UINT8  status;
    UINT8 setup_buf[8];

    setup_buf[0] = 0x00;
    setup_buf[1] = 0x09;
    setup_buf[2]  = cfg;
    setup_buf[3]  = 0x00;
    setup_buf[4]  = 0x00;
    setup_buf[5]  = 0x00;
    setup_buf[6]  = 0x00;
    setup_buf[7]  = 0x00;
    status = HubCtrlTransfer( phub,setup_buf,NULL,NULL, hubaddr);
    if( status == ERR_SUCCESS ){
    }
    return status;

}

/*******************************************************************************
* Function Name  : U20HOST_DevSetAddress
* Description    : Set the USB device address of the current operation of the USB host
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void U20HOST_DevSetAddress( UINT8 addr )
{
    R8_USB_DEV_AD = addr;
}

/*******************************************************************************
* Function Name  : U20HOST_Enumerate
* Description    : USB2.0 enumeration function
* Input          : *pbuf---Descriptor buffer
* Output         : None
* Return         : device status
*******************************************************************************/
UINT8 U20HOST_Enumerate(UINT8 depth, UINT8 *pbuf,UINT8 addr, UINT8 port)
{
    UINT8 status;
    UINT8 cfg,i;
    UINT32 timeout;
    UINT16 len;
    if( port == 0 )
    {
        SetBusReset();
        timeout = 0;
        while(1)
        {
            R8_USB_INT_FG = RB_USB_IF_DETECT;
            if( R8_USB_MIS_ST&RB_USB_ATTACH )
            {
                printf("USB2.0 DEVICE ATTACH1111 !\n");
                break;
            }
            else
            {
                return USB_INT_DISCONNECT;
            }
            timeout++;
            if( timeout>=0x10000 )return USB_CH56XUSBTIMEOUT;
        }
    }
    len = 7;
    status = U20HOST_GetDeviceDescr( pbuf,&len );
    if( status != USB_INT_SUCCESS ){
        printf("GetDev_ERROR=%02x\n",status);
        return USB_OPERATE_ERROR;
    }
    UsbDevEndp0Size = *(pbuf+4);

    status = U20HOST_SetAddress( addr );
    if( status != USB_INT_SUCCESS ){
        printf("SetAddr_ERROR=%02x\n",status);
        return USB_OPERATE_ERROR;
    }
    len = 0x12;
    status = U20HOST_GetDeviceDescr( pbuf,&len );
    if( status != USB_INT_SUCCESS ){
        printf("GetDev_ERROR=%02x\n",status);
        return USB_OPERATE_ERROR;
    }
    status = U20HOST_GetConfigDescr( &hs_hub_info[depth].portD[port],pbuf,&len );
    if( status != USB_INT_SUCCESS )
    {
        printf("GetDev_ERROR=%02x\n",status);
        return USB_OPERATE_ERROR;
    }
    cfg = ( (PUSB_CFG_DESCR)pbuf ) -> bConfigurationValue;
    printf("cfg=%02x\n",cfg);
    printf("ConfigDescr:");
    for( i=0;i!=len;i++ ){
        printf("%02x ",*(pbuf+i));
    }
    printf("\n");
    status=U20HOST_CofDescrAnalyse(depth,pbuf,port);
    if( status != USB_INT_SUCCESS )
    {
        printf( "Current Device is Not USB Mass Storage Device\n" );
        return( USB_OPERATE_ERROR );
    }

    status = U20HOST_SetConfig( cfg );
    if ( status != USB_INT_SUCCESS )
    {
        printf( "SetConfig_ERROR = %02X\n", (UINT16)status );
        return( USB_OPERATE_ERROR );
    }
    if( (hs_hub_info[depth].device_type == 0x09)  && (port == 0))
    {

        printf("GetHUBDevDescr\n");
        status = U20HOST_GetHUBDevDescr( pbuf,&len );
        if ( status != USB_INT_SUCCESS ){
            printf( "GetHUBDevDescr_ERROR = %02X\n", (UINT16)status );
            return( USB_OPERATE_ERROR );
        }
        hs_hub_info[depth].rootnumofport = *(pbuf+2);

        if( hs_hub_info[depth].rootnumofport>=MAX_HUBNUMPORT  )hs_hub_info[depth].rootnumofport = MAX_HUBNUMPORT;
        printf("status=%02x,%d\n",status,len);
        for( i=0;i!=len;i++ ){
            printf("%02x ",*(pbuf+i));
        }
        printf("\n");


        for( i=0;i!=hs_hub_info[depth].rootnumofport;i++ )
        {
            status = U20HOST_SetPortFeatrue(i+1,PORT_POWER);
            if ( status != USB_INT_SUCCESS )
            {
                printf( "SetPortFeatrue_ERROR = %02X\n", (UINT16)status );
                return( USB_OPERATE_ERROR );
            }
        }
    }
    return status;
}

/*******************************************************************************
* Function Name  : USBHS_HUBCheckPortConnect
* Description    : Detect HUB port status
* Input          : depth - hub depth
*                  port  - hub port
* Output         : None
* Return         : hub port status
*******************************************************************************/
UINT8 USBHS_HUBCheckPortConnect( UINT8 depth,UINT8 port )
{
    UINT8 ret;
    ret = U20HOST_GetPortStstus( depth,port+1 );
    if( ret != ERR_SUCCESS )return ret;
    /* Determine the current port connection status */
    if( endpRXbuff[ 2 ] & 0x01 )                               /* The connection status of this port has changed */
    {
        if( endpRXbuff[ 0 ] & 0x01 )
        {
            if( hs_hub_info[depth].portD[port].status<HUB_ERR_SCUESS ){
                hs_hub_info[depth].portD[port].status = HUB_ERR_CONNECT;
            }
            return( 0x18 );                                    /* This port: Device connection detected */
        }
        else
        {
            hs_hub_info[depth].portD[port].status = HUB_ERR_DISCONNECT;
            hs_hub_info[depth].portD[port].status = 0;
            hs_hub_info[depth].portD[port].addr = 0;
            hs_hub_info[depth].portD[port].speed = 0;
            hs_hub_info[depth].portD[port].devicetype = 0;
            return( 0x19 );                                    /* This port: Device disconnection detected */
        }
    }
    else                                                       /* The connection status of this port has not changed */
    {
        if( endpRXbuff[ 0 ] & 0x01 )
        {
            if( hs_hub_info[depth].portD[port].status<HUB_ERR_SCUESS ){
                hs_hub_info[depth].portD[port].status = HUB_ERR_CONNECT;
            }
            return( 0x02 );                                                     /* This port: has devices */
        }
        else
        {
            hs_hub_info[depth].portD[port].status = HUB_ERR_DISCONNECT;
            hs_hub_info[depth].portD[port].status = 0;
            hs_hub_info[depth].portD[port].addr = 0;
            hs_hub_info[depth].portD[port].speed = 0;
            hs_hub_info[depth].portD[port].devicetype = 0;
            return( 0x01 );                                                     /* This port: No device */
        }
    }

}

/*******************************************************************************
* Function Name  : USBHS_HUBHostEnum
* Description    : Enumerate devices under HUB
* Input          : depth       - hub depth
*                  *Databuf    - Enumerate Buffers
*                  port        - hub port
*                  uplevelport - The port number of the upper level HUB
*                  hubaddr     - HUB's address
* Return         : Enumerate status
*******************************************************************************/
UINT8 USBHS_HUBHostEnum( UINT8 depth, UINT8 *Databuf,UINT8 port ,UINT8 uplevelport,UINT8 hubaddr)
{
  UINT8 ret;
  UINT16 s,i;
  UINT16 temp16,len;
  UINT8 cfg;
  UINT8 l;

  ret = U20HOST_GetPortStstus(depth, port+1 );
  if( ret != USB_INT_SUCCESS )
  {
      return ret;
  }

  for (i = 0; i < 100; ++i) {
    mDelayuS(1000);
  }

  ret = U20HOST_SetPortFeatrue(port+1,PORT_RESET);
  if( ret != USB_INT_SUCCESS )
  {
      return ret;
  }
  do
  {
      ret = U20HOST_GetPortStstus(depth, port+1 );
      if( ret != USB_INT_SUCCESS )
      {
          return( ret );
      }

  }while( endpRXbuff[ 0 ] & C_PORT_CONNECTION );

  ret = U20HOST_ClearPortFeatrue_Process( depth,port+1 );
  if( ret != USB_INT_SUCCESS )
  {
      return ret;
  }
  ret = U20HOST_GetPortStstus( depth,port+1 );
  if( ret != USB_INT_SUCCESS )
  {
      return ret;
  }

  for( ret=0;ret!=4;ret++ ){
      printf("%02x ",endpRXbuff[ret]);
  }
  printf("\n");

  if( ( endpRXbuff[ 0 ] & 0x01 ) == 0x00 )
  {
      return( 0x18 );
  }

  temp16 = *((uint16_t *)&endpRXbuff[0]);

  if( temp16 & 0x01 )//Low speed and full speed
  {
      hs_hub_info[depth].portD[port].port_num = port+1;
      hs_hub_info[depth].portD[port].addr = 0;
      hs_hub_info[depth].portD[port].status = 1;
      hs_hub_info[depth].portD[port].speed = (temp16&(1<<9))?2:((temp16&(1<<10))?1:0);//1--high speed   2--low speed  0--full speed
  }
  else hs_hub_info[depth].portD[port].status = 0;            //high speed
  printf("Speed=%02x,%02x\n",hs_hub_info[depth].portD[port].speed,hs_hub_info[depth].portD[port].status);

  U20HOST_DevSetAddress(0);//set address 0

  if( hs_hub_info[depth].portD[port].speed == 1)//high speed
  {
      for (i = 0; i < 30000; ++i)
      {
        mDelayuS(1);
      }

      len = 7;
      s = U20HOST_GetDeviceDescr( Databuf,&len );
      if( s != USB_INT_SUCCESS ){
          printf("GetDev_ERROR=%02x\n",s);
          return USB_OPERATE_ERROR;
      }
      hs_hub_info[depth].portD[port].addr = USB_SetAddressNumber();

      ret = U20HOST_SetAddress( hs_hub_info[depth].portD[port].addr );
      if(ret != USB_INT_SUCCESS)
      {
          printf("set address:%02x,%d\n",ret,port+5);
          return( ret );
      }
      ret = U20HOST_GetConfigDescr( &hs_hub_info[depth].portD[port],Databuf,NULL );    //High speed also requires data analysis
      if(ret != USB_INT_SUCCESS)
      {
          printf("get configuration descriptor:%02x\n",ret);
          return( ret );
      }

      ret = U20HOST_SetConfig( ( (PUSB_CFG_DESCR)Databuf ) -> bConfigurationValue );
      if( ret != USB_INT_SUCCESS )
      {
          printf("set configuration:%02x\n",ret);
          return( ret );
      }

      if( hs_hub_info[depth].portD[port].devicetype == 0x09 ){         //Initialization of HUB
          ret = U20HOST_GetHUBDevDescr( Databuf,NULL );
          if( ret != USB_INT_SUCCESS )
          {
              printf("get_hub_desc:%02x\n",ret);
              return( ret );
          }


          hs_hub_info[depth].numofport = *(Databuf+2);
          if( hs_hub_info[depth].numofport > MAX_HUBNUMPORT)
          {
              hs_hub_info[depth].numofport = MAX_HUBNUMPORT;
          }

          for( i=0;i!=hs_hub_info[depth].numofport;i++ ){

              ret = U20HOST_SetPortFeatrue( i+1,0x08 );
              printf("hub_power_status=%02x,%d\n",ret,i);
              if( ret != USB_INT_SUCCESS )return ret;
          }
          hs_hub_info[depth+1].devaddr = hs_hub_info[depth].portD[port].addr;   //Root directory address of subordinate HUB

      }
      hs_hub_info[depth].portD[port].status = HUB_ERR_SCUESS;          //Enumeration successful

      R32_UH_TX_DMA = (UINT32)(UINT8 *)endpTXbuff;
      R32_UH_RX_DMA = (UINT32)(UINT8 *)endpRXbuff;

      return( ERR_SUCCESS );
  }
  else  //Low speed and full speed
  {
      printf("full&low_speed_enmu\n");

      l = 0x08;
      UsbDevEndp0Size = 0x08;
      s = U20HubUSBGetDevDescr( &hs_hub_info[depth].portD[port],Databuf,&l ,hubaddr);
      if( s != ERR_SUCCESS ){
          printf("Get_Desrc_Error=%02x\n",s);
          return s;
      }
      UsbDevEndp0Size = *(Databuf+7);

      s = U20HubUSBSetAddr( &hs_hub_info[depth].portD[port],USB_SetAddressNumber(),hubaddr);

      if( s != ERR_SUCCESS )
      {
          printf("Set_Addr_Error\n");
          return s;
      }
      l = 0x12;
      s = U20HubUSBGetDevDescr( &hs_hub_info[depth].portD[port],Databuf,&l ,hubaddr);
      if( s != ERR_SUCCESS ){
          printf("Get_Desrc_Error\n");
      }
      l = 0x09;
      s = U20HubUSBGetConfigDescr( &hs_hub_info[depth].portD[port],Databuf,&l,hubaddr );
      if( s != ERR_SUCCESS ){
          printf("Get_Config_Error\n");
      }
      cfg = ( (PUSB_CFG_DESCR)Databuf ) -> bConfigurationValue;
      printf("cfg=%02x\n",cfg);
      s=U20HOST_CofDescrAnalyse(depth,Databuf,port);
      if( s != USB_INT_SUCCESS )
      {
          printf( "Current Device is Not USB Mass Storage Device\n" );
          return( USB_OPERATE_ERROR );
      }
      s=U20HubUSBSetConfig( &hs_hub_info[depth].portD[port],cfg ,hubaddr);
      if( s != ERR_SUCCESS ){
          printf("Set_Cfg_Error\n");
      }
      printf("fls_hub_info.device_type=%02x\n",hs_hub_info[depth].portD[port-1].devicetype);
      if( hs_hub_info[depth].device_type == 0x03 )//HID
      {
      }
      hs_hub_info[depth].portD[port].status = HUB_ERR_SCUESS;
      R32_UH_TX_DMA = (UINT32)(UINT8 *)endpTXbuff;
      R32_UH_RX_DMA = (UINT32)(UINT8 *)endpRXbuff;

      return( ERR_SUCCESS );
  }
}

/*******************************************************************************
* Function Name  : USBHS_HUB_Main_Process
* Description    : HUB Operations and Recursive Programs
* Input          : depth       - hub depth
*                  port        - hub port
*                  uplevelport - The port number of the upper level HUB
*                  portnum     - Number of ports in HUB
* Return         : Operational state
*******************************************************************************/
UINT8  USBHS_HUB_Main_Process( UINT8 depth ,UINT8 addr ,UINT8 uplevelport,UINT8 portnum)
{
    UINT8 i,ret;
    USB_HUB_SaveData hubdata;

      for( i=0;i!=portnum;i++ )
      {
          R32_UH_TX_DMA = (UINT32)(UINT8 *)endpTXbuff;
          R32_UH_RX_DMA = (UINT32)(UINT8 *)endpRXbuff;

          AssignHubData(&hubdata,depth,uplevelport,i+1,hs_hub_info[depth]);
          ret = SearchHubData(Hub_LinkHead , &hubdata);//Determine whether the current node has saved data

          if( ret == 0 )
          {
              hs_hub_info[depth] = hubdata.HUB_Info;//Replace if there is saved data
          }
          else
          {
              memset( &hs_hub_info[depth].portD[i],0x00,sizeof( hs_hub_info[depth].portD[i] ) );//Clear without saved data to prevent the use of previous HUB data
          }

          U20HOST_DevSetAddress(addr);
          USBHS_HUBCheckPortConnect( depth, i );

          if( hs_hub_info[depth].portD[i].portpchangefield )
          {
              if( (hs_hub_info[depth].portD[i].status == HUB_ERR_CONNECT) ){            //device connect

                  ret = USBHS_HUBHostEnum( depth,Test_Buf,i ,uplevelport,addr);
                  if( ret == ERR_SUCCESS )
                  {
                      hs_hub_info[depth].portD[i].status = HUB_ERR_SCUESS;          //Enumeration successful

                      AssignHubData(&hubdata,depth,uplevelport,i+1,hs_hub_info[depth]);
                      ret = InsertHubData(Hub_LinkHead,hubdata);
                      if( ret == 0 )
                      {
                          printf("#Save: depth-%x,uplevelport-%x,port-%x,Status-%x,hubaddr-%x#\n",depth,uplevelport,i+1,hs_hub_info[depth].portD[i].status,hs_hub_info[depth].portD[i].addr);
                      }

                  }
                  else
                  {
                      hs_hub_info[depth].portD[i].status = HUB_ERR_DISCONNECT;          //Enumeration failed. Re enumerating
                  }
              }
              else if( (hs_hub_info[depth].portD[i].status ) == PORT_DISCONNECT )
              {

                  AssignHubData(&hubdata,depth,uplevelport,i+1,hs_hub_info[depth]);
                  if( hubdata.HUB_Info.portD[i].devicetype == 0x09 )//If it is a HUB, it is necessary to determine whether there are subordinate nodes when deleting, and if so, it needs to be deleted
                  {
                      Hublink_finesubstructures(hubdata);
                  }

                  ret = DeleteHubData(Hub_LinkHead,hubdata);//Delete linked list nodes
                  if( ret == 0 )
                  {
                      printf("#Delete: depth-%x,uplevelport-%x,port-%x#\n",depth,uplevelport,i+1);
                      USB_DelAddressNumber(hs_hub_info[depth].portD[i].addr);
                  }

                  ret = U20HOST_ClearPortFeatrue_Process( depth,i+1);
                  memset( &hs_hub_info[depth].portD[i],0x00,sizeof( hs_hub_info[depth].portD[i] ) );
                  printf("hub_device_disconnect_port=%02x\r\n",i+1);

              }
              else
              {
                  ret = U20HOST_ClearPortFeatrue_Process( depth,i+1 );
                  if( ret != USB_INT_SUCCESS )return ret;
                  ret = U20HOST_GetPortStstus( depth,i+1 );
                  if( ret != USB_INT_SUCCESS )return ret;
              }
          }

          if( hs_hub_info[depth].portD[i].status == HUB_ERR_SCUESS )//Device port operation successful
          {
              if( hs_hub_info[depth].portD[i].devicetype == 0x09 )
              {   //HUB
                  Global_Index++;
                  ret = USBHS_HUB_Main_Process( Global_Index,hs_hub_info[depth].portD[i].addr,i+1,hs_hub_info[depth].numofport);
                  Global_Index--;
                  if( ret != ERR_SUCCESS )return ret;
              }
              else
              {   //Other device types

              }
          }
          mDelaymS(2);

      }
      if( (R8_USB_MIS_ST&RB_USB_ATTACH) == 0){                //The overall device is disconnected and directly jumps away. It is necessary to clear all changes in the HUB
          printf("dis\n");
          for( i=0;i!=portnum;i++ )
          {
              hs_hub_info[depth].portD[i].status = 0;
              hs_hub_info[depth].portD[i].addr = 0;
              hs_hub_info[depth].portD[i].speed = 0;
              hs_hub_info[depth].portD[i].devicetype = 0;
          }
          memset( hs_hub_info,0x00,sizeof( hs_hub_info ) );
          return ERR_USB_DISCON;

      }
      return ERR_SUCCESS;
}

