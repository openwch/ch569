/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb20.h
* Author             : WCH
* Version            : V1.1
* Date               : 2021/11/3
* Description        :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_host_hs.h"

__attribute__ ((aligned(4))) const UINT8  GetDevDescrptor[]={USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof( USB_DEV_DESCR ), 0x00};
__attribute__ ((aligned(4))) const UINT8  GetConfigDescrptor[]= {USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00};
__attribute__ ((aligned(4))) const UINT8  SetAddress[]={USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const UINT8  SetConfig[]={USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
__attribute__ ((aligned(4))) const UINT8  Clear_EndpStall[]={USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


extern __attribute__ ((aligned(16))) UINT8 endpRXbuff[4096] __attribute__((section(".DMADATA"))); //数据接收缓冲区
extern __attribute__ ((aligned(16))) UINT8 endpTXbuff[4096] __attribute__((section(".DMADATA"))); //数据发送缓冲区

/*global define */
#define pSetupReq    ((PUSB_SETUP_REQ)endpTXbuff)

/*global variable */
DEV_INFO_Typedef g_U20DevInfo;
UINT8 U20_Endp0MaxSize = 0;

/*******************************************************************************
 * @fn        user2mem_copy
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
void SetBusReset( void )
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
    R8_UHOST_CTRL = RB_UH_AUTOSOF_EN;     //自动发送SOF包使能
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
void USBHS_Host_Init(FunctionalState sta)
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
 * @fn        USBHS_Transact
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
UINT8 USBHS_Transact( UINT8 endp_pid, UINT8 toggle,UINT32 timeout)
{
    UINT8 TransRetry=0;
    UINT8  r;
    UINT16  i;
    R8_UH_TX_CTRL = R8_UH_RX_CTRL = toggle ;    //表示期望接收/发送
    do
    {
        R16_UH_EP_PID = endp_pid;                  //启动传输
        R8_USB_INT_FG = RB_USB_IF_TRANSFER;       //清除中断标志
        for ( i = 2000; i != 0 && (( R8_USB_INT_FG & RB_USB_IF_TRANSFER ) == 0); i -- );
        R16_UH_EP_PID = 0;
        if ( (R8_USB_INT_FG & RB_USB_IF_TRANSFER) == 0 )
        {
            return( ERR_USB_UNKNOWN );
        }

        if( R8_USB_INT_FG & RB_USB_IF_DETECT )    //插拔中断
        {
            mDelayuS(200);
            R8_USB_INT_FG = RB_USB_IF_DETECT;
            if( R8_USB_MIS_ST & RB_USB_ATTACH )
            {
                if(R8_UHOST_CTRL & RB_UH_AUTOSOF_EN)  return ( ERR_USB_CONNECT );
            }
            else    return ( ERR_USB_DISCON );
        }

        if( R8_USB_INT_FG & RB_USB_IF_TRANSFER )    //传输完成中断
        {
            if ( R8_USB_INT_ST & RB_USB_ST_TOGOK )      {return( ERR_SUCCESS );}
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
                    if ( r ) {return( r | ERR_USB_TRANSFER );}  // 不是超时/出错,意外应答
                    break;

                case USB_PID_IN:
                    if ( r == USB_PID_DATA0 || r == USB_PID_DATA1 ) { } // 不同步则需丢弃后重试
                    else if ( r ) {return( r | ERR_USB_TRANSFER );}  // 不是超时/出错,意外应答
                    break;

                default:
                    return( ERR_USB_UNKNOWN );  // 不可能的情况
                    break;
                }
            }
        }
        else        R8_USB_INT_FG = 0xFF;       //其他中断，不应该发生的情况
        mDelayuS(15);
    }while(++ TransRetry < 3);
    return (ERR_USB_TRANSFER);      //应答超时
}

/*******************************************************************************
 * @fn        USBHS_HostCtrlTransfer
 *
 * @briefI    USB transfer.
 *
 * @param     databuf - receiving or send buffer
 *            len - receiving or send length
 *
 * @return    None
 */
UINT8 USBHS_HostCtrlTransfer(UINT8 *databuf,PUINT16 len)
{
   UINT8   ret;
   UINT8   stageFlag = 0;
   UINT8   tog = 1;
   UINT16  rxlen = 0;
   UINT16  ReLen;
   PUINT16 pLen;
   PUINT8  pBuf;

   pBuf = databuf;
   pLen = len;

   if( pLen )  *pLen = 0;
   DelayUs( 100 );
   R16_UH_TX_LEN = sizeof(USB_SETUP_REQ);
   ret = USBHS_Transact( (USB_PID_SETUP<<4)|ENDP_0, 0, 200000);
   if(ret != ERR_SUCCESS)  return ret;                    //error

   ReLen = pSetupReq->wLength;
   if(ReLen && pBuf)                                    //data stage
   {
      if( (pSetupReq->bRequestType) & USB_REQ_TYP_IN )    //device to host
      {
          while(ReLen)
          {
              DelayUs( 100 );
              R32_UH_RX_DMA = (UINT32)pBuf + *pLen;
              ret = USBHS_Transact( (USB_PID_IN<<4)|ENDP_0, tog<<3 ,200000);
              if(ret != ERR_SUCCESS)              return ret;

              rxlen = ( R16_USB_RX_LEN <= ReLen) ? R16_USB_RX_LEN : ReLen;
              ReLen -= rxlen;
              tog ^=1;
              if(pLen)  *pLen += rxlen;
              if((R16_USB_RX_LEN == 0)||(R16_USB_RX_LEN & (U20_Endp0MaxSize - 1))) break;
          }
          stageFlag = 1;
      }
      else
      {                                             // host to device
         while(ReLen)
         {
              DelayUs( 100 );
              R32_UH_TX_DMA= (UINT32)pBuf + *pLen;
              R16_UH_TX_LEN = (ReLen > U20_Endp0MaxSize)? U20_Endp0MaxSize : ReLen;

              ret = USBHS_Transact( (USB_PID_OUT<<4)|ENDP_0, tog<<3, 200000 );
              if(ret != ERR_SUCCESS)  return  rxlen;
              ReLen -= R16_UH_TX_LEN;
              tog ^=1;
              if( pLen )  *pLen += R16_UH_TX_LEN;
         }
      }
   }
   DelayUs( 100 );
   ret = USBHS_Transact(  (stageFlag ? (USB_PID_OUT<<4): (USB_PID_IN<<4))| ENDP_0,  RB_UEP_R_TOG_1|RB_UEP_T_TOG_1, 20000);    //status stage is in or out

   if(ret != ERR_SUCCESS)                return ret;

   if ( R16_UH_TX_LEN == 0 )         return( ERR_SUCCESS );    //status stage is out, send a zero-length packet.

   if ( R16_USB_RX_LEN == 0 )        return( ERR_SUCCESS );    //status stage is in, a zero-length packet is returned indicating success.

   return ERR_USB_BUF_OVER;
}

/*******************************************************************************
 * @fn        USBHS_Host_Enum
 *
 * @briefI    enumerate device.
 *
 * @param     Databuf - receive buffer
 *
 * @return    ERR_SUCCESS
 *            ERR_USB_TRANSFER
 */
UINT8 USBHS_Host_Enum( UINT8 *Databuf )
{
   UINT8 ret;

   ret = CtrlGetDevDescr( Databuf );
   if(ret != ERR_SUCCESS)
   {
      printf("get device descriptor error:%02x\n",ret);
      return  ret;
   }

   ret = CtrlSetAddress( 8 );
   if(ret != ERR_SUCCESS)
   {
       printf("set address error\n",ret);
      return  ret;
   }

   ret = CtrlGetConfigDescr( Databuf );
   if(ret != ERR_SUCCESS)
   {
       printf("get configuration descriptor error\n",ret);
      return  ret;
   }

   ret = CtrlSetConfig( g_U20DevInfo.DeviceCongValue );
   if(ret != ERR_SUCCESS)
   {
       printf("set configuration error\n",ret);
       return  ret;
   }

   return ERR_SUCCESS;
}

/*******************************************************************************
 * @fn        CtrlGetDevDescr
 *
 * @briefI    get device descriptor.
 *
 * @param     pdev - pointer
 *            usrbuf - buffer address
 *
 * @return    ERR_SUCCESS
 *            ERR_USB_BUF_OVER
 *            ERR_USB_TRANSFER
 */
UINT8 CtrlGetDevDescr(UINT8 *databuf)
{
    UINT8 ret;
    UINT16 len;
    U20_Endp0MaxSize = DEFAULT_ENDP0_SIZE ;
    CopySetupReqPkg( GetDevDescrptor );
    pSetupReq->wLength = DEFAULT_ENDP0_SIZE;

    ret = USBHS_HostCtrlTransfer(databuf,&len);
    if(ret != ERR_SUCCESS)         return  ret;
    if(len < DEFAULT_ENDP0_SIZE )  return  ERR_USB_BUF_OVER;
    U20_Endp0MaxSize = ((PUSB_DEV_DESCR)databuf)->bMaxPacketSize0;

    CopySetupReqPkg( GetDevDescrptor );
    pSetupReq->wLength = sizeof(USB_DEV_DESCR);
    ret = USBHS_HostCtrlTransfer(databuf,&len);
    return  ret;
}

/*******************************************************************************
 * @fn        CtrlGetConfigDescr
 *
 * @briefI    get configuration descriptor.
 *
 * @param     pdev - pointer
 *            usrbuf - buffer address
 *
 * @return    ERR_SUCCESS
 *            ERR_USB_BUF_OVER
 *            ERR_USB_TRANSFER
 */
UINT8 CtrlGetConfigDescr(UINT8 *databuf)
{
    UINT8 ret;
    UINT16 reallen,len;

    CopySetupReqPkg( GetConfigDescrptor );
    ret = USBHS_HostCtrlTransfer( databuf, &len );                //获取配置描述符
    if(ret != ERR_SUCCESS)         return  ERR_USB_TRANSFER;
    if(len < pSetupReq->wLength )  return  ERR_USB_BUF_OVER;

    reallen = ((PUSB_CFG_DESCR)databuf)->wTotalLength;             //解析全部配置描述符的长度

    CopySetupReqPkg( GetConfigDescrptor );
    pSetupReq->wLength = reallen;
    ret = USBHS_HostCtrlTransfer( databuf, &len );                 //获取全部配置描述符
    if(ret != ERR_SUCCESS)        return  ERR_USB_TRANSFER;
    if(len < pSetupReq->wLength)  return  ERR_USB_BUF_OVER;
    g_U20DevInfo.DeviceCongValue = ((PUSB_CFG_DESCR)databuf)->bConfigurationValue;

    USBHS_Analysis_Descr( &g_U20DevInfo, databuf, pSetupReq->wLength );
    return  ret;
}

/*******************************************************************************
 * @fn        CtrlSetAddress
 *
 * @briefI    set device address.
 *
 * @param     addr - address of device
 *
 * @return    ERR_SUCCESS
 *            ERR_USB_TRANSFER
 */
UINT8 CtrlSetAddress( UINT8 addr )
{
    UINT32 rxlen;

    CopySetupReqPkg( SetAddress );
    pSetupReq->wValue = addr;
    rxlen = USBHS_HostCtrlTransfer(NULL,NULL);
    if(rxlen != ERR_SUCCESS)  return  ERR_USB_TRANSFER;

    USBHS_CurrentAddr(addr);
    return  ERR_SUCCESS;
}

/*******************************************************************************
 * @fn        CtrlSetConfig
 *
 * @briefI    set device configuration value.
 *
 * @param     cfg_val - value of  configuration
 *
 * @return    ERR_SUCCESS
 *            ERR_USB_TRANSFER
 */
UINT8 CtrlSetConfig(UINT8 cfg_val)
{
    UINT8 ret;

    CopySetupReqPkg( SetConfig );
    pSetupReq->wValue = cfg_val;
    ret = USBHS_HostCtrlTransfer(NULL,NULL);
    if(ret != ERR_SUCCESS)  return  ERR_USB_TRANSFER;

    return  ERR_SUCCESS;
}

/*******************************************************************************
 * @fn        CtrlSetAddress
 *
 * @briefI    set current target address of host.
 *
 * @param     addr - target address
 *
 * @return    None
 */
void USBHS_CurrentAddr( UINT8 addr )
{
    R8_USB_DEV_AD = addr;           // SET ADDRESS
}

/*******************************************************************************
 * @fn        Anaylisys_Descr
 *
 * @briefI    descriptor analysis.
 *
 * @param     pusbdev - device information variable.
 *            pdesc - descriptor buffer to analyze
 *            l - length
 *
 * @return    None
 */
void USBHS_Analysis_Descr( pDEV_INFO_Typedef pusbdev,PUINT8 pdesc, UINT16 l)
{
    UINT16 i,EndPMaxSize;
    for(i=0;i<l;i++)                                                //分析描述符
    {
     if((pdesc[i]==0x09)&&(pdesc[i+1]==0x02))
        {
            printf("bNumInterfaces:%02x \n",pdesc[i+4]);            //配置描述符里的接口数-第5个字节
        }

     if((pdesc[i]==0x07)&&(pdesc[i+1]==0x05))
        {
            if((pdesc[i+2])&0x80)
            {
                 printf("endpIN:%02x \n",pdesc[i+2]&0x0f);              //取in端点号
                 pusbdev->DevEndp.InEndpNum = pdesc[i+2]&0x0f;
                 pusbdev->DevEndp.InEndpCount++;
                 EndPMaxSize = ((UINT16)pdesc[i+5]<<8)|pdesc[i+4];     //取端点大小
                 pusbdev->DevEndp.InEndpMaxSize = EndPMaxSize;
                 printf("In_endpmaxsize:%02x \n",EndPMaxSize);
            }
            else
            {
                printf("endpOUT:%02x \n",pdesc[i+2]&0x0f);              //取out端点号
                pusbdev->DevEndp.OutEndpNum = pdesc[i+2]&0x0f;
                pusbdev->DevEndp.OutEndpCount++;
                EndPMaxSize =((UINT16)pdesc[i+5]<<8)|pdesc[i+4];        //取端点大小
                pusbdev->DevEndp.OutEndpMaxSize = EndPMaxSize;
                printf("Out_endpmaxsize:%02x \n",EndPMaxSize);
            }
        }
    }
}
