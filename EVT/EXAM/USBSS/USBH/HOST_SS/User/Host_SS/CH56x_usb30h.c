/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30h.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56xusb30h_LIB.h"
#include "CH56x_usb30h.h"

/* Function */
void LINK_IRQHandler( void ) __attribute__((interrupt("WCH-Interrupt-fast")));

__attribute__ ((aligned(16))) UINT8 endpRXbuff[4096] __attribute__((section(".DMADATA"))); //data recive buffer
__attribute__ ((aligned(16))) UINT8 endpTXbuff[4096] __attribute__((section(".DMADATA"))); //data send buffer

/* Global define */
#define pSetupreq    ((PUSB_SETUP_REQ)endpTXbuff)

/* Global Variable */
UINT8V tx_lmp_port = 0;
UINT8V device_link_status = 0;
UINT8V config_value = 0;
UINT8V g_DeviceConnectstatus = 0;
UINT8V g_DeviceUsbType = 0;
extern volatile DevInfo g_DevInfo;

const uint8_t get_dev_descriptor[] =
{
    0x80,0x06,0x00,0x01,0x00,0x00,0x40,0x00
};

const uint8_t get_cfg_descriptor[] =
{
    0x80,0x06,0x00,0x02,0x00,0x00,0x09,0x00
};

const uint8_t get_cfg_descriptor_all[] =
{
    0x80,0x06,0x00,0x02,0x00,0x00,0xff,0x00
};

const uint8_t get_bos_descriptor[] =
{
    0x80,0x06,0x00,0x0f,0x00,0x00,0xff,0x00
};


const uint8_t get_string_descriptor0[] =
{
    0x80,0x06,0x00,0x03,0x00,0x00,0xff,0x00
};

const uint8_t get_string_descriptor1[] =
{
    0x80,0x06,0x01,0x03,0x09,0x04,0xff,0x00
};

const uint8_t get_string_descriptor2[] =
{
    0x80,0x06,0x02,0x03,0x09,0x04,0xff,0x00
};

const uint8_t get_string_descriptor3[] =
{
    0x80,0x06,0x03,0x03,0x09,0x04,0xff,0x00
};

const uint8_t get_interface[] =
{
    0x81,0x0a,0x00,0x00,0x00,0x00,0x09,0x00
};
const uint8_t set_configuration[] =
{
    0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00
};

const uint8_t set_address[] =
{
    0x00,0x05,0x08,0x00,0x00,0x00,0x00,0x00
};

const uint8_t set_isoch_delay[] =
{
    0x00,0x31,0x88,0x00,0x00,0x00,0x00,0x00
};

const uint8_t set_sel[] =
{
    0x00,0x30,0x00,0x00, 0x00,0x00,0x06,0x00
};

const uint8_t tx_sel_data[] =
{
    0x5f,0x0a,0x54,0x08,0xff,0x07
};

const uint8_t set_feature_U1[] =
{
    0x00,0x03,0x30,0x00,0x00,0x00,0x00,0x00
};

const uint8_t set_feature_U2[] =
{
    0x00,0x03,0x31,0x00,0x00,0x00,0x00, 0x00
};

/*******************************************************************************
 * @fn        USB30HOST_INTransaction
 *
 * @brief     bulk Transmit IN transaction
 *
 * @param     seq_num -         The host is ready to receive the packet sequence of the first packet
 *            recv_packnum -    The number of packets the host is ready to receive
 *            endp_num -        endpoint number
 *
 * @return    None
 */
UINT16 USB30HOST_INTransaction(UINT8 seq_num,UINT8 *recv_packnum ,UINT8 endp_num,UINT16 *status)
{
    UINT8  packnum = 0;
    UINT8  nump = 0;
    UINT16 len;
    UINT16 num =0;
    UINT32 i = 0;

    num = USB30H_IN_Data(seq_num, recv_packnum, endp_num );
    mDelayuS(200);
    switch( num & 0x7000 )
    {
        case 0x1000:                                                           //ACK
            break;
        default:                                                               //NRDY
            i=0;
            while( !USB30H_Erdy_Status( &packnum, &endp_num  ) )               //wati ERDY flag
            {
                i++;
                if(i==10000000)
                {
                    i=0;
                    num = USB30H_IN_Data(seq_num, recv_packnum, endp_num);    //retry
                    if( (num & 0x7000) == 0x1000 )       break;               //Retry succeeded
                }
            }
            if(packnum > 1){                                                  //Only one package
                packnum = 1;
            }
            if( (num & 0x7000) == 0x2000)
            {
                nump = packnum;
                num = USB30H_IN_Data(seq_num,&packnum, endp_num );
                *recv_packnum -= nump;
            }
            break;
    }
    *status = num ;
    len = num & 0x0fff;
    return len;
}

/*******************************************************************************
 * @fn        USB30HOST_OUTTransaction
 *
 * @brief     bulk Transmit OUT transaction
 *
 * @param     seq_num:              The packet sequence of the first packet of the distributed packet
 *            recv_packnum:         The number of packets that the host is ready to send
 *            endp_num:             endpoint number
 *            txlen:                The packet length of the last packet of data
 *
 * @return    Number of unsent packets remaining
 */
UINT8 USB30HOST_OUTTransaction(UINT8 seq_num,UINT8 send_packnum ,UINT8 endp_num,UINT32 txlen)
{
    UINT8  sta;
    UINT8  packnump;
    UINT32 i;

    sta = USB30H_OUT_Data(seq_num, send_packnum, endp_num, txlen);
    switch(sta&0x07)
    {
        case 1:                                                                        //ACK
            send_packnum = 0;
            break;
        default:                                                                       //NRDY
            i=0;
            while( !USB30H_Erdy_Status(  &packnump, &endp_num  ) )                     //wait ERDY flag
            {
                i++;
                if(i==2000000)  //ERDY timeout
                {
                    i=0;
                    sta = USB30H_OUT_Data( seq_num, send_packnum, endp_num, txlen);   //Retry
                    if( (sta&0x07) == 1 )                                             //Retry succeeded
                    {
                        send_packnum = 0;
                        break;
                    }
                }
            }
            if(packnump > 1){                                                        //Only one package
                packnump = 1;
            }
            if((sta&0x07) == 2)
            {
                sta = USB30H_OUT_Data( seq_num, packnump, endp_num, txlen);
                send_packnum -= packnump ;
            }
            break;
    }

    return send_packnum;                                                             //Remaining number of packages not issued
}


/*******************************************************************************
 * @fn        USB30HOST_CtrlTransaciton
 *
 * @brief     Control transmission
 *
 * @param     databuf - receiving or send buffer
 *
 * @return    0x7000 -  The error should not occur
 *            bit0~11 - The data stage is device-to-host, and the data length is returned by the device.
 *                      The data stage is host-to-device, and the status value is returned by the device
 */
UINT16 USB30HOST_CtrlTransaciton(UINT8 *databuf)
{

    UINT8   *pBuf;
    UINT8   seq_num =0;
    UINT8   req_nump = 1;
    UINT16  ReLen;
    UINT16  txlen=0;
    UINT16  len = 0;
    UINT16  trans_len = 0;
    UINT32  i=0;

    pBuf = databuf;
    ReLen = pSetupreq->wLength;

    if(pSetupreq->bRequestType & USB_REQ_TYP_IN)                         //Data stage device-to-host
    {
      while(ReLen)
      {
          USBSSH->UH_RX_DMA = (UINT32)pBuf + trans_len ;
          len = USB30H_IN_Data( seq_num, &req_nump, 0 );                 // req_nump=1, endp=0
          mDelayuS(200);
          switch( len & 0x7000 )
          {
              case 0x1000:                                               //ACK
                  break;
              default:                                                   //NRDY or stall
                  i=0;
                  while( !USB30H_Erdy_Status( NULL, NULL) )
                  {
                      i++;
                      if(i==10000000)
                      {
                        printf("IN ERDY timeout...\n");
                        i=0;
                        len = USB30H_IN_Data(seq_num, &req_nump, 0 );
                        if( (len & 0x7000) == 0x1000 )    break;
                        else
                        {
                            break;
                        }
                      }
                  }
                  if( (len & 0x7000) == 0x2000)
                  {
                    len = USB30H_IN_Data(seq_num,&req_nump, 0 );
                  }
                  break;
          }
         len &= 0x0fff;
         len = len > ReLen ? ReLen :len;
         trans_len += len;
         ReLen -= len;
         if(  (len == 0) || ( len & (USBSS_ENDP0_MAXSIZE -1) ) )  break;
         seq_num++;
      }
      len = trans_len;
    }
    else
    {
      while(ReLen)
      {
         USBSSH->UH_TX_DMA = (UINT32)pBuf  + trans_len;
         txlen = (ReLen > USBSS_ENDP0_MAXSIZE) ? USBSS_ENDP0_MAXSIZE : ReLen;
         len = USB30H_OUT_Data( seq_num, req_nump, 0 ,txlen);      // nump =1, endp=0
         switch(len & 0x07)
         {
             case 1:                                               //ACK
                 break;
             default:                                              //NRDY
                 i=0;
                 while( !USB30H_Erdy_Status(  NULL, NULL  ) )
                 {
                     i++;
                     if(i==1000000)
                     {
                         printf("OUT ERDY timeout...\n");
                         i=0;
                         len = USB30H_OUT_Data(seq_num,req_nump,0,txlen);
                         if( (len & 0x07) == 1 )
                         {
                             break;
                         }
                         else
                         {
                             break;
                         }
                     }
                 }
                 if((len &0x07) == 2)
                 {
                     len = USB30H_OUT_Data(seq_num,req_nump,0,txlen);
                 }
                 break;
         }
         trans_len += txlen;
         ReLen -= txlen;
         seq_num++;
      }
    }
    return len;

}

/*******************************************************************************
 * @fn        GetDEV_Descrptor
 *
 * @brief     GetDEV_Descrptor
 *
 * @return    len
 */
UINT16 GetDEV_Descriptor(void)
{
  UINT16 len;
  memcpy( endpTXbuff , get_dev_descriptor , 8);
  USB30H_Send_Setup( 8 );
  len = USB30HOST_CtrlTransaciton(endpRXbuff);
  USB30H_Send_Status();
  return len;
}

/*******************************************************************************
 * @fn        GetConfig_Descrptor
 *
 * @brief     GetConfig_Descrptor
 *
 * @return    len
 */
UINT16 GetConfig_Descriptor(void)
{
  UINT16 reallen;
  UINT16 len;

  memcpy( endpTXbuff , get_cfg_descriptor , 8);
  USB30H_Send_Setup( 8 );
  len = USB30HOST_CtrlTransaciton(endpRXbuff);
  USB30H_Send_Status();
  reallen = ((PUSB_CFG_DESCR)endpRXbuff)->wTotalLength ;             //Gets the total length of the configuration descriptor
  config_value = ((PUSB_CFG_DESCR)endpRXbuff)->bConfigurationValue;  //Get device configuration values

  memcpy( endpTXbuff , get_cfg_descriptor , 8);                      //Get all configuration descriptors
  pSetupreq->wLength = reallen;
  USB30H_Send_Setup( 8 );
  len = USB30HOST_CtrlTransaciton(endpRXbuff);
  USB30H_Send_Status();
  if(len < pSetupreq->wLength)   printf("error\n");
  Analysis_Descr(endpRXbuff,len);
  return len;
}

/*******************************************************************************
 * @fn        Set_Address
 *
 * @brief     Set device address
 *
 * @param     addr - device address
 *
 * @return    None
 */
void Set_Address(UINT8 addr)
{
  memcpy( endpTXbuff , set_address , 8);
  pSetupreq->wValue = addr;
  USB30H_Send_Setup( 8 );
  USB30H_Send_Status();
  USB30H_Set_Address(addr);
}

/*******************************************************************************
 * @fn        Set_IsochDelay
 *
 * @brief     Set isoch_delay
 *
 * @return    None
 */
void Set_IsochDelay(void)
{
  memcpy( endpTXbuff , set_isoch_delay , 8);
  USB30H_Send_Setup( 8 );
  USB30H_Send_Status();
}

/*******************************************************************************
 * @fn        Set_Sel
 *
 * @brief     set Set_Sel
 *
 * @return    None
 */
void Set_Sel(void)
{
    UINT8 buf[6];
    buf[0] = 0x5f;buf[1] = 0x0a;buf[2] = 0x54;
    buf[3] = 0x08;buf[4] = 0xff;buf[5] = 0x07;
    memcpy( endpTXbuff , set_sel , 8);
    USB30H_Send_Setup( 8 );

    memcpy( endpTXbuff , buf , 6);
    USB30HOST_CtrlTransaciton(endpTXbuff);
    USB30H_Send_Status();
}

/*******************************************************************************
 * @fn        Set_Configuration
 *
 * @brief     set configuration
 *
 * @return    None
 */
void Set_Configuration(void)
{
  memcpy( endpTXbuff , set_configuration , 8);
  pSetupreq->wValue = config_value;
  USB30H_Send_Setup( 8 );
  USB30H_Send_Status();
}

/*******************************************************************************
 * @fn        Anaylisys_Descr
 *
 * @brief     descriptor analysis.
 *
 * @param     pdesc - descriptor buffer to analyze
 *            l - length
 *
 * @return    None
 */
void Analysis_Descr(UINT8 *pdesc, UINT16 l)
{
    UINT8 in_endp_num = 1;
    UINT8 out_endp_num = 1;
    UINT16 i;
    for(i=0;i<l;i++)                                                //Analysis Descriptor
    {
        if((pdesc[i]==0x09)&&(pdesc[i+1]==0x02))
        {
            printf("bNumInterfaces:%02x \n",pdesc[i+4]);
        }

        if((pdesc[i]==0x07)&&(pdesc[i+1]==0x05))
        {
            if((pdesc[i+2])&0x80)
            {
                g_DevInfo.InMaxBurstSize[in_endp_num] = pdesc[i+9] + 1;
                printf("endp%lx max burst size:%d\n",(in_endp_num|0x80),g_DevInfo.InMaxBurstSize[in_endp_num]);
                in_endp_num++;
            }
            else
            {
                g_DevInfo.OutMaxBurstSize[out_endp_num] = pdesc[i+9] + 1;
                printf("endp%lx max burst size:%d\n",out_endp_num,g_DevInfo.OutMaxBurstSize[out_endp_num]);
                out_endp_num++;
            }
        }
    }
}

/*******************************************************************************
 * @fn        USB30_host_enum
 *
 * @brief     enumerate device
 *
 * @return    None
 */
void USB30_Host_Enum(void)
{
    UINT32 i=0;
    UINT16 len;
//================= set address ===================
        printf("****** set address : 0x08 \n");
        Set_Address(8);
        USB30H_Set_Address(8);
        mDelaymS(1);
//================= get descriptor ===================
        printf("****** get dev descriptor : \n");
        len = GetDEV_Descriptor();
        for(i=0;i<len;i++)
            printf("%02x ",endpRXbuff[i]);
        printf("\n");
        mDelaymS(1);
//================= get cfg descriptor ===================
        printf("****** get cfg descriptor : \n");
        len = GetConfig_Descriptor();
        for(i=0;i<len;i++)
            printf("%02x ",endpRXbuff[i]);
        printf("\n");
        mDelaymS(1);
//================= set isoch delay===================
        printf("****** set isoch delay : \n");
        Set_IsochDelay();
        mDelaymS(1);
//================= set sel ===================
        printf("******set sel : \n");
        Set_Sel();
        mDelaymS(1);
//========================= set config =============================
        printf("****** set config : \n");
        //----- setup stage -----
        Set_Configuration();
}


/*******************************************************************************
 * @fn        USB30_link_status
 *
 * @brief     set link status
 *
 * @param     s - value of link status
 *
 * @return    None
 */
void USB30_link_status(UINT8 s)
{
    if(s)
    {    //Successfully connected to the device
        printf("Linked!\n");
        device_link_status = 1;
    }
    else
    {     //Device disconnected
        device_link_status = 0;
        printf("link is disconnect !\n\n");
    }
}

/*******************************************************************************
 * @fn        LINK_IRQHandler
 *
 * @brief     LINK_IRQHandler
 *
 * @return    None
 */
void LINK_IRQHandler (void)         //USBSSH interrupt service
{
    UINT32 temp = 0;
    temp = USBSS->LINK_ERR_STATUS;
    if( USBSSH->LINK_INT_FLAG & LINK_INACT_FLAG )
    {
        USBSSH->LINK_INT_FLAG = LINK_INACT_FLAG;
        USB30H_Switch_Powermode(POWER_MODE_2);
        printf("link inactive, error status = %0x\n", temp>>16);
    }
    else if( USBSS->LINK_INT_FLAG & LINK_DISABLE_FLAG ) // GO DISABLED
    {
        USBSSH->LINK_INT_FLAG = LINK_DISABLE_FLAG;
        USBSS->LINK_CTRL = POWER_MODE_2;// GO RX DETECT
    }
    else if( USBSSH->LINK_INT_FLAG & LINK_RX_DET_FLAG )
    {
        USBSSH->LINK_INT_FLAG = LINK_RX_DET_FLAG;
        USB30H_Switch_Powermode(POWER_MODE_2);
    }
    else if( USBSSH->LINK_INT_FLAG & TERM_PRESENT_FLAG ) // term present , begin POLLING
    {
        USBSSH->LINK_INT_FLAG = TERM_PRESENT_FLAG;
        if( USBSS->LINK_STATUS & LINK_PRESENT )
        {
            USB30H_Switch_Powermode(POWER_MODE_2);
            USBSSH->LINK_CTRL |= POLLING_EN;
        }
        else
        {
            USB30HOST_Init(DISABLE,endpTXbuff,endpRXbuff);
            USB30_link_status(0);       //disconnect link
            g_DeviceConnectstatus = USB_INT_DISCONNECT;
            g_DeviceUsbType = 0;
            printf("dis\n");
            mDelaymS(1);
            R8_SAFE_ACCESS_SIG = 0x57;                  //enable safe access mode
            R8_SAFE_ACCESS_SIG = 0xa8;
            R8_RST_WDOG_CTRL = 0x40 | RB_SOFTWARE_RESET;
            while(1);
        }

    }
    else if( USBSSH->LINK_INT_FLAG & LINK_TXEQ_FLAG ) // POLLING SHAKE DONE
    {
        tx_lmp_port = 1;
        USBSSH->LINK_INT_FLAG = LINK_TXEQ_FLAG;
        if( USBSS->LINK_STATUS & LINK_PRESENT )
        {
            while(USBSS->LINK_STATUS & LINK_RX_DETECT);
            USB30H_Switch_Powermode(POWER_MODE_0);
        }
    }
    else if( USBSSH->LINK_INT_FLAG & LINK_RDY_FLAG ) // POLLING SHAKE DONE
    {
        USBSSH->LINK_INT_FLAG = LINK_RDY_FLAG;
        if( tx_lmp_port ) // LMP, TX PORT_CAP & RX PORT_CAP
        {
            tx_lmp_port = 0;
            USB30H_Lmp_Init();
            USB30_link_status(1);   //link success
            g_DeviceConnectstatus = USB_INT_CONNECT;
            g_DeviceUsbType = USB_U30_SPEED;
        }
    }
}

/*******************************************************************************
 * @fn        USBSS_IRQHandler
 *
 * @brief     USBSS_IRQHandler
 *
 * @return    None
 */
void USBSS_IRQHandler (void)            //USBSSH interrupt service
{
    ;
}

