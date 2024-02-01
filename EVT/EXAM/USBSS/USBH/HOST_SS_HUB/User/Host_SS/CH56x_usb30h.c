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
#include "hub.h"
#include "CH56x_host_hs.h"
#include "stdlib.h"

/* Function */
void LINK_IRQHandler( void ) __attribute__((interrupt("WCH-Interrupt-fast")));
void Analysis_Descr(UINT8 *pdesc, UINT16 len);

/* Global define */
#define pSetupreq    ((PUSB_SETUP_REQ)endpTXbuff)

/* Global Variable */
__attribute__ ((aligned(16))) UINT8 endpRXbuff[512] __attribute__((section(".DMADATA"))); //data recive buffer
__attribute__ ((aligned(16))) UINT8 endpTXbuff[512] __attribute__((section(".DMADATA"))); //data send buffer
__attribute__ ((aligned(16))) UINT8 Test_Buf[3072] __attribute__((section(".DMADATA")));

UINT8V tx_lmp_port = 0;
UINT8V device_link_status = 0;              //USBSS link flag
UINT8V config_value = 0;
UINT8V gDeviceConnectstatus;                /* USB connection status */
UINT8  gDeviceClassType;
UINT8  gDeviceUsbType = 0;                  /* 01--USB2.0&1.1  02--USB3.0*/
UINT8 AddressNum[127] = {0};


Link_HUBSaveData* Hub_LinkHead;//HUB link head

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
 * @fn        InitHubLink
 *
 * @brief     Create a link header node for HUB
 *
 * @param     None
 *
 * @return    None
 */
Link_HUBSaveData* InitHubLink()
{
    //Create a header pointer
    Link_HUBSaveData* p = NULL;
    //Create a header node
    Link_HUBSaveData* temp = (Link_HUBSaveData*)malloc(sizeof(Link_HUBSaveData));

    memset(&temp->HUB_SaveData,0,sizeof(USB_HUB_SaveData));
    temp->HUB_SaveData.Depth = 0xff;
    temp->HUB_SaveData.CurrentPort = 0xff;
    temp->HUB_SaveData.UpLevelPort = 0xff;
    temp->HUB_SaveData.HUB_Info.devaddr = 0xff;

    temp->next = NULL;
    //The head pointer points to the head node
    p = temp;

    return p;
}

/*******************************************************************************
 * @fn        AssignHubData
 *
 * @brief     Assign HUB data
 *
 * @param     hubdata - Node to be assigned
 *
 * @return    None
 */
void AssignHubData(USB_HUB_SaveData *hubdata ,UINT8 depth,UINT8 uplevelport,UINT8 currentport,USB_HUB_Info hub_info)
{
   (*hubdata).Depth = depth;
   (*hubdata).UpLevelPort = uplevelport;
   (*hubdata).CurrentPort = currentport;
   (*hubdata).HUB_Info = hub_info;
}

/*******************************************************************************
 * @fn        SearchHubData
 *
 * @brief     Find elements in a linked list
 *
 * @param     p - Original linked list
 *            HUB_SaveData - Represents the searched element
 *
 * @return    None
 */
UINT8 SearchHubData(Link_HUBSaveData* p, USB_HUB_SaveData *HUB_SaveData)
{
    p = p->next;
    while (p)
    {
        if (p->HUB_SaveData.Depth == (*HUB_SaveData).Depth
            && p->HUB_SaveData.UpLevelPort == (*HUB_SaveData).UpLevelPort
            && p->HUB_SaveData.CurrentPort == (*HUB_SaveData).CurrentPort
            && p->HUB_SaveData.HUB_Info.devaddr == (*HUB_SaveData).HUB_Info.devaddr)
        {
            *HUB_SaveData = p->HUB_SaveData;
            return 0;//found
        }
        p = p->next;
    }
    return 1;//not found
}

/*******************************************************************************
 * @fn        SearchHubData
 *
 * @brief     Insert Linked List Node
 *
 * @param     p - Original linked list
 *            HUB_SaveData - Node elements that need to be inserted
 *
 * @return    None
 */
UINT8 InsertHubData(Link_HUBSaveData* p, USB_HUB_SaveData HUB_SaveData)
{
    Link_HUBSaveData* c = NULL;
    Link_HUBSaveData* temp = p;//Create temporary nodes temp

    if(SearchHubData(p,&HUB_SaveData) == 0)
    {
        return 1;//Existing devices to prevent duplicate creation
    }

    while( temp->next )
    {
        temp = temp->next;
    }

    //Create Insert Node c
    c = (Link_HUBSaveData*)malloc(sizeof(Link_HUBSaveData));
    c->HUB_SaveData = HUB_SaveData;
    //Point the next pointer of the new node to the node after the insertion position
    c->next = temp->next;
    //Point the next pointer of the node before the insertion position to the insertion node
    temp->next = c;

    return 0;//success
}

/*******************************************************************************
 * @fn        DeleteHubData
 *
 * @brief     Delete elements from HUB linked list
 *
 * @param     p - Original linked list
 *            HUB_SaveData - The target element to be deleted
 *
 * @return    None
 */
UINT8 DeleteHubData(Link_HUBSaveData* p, USB_HUB_SaveData HUB_SaveData)
{
    Link_HUBSaveData* del = NULL, *temp = p;
    UINT8 find = 0;
    //Find the direct precursor node of the target element
    while (temp->next)
    {
        if (temp->next->HUB_SaveData.Depth == HUB_SaveData.Depth
            && temp->next->HUB_SaveData.UpLevelPort == HUB_SaveData.UpLevelPort
            && temp->next->HUB_SaveData.CurrentPort == HUB_SaveData.CurrentPort
            && temp->next->HUB_SaveData.HUB_Info.devaddr == HUB_SaveData.HUB_Info.devaddr)
        {
            find = 1;
            break;
        }
        temp = temp->next;
    }
    if (find == 0)
    {
        return 1;//Delete failed
    }
    else
    {
        //Mark nodes to be deleted
        del = temp->next;
        //Remove the target node from the linked list
        temp->next = temp->next->next;
        //Release target node
        free(del);
        return 0;//Successfully deleted
    }
}

/*******************************************************************************
 * @fn        ModifyHubData
 *
 * @brief     Modify HUB linked list elements
 *
 * @param     p - Original linked list
 *            oldhubdata - Old Elements
 *            newhubdata - New Elements
 *
 * @return    None
 */
UINT8 ModifyHubData(Link_HUBSaveData* p, USB_HUB_SaveData oldhubdata, USB_HUB_SaveData newhubdata)
{
    p = p->next;
    while (p)
    {
        if (p->HUB_SaveData.Depth == oldhubdata.Depth
            && p->HUB_SaveData.UpLevelPort == oldhubdata.UpLevelPort
            && p->HUB_SaveData.CurrentPort == oldhubdata.CurrentPort
            && p->HUB_SaveData.HUB_Info.devaddr == oldhubdata.HUB_Info.devaddr)
        {
            p->HUB_SaveData = newhubdata;
            return 0;//success
        }
        p = p->next;
    }
    return 1;//fail
}

/*******************************************************************************
 * @fn        Hublink_judgingstructures
 *
 * @brief     Determine and delete sub devices under the device
 *
 * @param     hubdata - Variables used for judgment
 *
 * @return    None
 */
UINT8  Hublink_judgingstructures( USB_HUB_SaveData hubdata )
{
    UINT8 ret = 0;
    UINT8 del_flg = 0;
    Link_HUBSaveData *deletehub = NULL;
    deletehub = Hub_LinkHead;

    while(deletehub != NULL)
    {
        if( deletehub->HUB_SaveData.Depth-1 == hubdata.Depth )
        {
                if( deletehub->HUB_SaveData.HUB_Info.devaddr == hubdata.HUB_Info.portD[hubdata.CurrentPort-1].addr )
                {
                    if( deletehub->HUB_SaveData.HUB_Info.portD[deletehub->HUB_SaveData.CurrentPort-1].devicetype == 0x09 )
                    {
                        Hublink_judgingstructures(deletehub->HUB_SaveData);
                        del_flg = 1;
                        break;
                    }
                    else
                    {
                        del_flg = 1;
                        break;
                    }
                }
        }
        deletehub = deletehub->next;
    }

    if( del_flg == 1 )
    {
        USB_DelAddressNumber(deletehub->HUB_SaveData.HUB_Info.portD[deletehub->HUB_SaveData.CurrentPort-1].addr);
        ret = DeleteHubData(Hub_LinkHead,deletehub->HUB_SaveData);//Delete linked list nodes
        if( ret == 0 )
        {
        }
        return 0;
    }
    else
    {
        return 1;
    }

}

/*******************************************************************************
 * @fn        Hublink_finesubstructures
 *
 * @brief     Used to query whether there are sub devices under this variable
 *
 * @param     hubdata - Variables used for judgment
 *
 * @return    None
 */
void Hublink_finesubstructures( USB_HUB_SaveData hubdata )
{
    while( 1 )
    {
        if(Hublink_judgingstructures(hubdata) == 0)//If there are devices, proceed to the next judgment until there are no devices available
        {
            continue;
        }
        else
        {
            break;
        }
    }
}

/*******************************************************************************
 * @fn        Hublink_finesubstructures
 *
 * @brief     Number of assigned addresses
 *
 * @param     None
 *
 * @return    Number of addresses to be set
 */
UINT8 USB_SetAddressNumber(void)
{
    UINT8 i = 0;
    for (i = DEVICE_ADDR+1; i < 127; i++)
    {
        if( AddressNum[i] == 0 )
        {
            AddressNum[i] = 1;
            return i;
        }
    }
    return 0xff;
}

/*******************************************************************************
 * @fn        Hublink_finesubstructures
 *
 * @brief     Number of assigned addresses
 *
 * @param     None
 *
 * @return    0 - success 1 - faild
 */
UINT8 USB_DelAddressNumber(UINT8 addr)
{
    if( AddressNum[addr] == 1)
    {
        AddressNum[addr] = 0;
        return 0;//success
    }

    return 1;//fail
}

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
                if(i == 10000000)
                {
                    printf("IN ERDY timeout...\n");
                    i=0;
                    num = USB30H_IN_Data(seq_num, recv_packnum, endp_num);    //retry
                    if( (num & 0x7000) == 0x1000 )
                    {
                        break;                                      //Retry succeeded
                    }
                    else
					{
                        return USB30_IN_DISCONNECT;
                    }
                }
            }
            if(packnum > 1)
            {                                                  //Only one package
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
 * @param     seq_num       -   The packet sequence of the first packet of the distributed packet
 *            recv_packnum  -   The number of packets that the host is ready to send
 *            endp_num      -   endpoint number
 *            txlen         -   The packet length of the last packet of data
 *
 * @return    Number of unsent packets remaining
 */
UINT8 USB30HOST_OUTTransaction(UINT8 seq_num,UINT8 send_packnum ,UINT8 endp_num,UINT32 txlen)
{
    UINT8  sta;
    UINT8  packnump;
    UINT32 i = 0;

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
                if(i==1000000)  //ERDY timeout
                {
                    printf("OUT ERDY timeout...\n");
                    i=0;
                    sta = USB30H_OUT_Data( seq_num, send_packnum, endp_num, txlen);    //Retry
                    if( (sta&0x07) == 1 )                                              //Retry succeeded
                    {
                        send_packnum = 0;
                        break;
                    }
                    else
                    {
                        return USB30_OUT_DISCONNECT;
                    }
                }
            }
            if(packnump > 1)
            {                                                         //Only one package
                packnump = 1;
            }
            if((sta&0x07) == 2)
            {
                sta = USB30H_OUT_Data( seq_num, packnump, endp_num, txlen);
                send_packnum -= packnump ;
            }
            break;
    }
    return send_packnum;                                                              //Remaining number of packages not issued
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
                            return USB30_IN_DISCONNECT;
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
                     if(i==2000000)
                     {
                         printf("OUT ERDY timeout...\n");
                         i=0;
                         len = USB30H_OUT_Data(seq_num,req_nump,0,txlen);
                         if( (len&0x07) == 1 )                                              //Retry succeeded
                         {
                             break;
                         }
                         else
                         {
                             return USB30_OUT_DISCONNECT;
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
UINT16 GetConfig_Descriptor(PHUB_Port_Info phub,UINT8 depth , UINT8 port)
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

  thisUsbDev.DeviceCongValue = ( (PUSB_CFG_DESCR)endpRXbuff )-> bConfigurationValue;
  printf("thisUsbDev.DeviceType=%02x\n",thisUsbDev.DeviceType);
  if( thisUsbDev.DeviceType == 0x09 )//HUB processing
  {
      HubAnalysis_Descr( phub,(UINT8 *)endpRXbuff, pSetupreq->wLength );
  }
  else//Other device processing
  {
      USBHS_Analysis_Descr( &thisUsbDev,(UINT8 *)endpRXbuff, pSetupreq->wLength );
  }

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
  USB30H_Set_Address(0);
  memcpy( endpTXbuff , set_address , 8);
  pSetupreq->wValue = addr;
  USB30H_Send_Setup( 8 );
  USB30H_Send_Status();
  USB30H_Set_Address(addr);
}

/*******************************************************************************
 * @fn        U30_RootSetAddress
 *
 * @brief     Set root device address
 *
 * @param     addr - device address
 *
 * @return    None
 */
void U30_RootSetAddress(UINT8 addr)
{
  UINT8 s = 0;

  USB30H_Set_Address(0);
  memcpy( endpTXbuff , set_address , 8);
  pSetupreq->wValue = addr;
  s |= USB30H_Send_Setup( 8 );
  s |= USB30H_Send_Status();

  if( s )
  {
      R8_SAFE_ACCESS_SIG = 0x57;                  //enable safe access mode
      R8_SAFE_ACCESS_SIG = 0xa8;
      R8_RST_WDOG_CTRL = 0x40 | RB_SOFTWARE_RESET;
      while(1);
  }

  USB30H_Set_Address(addr);
}

/*******************************************************************************
 * @fn        Set_IsochDelay
 *
 * @brief     set isoch_delay
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
 * @fn        SetFeatureU1
 *
 * @brief     None
 *
 * @return    None
 */
void SetFeatureU1(void)
{
  UINT8 buf[8] = {0x00,0x03,0x30,0x00,0x00,0x00,0x00,0x00};

  memcpy( endpTXbuff , buf , 8);
  USB30H_Send_Setup( 8 );
  USB30H_Send_Status();
}

/*******************************************************************************
 * @fn        SetFeatureU2
 *
 * @brief     None
 *
 * @return    None
 */
void SetFeatureU2(void)
{
  UINT8 buf[8] = {0x00,0x03,0x31,0x00,0x00,0x00,0x00,0x00};
  memcpy( endpTXbuff , buf , 8);
  USB30H_Send_Setup( 8 );
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
void Analysis_Descr(UINT8 *pdesc, UINT16 len)
{
    UINT8 in_endp_num = 1;
    UINT8 out_endp_num = 1;
    UINT16 i;
    for(i=0;i<len;i++)                                                //Analysis Descriptor
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
* Function Name  : USB30HOST_ClearEndpStall
* Description    : USB Host Purge Endpoint
* Input          : *Device---Current operating device
                   endp------Endpoint number to clear
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 USB30HOST_ClearEndpStall( UINT8 endp )
{
    UINT8  setup_buf[8];
    setup_buf[0] = 0x02;
    setup_buf[1] = 0x01;
    setup_buf[2] = 0X00;
    setup_buf[3] = 0x00;
    setup_buf[4] = endp;
    setup_buf[5] = 0x00;
    setup_buf[6] = 0x00;
    setup_buf[7] = 0x00;

    memcpy( endpTXbuff , setup_buf , 8);
    USB30H_Send_Setup( 8 );
    USB30HOST_CtrlTransaciton(endpRXbuff);
    USB30H_Send_Status();

    return( USB_INT_SUCCESS );
}

/*******************************************************************************
* Function Name  : U30HOST_CofDescrAnalyse
* Description    : USB Host Analysis Configuration Descriptor
* Input          : *pbuf---Descriptor buffer
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 U30HOST_CofDescrAnalyse( UINT8 depth,UINT8 *pbuf, UINT8 port )
{
    gDeviceClassType = ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> itf_descr.bInterfaceClass;
    if( ( gDeviceClassType <= 0x09 ) || ( gDeviceClassType == 0xFF ) )
    {
        if( gDeviceClassType == 0x09 )
        {         //HUB
            if( port == 0 )
            {
                ss_hub_info[depth].device_type = gDeviceClassType;
            }
            else{
                ss_hub_info[depth].portD[port-1].devicetype = gDeviceClassType;
            }
        }
        else if( gDeviceClassType == 0x08 )
        {    //U-DISK
            if( port == 0 ){
                ss_hub_info[depth].device_type = gDeviceClassType;
            }
            else
            {
                ss_hub_info[depth].portD[port-1].devicetype = gDeviceClassType;
            }
        }
        else
        {
            if( port == 0 ){
                ss_hub_info[depth].device_type = gDeviceClassType;
            }
            else
            {
                ss_hub_info[depth].portD[port-1].devicetype = gDeviceClassType;
            }
        }
        return( USB_INT_SUCCESS );
    }
    else
    {
        gDeviceClassType = USB_DEV_CLASS_RESERVED;
        return( 0xfe );
    }
}

/*******************************************************************************
 * @fn        USB30Host_Enum
 *
 * @brief     enumerate device
 *
 * @return    None
 */
void USB30Host_Enum(UINT8 depth,UINT8 addr, UINT8 port )
{
    UINT8 status,i;
    UINT16 len;

//============= set address ===================
        U30_RootSetAddress(addr);
        printf("Set_Address\n");
        USB30H_Set_Address(addr);
//============= get descriptor ================
        GetDEV_Descriptor();
//============= get cfg descriptor ============
        GetConfig_Descriptor( &ss_hub_info[depth].portD[port],depth,port );

        U30HOST_CofDescrAnalyse( depth,endpRXbuff,port);

//============= set isoch delay================
        Set_IsochDelay();

//============= set sel =======================
        Set_Sel();

        if( ss_hub_info[depth].device_type == 0x09 )
        {
            status = U30HOST_GetHubDescr( endpRXbuff,&len );
            if( status != USB_INT_SUCCESS )
            {
                printf("get_hub_des_error\n");
            }
            printf("gen_hub_des:");
            for( i=0;i!=len;i++ )
            {
                printf("%02x ",endpRXbuff[i]);
            }
            printf("\n");
            printf("hub_num = %02x\n",endpRXbuff[2]);
            ss_hub_info[depth].numofport = endpRXbuff[2];
            if( ss_hub_info[depth].numofport>=4  )ss_hub_info[depth].numofport = 4;
            printf("U30HOST_GetDeviceStstus:\n");
            status = U30HOST_GetDeviceStstus();
            if( status != USB_INT_SUCCESS )
            {
               printf("U30HOST_GetDeviceStstus_Error\n");
            }
        }
//============= set config ====================
        Set_Configuration();

//        SetFeatureU1();
//
//        SetFeatureU2();

        if( ss_hub_info[depth].device_type == 0x09 )/*HUB*/
        {
            printf("U30HOST_SetHubDepth\n");
            status = U30HOST_SetHubDepth( 0 );
            if ( status != USB_INT_SUCCESS )
            {
                printf( "U30HOST_SetHubDepth_ERROR = %02X\n", (UINT16)status );
            }
            for( i=0;i!=ss_hub_info[depth].numofport;i++ )
            {
                status = U30HOST_GetPortStstus( depth,i+1 );
                if ( status != USB_INT_SUCCESS )
                {
                    printf( "U30HOST_GetPortStstus = %02X\n", (UINT16)status );
                }
            }
            for( i=0;i!=ss_hub_info[depth].numofport;i++ )
            {
                status = U30HOST_SetPortFeatrue( i+1 );
                if ( status != USB_INT_SUCCESS )
                {
                    printf( "U30HOST_SetPortFeatrue = %02X\n", (UINT16)status );
                }
            }
            ss_hub_info[depth].speed = PORT_SS_SPEED;
            ss_hub_info[depth].time_out = 0;
        }
        else if( ss_hub_info[depth].device_type == 0x08 ) /* udisk */
        {

        }

}

/*******************************************************************************
* Function Name  : USB30HSOT_Enumerate_Hotrst
* Description    : HOT_RST Dedicated enumeration function
* Input          : *pbuf
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 USB30HSOT_Enumerate_Hotrst( UINT8 *pbuf )
{
    UINT8 status = 0;

    USB30H_Set_Address( 0 );
    mDelaymS(1);
    Set_Address( DEVICE_ADDR-1 );

    GetDEV_Descriptor();

    mDelaymS( 20 );
    /**Get configuration descriptor**/
    GetConfig_Descriptor(NULL,0,0);

    Set_Configuration(  );

    return status;
}

/*******************************************************************************
 * @fn        USB30_HUBHostEnum
 *
 * @brief     Enumerate USB 3.0 devices under HUB
 *
 * @return    None
 */
void USB30_HUBHostEnum(UINT8 depth, UINT8 addr, UINT8 port )
{
    UINT8 status,i;
    UINT16 len;

//============= set address ===================
        Set_Address(addr);
        printf("Set_Address\n");
        USB30H_Set_Address(addr);
//============= get descriptor ================
        GetDEV_Descriptor();
//============= get cfg descriptor ============
        GetConfig_Descriptor( &ss_hub_info[depth].portD[port] ,depth,port);

        U30HOST_CofDescrAnalyse(depth,endpRXbuff,port);
//============= set isoch delay================
        Set_IsochDelay();

//============= set sel =======================
        Set_Sel();

//============= set config ====================
        Set_Configuration();

//        SetFeatureU1();
//
//        SetFeatureU2();

        if( ss_hub_info[depth].portD[port].devicetype == 0x09 )
        {             /*HUB*/
            status = U30HOST_GetHubDescr( endpRXbuff,&len );
            if( status != USB_INT_SUCCESS )
            {
                printf("get_hub_des_error\n");
            }
            printf("gen_hub_des:");
            for( i=0;i!=len;i++ )
            {
                printf("%02x ",endpRXbuff[i]);
            }
            printf("\n");
            printf("hub_num = %02x\n",endpRXbuff[2]);
            ss_hub_info[depth].numofport = endpRXbuff[2];
            if( ss_hub_info[depth].numofport>=4  )ss_hub_info[depth].numofport = 4;
            printf("U30HOST_GetDeviceStstus:\n");
            status = U30HOST_GetDeviceStstus();
            if( status != USB_INT_SUCCESS )
            {
               printf("U30HOST_GetDeviceStstus_Error\n");
            }

            printf("U30HOST_SetHubDepth\n");
            status = U30HOST_SetHubDepth( (depth)+1 );
            if ( status != USB_INT_SUCCESS )
            {
                printf( "U30HOST_SetHubDepth_ERROR = %02X\n", (UINT16)status );
            }

            for( i=0;i!=ss_hub_info[depth].numofport;i++ )
            {
                status = U30HOST_GetPortStstus( depth,i+1 );
                if ( status != USB_INT_SUCCESS )
                {
                    printf( "U30HOST_GetPortStstus = %02X\n", (UINT16)status );
                }
            }

            for( i=0;i!=ss_hub_info[depth].numofport;i++ )
            {
                status = U30HOST_SetPortFeatrue( i+1 );
                if ( status != USB_INT_SUCCESS )
                {
                    printf( "U30HOST_SetPortFeatrue = %02X\n", (UINT16)status );
                }
            }

            ss_hub_info[depth].speed = PORT_SS_SPEED;
            ss_hub_info[depth].time_out = 0;
        }
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
    UINT32 temp;
    temp = USBSS->LINK_ERR_STATUS;

    if( USBSSH->LINK_INT_FLAG & LINK_RECOV_FLAG )
    {
        USBSSH->LINK_INT_FLAG = LINK_RECOV_FLAG;
    }
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
        printf("go disabled \n");
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
            printf("rx term present!\n\n");
        }
        else
        {
            USBSSH->LINK_INT_CTRL = 0;
            printf("link is disconnect !\n\n");
            gDeviceConnectstatus = USB_INT_DISCONNECT;
            gDeviceUsbType = 0;
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
            printf("link is tx EQ !%0x\n\n", USBSS->LINK_STATUS);
        }
    }
    else if( USBSSH->LINK_INT_FLAG & LINK_RDY_FLAG ) // POLLING SHAKE DONE
    {
        USBSSH->LINK_INT_FLAG = LINK_RDY_FLAG;
        if( tx_lmp_port ) // LMP, TX PORT_CAP & RX PORT_CAP
        {
            tx_lmp_port = 0;
            USB30H_Lmp_Init();
            printf("lmp rtx successfully !\n\n");
            gDeviceConnectstatus = USB_INT_CONNECT;
            gDeviceUsbType = USB_U30_SPEED;
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

/*******************************************************************************
* Function Name  : U30OST_GetPortStstus
* Description    : USB Host gets port status
* Input          :
                   port   port number

* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U30HOST_GetPortStstus( UINT8 depth,UINT8 port )
{
    UINT16  Port_Status_Bits,Port_Change_Field,timeoutcnt;
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

    memcpy( endpTXbuff , setup_buf , 8);

    status =  USB30H_Send_Setup( 8 );
    if(status == 1)
    {
        return USB_CH56XUSBTIMEOUT;
    }

    timeoutcnt = 0;
    do
    {
        if( timeoutcnt >= 100 )
        {
            return USB_CH56XUSBTIMEOUT;
        }
        status = USB30HOST_CtrlTransaciton(endpRXbuff);
        timeoutcnt++;
    }
    while (status == 0);

    status = USB30H_Send_Status();
    if( status == 1)
    {
        return USB_CH56XUSBTIMEOUT;
    }

    status = USB_INT_SUCCESS;
    memcpy( buf,endpRXbuff,4 );
    if( status == USB_INT_SUCCESS )
    {

        Port_Status_Bits = buf[0];
        Port_Status_Bits |= ((UINT16)buf[1]<<8);
        Port_Change_Field = buf[2];
        Port_Change_Field |= ((UINT16)buf[3]<<8);

        if( Port_Status_Bits&PORT_CONNECTION )
        {                        //Device connection detected
            if(ss_hub_info[depth].portD[port-1].status<PORT_CONNECTION)
            {
                ss_hub_info[depth].portD[port-1].speed = PORT_SS_SPEED;
                ss_hub_info[depth].portD[port-1].status = PORT_CONNECTION;    //Device connection
            }
        }
        else
        {
            ss_hub_info[depth].portD[port-1].status = PORT_DISCONNECT;        //Device disconnection
        }
        ss_hub_info[depth].portD[port-1].portpchangefield = Port_Change_Field;

    }
    return status;
}

/*******************************************************************************
* Function Name  : U30HOST_SetPortFeatrue
* Description    : USB Host Set Port Properties
* Input          :
                   port  port number
                   port_status  Clear Port Status
                   NULL
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U30HOST_ClearPortFeatrue( UINT8 port ,UINT8 port_status )
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

    memcpy( endpTXbuff , setup_buf , 8);
    USB30H_Send_Setup( 8 );
    USB30HOST_CtrlTransaciton(endpRXbuff);
    USB30H_Send_Status();
    status = USB_INT_SUCCESS;
    if( status == USB_INT_SUCCESS )
    {
    }
    return status;
}

/*******************************************************************************
* Function Name  : U30HOST_ClearPortFeatrue_Process
* Description    : USB Host clears port status
* Input          :
                   port  port number
                   NULL
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U30HOST_ClearPortFeatrue_Process( UINT8 depth,UINT8 port )
{
    UINT8 status;
    if( ss_hub_info[depth].portD[port-1].portpchangefield&0x01 )
    {      //Connection status changes
        status = U30HOST_ClearPortFeatrue( port,C_PORT_CONNECTION );
        if( status == USB_INT_SUCCESS )return status;
    }
    if( ss_hub_info[depth].portD[port-1].portpchangefield&0x08 )
    {      //Overcurrent indication changes
        status = U30HOST_ClearPortFeatrue( port,C_PORT_OVER_CURRENT );
        if( status == USB_INT_SUCCESS )return status;
    }
    if( ss_hub_info[depth].portD[port-1].portpchangefield&0x10 )
    {      //Reset change
        status = U30HOST_ClearPortFeatrue( port,C_PORT_RESET );
        if( status == USB_INT_SUCCESS )return status;
    }
    if( ss_hub_info[depth].portD[port-1].portpchangefield&0x20 )
    {      //This field will be set when the hot reset processing on this port is completed
        status = U30HOST_ClearPortFeatrue( port,C_BH_PORT_RESET );
        if( status == USB_INT_SUCCESS )return status;
    }
    if( ss_hub_info[depth].portD[port-1].portpchangefield&0x40 )
    {      //Port connection changes
        status = U30HOST_ClearPortFeatrue( port,C_PORT_LINK_STATE );
        if( status == USB_INT_SUCCESS )return status;
    }
    if( ss_hub_info[depth].portD[port-1].portpchangefield&0x80 )
    {      //Configuration error change
        status = U30HOST_ClearPortFeatrue( port,C_PORT_CONFIG_ERROR );
        if( status == USB_INT_SUCCESS )return status;
    }
}

/*******************************************************************************
* Function Name  : U30OST_GetHubDescr
* Description    : USB Host gets HUB descriptor
* Input          :
                   *buf------data buffer
                   *len------data length
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U30HOST_GetHubDescr( UINT8 *buf ,UINT16 *len )
{
    UINT16 l;
    UINT8 setup_buf[8];
    setup_buf[0] = 0xa0;
    setup_buf[1] = 0x06;
    setup_buf[2]  = 0x00;
    setup_buf[3]  = 0x2a;
    setup_buf[4]  = 0x00;
    setup_buf[5]  = 0x00;
    setup_buf[6] = 0x0c;
    setup_buf[7] = 0x00;
    l = 0x0c;
    memcpy( endpTXbuff , setup_buf , 8);
    USB30H_Send_Setup( 8 );
    USB30HOST_CtrlTransaciton(endpRXbuff);
    USB30H_Send_Status();

    *len = l;

    return USB_INT_SUCCESS;
}

/*******************************************************************************
* Function Name  : U30OST_GetDeviceStstus
* Description    : USB host obtains device status
* Input          : None
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U30HOST_GetDeviceStstus( void )
{
    UINT8 setup_buf[8];
    setup_buf[0] = 0x80;
    setup_buf[1] = 0x00;
    setup_buf[2] = 0x00;
    setup_buf[3] = 0x00;
    setup_buf[4] = 0x00;
    setup_buf[5] = 0x00;
    setup_buf[6] = 0x02;
    setup_buf[7] = 0x00;
    memcpy( endpTXbuff , setup_buf , 8);
    USB30H_Send_Setup( 8 );
    USB30HOST_CtrlTransaciton(endpRXbuff);
    USB30H_Send_Status();

    return USB_INT_SUCCESS;
}

/*******************************************************************************
* Function Name  : U30HOST_SetHubDepth
* Description    : USBHost set HUB depth
* Input          :
                   depth HUB depth
                   NULL
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U30HOST_SetHubDepth( UINT8 depth )
{
    UINT8 setup_buf[8];
    setup_buf[0] = 0x20;
    setup_buf[1] = 0x0c;
    setup_buf[2]  = depth;
    setup_buf[3]  = 0x00;
    setup_buf[4]  = 0x00;
    setup_buf[5]  = 0x00;
    setup_buf[6] = 0x00;
    setup_buf[7] = 0x00;
    memcpy( endpTXbuff , setup_buf , 8);
    USB30H_Send_Setup( 8 );
    USB30HOST_CtrlTransaciton(endpRXbuff);
    USB30H_Send_Status();

    return USB_INT_SUCCESS;
}

/*******************************************************************************
* Function Name  : U30HOST_SetPortFeatrue
* Description    : USB Host Set Port Properties
* Input          :
                   port  port number
                   NULL
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 U30HOST_SetPortFeatrue( UINT8 port )
{
    UINT8 setup_buf[8];
    setup_buf[0] = 0x23;
    setup_buf[1] = 0x03;
    setup_buf[2]  = 0x1b;
    setup_buf[3]  = 0x00;
    setup_buf[4]  = port;
    setup_buf[5]  = 0x07;
    setup_buf[6] = 0x00;
    setup_buf[7] = 0x00;
    memcpy( endpTXbuff , setup_buf , 8);
    USB30H_Send_Setup( 8 );
    USB30HOST_CtrlTransaciton(endpRXbuff);
    USB30H_Send_Status();

    return USB_INT_SUCCESS;
}

/*******************************************************************************
* Function Name  : USBSS_HUBCheckPortConnect
* Description    : Check the port status under HUB
* Input          : depth --- hub depth
*                  port  --- hub port
                   NULL
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 USBSS_HUBCheckPortConnect( UINT8 depth,UINT8 port )
{
    UINT8 ret;
    ret = U30HOST_GetPortStstus( depth,port+1 );
    if( ret != ERR_SUCCESS )return ret;
    /* Determine the current port connection status */
    if( endpRXbuff[ 2 ] & 0x01 )                                /* The connection status of this port has changed */
    {
        if( endpRXbuff[ 0 ] & 0x01 )
        {
            if( ss_hub_info[depth].portD[port].status<HUB_ERR_SCUESS )
            {
                ss_hub_info[depth].portD[port].status = HUB_ERR_CONNECT;
            }
            return( 0x18 );                                    /* This port: Device connection detected */
        }
        else
        {
            ss_hub_info[depth].portD[port].status = HUB_ERR_DISCONNECT;
            return( 0x19 );                                 /* This port: Device disconnection detected */
        }
    }
    else                                                                        /* The connection status of this port has not changed */
    {
        if( endpRXbuff[ 0 ] & 0x01 )
        {
            if( ss_hub_info[depth].portD[port].status<HUB_ERR_SCUESS )
            {
                ss_hub_info[depth].portD[port].status = HUB_ERR_CONNECT;
            }
            return( 0x02 );                                                     /* This port: has devices */
        }
        else
        {
            ss_hub_info[depth].portD[port].status = HUB_ERR_DISCONNECT;
            return( 0x01 );                                                     /* This port: No device */
        }
    }
}

/*******************************************************************************
* Function Name  : USBSS_HUBHostEnum
* Description    : Check the port status under HUB
* Input          : depth --- hub depth
*                  *Databuf --- Enumerate Buffers
*                  port  --- hub port
*                  route_string --- USB3.0 Route string
                   NULL
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 USBSS_HUBHostEnum( UINT8 depth, UINT8 *Databuf,UINT8 port ,UINT32 route_string)
{
    UINT8 ret;
    UINT16 s;

    mDelaymS( 10 );

    s = U30HOST_GetPortStstus( depth,port+1 );
    if( s != USB_INT_SUCCESS )
    {
      printf( "U30HOST_GetPortStstus = %02x,%02x\n", (UINT16)s,port+1 );
      return s;
    }

    s = U30HOST_SetPortFeatrue(port+1);
    if( s != USB_INT_SUCCESS )return s;


    do
    {
      mDelaymS( 10 );
      ret = U30HOST_GetPortStstus(depth, port+1 );
      if( ret != USB_INT_SUCCESS )
      {
          return( ret );
      }

      printf("0x10\n");

    }
    while( endpRXbuff[ 0 ] & 0x10 );


    printf("hub_device_connect_port=%02x\n",port+1);
    ss_hub_info[depth].portD[port].speed = PORT_SS_SPEED;
    s = U30HOST_ClearPortFeatrue_Process( depth,port+1 );       //Clear the device status. All status needs to be cleared
    if( s != USB_INT_SUCCESS )return s;

    s = U30HOST_GetPortStstus( depth,port+1 );                  //Get it again
    if( s != USB_INT_SUCCESS )return s;


    for( ret=0;ret!=4;ret++ )
    {
      printf("%02x ",endpRXbuff[ret]);
    }
    printf("\n");
    if( ( endpRXbuff[ 0 ] & 0x01 ) == 0x00 )
    {
      return( 0x18 );
    }

    printf("Speed=%02x,%02x\n",ss_hub_info[depth].portD[port].speed,ss_hub_info[depth].portD[port].status);


    USB30H_Set_Address(0x00);
    ss_hub_info[depth].portD[port].addr = USB_SetAddressNumber();

    route_string = ((port+1)<<((depth)*4)|route_string);
    USBSS->LINK_ROUTE_STRING = route_string;

    USB30_HUBHostEnum( depth,ss_hub_info[depth].portD[port].addr,port);

    USBSS->LINK_ROUTE_STRING = 0;
    USB30H_Set_Address(ss_hub_info[depth].devaddr);

    ss_hub_info[(depth)+1].devaddr = ss_hub_info[depth].portD[port].addr;   //Root directory address of subordinate HUB
    ss_hub_info[depth].portD[port].status = PORT_INIT;

    USBSSH->UH_TX_DMA = (UINT32)(UINT8 *)endpTXbuff;
    USBSSH->UH_RX_DMA = (UINT32)(UINT8 *)endpRXbuff;

    return( ERR_SUCCESS );

}

/*******************************************************************************
* Function Name  : USBSS_HUB_Main_Process
* Description    : HUB Operations and Recursive Programs
* Input          : depth       - hub depth
*                  port        - hub port
*                  uplevelport - The port number of the upper level HUB
*                  route_string --- USB3.0 Route string
* Return         : Operational state
*******************************************************************************/
UINT8  USBSS_HUB_Main_Process( UINT8 depth,UINT8 addr ,UINT8 uplevelport,UINT32 route_string,UINT8 portnum)
{
    UINT8 i,ret;
    USB_HUB_SaveData hubdata;

      for( i = 0; i != portnum; i++ )
      {
          USBSSH->UH_TX_DMA = (UINT32)(UINT8 *)endpTXbuff;
          USBSSH->UH_RX_DMA = (UINT32)(UINT8 *)endpRXbuff;

          ss_hub_info[depth].devaddr = addr;

          AssignHubData(&hubdata,depth,uplevelport,i+1,ss_hub_info[depth]);
          ret = SearchHubData(Hub_LinkHead , &hubdata);//Determine whether the current node has saved data

          if( ret == 0 )
          {
              ss_hub_info[depth] = hubdata.HUB_Info;//Replace if there is saved data
          }
          else
          {
              memset( &ss_hub_info[depth].portD[i],0,sizeof( ss_hub_info[depth].portD[i] ) );//Clear without saved data to prevent the use of previous HUB data
          }

          USBSS->LINK_ROUTE_STRING = route_string;
          USB30H_Set_Address(addr);
          USBSS_HUBCheckPortConnect( depth, i );

          if( ss_hub_info[depth].portD[i].portpchangefield )
          {
              if( ss_hub_info[depth].portD[i].status == HUB_ERR_CONNECT )//device connect
              {
                  mDelaymS(5);
                  ret = USBSS_HUBHostEnum( depth,Test_Buf,i,route_string );
                  if( ret == ERR_SUCCESS )
                  {
                      ss_hub_info[depth].portD[i].status = HUB_ERR_SCUESS;          //Enumeration successful
                      AssignHubData(&hubdata,depth,uplevelport,i+1,ss_hub_info[depth]);
                      ret = InsertHubData(Hub_LinkHead,hubdata);
                      if( ret == 0 )
                      {
                          printf("#Save: depth-%x,uplevelport-%x,port-%x,Status-%x,hubaddr-%x#\n",depth,uplevelport,i+1,ss_hub_info[depth].portD[i].status,ss_hub_info[depth].portD[i].addr);
                      }
                  }
                  else
                  {
                      ss_hub_info[depth].portD[i].status = HUB_ERR_DISCONNECT;   //Enumeration failed. Re enumerating
                  }
              }
              else if( (ss_hub_info[depth].portD[i].status ) == PORT_DISCONNECT )//Device disconnection requires clearing variables
              {
                  AssignHubData(&hubdata,depth,uplevelport,i+1,ss_hub_info[depth]);
                  if( hubdata.HUB_Info.portD[i].devicetype == 0x09 )//If it is a HUB, it is necessary to determine whether there are subordinate nodes when deleting, and if so, it needs to be deleted
                  {
                      Hublink_finesubstructures(hubdata);
                  }

                  ret = DeleteHubData(Hub_LinkHead,hubdata);//Delete linked list nodes
                  if( ret == 0 )
                  {
                      printf("#Delete: depth-%x,uplevelport-%x,port-%x#\n",depth,uplevelport,i+1);
                      USB_DelAddressNumber(ss_hub_info[depth].portD[i].addr);//Clear Address
                  }

                  ret = U30HOST_ClearPortFeatrue_Process( depth,i+1 );           //Clear device status
                  memset( &ss_hub_info[depth].portD[i],0x00,sizeof( ss_hub_info[depth].portD[i] ) );    //Clear all states
                  printf("hub_device_disconnect_port=%02x\r\n",i+1);
              }
              else
              {
                  ret = U30HOST_ClearPortFeatrue_Process( depth,i+1 );       //Clear the device status. All status needs to be cleared
                  if( ret != USB_INT_SUCCESS )return ret;
                  ret = U30HOST_GetPortStstus( depth,i+1 );                  //Get it again
                  if( ret != USB_INT_SUCCESS )return ret;
              }
          }

          if( ss_hub_info[depth].portD[i].status == HUB_ERR_SCUESS )//Device port operation successful
          {
              if( ss_hub_info[depth].portD[i].devicetype == 0x09 ) //HUB
              {
                  Global_Index++;
                  ret = USBSS_HUB_Main_Process( Global_Index ,ss_hub_info[depth].portD[i].addr,i+1, ((i+1)<<((depth)*4))|route_string,ss_hub_info[depth].numofport);
                  Global_Index--;
                  if( ret != ERR_SUCCESS )return ret;
              }
              else//Other device types
              {

              }
          }

          mDelaymS(2);
      }

      return ERR_SUCCESS;
}

