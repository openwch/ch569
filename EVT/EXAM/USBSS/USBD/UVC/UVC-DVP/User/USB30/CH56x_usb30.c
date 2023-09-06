/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "CH56x_usb30.h"
#include "CH56x_usb20.h"
#include "usb30_desc.h"
#include "UVCLIB.H"

/* Global Variable */
UINT8V      tx_lmp_port = 0;
UINT8V      Link_sta = 0;
UINT32      SetupLen = 0;
UINT8       SetupReqCode = 0;
PUINT8      pDescr;
__attribute__ ((aligned(16))) UINT8 endp0RTbuff[512] __attribute__((section(".DMADATA")));  //Endpoint 0 data send / receive buffer
__attribute__ ((aligned(16))) UINT8 endp1RTbuff[1024] __attribute__((section(".DMADATA"))); //Endpoint 1 data send / receive buffer
/*******************************************************************************
 * @fn      USB30_BUS_RESET
 *
 * @brief   USB3.0 bus reset
 *
 * @return  None
 */
void USB30_BUS_RESET()
{
    R8_SAFE_ACCESS_SIG = 0x57; // enable safe access mode
    R8_SAFE_ACCESS_SIG = 0xa8;
    R8_RST_WDOG_CTRL = 0x40 | RB_SOFTWARE_RESET;
}

/*******************************************************************************
 * @fn      USB30D_init
 *
 * @brief   USB3.0 initialization
 *
 * @return   None
 */
void USB30D_init(FunctionalState sta)
{
    UINT16 i, s;
    if(sta)
    {
        USB30_Device_Init();
        USBSS->UEP_CFG = EP0_R_EN | EP0_T_EN  | EP1_T_EN ; // set end point rx/tx enable
        USBSS->UEP0_DMA = (UINT32)(UINT8 *)endp0RTbuff;
        USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)endp1RTbuff;
        USB30_ISO_Setendp(endp_1|endp_in,ENABLE);
    }
    else
    {
        USB30_Switch_Powermode(POWER_MODE_2);
        USBSS->LINK_CFG = PIPE_RESET | LFPS_RX_PD;
        USBSS->LINK_CTRL = GO_DISABLED | POWER_MODE_3;
        USBSS->LINK_INT_CTRL = 0;
        USBSS->USB_CONTROL = USB_FORCE_RST | USB_ALL_CLR;
    }

}
/*******************************************************************************
 * @fn      USBSS_IRQHandler
 *
 * @brief   USB3.0 Interrupt Handler.
 *
 * @return  None
 */
void USBSS_IRQHandler(void)
{
    USB30_IRQHandler();
}

/*******************************************************************************
 * @fn      TMR0_IRQHandler
 *
 * @brief   USB3.0 Connection failure timeout processing
 *
 * @return  None
 */
void TMR0_IRQHandler()
{
    R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;
    PRINT("TMR0_IRQHandler\n");
    if(Link_sta == 1)
    {
        Link_sta = 0;
        PFIC_DisableIRQ(USBSS_IRQn);
        PFIC_DisableIRQ(LINK_IRQn);
        USB30D_init(DISABLE);
        PRINT("USB3.0 disable\n");
        return;
    }
    if(Link_sta != 3)
    {
        PFIC_DisableIRQ(USBSS_IRQn);
        PFIC_DisableIRQ(LINK_IRQn);
        USB30D_init(DISABLE);
        R32_USB_CONTROL = 0;
        PFIC_EnableIRQ(USBHS_IRQn);
        USB20_Device_Init(ENABLE);
    }
    Link_sta = 1;
    R8_TMR0_INTER_EN = 0;
    PFIC_DisableIRQ(TMR0_IRQn);
    R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
}

/*******************************************************************************
 * @fn      LINK_IRQHandler
 *
 * @brief   USB3.0 Link Interrupt Handler.
 *
 * @return  None
 */
void LINK_IRQHandler() //USBSS link interrupt service
{
    if(USBSS->LINK_INT_FLAG & LINK_Ux_EXIT_FLAG) // device enter U2
    {
        USBSS->LINK_CFG = CFG_EQ_EN | DEEMPH_CFG | TERM_EN;
        USB30_Switch_Powermode(POWER_MODE_0);
        USBSS->LINK_INT_FLAG = LINK_Ux_EXIT_FLAG;
    }
    if(USBSS->LINK_INT_FLAG & LINK_RDY_FLAG) // POLLING SHAKE DONE
    {
        USBSS->LINK_INT_FLAG = LINK_RDY_FLAG;
        if(tx_lmp_port) // LMP, TX PORT_CAP & RX PORT_CAP
        {
            USBSS->LMP_TX_DATA0 = LINK_SPEED | PORT_CAP | LMP_HP;
            USBSS->LMP_TX_DATA1 = UP_STREAM | NUM_HP_BUF;
            USBSS->LMP_TX_DATA2 = 0x0;
            tx_lmp_port = 0;
        }
        /*Successful USB3.0 communication*/
        Link_sta = 3;
        PFIC_DisableIRQ(TMR0_IRQn);
        R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
        R8_TMR0_INTER_EN = 0;
        PFIC_DisableIRQ(USBHS_IRQn);
        USB20_Device_Init(DISABLE);
    }

    if(USBSS->LINK_INT_FLAG & LINK_INACT_FLAG)
    {
        USBSS->LINK_INT_FLAG = LINK_INACT_FLAG;
        USB30_Switch_Powermode(POWER_MODE_2);
    }
    if(USBSS->LINK_INT_FLAG & LINK_DISABLE_FLAG) // GO DISABLED
    {
        USBSS->LINK_INT_FLAG = LINK_DISABLE_FLAG;
        Link_sta = 1;
        USB30D_init(DISABLE);
        PFIC_DisableIRQ(USBSS_IRQn);
        R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
        R8_TMR0_INTER_EN = 0;
        PFIC_DisableIRQ(TMR0_IRQn);
        PFIC_EnableIRQ(USBHS_IRQn);
        USB20_Device_Init(ENABLE);
    }
    if(USBSS->LINK_INT_FLAG & LINK_RX_DET_FLAG)
    {
        USBSS->LINK_INT_FLAG = LINK_RX_DET_FLAG;
        USB30_Switch_Powermode(POWER_MODE_2);
    }
    if(USBSS->LINK_INT_FLAG & TERM_PRESENT_FLAG) // term present , begin POLLING
    {
        USBSS->LINK_INT_FLAG = TERM_PRESENT_FLAG;
        if(USBSS->LINK_STATUS & LINK_PRESENT)
        {
            USB30_Switch_Powermode(POWER_MODE_2);
            USBSS->LINK_CTRL |= POLLING_EN;
        }
        else
        {
            USBSS->LINK_INT_CTRL = 0;
            mDelayuS(2);
            USB30_BUS_RESET();
        }
    }
    if(USBSS->LINK_INT_FLAG & LINK_TXEQ_FLAG) // POLLING SHAKE DONE
    {
        tx_lmp_port = 1;
        USBSS->LINK_INT_FLAG = LINK_TXEQ_FLAG;
        USB30_Switch_Powermode(POWER_MODE_0);
    }
    if(USBSS->LINK_INT_FLAG & WARM_RESET_FLAG)
    {
        USBSS->LINK_INT_FLAG = WARM_RESET_FLAG;
        USB30_Switch_Powermode(POWER_MODE_2);
        USBSS->LINK_CTRL |= TX_WARM_RESET;
        while(USBSS->LINK_STATUS & RX_WARM_RESET);
        USBSS->LINK_CTRL &= ~TX_WARM_RESET;
        mDelayuS(2);
        USB30_BUS_RESET();
        USB30_Device_Setaddress(0);

    }
    if(USBSS->LINK_INT_FLAG & HOT_RESET_FLAG) //The host may send hot reset,Note the configuration of the endpoint
    {
        USBSS->USB_CONTROL |= 1 << 31;
        USBSS->LINK_INT_FLAG = HOT_RESET_FLAG; // HOT RESET begin
        USBSS->UEP0_TX_CTRL = 0;
        USB30_IN_Set(ENDP_1, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_2, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_3, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_4, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_5, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_6, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_7, DISABLE, NRDY, 0, 0);
        USB30_OUT_Set(ENDP_1, NRDY, 0);
        USB30_OUT_Set(ENDP_2, NRDY, 0);
        USB30_OUT_Set(ENDP_3, NRDY, 0);
        USB30_OUT_Set(ENDP_4, NRDY, 0);
        USB30_OUT_Set(ENDP_5, NRDY, 0);
        USB30_OUT_Set(ENDP_6, NRDY, 0);
        USB30_OUT_Set(ENDP_7, NRDY, 0);

        USB30_Device_Setaddress(0);
        USBSS->LINK_CTRL &= ~TX_HOT_RESET;     // HOT RESET end
    }
    if(USBSS->LINK_INT_FLAG & LINK_GO_U1_FLAG) // device enter U1
    {
        USB30_Switch_Powermode(POWER_MODE_1);
        USBSS->LINK_INT_FLAG = LINK_GO_U1_FLAG;
    }
    if(USBSS->LINK_INT_FLAG & LINK_GO_U2_FLAG) // device enter U2
    {
        USB30_Switch_Powermode(POWER_MODE_2);
        USBSS->LINK_INT_FLAG = LINK_GO_U2_FLAG;
    }
    if(USBSS->LINK_INT_FLAG & LINK_GO_U3_FLAG) // device enter U2
    {
        USB30_Switch_Powermode(POWER_MODE_2);
        USBSS->LINK_INT_FLAG = LINK_GO_U3_FLAG;
    }
}

/*******************************************************************************
 * @fn      USB30_NonStandardReq
 *
 * @brief   USB3.0 Nonstandard request processing function
 *
 * @return  Length
 */
UINT16 USB30_NonStandardReq()
{
    SetupReqCode = UsbSetupBuf->bRequest;
    SetupLen = UsbSetupBuf->wLength;
    UINT16 len = 0xFFFF;
    /*Upload data*/
    if(UsbSetupBuf->bRequestType & 0x80){
        len = UVC_NonStandardReq(&pDescr);
        if(len != 0xFFFF){
                    len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen;
                    memcpy(endp0RTbuff, pDescr,len );
                    SetupLen -= len;
                    pDescr += len;
                }
    }else{
        /*set cur*/
        if(UsbSetupBuf->bRequestType==0x21 && UsbSetupBuf->bRequest == 0x01){
            len = 0;
        }
    }
    return len;

}
/*******************************************************************************
 * @fn      USB30_StandardReq
 *
 * @brief   USB3.0 Standard request
 *
 * @return  Length
 */
UINT16 USB30_StandardReq()
{
    SetupReqCode = UsbSetupBuf->bRequest;
    SetupLen = UsbSetupBuf->wLength;
    UINT16 len = 0;
#if 0
    printf("S:%02x %02x %02x %02x %02x %02x %02x %02x\n", endp0RTbuff[0], endp0RTbuff[1],
            endp0RTbuff[2], endp0RTbuff[3], endp0RTbuff[4], endp0RTbuff[5],
            endp0RTbuff[6], endp0RTbuff[7]);
#endif
    if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD)
    {
        len = USB30_NonStandardReq();
    }
    else
    {
        switch(SetupReqCode)
        {
            case USB_GET_DESCRIPTOR:
                switch(UsbSetupBuf->wValueH)
                {
                        case USB_DESCR_TYP_DEVICE:
                                if(SetupLen>SIZE_DEVICE_DESC) SetupLen  = SIZE_DEVICE_DESC;
                                pDescr = (PUINT8)DeviceDescriptor;
                                break;
                        case USB_DESCR_TYP_CONFIG:
                                if(SetupLen > SIZE_CONFIG_DESC) SetupLen = SIZE_CONFIG_DESC;
                                pDescr = (PUINT8)ConfigDescriptor;
                                break;
                        case USB_DESCR_TYP_BOS:
                                if(SetupLen > SIZE_BOS_DESC) SetupLen = SIZE_BOS_DESC;
                                pDescr = (PUINT8)BOSDescriptor;
                                break;
                        case    USB_DESCR_TYP_STRING:
                            switch(UsbSetupBuf->wValueL)
                                {
                                    case USB_DESCR_LANGID_STRING:
                                        if(SetupLen > SIZE_STRING_LANGID) SetupLen = SIZE_STRING_LANGID;
                                        pDescr = (PUINT8)StringLangID;
                                        break;
                                    case USB_DESCR_VENDOR_STRING:
                                        if(SetupLen > SIZE_STRING_VENDOR) SetupLen = SIZE_STRING_VENDOR;
                                        pDescr = (PUINT8)StringVendor;
                                        break;
                                    case USB_DESCR_PRODUCT_STRING:
                                        if(SetupLen > SIZE_STRING_PRODUCT) SetupLen = SIZE_STRING_PRODUCT;
                                        pDescr = (PUINT8)StringProduct;
                                        break;
                                    case USB_DESCR_SERIAL_STRING:
                                        if(SetupLen > SIZE_STRING_SERIAL) SetupLen = SIZE_STRING_SERIAL;
                                        pDescr = (PUINT8)StringSerial;
                                        break;
                                    case USB_DESCR_OS_STRING:
                                        if(SetupLen >SIZE_STRING_OS) SetupLen = SIZE_STRING_OS;
                                        pDescr = (PUINT8)OSStringDescriptor;
                                        break;
                                    default:
                                        len = USB_DESCR_UNSUPPORTED;
                                        SetupReqCode = INVALID_REQ_CODE;
                                        break;
                                }
                                break;
                        default:
                            len = USB_DESCR_UNSUPPORTED;
                            SetupReqCode = INVALID_REQ_CODE;
                            break;
                }
                if(len != USB_DESCR_UNSUPPORTED){
                    len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen;
                    memcpy(endp0RTbuff, pDescr,len );
                    SetupLen -= len;
                    pDescr += len;
                }
                break;
            case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;
                        break;
            case 0x31:
                        SetupLen = UsbSetupBuf->wValueL;
                        break;
            case 0x30:
                        break;
            case USB_SET_CONFIGURATION:
                        break;
            case USB_GET_STATUS:
                        len=2;
                        endp0RTbuff[0]=0x00;
                        endp0RTbuff[1]=0x00;
                        SetupLen = 0;
                        break;
            case USB_CLEAR_FEATURE:
                        switch( UsbSetupBuf->wIndexL )
                        {
                            case 0x82:
                                USB30_IN_ClearIT( ENDP_2 );
                                USB30_IN_Set( ENDP_2, DISABLE , STALL , 0, 0 );
                                break;
                            case 0x02:
                                USB30_OUT_ClearIT( ENDP_2 );
                                USB30_OUT_Set( ENDP_2 , STALL , 0 );
                                break;
                            case 0x81:
                                USB30_IN_ClearIT( ENDP_1 );
                                USB30_IN_Set( ENDP_1, DISABLE , STALL , 0, 0 );
                                break;
                            case 0x01:
                                USB30_OUT_ClearIT( ENDP_1 );
                                USB30_OUT_Set( ENDP_1 , STALL , 0 );
                                break;
                            default:
                                SetupLen = USB_DESCR_UNSUPPORTED;
                                break;
                        }
                        break;
            case USB_SET_FEATURE:
                        break;
            case USB_SET_INTERFACE:
                        SS_CtrlCamera();
                        break;
            default:
                    len =USB_DESCR_UNSUPPORTED;
                    SetupReqCode = INVALID_REQ_CODE;
                    printf(" stall \n");
                        break;
        }
    }
    return len;
}

/*******************************************************************************
 * @fn      EP0_IN_Callback
 *
 * @brief   USB3.0 Endpoint 0 IN transaction callback
 *
 * @return  Send length
 */
UINT16 EP0_IN_Callback(void)
{

    UINT16 len = 0;
    switch(SetupReqCode)
    {
        case USB_GET_DESCRIPTOR:
             len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen;
             memcpy(  endp0RTbuff, pDescr,len );
             SetupLen -= len;
             pDescr += len;
             break;
    }
    return len;
}

/*******************************************************************************
 * @fn      EP0_OUT_Callback
 *
 * @brief   USB3.0 Endpoint 0 OUT transaction callback
 *
 * @return  None
 */
UINT16 EP0_OUT_Callback()
{
    UINT16 rx_len = 0;
    static UINT8 format = 0;
    static UINT8 frame = 0;
    USB30_OUT_Status( ENDP_0,NULL, &rx_len, NULL );

    /* Switch video format or resolution */
    if(rx_len == 0x1A)
    {
        if( (format != *(UINT8 *)(endp0RTbuff+2)) || (frame != *(UINT8 *)(endp0RTbuff+3)))
        {
            format = *(UINT8 *)(endp0RTbuff+2);
            frame = *(UINT8 *)(endp0RTbuff+3);
            Get_Curr[2] = format;
            Get_Curr[3] = frame;
            Formatchange_flag = *(UINT8 *)(endp0RTbuff+2);
            OV2640_Format_Mode(Formatchange_flag);
            OV2640_Change_Resolution(format,frame);
        }
    }

    return 0;
}

/*******************************************************************************
 * @fn      USB30_Setup_Status
 *
 * @brief   USB3.0 Control transfer status stage callback
 *
 * @return  None
 */
void USB30_Setup_Status( void)
{
    switch(SetupReqCode)
    {
        case USB_SET_ADDRESS:
            USB30_Device_Setaddress(SetupLen );
             break;
        case 0x81:
            ClearError();
            break;
        case 0x31:
            USB30_ISO_Setdelay(SetupLen);
            break;
    }
}


/***************Endpointn IN Transaction Processing*******************/
void EP1_IN_Callback()
{
    SS_Endp1_Hander();
}

void EP2_IN_Callback()
{

}
void EP3_IN_Callback()
{

}
void EP4_IN_Callback()
{

}
void EP5_IN_Callback()
{

}
void EP6_IN_Callback()
{

}
void EP7_IN_Callback()
{

}

/***************Endpointn OUT Transaction Processing*******************/
void EP1_OUT_Callback()
{

}
void EP2_OUT_Callback()
{

}
void EP3_OUT_Callback()
{

}
void EP4_OUT_Callback()
{

}
void EP5_OUT_Callback()
{

}
void EP6_OUT_Callback()
{

}
void EP7_OUT_Callback()
{

}

/*******************************************************************************
 * @fn      USB30_ITP_Callback
 *
 * @brief   USB3.0 ITP callback function
 *
 * @return  None
 */
void USB30_ITP_Callback(UINT32 ITPCounter)
{

}

