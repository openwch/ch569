/********************************** (C) COPYRIGHT *******************************
* File Name          : HSPI.C
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/******************************************************************************/
#include "MAIN.h"
#include "CH56x_usb30.h"
/******************************************************************************/
UINT8V  HSPI_Tx_PackCnt = 0x00;                                                 /* HSPI Send packet count */
UINT8V  HSPI_Tx_AddrTog = 0x00;                                                 /* HSPI Send packet address synchronization count */
UINT8V  HSPI_Tx_Status = 0x00;                                                  /* HSPI Send status */
UINT8V  HSPI_Tx_ErrFlag = 0x00;                                                 /* HSPI Send error flag*/
UINT8V  HSPI_Tx_BurstPackNum = 0x00;                                            /* HSPI Number of burst packets sent this time */
UINT8V  HSPI_Rx_PackCnt = 0x00;                                                 /* HSPI Received packet count */
UINT8V  HSPI_Rx_AddrTog = 0x00;                                                 /* HSPI Receive packet address synchronization count */
UINT8V  HSPI_Rx_ErrFlag = 0x00;                                                 /* HSPI Receive error flag */
UINT32V HSPI_Tx_Data_LoadAddr = 0x00;                                           /* HSPI Send data load pointer address*/
UINT32V HSPI_Tx_Data_DealAddr = 0x00;                                           /* HSPI Send data processing pointer address */
UINT32V HSPI_Tx_Data_RemainLen = 0x00;                                          /* HSPI Remaining length of sending data */
UINT16V HSPI_Tx_LastPackLen = 0x00;                                             /* HSPI Send burst end packet length */
UINT8V  HSPI_Int_En_Save = 0x00;                                                /* HSPI Interrupt enable save */
UINT32V HSPI_Rx_Data_LoadAddr = 0x00;                                           /* HSPI Receive data loading pointer address */
UINT32V HSPI_Rx_Data_DealAddr = 0x00;                                           /* HSPI Receive data processing pointer address */
UINT32V HSPI_Rx_Data_RemainLen = 0x00;                                          /* HSPI Remaining length of received data */
UINT8V  HSPI_Rx_Notice_Status = 0x00;                                           /* HSPI Receive notification status */
UINT8V  USB_Stop_UpFlag = 0x00;                                                 /* USB Stop uploading flag */
UINT16V USB3_2_Switchover = 0x00;                                               /* USB mode switch */

/*******************************************************************************
* Function Name  : HSPI_GPIO_Init
* Description    : HSPI Interface related GPIO initialization
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HSPI_GPIO_Init( void )
{
    //TX GPIO PA9 11 21 push-pull output
    R32_PA_DIR |= (1<<9) | (1<<11) | (1<<21);

    //clk 16mA
    R32_PA_DRV |= (1<<11);

    //Rx GPIO PA10 push-pull output
    R32_PA_DIR |= (1<<10);
}

/*******************************************************************************
* Function Name  : HSPI_Init
* Description    : HSPI Interface initialization
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HSPI_Init( void )
{
    UINT16 i;

    /* Configure working mode */
    R8_HSPI_CFG &= ~( RB_HSPI_MODE | RB_HSPI_MSK_SIZE );
#if( DEF_HSPI_MODE == DEF_HSPI_HOST_MODE )
       R8_HSPI_CFG |= RB_HSPI_MODE;
#else
    R8_HSPI_CFG &= ~( RB_HSPI_MODE );
#endif

    /* Configure data bits */
#if( DEF_HSPI_DATASIZE == DEF_HSPI_DATASIZE_8 )
    R8_HSPI_CFG |= RB_HSPI_DAT8_MOD;

#elif( DEF_HSPI_DATASIZE == DEF_HSPI_DATASIZE_16 )
    R8_HSPI_CFG |= RB_HSPI_DAT16_MOD;

#elif( DEF_HSPI_DATASIZE == DEF_HSPI_DATASIZE_32 )
    R8_HSPI_CFG |= RB_HSPI_DAT32_MOD;
#endif

    /* ACk mode 1 */
    R8_HSPI_CFG |= RB_HSPI_HW_ACK;

    /* Tx ToG En */
    R8_HSPI_CFG |= RB_HSPI_TX_TOG_EN;

    /* Rx ToG En */
    R8_HSPI_CFG |= RB_HSPI_RX_TOG_EN;

    /* This bit represents the request for dual DMA */
    R8_HSPI_AUX |= ( 1 << 5 );

    /* TX Falling edge sampling */
    R8_HSPI_AUX |= RB_HSPI_TCK_MOD;

    /* Hardware Auto ACK time*/
    R8_HSPI_AUX |= RB_HSPI_ACK_TX_MOD;

    /* delay time(delay 2T) */
    R8_HSPI_AUX &= ~RB_HSPI_ACK_CNT_SEL;

    /* Clear ALL_CLR And TRX_RST */
    R8_HSPI_CTRL &= ~( RB_HSPI_ALL_CLR | RB_HSPI_TRX_RST );

    /* DMA TX/RX Addr0/1 */
    R32_HSPI_TX_ADDR0 = DEF_HPSI_DMA_TX_ADDR0;
    R32_HSPI_TX_ADDR1 = DEF_HPSI_DMA_TX_ADDR0 + DEF_HSPI_DMA_PACK_LEN;
    R32_HSPI_RX_ADDR0 = DEF_HPSI_DMA_RX_ADDR0;
    R32_HSPI_RX_ADDR1 = DEF_HPSI_DMA_RX_ADDR0 + DEF_HSPI_DMA_PACK_LEN;

    /* DMA TX/RX Len */
    R16_HSPI_DMA_LEN0 = DEF_HSPI_DMA_PACK_LEN - 1;
    R16_HSPI_DMA_LEN1 = DEF_HSPI_DMA_PACK_LEN - 1;

//  R16_HSPI_BURST_CFG |= (1<<8)|RB_HSPI_BURST_EN;
    if( ( R8_HSPI_RX_SC & RB_HSPI_RX_TOG ) == RB_HSPI_RX_TOG )
    {
        R8_HSPI_RX_SC = RB_HSPI_RX_TOG;
    }
    R8_HSPI_RX_SC = 0x00;
    if( ( R8_HSPI_TX_SC & RB_HSPI_TX_TOG ) == RB_HSPI_TX_TOG )
    {
        R8_HSPI_TX_SC = RB_HSPI_TX_TOG;
    }
    R8_HSPI_TX_SC = 0x00;

    /* Enable HSPI  DMA */
    R8_HSPI_CTRL |= RB_HSPI_ENABLE | RB_HSPI_DMA_EN;

    /* Configure TX Customization  Header */
    R32_HSPI_UDF0 = 0x00;
    R32_HSPI_UDF1 = 0x00;

    /* Enable Interupt */
    R8_HSPI_INT_EN |= RB_HSPI_IE_T_DONE;
    R8_HSPI_INT_EN |= RB_HSPI_IE_FIFO_OV;
    R8_HSPI_INT_EN |= RB_HSPI_IE_B_DONE;
    R8_HSPI_INT_EN |= RB_HSPI_IE_R_DONE;
    HSPI_Int_En_Save = R8_HSPI_INT_EN;
    R8_HSPI_INT_FLAG = 0x0F;

    PFIC_EnableIRQ( HSPI_IRQn );

    /* Initialize related variables */
    HSPI_Tx_PackCnt = 0x00;
    HSPI_Rx_PackCnt = 0x00;

    HSPI_Tx_Data_LoadAddr = DEF_HPSI_DMA_TX_ADDR0;
    HSPI_Tx_Data_DealAddr = DEF_HPSI_DMA_TX_ADDR0;
    HSPI_Tx_Data_RemainLen = 0x00;

    HSPI_Rx_Data_LoadAddr = DEF_HPSI_DMA_RX_ADDR0;
    HSPI_Rx_Data_DealAddr = DEF_HPSI_DMA_RX_ADDR0;
    HSPI_Rx_Data_RemainLen = 0x00;
}

/*******************************************************************************
* Function Name  : HSPI_IRQHandler
* Description    : HSPI Interface interrupt handling
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HSPI_IRQHandler( void ) __attribute__((interrupt("WCH-Interrupt-fast")));
void HSPI_IRQHandler( void )
{
    UINT32V i,j;
    UINT32V addr;
    UINT32V packnum;
    UINT32V len ;
    if( R8_HSPI_INT_FLAG & RB_HSPI_IF_T_DONE )
    {
        /* Single packet transmission completion interrupt */
        R8_HSPI_INT_FLAG = RB_HSPI_IF_T_DONE;

        HSPI_Tx_PackCnt++;
        HSPI_Tx_AddrTog++;
        if( HSPI_Tx_AddrTog % 2 )
        {
            R32_HSPI_TX_ADDR0 += ( DEF_HSPI_DMA_PACK_LEN * 2 );
        }
        else
        {
            R32_HSPI_TX_ADDR1 += ( DEF_HSPI_DMA_PACK_LEN * 2 );
        }
	}
    else if( R8_HSPI_INT_FLAG & RB_HSPI_IF_R_DONE )
	{
        /* Single packet receiving completion interrupt */
        R8_HSPI_INT_FLAG = RB_HSPI_IF_R_DONE;

        /* The slave pulls up the HRTS pin to notify the host to suspend the next packet of data transmission */
        PIN_HRTS_HIGH( );

        if( R8_HSPI_RTX_STATUS == 0x00 || R8_HSPI_RTX_STATUS == 0x01 )
        {
            HSPI_Rx_PackCnt++;
            if( HSPI_Rx_PackCnt % 2 )
            {
                R32_HSPI_RX_ADDR0 += ( DEF_HSPI_DMA_PACK_LEN * 2 );
            }
            else
            {
                R32_HSPI_RX_ADDR1 += ( DEF_HSPI_DMA_PACK_LEN * 2 );
            }

            if(  ( R32_HSPI_UDF0 & ( 1 << 13 ) ) || ( R32_HSPI_UDF1 & ( 1 << 13 ) ) )
            {
                if( R32_HSPI_UDF0 & ( 1 << 13 ) )
                {
                    HSPI_Rx_Data_LoadAddr += DEF_HSPI_DMA_PACK_LEN;
                    if( ( R32_HSPI_UDF0 & 0x00000FFF ) == 0x00 )
                    {
                        HSPI_Rx_Data_RemainLen += DEF_HSPI_DMA_PACK_LEN;
                    }
                    else
                    {
                        HSPI_Rx_Data_RemainLen += ( R32_HSPI_UDF0 & 0x00000FFF );
                        HSPI_RX_StopFlag = 1;
                    }
                }
                else if( R32_HSPI_UDF1 & ( 1 << 13 ) )
                {
                    HSPI_Rx_Data_RemainLen +=  DEF_HSPI_DMA_PACK_LEN;
                    HSPI_Rx_Data_LoadAddr += ( 2 * DEF_HSPI_DMA_PACK_LEN );
                    if( ( R32_HSPI_UDF1 & 0x00000FFF ) == 0x00 )
                    {
                        HSPI_Rx_Data_RemainLen += DEF_HSPI_DMA_PACK_LEN;
                    }
                    else
                    {
                        HSPI_Rx_Data_RemainLen += ( R32_HSPI_UDF1 & 0x00000FFF );
                        HSPI_RX_StopFlag = 1;
                    }
                }

                if( HSPI_Rx_Data_LoadAddr >= DEF_HPSI_RX_DMA_ADDR_MAX )
                {
                    HSPI_Rx_Data_LoadAddr = DEF_HPSI_DMA_RX_ADDR0;
                }
                R32_HSPI_RX_ADDR0 = HSPI_Rx_Data_LoadAddr;
                R32_HSPI_RX_ADDR1 = HSPI_Rx_Data_LoadAddr + DEF_HSPI_DMA_PACK_LEN;


                if( ( R8_HSPI_RX_SC & RB_HSPI_RX_TOG ) == RB_HSPI_RX_TOG )
                {
                    R8_HSPI_RX_SC = RB_HSPI_RX_TOG;
                }
                R8_HSPI_RX_SC = 0x00;
                HSPI_Rx_PackCnt = 0x00;

                R32_HSPI_UDF0 = 0x00;
                R32_HSPI_UDF1 = 0x00;

                HSPI_Rx_Notice_Status = 0x01;
            }
        }
        else if( R8_HSPI_RTX_STATUS & RB_HSPI_CRC_ERR )
        {
            /* CRC Verification error */
#if( DEF_FUN_DEBUG_EN == 0x01 )
            DUG_PRINTF("CRC Err: %d\r\n",(UINT16)HSPI_Rx_PackCnt);
            DUG_PRINTF("R8_HSPI_RX_SC:%d\r\n",R8_HSPI_RX_SC);
#endif
            R32_HSPI_RX_ADDR0 = DEF_HPSI_DMA_RX_ADDR0;
            R32_HSPI_RX_ADDR1 = DEF_HPSI_DMA_RX_ADDR0 + DEF_HSPI_DMA_PACK_LEN;
            if( ( R8_HSPI_RX_SC & RB_HSPI_RX_TOG ) == RB_HSPI_RX_TOG )
            {
                R8_HSPI_RX_SC = RB_HSPI_RX_TOG;
            }

            R8_HSPI_RX_SC = 0x00;
            HSPI_Rx_PackCnt = 0x00;

            HSPI_Rx_Notice_Status = 0x01;
        }
        else if( R8_HSPI_RTX_STATUS & RB_HSPI_NUM_MIS )
        {
            /* Receive serial number does not match */
#if( DEF_FUN_DEBUG_EN == 0x01 )
            DUG_PRINTF("NUM_MIS Err: %d\r\n",(UINT16)HSPI_Rx_PackCnt);
            DUG_PRINTF("R8_HSPI_RX_SC:%d\r\n",R8_HSPI_RX_SC);
#endif
            R32_HSPI_RX_ADDR0 = DEF_HPSI_DMA_RX_ADDR0;
            R32_HSPI_RX_ADDR1 = DEF_HPSI_DMA_RX_ADDR0 + DEF_HSPI_DMA_PACK_LEN;
            if( ( R8_HSPI_RX_SC & RB_HSPI_RX_TOG ) == RB_HSPI_RX_TOG )
            {
                R8_HSPI_RX_SC = RB_HSPI_RX_TOG;
            }

            R8_HSPI_RX_SC = 0x00;
            HSPI_Rx_PackCnt = 0x00;
        }

	}
    else if( R8_HSPI_INT_FLAG & RB_HSPI_IF_B_DONE )
    {
        /* In burst mode, burst sequence packet transmission is completed */
        R8_HSPI_INT_FLAG = RB_HSPI_IF_B_DONE;

        if( HSPI_Tx_PackCnt != ( R16_HSPI_BURST_CFG >> 8 ) )
        {
#if( DEF_FUN_DEBUG_EN == 0x01 )
            DUG_PRINTF("BURST Tx Err:%d\r\n",HSPI_Tx_PackCnt);
#endif
            if( ( R8_HSPI_TX_SC & RB_HSPI_TX_TOG ) == RB_HSPI_TX_TOG )
            {
                R8_HSPI_TX_SC = RB_HSPI_TX_TOG;
            }
            R8_HSPI_TX_SC = 0x00;
            HSPI_Tx_PackCnt = 0x00;

            HSPI_Tx_Status = 0x00;

            if( ( PIN_HCTS_RD( ) == 0x00 ) && ( HSPI_Rx_PackCnt == 0x00 ) )
            {
                HSPI_Tx_Data_Deal( );
            }
        }
        else
        {
            if( ( R8_HSPI_TX_SC & RB_HSPI_TX_TOG ) == RB_HSPI_TX_TOG )
            {
                R8_HSPI_TX_SC = RB_HSPI_TX_TOG;
            }
            R8_HSPI_TX_SC = 0x00;
            HSPI_Tx_PackCnt = 0x00;

            HSPI_Tx_Status = 0x00;

            HSPI_Tx_Data_RemainLen -= HSPI_Tx_LastPackLen;

            HSPI_Tx_Data_DealAddr += ( ( HSPI_Tx_BurstPackNum - 1 ) * DEF_HSPI_DMA_PACK_LEN );
            if( ( HSPI_Tx_LastPackLen % 4096 ) == 0 )
            {
                HSPI_Tx_Data_DealAddr += DEF_HSPI_DMA_PACK_LEN;
            }
            else
            {
                HSPI_Tx_Data_DealAddr += ( HSPI_Tx_LastPackLen % 4096 ) / USB3_2_Switchover * USB3_2_Switchover;
            }

            if( HSPI_Tx_Data_DealAddr >= DEF_HPSI_TX_DMA_ADDR_MAX )
            {
                HSPI_Tx_Data_DealAddr = DEF_HPSI_DMA_TX_ADDR0;
            }

            R32_HSPI_UDF0 = 0x00;
            R32_HSPI_UDF1 = 0x00;
            if( ( PIN_HCTS_RD( ) == 0x00 ) && ( HSPI_Rx_PackCnt == 0x00 ) )
            {
                HSPI_Tx_Data_Deal( );
            }

        }

    }
	else if( R8_HSPI_INT_FLAG & RB_HSPI_IF_FIFO_OV )
	{
	    /* FIFO Overflow interrupt */
		R8_HSPI_INT_FLAG = RB_HSPI_IF_FIFO_OV;
#if( DEF_FUN_DEBUG_EN == 0x01 )
		DUG_PRINTF("FIFO OV\r\n");
#endif
	}

}

/*******************************************************************************
* Function Name  : HSPI_usb30_IN_handle
* Description    : HSPI USB30 Upload processing
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HSPI_usb30_IN_handle( void )
{
    UINT32 len;
    UINT32 remanlen;
    UINT32 packnum;
    UINT32 packlen;
    UINT32 offset;
    /* HSPI data is uploaded and processed through USB endpoint */
    if( Endp1_Up_Status == 0x00 )
    {
        /* Determine whether there is any data to upload */
        R8_HSPI_INT_EN = 0x00;
        R8_HSPI_INT_EN = 0x00;
        remanlen = HSPI_Rx_Data_RemainLen;
        R8_HSPI_INT_EN = HSPI_Int_En_Save;

        if( remanlen )
        {
            /* Calculate the length and number of packets uploaded this time */
            offset = ( DEF_HPSI_RX_DMA_ADDR_MAX - HSPI_Rx_Data_DealAddr );
            if( remanlen >= ( DEF_ENDP1_IN_BURST_LEVEL * 1024 ) )
            {
                if( offset >= ( DEF_ENDP1_IN_BURST_LEVEL * 1024 ) )
                {
                    Endp1_Up_LastPackLen = DEF_ENDP1_IN_BURST_LEVEL * 1024;
                    Endp1_Up_LastPackNum = DEF_ENDP1_IN_BURST_LEVEL;
                    len = 1024;
                }
                else
                {
                    Endp1_Up_LastPackLen = offset;
                    Endp1_Up_LastPackNum = Endp1_Up_LastPackLen / 1024;
                    len = Endp1_Up_LastPackLen % 1024;
                    if( len )
                    {
                        Endp1_Up_LastPackNum++;
                    }
                    else if( len == 0 )
                    {
                        len = 1024;
                    }
                }
            }
            else
            {
                if( offset >= remanlen )
                {
                    Endp1_Up_LastPackLen = remanlen;
                }
                else
                {
                    Endp1_Up_LastPackLen = offset;
                }
                Endp1_Up_LastPackNum = Endp1_Up_LastPackLen / 1024;
                len = Endp1_Up_LastPackLen % 1024;
                if( len )
                {
                    Endp1_Up_LastPackNum++;
                }
                else if( len == 0 )
                {
                    len = 1024;
                }
            }
              USB30_IN_ClearIT( ENDP_1 );
              USB30_IN_Set( ENDP_1, ENABLE , ACK , Endp1_Up_LastPackNum, len );
              USB30_Send_ERDY( ENDP_1 | IN , Endp1_Up_LastPackNum );

            Endp1_Up_Status = 0x01;
        }
    }

    R8_HSPI_INT_EN = 0x00;
    R8_HSPI_INT_EN = 0x00;
    R8_HSPI_INT_EN = 0x00;
    remanlen = HSPI_Rx_Data_RemainLen;
    R8_HSPI_INT_EN = HSPI_Int_En_Save;

    if( remanlen <= ( DEF_ENDP1_TX_BUF_LEN_BULK - DEF_HSPI_BULK_BPACK_LEN*2 ) && HSPI_RX_StopFlag == 0 )
    {
        if( HSPI_Rx_Notice_Status == 0x01 )
        {
            HSPI_Rx_Notice_Status = 0x00;
            if( USB_Stop_UpFlag == 0x00 )
            {
                PIN_HRTS_LOW( );
            }
        }
    }
}

/*******************************************************************************
* Function Name  : HSPI_usb30_OUT_handle
* Description    : HSPI USB30 Download processing
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HSPI_usb30_OUT_handle( void )
{
    UINT32 len;
    UINT32 remanlen;
    UINT32 packnum;
    UINT32 packlen;
    UINT32 offset;
    /*************************************************************************/
    /* USB endpoint downloading data sent through HSPI interface */
    R8_HSPI_INT_EN = 0x00;
    R8_HSPI_INT_EN = 0x00;
    R8_HSPI_INT_EN = 0x00;
    if( ( HSPI_Tx_Status == 0x00 ) && ( PIN_HCTS_RD( ) == 0x00 ) )
    {
        HSPI_Tx_Data_Deal( );
    }
    R8_HSPI_INT_EN = HSPI_Int_En_Save;

    /* Determine whether USB endpoint download is paused */
    if( Endp1_Down_Status )
    {
        /* Determine whether the current pause is a forced pause with non full packets or a pause with insufficient buffers */
        R8_HSPI_INT_EN = 0x00;
        R8_HSPI_INT_EN = 0x00;
        R8_HSPI_INT_EN = 0x00;
        remanlen = HSPI_Tx_Data_RemainLen;
        R8_HSPI_INT_EN = HSPI_Int_En_Save;
        if( USB_Down_StopFlag )
        {
            /* After all data is sent, enable USB download */
            if( remanlen == 0x00 )
            {
                USB_Down_StopFlag = 0x00;
                Endp1_Down_Status = 0x00;

                /* HSPI send buffer and USB endpoint receive buffer pointer back */
                HSPI_Tx_Data_LoadAddr = DEF_HPSI_DMA_TX_ADDR0;
                HSPI_Tx_Data_DealAddr = DEF_HPSI_DMA_TX_ADDR0;
                USBSS->UEP1_RX_DMA = HSPI_Tx_Data_LoadAddr;

                /* Notify the computer to continue downloading N packets of data */
                packnum = DEF_ENDP1_OUT_BURST_LEVEL;
                USB30_OUT_Set( ENDP_1, ACK, packnum );
                USB30_Send_ERDY( ENDP_1 | OUT , packnum );
            }
        }
        else
        {
            /* If there is enough space in the buffer, then enable USB download */
            if( remanlen <= ( DEF_ENDP1_RX_BUF_LEN -  DEF_ENDP1_OUT_BURST_LEVEL * 1024 * 2 ) )
            {
                Endp1_Down_Status = 0x00;

                /* Determine whether the buffer offset is sufficient, and if not, limit it */
                packnum = DEF_ENDP1_OUT_BURST_LEVEL;
                packlen = ( DEF_HPSI_TX_DMA_ADDR_MAX - HSPI_Tx_Data_LoadAddr );
                if( packlen < ( DEF_ENDP1_OUT_BURST_LEVEL * 1024 ) )
                {
                    packnum = packlen / 1024;
                }

                /* Notify the computer to continue downloading N packets of data */
                USB30_OUT_Set( ENDP_1, ACK, packnum );
                USB30_Send_ERDY( ENDP_1 | OUT , packnum );

            }
        }
    }
}

/*******************************************************************************
* Function Name  : HSPI_usb20_IN_handle
* Description    : HSPI USB20 Upload processing
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HSPI_usb20_IN_handle( void )
{
    UINT32 len;
    UINT32 remanlen;
    UINT32 packnum;
    UINT32 packlen;
    UINT32 offset;
    /*************************************************************************/
    /* HSPI data is uploaded and processed through USB endpoint */
    if( Endp1_Up_Status == 0x00 )
    {
        /* Determine whether there is any data to upload */
        R8_HSPI_INT_EN = 0x00;
        R8_HSPI_INT_EN = 0x00;
        remanlen = HSPI_Rx_Data_RemainLen;
        R8_HSPI_INT_EN = HSPI_Int_En_Save;
        if( remanlen )
        {
            /* Calculate the length and number of packets uploaded this time */
            offset = ( DEF_HPSI_RX_DMA_ADDR_MAX - HSPI_Rx_Data_DealAddr );
            if( remanlen >= ( 1 * 512 ) )
            {
                if( offset >= ( 1 * 512 ) )
                {
                    Endp1_Up_LastPackLen = ( 1 * 512 );
                    Endp1_Up_LastPackNum = 1;
                    len = 512;
                }
                else
                {
                    Endp1_Up_LastPackLen = offset;
                    Endp1_Up_LastPackNum = Endp1_Up_LastPackLen / 512;
                    len = Endp1_Up_LastPackLen % 512;
                    if( len )
                    {
                        Endp1_Up_LastPackNum++;
                    }
                    else if( len == 0 )
                    {
                        len = 512;
                    }
                }
            }
            else
            {
                if( offset >= remanlen )
                {
                    Endp1_Up_LastPackLen = remanlen;
                }
                else
                {
                    Endp1_Up_LastPackLen = offset;
                }
                len = Endp1_Up_LastPackLen % 512;
                if( len == 0 )
                {
                    len = 512;
                }
            }
            R16_UEP1_T_LEN = len;
            R8_UEP1_TX_CTRL ^= RB_UEP_T_TOG_1;
            R8_UEP1_TX_CTRL = ( R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK ) | UEP_T_RES_ACK;
            Endp1_Up_Status = 0x01;
        }

    }
    R8_HSPI_INT_EN = 0x00;
    R8_HSPI_INT_EN = 0x00;
    R8_HSPI_INT_EN = 0x00;
    remanlen = HSPI_Rx_Data_RemainLen;
    R8_HSPI_INT_EN = HSPI_Int_En_Save;
    if( remanlen <= ( DEF_ENDP1_TX_BUF_LEN_BULK - ( DEF_HSPI_BULK_BPACK_LEN * 2 ) ) )
    {
        if( HSPI_Rx_Notice_Status == 0x01 )
        {
            HSPI_Rx_Notice_Status = 0x00;
            if( USB_Stop_UpFlag == 0x00 )
            {
                PIN_HRTS_LOW( );
            }
        }
    }
}

/*******************************************************************************
* Function Name  : HSPI_usb20_OUT_handle
* Description    : HSPI USB20 Download processing
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HSPI_usb20_OUT_handle( void )
{
    UINT32 len;
    UINT32 remanlen;
    UINT32 packnum;
    UINT32 packlen;
    UINT32 offset;
    /*************************************************************************/
    /* USB endpoint downloading data sent through HSPI interface */
    R8_HSPI_INT_EN = 0x00;
    R8_HSPI_INT_EN = 0x00;
    R8_HSPI_INT_EN = 0x00;
    if( ( HSPI_Tx_Status == 0x00 ) && ( PIN_HCTS_RD( ) == 0x00 ) )
    {
        HSPI_Tx_Data_Deal( );
    }
    R8_HSPI_INT_EN = HSPI_Int_En_Save;

    /* Determine whether USB endpoint download is paused */
    if( Endp1_Down_Status )
    {
        /* Determine whether the current pause is a forced pause with non full packets or a pause with insufficient buffers */
        R8_HSPI_INT_EN = 0x00;
        R8_HSPI_INT_EN = 0x00;
        R8_HSPI_INT_EN = 0x00;
        remanlen = HSPI_Tx_Data_RemainLen;
        R8_HSPI_INT_EN = HSPI_Int_En_Save;
        if( USB_Down_StopFlag )
        {
            /* After all data is sent, enable USB download */
            if( remanlen == 0x00 )
            {
                USB_Down_StopFlag = 0x00;
                Endp1_Down_Status = 0x00;

                /* HSPI send buffer and USB endpoint receive buffer pointer back */
                HSPI_Tx_Data_LoadAddr = DEF_HPSI_DMA_TX_ADDR0;
                HSPI_Tx_Data_DealAddr = DEF_HPSI_DMA_TX_ADDR0;

                R32_UEP1_RX_DMA = (UINT32)(UINT8 *)HSPI_Tx_Data_LoadAddr;
                /* Notify the computer to continue downloading N packets of data */
                R8_UEP1_RX_CTRL ^= RB_UEP_R_TOG_1;
                R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;
            }
        }
        else
        {
            /* If there is enough space in the buffer, then enable USB download */
            if( remanlen <= ( DEF_ENDP1_RX_BUF_LEN - ( 512 ) ) )
            {
                Endp1_Down_Status = 0x00;

                /* Notify the computer to continue downloading N packets of data */
                R8_UEP1_RX_CTRL ^= RB_UEP_R_TOG_1;
                R8_UEP1_RX_CTRL = (R8_UEP1_RX_CTRL & ~RB_UEP_RRES_MASK) | UEP_R_RES_ACK;
            }
        }
    }
}

/*******************************************************************************
* Function Name  : HSPI_DataTrans
* Description    : HSPI Data Trans
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HSPI_DataTrans( void )
{
    UINT32 i,j;
    UINT32 addr;
    UINT32 len;
    UINT32 remanlen;
    UINT32 packnum;
    UINT32 packlen;
    UINT32 offset;

    HSPI_Init( );                                                             	/* HSPI Interface initialization */
    /************************************************************************/
    USB_Stop_UpFlag = 0x00;
    HSPI_Rx_Notice_Status = 0x01;
    HSPI_Rx_Data_RemainLen = 0x00;

    PIN_HRTS_LOW( );

    while( 1 )
    {
        if( Link_Sta == LINK_STA_1 )
        {
            HSPI_usb20_IN_handle( );
            HSPI_usb20_OUT_handle( );
        }
        else if( Link_Sta == LINK_STA_3 )
        {
            HSPI_usb30_IN_handle( );
            HSPI_usb30_OUT_handle( );
        }
    }
}

/*******************************************************************************
* Function Name  : HSPI_Tx_Data_Deal
* Description    : HSPI Interface sending data processing
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HSPI_Tx_Data_Deal( void )
{
    UINT32V remanlen;
    UINT32V len;
    UINT32V packlen;
    UINT32V i;

    /* Judge whether there is data to be sent through HSPI */
    remanlen = HSPI_Tx_Data_RemainLen;
    if( remanlen )
    {
        /* Calculate the number of burst packets sent this time and the length of the last packets*/
        if( remanlen >= DEF_HSPI_BULK_BPACK_LEN )
        {
            packlen = DEF_HSPI_BULK_BPACK_LEN;

            if( packlen > ( DEF_HPSI_TX_DMA_ADDR_MAX - HSPI_Tx_Data_DealAddr ) )
            {
                packlen = ( DEF_HPSI_TX_DMA_ADDR_MAX - HSPI_Tx_Data_DealAddr );
            }
        }
        else
        {
            /* Judge whether the timeout has been reached */
            if( Endp1_Down_IdleCount >= DEF_USB_DOWN_IDLE_TIMEOUT )
            {
                Endp1_Down_IdleCount = 0x00;

                packlen = remanlen;

                /* Judge whether the buffer offset is sufficient. If not, it will be sent to the end of the buffer at most */
                if( packlen > ( DEF_HPSI_TX_DMA_ADDR_MAX - HSPI_Tx_Data_DealAddr ) )
                {
                    packlen = ( DEF_HPSI_TX_DMA_ADDR_MAX - HSPI_Tx_Data_DealAddr );
                }
            }
            else
            {
                packlen = 0x00;
            }
        }

        /* Start burst mode for data transmission */
        if( packlen )
        {
            HSPI_Tx_LastPackLen = packlen;
            HSPI_Tx_BurstPackNum = ( HSPI_Tx_LastPackLen + ( DEF_HSPI_DMA_PACK_LEN - 1 ) ) / DEF_HSPI_DMA_PACK_LEN;

            if( HSPI_Tx_BurstPackNum == 1 )
            {
                len = HSPI_Tx_LastPackLen % DEF_HSPI_DMA_PACK_LEN;
                R32_HSPI_UDF0 = len | ( 1 << 13 );
            }
            else if( HSPI_Tx_BurstPackNum == 2 )
            {
                len = HSPI_Tx_LastPackLen % DEF_HSPI_DMA_PACK_LEN;
                R32_HSPI_UDF1 = len | ( 1 << 13 );
            }

            /* Set related variables */
            HSPI_Tx_PackCnt = 0x00;
            HSPI_Tx_Status = 0x01;

            /* Start HSPI data transmission */
            R32_HSPI_TX_ADDR0 = HSPI_Tx_Data_DealAddr;
            R32_HSPI_TX_ADDR1 = HSPI_Tx_Data_DealAddr + DEF_HSPI_DMA_PACK_LEN;
            R16_HSPI_BURST_CFG = ( HSPI_Tx_BurstPackNum << 8 ) | RB_HSPI_BURST_EN;
            R8_HSPI_INT_FLAG = RB_HSPI_IF_T_DONE | RB_HSPI_IF_B_DONE;           /* Clear all HSPI interrupt flags before sending */
            R8_HSPI_CTRL |= RB_HSPI_SW_ACT;                                     /* Software trigger sending */

        }
    }
}
