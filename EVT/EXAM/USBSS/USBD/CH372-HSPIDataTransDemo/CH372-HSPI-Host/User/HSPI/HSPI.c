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

#include <MAIN.h>

/******************************************************************************/
#define HPSI_DMA_TX_Addr0          0x20020000                                   /* HSPI send DMA0 address */
#define HPSI_DMA_TX_Addr1          HPSI_DMA_TX_Addr0 + DEF_HSPI_DMA_PACK_LEN    /* HSPI send DMA1 address */
#define HPSI_DMA_RX_Addr0          0x20021000                                   /* HSPI receive DMA0 address */
#define HPSI_DMA_RX_Addr1          HPSI_DMA_RX_Addr0 + DEF_HSPI_DMA_PACK_LEN    /* HSPI receive DMA1 address */


volatile UINT8  HSPI_Tx_PackCnt = 0x00;											/* HSPI Send packet count */
volatile UINT8  HSPI_Tx_AddrTog = 0x00;                                         /* HSPI Send packet address synchronization count */
volatile UINT8  HSPI_Tx_Status = 0x00;                                          /* HSPI Send Status */
volatile UINT8  HSPI_Tx_ErrFlag = 0x00;                                         /* HSPI Send error flag */
volatile UINT8  HSPI_Tx_BurstPackNum = 0x00;                                    /* HSPI Number of burst packets sent this time */
volatile UINT8  HSPI_Rx_PackCnt = 0x00;											/* HSPI Received packet count */
volatile UINT8  HSPI_Rx_AddrTog = 0x00;                                         /* HSPI Receive packet address synchronization count */
volatile UINT8  HSPI_Rx_ErrFlag = 0x00;                                         /* HSPI Receive error flag */
volatile UINT32 HSPI_Test_PackCount = 0x00;
volatile UINT32 DATA_TotalLen = 0x00;                                           /* Total data length */
volatile UINT32 DATA_SendLen = 0x00;                                            /* Data sent length */
volatile UINT32 DATA_Last_PackLen = 0x00;                                       /* Packet sending length at the end of data */
volatile UINT32 Pack_Send_Num = 0x00;
volatile UINT32 Pack_Recv_Num = 0x00;


/*******************************************************************************
* Function Name  : HSPI_GPIO_Init
* Description    : HSPI Interface related GPIO initialization
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HSPI_GPIO_Init( void )
{
    /* configure HTCLK(PA11),HTREQ(PA9),HTVLD(PA21),HTACK(PA10),output,16mA */
    R32_PA_DIR |= ( ( 1 << 9 ) | ( 1 << 10 ) | ( 1 << 11 ) | ( 1 << 21 ) );
    R32_PA_DRV |= ( ( 1 << 11 ) );
    R32_PA_PU &= ~( ( 1 << 9 ) | ( 1 << 10 ) | ( 1 << 11 ) | ( 1 << 21 ) );
    R32_PA_PD &= ~( ( 1 << 9 ) | ( 1 << 10 ) | ( 1 << 11 ) | ( 1 << 21 ) );

    /* configure HTRDY(PA23),HRCLK(PA19),HRACK(PA18),HRVLD(PA6),input */
    R32_PA_DIR &= ~( ( 1 << 6 ) | ( 1 << 18 ) | ( 1 << 19 ) | ( 1 << 23 ) );
    R32_PA_PU &= ~( ( 1 << 6 ) | ( 1 << 18 ) | ( 1 << 19 ) | ( 1 << 23 ) );
    R32_PA_PD |= ( ( 1 << 6 ) | ( 1 << 18 ) | ( 1 << 19 ) | ( 1 << 23 ) );

    /* configure HD0(PA6),HD1(PA4),HD2(PA22), HD3(PA3),HD4(PA2), HD5(PA1),HD6(PA0),HD7(PB21),
       HD8(PB20), HD9(PB19),HD10(PB18),HD11(PB17),HD12(PA17),HD13(PB16),HD14(PB15),HD15(PB14),
       HD16(PB0),HD17(PB1),HD18(PB2), HD19(PA20),HD20(PB3),HD21(PB4),HD22(PB5),HD23(PB6),
       HD24(PA16),HD25(PB7),HD26(PB8), HD27(PB9), HD28(PB10),HD29(PB11),HD30(PB12),HD31(PB13) floating input */
    R32_PA_DIR &= ~( ( 1 << 6 ) | ( 1 << 4 ) | ( 1 << 22 ) | ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 ) | ( 1 << 0 ) |
                     ( 1 << 17 ) | ( 1 << 20 ) | ( 1 << 16 ) );
    R32_PA_PU &= ~( ( 1 << 6 ) | ( 1 << 4 ) | ( 1 << 22 ) | ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 ) | ( 1 << 0 ) |
                    ( 1 << 17 ) | ( 1 << 20 ) | ( 1 << 16 ) );
    R32_PA_PD &= ~( ( 1 << 6 ) | ( 1 << 4 ) | ( 1 << 22 ) | ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 ) | ( 1 << 0 ) |
                    ( 1 << 17 ) | ( 1 << 20 ) | ( 1 << 16 ) );

    R32_PB_DIR &= 0xFFC00000;
    R32_PB_PU &= 0xFFC00000;
    R32_PB_PD &= 0xFFC00000;
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
	R32_HSPI_TX_ADDR0 = HPSI_DMA_TX_Addr0;
	R32_HSPI_RX_ADDR0 = HPSI_DMA_RX_Addr0;
	R32_HSPI_TX_ADDR1 = HPSI_DMA_TX_Addr1;
	R32_HSPI_RX_ADDR1 = HPSI_DMA_RX_Addr1;

    /* DMA TX/RX Len */
	R16_HSPI_DMA_LEN0 = DEF_HSPI_DMA_PACK_LEN - 1;
	R16_HSPI_DMA_LEN1 = DEF_HSPI_DMA_PACK_LEN - 1;

//	R16_HSPI_BURST_CFG |= (1<<8)|RB_HSPI_BURST_EN;
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

    /* Configure TX Customization Header */
    R32_HSPI_UDF0 = DEF_HSPI_UDF0;
    R32_HSPI_UDF1 = DEF_HSPI_UDF1;

    /* Enable Interupt */
    R8_HSPI_INT_EN |= RB_HSPI_IE_T_DONE;
    R8_HSPI_INT_EN |= RB_HSPI_IE_FIFO_OV;
    R8_HSPI_INT_EN |= RB_HSPI_IE_B_DONE;
    R8_HSPI_INT_EN |= RB_HSPI_IE_R_DONE;
    R8_HSPI_INT_FLAG = 0x0F;

    PFIC_EnableIRQ( HSPI_IRQn );

}

/*******************************************************************************
* Function Name  : HSPI_IRQHandler
* Description    : HSPI Interface interrupt handling
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HSPI_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HSPI_IRQHandler( void )
{
    UINT32V i,j;
    UINT32V addr;
    UINT32V packnum;

    if( R8_HSPI_INT_FLAG & RB_HSPI_IF_T_DONE )        /* Single packet transmission completion interrupt */
    {
        R8_HSPI_INT_FLAG = RB_HSPI_IF_T_DONE;

		/* Calculate related variables and switch buffers */
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
    else if( R8_HSPI_INT_FLAG & RB_HSPI_IF_R_DONE )        /* Single packet receiving completion interrupt */
	{
		R8_HSPI_INT_FLAG = RB_HSPI_IF_R_DONE;  

        /* The slave pulls up the HRTS pin to notify the host to suspend the next packet of data transmission */
        PIN_HRTS_HIGH( );

        if( R8_HSPI_RTX_STATUS & RB_HSPI_CRC_ERR )
		{  
			/* CRC Verification error */
			DUG_PRINTF("CRC Err:%d\r\n",(UINT16)HSPI_Rx_PackCnt);
			
			/* Reset DMA address and clear relevant variables */
            R32_HSPI_RX_ADDR0 = HPSI_DMA_RX_Addr0;
            R32_HSPI_RX_ADDR1 = HPSI_DMA_RX_Addr1;
            if( ( R8_HSPI_RX_SC & RB_HSPI_RX_TOG ) == RB_HSPI_RX_TOG )
            {
                R8_HSPI_RX_SC = RB_HSPI_RX_TOG;
            }
            /* In case of CRC error or MIS error at the receiver, RB_ HSPI_ RX_ NUM reset */
            R8_HSPI_RX_SC = 0x00;
            HSPI_Rx_AddrTog = 0x00;
            HSPI_Rx_PackCnt = 0x00;
        }
		else if( R8_HSPI_RTX_STATUS & RB_HSPI_NUM_MIS )
		{  
			/* Receive serial number does not match */
            DUG_PRINTF("NUM_MIS Err:%d\r\n",(UINT16)HSPI_Rx_PackCnt);

            R32_HSPI_RX_ADDR0 = HPSI_DMA_RX_Addr0;
            R32_HSPI_RX_ADDR1 = HPSI_DMA_RX_Addr1;
            if( ( R8_HSPI_RX_SC & RB_HSPI_RX_TOG ) == RB_HSPI_RX_TOG )
            {
                R8_HSPI_RX_SC = RB_HSPI_RX_TOG;
            }

            R8_HSPI_RX_SC = 0x00;
            HSPI_Rx_AddrTog = 0x00;
            HSPI_Rx_PackCnt = 0x00;

        }
		else
		{

            HSPI_Rx_PackCnt++;
            HSPI_Rx_AddrTog++;
            if( HSPI_Rx_AddrTog % 2 )
            {
                R32_HSPI_RX_ADDR0 += ( DEF_HSPI_DMA_PACK_LEN * 2 );
            }
            else
            {
                R32_HSPI_RX_ADDR1 += ( DEF_HSPI_DMA_PACK_LEN * 2 );
            }

            /* Judge whether the end packet is received */
            if( ( R32_HSPI_UDF0 & ( 1 << 13 ) ) || ( R32_HSPI_UDF1 & ( 1 << 13 ) ) )
            {
                /* Calculate related variables */
                if( R32_HSPI_UDF0 & ( 1 << 13 ) )
                {

                    Dbg_HSPI_Rx_TLen += R32_HSPI_UDF0^(1 << 13);

                }
                else if(R32_HSPI_UDF1 & ( 1 << 13 ))
                {
                    Dbg_HSPI_Rx_TLen +=  R32_HSPI_UDF1^(1 << 13)+DEF_HSPI_DMA_PACK_LEN;
                }


                R32_HSPI_RX_ADDR0 = HPSI_DMA_RX_Addr0;
                R32_HSPI_RX_ADDR1 = HPSI_DMA_RX_Addr0 + DEF_HSPI_DMA_PACK_LEN;


                /* CRC error or MIS error on the receiving end, RB needs to be updated_ HSPI_ RX_ NUM reset */
                if( ( R8_HSPI_RX_SC & RB_HSPI_RX_TOG ) == RB_HSPI_RX_TOG )
                {
                    R8_HSPI_RX_SC = RB_HSPI_RX_TOG;
                }
                R8_HSPI_RX_SC = 0x00;
                HSPI_Rx_PackCnt = 0x00;

                /* Clear custom register */
                R32_HSPI_UDF0 = 0x00;
                R32_HSPI_UDF1 = 0x00;

              }
		}
        /* Notify the HSPI counterpart to continue sending the next packet of data */
        PIN_HRTS_LOW( );
	}
    if( R8_HSPI_INT_FLAG & RB_HSPI_IF_B_DONE )        /* In burst mode, burst sequence packet transmission is completed */
    {
        R8_HSPI_INT_FLAG = RB_HSPI_IF_B_DONE;

		/* Compare the actual number of sent packets with the set number of burst packets */
		if( HSPI_Tx_PackCnt != ( R16_HSPI_BURST_CFG >> 8 ) )
		{
            DUG_PRINTF("BURST Tx Err:%d\r\n",HSPI_Tx_PackCnt);
            DUG_PRINTF("R8_HSPI_TX_SC:%d\r\n",R8_HSPI_TX_SC);

            R32_HSPI_TX_ADDR0 = HPSI_DMA_TX_Addr0;
            R32_HSPI_TX_ADDR1 = HPSI_DMA_TX_Addr1;
            if( ( R8_HSPI_TX_SC & RB_HSPI_TX_TOG ) == RB_HSPI_TX_TOG )
            {
                R8_HSPI_TX_SC = RB_HSPI_TX_TOG;
            }
            R8_HSPI_TX_SC = 0x00;
            HSPI_Tx_PackCnt = 0x00;
            HSPI_Tx_AddrTog = 0x00;

            HSPI_Tx_ErrFlag = 0x01;
		}
		else
		{
            R32_HSPI_TX_ADDR0 = HPSI_DMA_TX_Addr0;
            R32_HSPI_TX_ADDR1 = HPSI_DMA_TX_Addr1;
            if( ( R8_HSPI_TX_SC & RB_HSPI_TX_TOG ) == RB_HSPI_TX_TOG )
            {
                R8_HSPI_TX_SC = RB_HSPI_TX_TOG;
            }
            R8_HSPI_TX_SC = 0x00;
            HSPI_Tx_AddrTog = 0x00;

            HSPI_Tx_Status = 0x02;
		}
    }
	else if( R8_HSPI_INT_FLAG & RB_HSPI_IF_FIFO_OV )        /* FIFO Overflow interrupt */
	{
		R8_HSPI_INT_FLAG = RB_HSPI_IF_FIFO_OV;
		DUG_PRINTF("FIFO OV\r\n");
	}

    Dbg_Idle_TimeCount = 0;
}

/*******************************************************************************
* Function Name  : BULKMode_HSPI_Test
* Description    : Bulk transmission mode HSPI interface test
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BULKMode_HSPI_Test( void )
{
    UINT32 i,j;
    UINT32 addr;

    HSPI_Init( );

    HSPI_Tx_PackCnt = 0x00;
    HSPI_Tx_AddrTog = 0x00;
    HSPI_Rx_PackCnt = 0x00;
    HSPI_Rx_AddrTog = 0x00;
    HSPI_Tx_Status = 0x00;
    HSPI_Test_PackCount = 0x00;

    /* Fill in sending data */
    addr = HPSI_DMA_TX_Addr0;
    for( i = 0; i < ( ( DEF_HSPI_DMA_PACK_LEN * DEF_HSPI_BULK_BPACK_NUM ) * 2 / 1024 ); i++ )
    {
        for( j = 0; j < 1024; j++ )
        {
            *(UINT8 *)( addr + j ) = (UINT8)j;
        }
        addr += 1024;
    }

    /************************************************************************/
    DUG_PRINTF("HSPI Double Dir Test\n");
    DUG_PRINTF("HSPI Host Mode\n");

    DATA_TotalLen = ( DEF_HSPI_DMA_PACK_LEN * DEF_HSPI_BULK_BPACK_NUM ) * 2;
    DATA_SendLen = 0x00;
    DATA_Last_PackLen = DATA_TotalLen % DEF_HSPI_DMA_PACK_LEN;

    PRINT("Pic_TotalLen: %x\n", (UINT32)DATA_TotalLen );
    PRINT("Pic_Last_PackLen: %x\n", (UINT32)DATA_Last_PackLen );

    /* Custom bytes BIT0-BIT12 length; BIT13: end Package  */
    R32_HSPI_UDF0 = DEF_HSPI_DMA_PACK_LEN;
    R32_HSPI_UDF1 = DEF_HSPI_DMA_PACK_LEN;
    Pack_Send_Num = 0x00;
    Pack_Recv_Num = 0x00;

    PIN_HRTS_LOW( );
    while( 1 )
    {
        /* Judge whether to allow sending data */
        if( ( HSPI_Tx_Status == 0x00 ) && PIN_HCTS_RD( ) == 0x00 )
        {
            /* Calculate the number of currently sent burst packets */
            HSPI_Tx_BurstPackNum = DEF_HSPI_BULK_BPACK_NUM;
            if( ( DATA_TotalLen - DATA_SendLen ) < ( DEF_HSPI_BULK_BPACK_NUM * DEF_HSPI_DMA_PACK_LEN ) )
            {
                HSPI_Tx_BurstPackNum = ( DATA_TotalLen - DATA_SendLen + ( DEF_HSPI_DMA_PACK_LEN - 1 ) ) / DEF_HSPI_DMA_PACK_LEN;
            }


            if( HSPI_Tx_BurstPackNum == 1 )
            {
                R32_HSPI_UDF0 = DATA_Last_PackLen | ( 1 << 13 );
            }
            else if( HSPI_Tx_BurstPackNum == 2 )
            {
                R32_HSPI_UDF1 = DATA_Last_PackLen | ( 1 << 13 );
            }


            /* Start burst mode for data transmission */
            R32_HSPI_TX_ADDR0 = HPSI_DMA_TX_Addr0 + DATA_SendLen;
            R32_HSPI_TX_ADDR1 = HPSI_DMA_TX_Addr1 + DATA_SendLen;


            HSPI_Tx_PackCnt = 0x00;
            R16_HSPI_BURST_CFG = ( HSPI_Tx_BurstPackNum << 8 ) | RB_HSPI_BURST_EN;
            R8_HSPI_INT_FLAG = 0x0F;                                            /* Clear all HSPI interrupt flags before sending */
            R8_HSPI_CTRL |= RB_HSPI_SW_ACT;                                     /* Software trigger sending */
            HSPI_Tx_Status = 0x01;

        }

        /* Wait for the data to be sent or failed */
        if( HSPI_Tx_Status == 0x02 )
        {

            HSPI_Tx_Status = 0x00;

            DATA_SendLen += ( HSPI_Tx_PackCnt * DEF_HSPI_DMA_PACK_LEN );

            Dbg_HSPI_Tx_TLen += ( HSPI_Tx_PackCnt * DEF_HSPI_DMA_PACK_LEN );

            if( DATA_SendLen >= DATA_TotalLen )
            {
                DATA_SendLen = 0x00;
                R32_HSPI_UDF0 = DEF_HSPI_DMA_PACK_LEN;
                R32_HSPI_UDF1 = DEF_HSPI_DMA_PACK_LEN;

                R32_HSPI_TX_ADDR0 = HPSI_DMA_TX_Addr0;
                R32_HSPI_TX_ADDR1 = HPSI_DMA_TX_Addr1;
                if( ( R8_HSPI_TX_SC & RB_HSPI_TX_TOG ) == RB_HSPI_TX_TOG )
                {
                    R8_HSPI_TX_SC = RB_HSPI_TX_TOG;
                }
                R8_HSPI_TX_SC = 0x00;
                HSPI_Tx_PackCnt = 0x00;
                HSPI_Tx_AddrTog = 0x00;
            }

        }
        else if( HSPI_Tx_ErrFlag )
        {
            DUG_PRINTF("Send Error!\r\n");
            HSPI_Tx_ErrFlag = 0x00;
        }
    }
}



