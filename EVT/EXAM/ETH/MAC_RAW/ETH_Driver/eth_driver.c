/********************************** (C) COPYRIGHT *******************************
* File Name          : eth_driver.c
* Author             : WCH
* Version            : V1.3.0
* Date               : 2023/03/03
* Description        : eth program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "string.h"
#include "eth_driver.h"
#include "ISPEM569.h"

__attribute__ ((aligned(16))) ETH_DMADESCTypeDef DMARxDscrTab[ETH_RXBUFNB];    /* MAC receive descriptor, 16-byte aligned*/

__attribute__ ((aligned(16))) ETH_DMADESCTypeDef DMATxDscrTab[ETH_TXBUFNB];    /* MAC send descriptor, 16-byte aligned */

__attribute__ ((aligned(16))) uint8_t MACRxBuf[ETH_RXBUFNB*ETH_RX_BUF_SZE];    /* MAC receive buffer, 16-byte aligned */

__attribute__ ((aligned(4))) uint8_t  MACTxBuf[ETH_TXBUFNB*ETH_TX_BUF_SZE];    /* MAC send buffer, 4-byte aligned */


ETH_DMADESCTypeDef *pDMARxSet;
ETH_DMADESCTypeDef *pDMATxSet;
volatile uint8_t LinkSta = 0; //0: No valid link established   1:Valid link established
/*********************************************************************
 * @fn      ETH_GetMacAddr
 *
 * @brief   Get the MAC address
 *
 * @return  none.
 */
void ETH_GetMacAddr( uint8_t *p )
{
    uint8_t rc, buf[8];

    rc = FLASH_ROMA_READ((0x7FFE4 - 0x8000), buf, 8);
    for(rc = 0; rc < 6; rc++)
        p[0 + rc] = buf[5 - rc];
}

/*********************************************************************
 * @fn      ETH_GPIOInit
 *
 * @brief   PHY RGMII interface GPIO initialization.
 *
 * @return  none
 */
void ETH_GPIOInit(void)
{
    R32_PA_DIR |=  bEMCO;
    R32_PA_DIR &= ~bEMCI;
    R32_PB_DIR |=  bMDCK | bETHT3 | bETHT2 | \
                   bETHT1 | bETHT0 | bETHTEN | bETHTC;
    R32_PB_DIR &= ~(bETHRC | bETHR3 | bETHR2 | \
                    bETHR1 | bETHR0 | bETHRDV | bMDIO);
}

/*********************************************************************
 * @fn      ETH_Start
 *
 * @brief   Enable ETH MAC and DMA reception/transmission.
 *
 * @return  none
 */
void ETH_Start(void)
{
    ETH->MACCR |= ETH_MACCR_RE;
    ETH->DMAOMR |= ETH_DMAOMR_FTF;
    ETH->DMAOMR |= ETH_DMAOMR_ST;
    ETH->DMAOMR |= ETH_DMAOMR_SR;
}

/*********************************************************************
 * @fn      ETH_PHYLink
 *
 * @brief   Configure MAC parameters after the PHY Link is successful.
 *
 * @param   none.
 *
 * @return  none.
 */
void ETH_PHYLink( void )
{
    uint32_t phy_stat;

    ETH_WritePHYRegister( PHY_ADDRESS, 0x1F,0x0a43 );
    /*In some cases the status is not updated in time,
     * so read this register twice to get the correct status value.*/
    ETH_ReadPHYRegister( PHY_ADDRESS, 0x1A);
    phy_stat = ETH_ReadPHYRegister( PHY_ADDRESS, 0x1A);

    if( phy_stat & 0x04 )
    {
        if( phy_stat & 0x08 )
        {
            ETH->MACCR |= ETH_Mode_FullDuplex;
        }
        else
        {
            ETH->MACCR &= ~ETH_Mode_FullDuplex;
        }
        if( (phy_stat & 0x30) == 0x00 )
        {
            ETH->MACCR &= ~(ETH_Speed_100M|ETH_Speed_1000M);
        }
        else if( (phy_stat & 0x30) == 0x10 )
        {
            ETH->MACCR &= ~(ETH_Speed_100M|ETH_Speed_1000M);
            ETH->MACCR |= ETH_Speed_100M;
        }
        else if( (phy_stat & 0x30) == 0x20 )
        {
            ETH->MACCR &= ~(ETH_Speed_100M|ETH_Speed_1000M);
            ETH->MACCR |= ETH_Speed_1000M;
        }
        ETH_Start( );
        LinkSta = 1;
    }
    else {
        LinkSta = 0;
    }
    phy_stat = ETH_ReadPHYRegister( PHY_ADDRESS, 0x1D);   /* Clear the Interrupt status */
}

/*********************************************************************
 * @fn      ETH_ETHIsr
 *
 * @brief   Ethernet Interrupt Service program
 *
 * @return  none
 */
void ETH_ETHIsr(void)
{
    uint32_t int_sta, length, buffer;

    int_sta = ETH->DMASR;

    if (int_sta & ETH_DMA_IT_AIS)
    {
        if (int_sta & ETH_DMA_IT_RBU)
        {
            ETH_DMAClearITPendingBit(ETH_DMA_IT_RBU);
        }
        ETH_DMAClearITPendingBit(ETH_DMA_IT_AIS);
    }
    if( int_sta & ETH_DMA_IT_NIS )
    {
        if( int_sta & ETH_DMA_IT_R )
        {
            ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
            /*If you don't use the Ethernet library,
             * you can do some data processing operations here*/
            for(uint8_t i = 0; i < ETH_RXBUFNB; i++)
            {
                if(!(DMARxDescToGet->Status & ETH_DMARxDesc_OWN))
                {
                    if( !(DMARxDescToGet->Status & ETH_DMARxDesc_ES) &&\
                      (DMARxDescToGet->Status & ETH_DMARxDesc_LS) &&\
                      (DMARxDescToGet->Status & ETH_DMARxDesc_FS) &&\
                      !(DMARxDescToGet->Status & ETH_DMARxDesc_MAMPCE))
                    {
                        /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
                        length = (DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> 16;
                        /* Get the addrees of the actual buffer */
                        buffer = DMARxDescToGet->Buffer1Addr;

                        /* Do something*/
                        printf("rec data:%d bytes\r\n",length);
                        printf("data:%x\r\n",*((uint8_t *)buffer));
                    }
                }
                DMARxDescToGet->Status = ETH_DMARxDesc_OWN;
                DMARxDescToGet = (ETH_DMADESCTypeDef *)DMARxDescToGet->Buffer2NextDescAddr;
            }
        }
        if( int_sta & ETH_DMA_IT_T )
        {
            ETH_DMAClearITPendingBit(ETH_DMA_IT_T);
        }
        if (int_sta & ETH_DMA_IT_TBU)
        {
            ETH_DMAClearITPendingBit(ETH_DMA_IT_TBU);
        }
        ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
    }
}

/*******************************************************************************
 * @fn     ETH_ClockEnable
 *
 * @brief  ETH peripheral clock initialization
 *
 * @return   None
 */
void ETH_ClockEnable(void)
{
    R8_SAFE_ACCESS_SIG = 0x57;   /* enable safe access mode */
    R8_SAFE_ACCESS_SIG = 0xa8;

    R8_CLK_MOD_AUX = (R8_CLK_MOD_AUX & (~RB_SRC_125M_MSK)) | RB_EXT_125M_EN;

    R8_SAFE_ACCESS_SIG = 0;
}

/*******************************************************************************
 * @fn      ETH_RegInit
 *
 * @brief   ETH register & PHY register initialization
 *
 * @param   ETH_InitStruct - pointer to the ETH config struct
 *          PHYAddress - PHY address
 *
 * @return  Execution status
 */
uint32_t ETH_RegInit(ETH_InitTypeDef* ETH_InitStruct, uint16_t PHYAddress)
{
    uint32_t tmpreg = 0;

    /*---------------------- Physical layer configuration -------------------*/
    /* Set the SMI interface clock, set as the main frequency divided by 42  */
    tmpreg = ETH->MACMIIAR;
    tmpreg &= MACMIIAR_CR_MASK;
    tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div42;
    ETH->MACMIIAR = (uint32_t)tmpreg;

    /*------------------------ MAC register configuration  ----------------------- --------------------*/
    tmpreg = ETH->MACCR;
    tmpreg &= MACCR_CLEAR_MASK;
    tmpreg |= (uint32_t)(ETH_InitStruct->ETH_AutoNegotiation |
                  ETH_InitStruct->ETH_Watchdog |
                  ETH_InitStruct->ETH_Jabber |
                  ETH_InitStruct->ETH_InterFrameGap |
                  ETH_InitStruct->ETH_CarrierSense |
                  ETH_InitStruct->ETH_Speed |
                  ETH_InitStruct->ETH_ReceiveOwn |
                  ETH_InitStruct->ETH_LoopbackMode |
                  ETH_InitStruct->ETH_Mode |
                  ETH_InitStruct->ETH_ChecksumOffload |
                  ETH_InitStruct->ETH_RetryTransmission |
                  ETH_InitStruct->ETH_AutomaticPadCRCStrip |
                  ETH_InitStruct->ETH_BackOffLimit |
                  ETH_InitStruct->ETH_DeferralCheck);
    /* Write MAC Control Register */
    ETH->MACCR = (uint32_t)tmpreg;

    ETH->MACFFR = (uint32_t)(ETH_InitStruct->ETH_ReceiveAll |
                          ETH_InitStruct->ETH_SourceAddrFilter |
                          ETH_InitStruct->ETH_PassControlFrames |
                          ETH_InitStruct->ETH_BroadcastFramesReception |
                          ETH_InitStruct->ETH_DestinationAddrFilter |
                          ETH_InitStruct->ETH_PromiscuousMode |
                          ETH_InitStruct->ETH_MulticastFramesFilter |
                          ETH_InitStruct->ETH_UnicastFramesFilter);
    /*--------------- ETHERNET MACHTHR and MACHTLR Configuration ---------------*/
    /* Write to ETHERNET MACHTHR */
    ETH->MACHTHR = (uint32_t)ETH_InitStruct->ETH_HashTableHigh;
    /* Write to ETHERNET MACHTLR */
    ETH->MACHTLR = (uint32_t)ETH_InitStruct->ETH_HashTableLow;
    /*----------------------- ETHERNET MACFCR Configuration --------------------*/
    /* Get the ETHERNET MACFCR value */
    tmpreg = ETH->MACFCR;
    /* Clear xx bits */
    tmpreg &= MACFCR_CLEAR_MASK;
    tmpreg |= (uint32_t)((ETH_InitStruct->ETH_PauseTime << 16) |
                     ETH_InitStruct->ETH_ZeroQuantaPause |
                     ETH_InitStruct->ETH_PauseLowThreshold |
                     ETH_InitStruct->ETH_UnicastPauseFrameDetect |
                     ETH_InitStruct->ETH_ReceiveFlowControl |
                     ETH_InitStruct->ETH_TransmitFlowControl);
    ETH->MACFCR = (uint32_t)tmpreg;

    ETH->MACVLANTR = (uint32_t)(ETH_InitStruct->ETH_VLANTagComparison |
                               ETH_InitStruct->ETH_VLANTagIdentifier);

    tmpreg = ETH->DMAOMR;
    tmpreg &= DMAOMR_CLEAR_MASK;
    tmpreg |= (uint32_t)(ETH_InitStruct->ETH_DropTCPIPChecksumErrorFrame |
                    ETH_InitStruct->ETH_ReceiveStoreForward |
                    ETH_InitStruct->ETH_FlushReceivedFrame |
                    ETH_InitStruct->ETH_TransmitStoreForward |
                    ETH_InitStruct->ETH_TransmitThresholdControl |
                    ETH_InitStruct->ETH_ForwardErrorFrames |
                    ETH_InitStruct->ETH_ForwardUndersizedGoodFrames |
                    ETH_InitStruct->ETH_ReceiveThresholdControl |
                    ETH_InitStruct->ETH_SecondFrameOperate);
    ETH->DMAOMR = (uint32_t)tmpreg;

    /* Reset the physical layer */
    ETH_WritePHYRegister(PHYAddress, PHY_BCR, PHY_Reset);
    return ETH_SUCCESS;
}

/*********************************************************************
 * @fn      RTL8211FS_Interrupt_Init
 *
 * @brief   Enable Link Status Change Interrupt
 *          and Auto-Negotiation Completed Interrupt
 *
 * @return  none
 */
void RTL8211FS_Interrupt_Init(void)
{
    uint16_t RegValue;

    ETH_WritePHYRegister(PHY_ADDRESS, 0x1f, 0x0a42 );
    RegValue = ETH_ReadPHYRegister(PHY_ADDRESS, 0x12);
    /* Enable Link Status Change Interrupt and Auto-Negotiation Completed Interrupt*/
    RegValue |= (1<<4)|(1<<3);
    ETH_WritePHYRegister(PHY_ADDRESS, 0x12, RegValue );

    ETH_WritePHYRegister(PHY_ADDRESS, 0x1f, 0x0a43 );
    ETH_ReadPHYRegister(PHY_ADDRESS, 0x1d);        /* Clear the Interrupt status */
}

/*********************************************************************
 * @fn      GPIO_Interrupt_Init
 *
 * @brief   Interrupt pin initialization
 *
 * @return  none
 */
void GPIO_Interrupt_Init(void)
{
    /* PB15 is set to GPIO interrupt trigger pin, Falling edge trigger interrupt  */
    R32_PB_PU = (1<<15);                            /* PB15 pull-up */
    GPIOB_ITModeCfg( GPIO_Pin_15, GPIO_ITMode_FallEdge );
    R8_GPIO_INT_FLAG = 0xff;                        /* Clear the Interrupt status */
    PFIC_EnableIRQ(GPIO_IRQn);
}

/*******************************************************************************
 * @fn       ETH_MACAddressConfig
 *
 * @brief    Mac address initialization
 *
 * @param    MacAddr - Mac address register
 *           PHYAddress - pointer to the Mac address string
 *
 * @return   None
 */
void ETH_MACAddressConfig(uint32_t MacAddr, uint8_t *Addr)
{
    uint32_t tmpreg;

    /* Calculate the selected MAC address high register */
    tmpreg = ((uint32_t)Addr[5] << 8) | (uint32_t)Addr[4];

    /* Load the selected MAC address high register */
    (*(__IO uint32_t *) (ETH_MAC_ADDR_HBASE + MacAddr)) = tmpreg;

    /* Calculate the selected MAC address low register */
    tmpreg = ((uint32_t)Addr[3] << 24) | ((uint32_t)Addr[2] << 16) | ((uint32_t)Addr[1] << 8) | Addr[0];

    /* Load the selected MAC address low register */
    (*(__IO uint32_t *) (ETH_MAC_ADDR_LBASE + MacAddr)) = tmpreg;
}

/*********************************************************************
 * @fn      ETH_Configuration
 *
 * @brief   Ethernet configure.
 *
 * @return  none
 */
void ETH_Configuration( uint8_t *macAddr )
{
    ETH_InitTypeDef ETH_InitStructure;
    uint16_t timeout = 10000;

    /* Enable Ethernet 125MHz clock */
    ETH_ClockEnable();

    /* Enable RGMII GPIO */
    ETH_GPIOInit();

    /* Software reset */
    ETH_SoftwareReset();

    /* Wait for software reset */
    do{
        DelayUs(10);
        if( !--timeout )  break;
    }while(ETH->DMABMR & ETH_DMABMR_SR);

    /* Enable MAC Transmitter */
    ETH->MACCR |= ETH_MACCR_TE;

    /* Mask the interrupt that Tx good frame count counter reaches half the maximum value */
    ETH->MMCTIMR = ETH_MMCTIMR_TGFM;
    /* Mask the interrupt that Rx good unicast frames counter reaches half the maximum value */
    /* Mask the interrupt that Rx crc error counter reaches half the maximum value */
    ETH->MMCRIMR = ETH_MMCRIMR_RGUFM | ETH_MMCRIMR_RFCEM;

    /* Configure MAC address */
    ETH_MACAddressConfig(ETH_MAC_Address0,macAddr);

    /* ETHERNET Configuration */
    /* Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter */
    ETH_StructInit(&ETH_InitStructure);
    /* Fill ETH_InitStructure parameters */
    /*------------------------   MAC   -----------------------------------*/
    ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
    ETH_InitStructure.ETH_Speed = ETH_Speed_1000M;
    ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
    ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
    ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
    ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
    /* Filter function configuration */
    ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Disable;
    ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
    ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
    ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
    ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
    /*------------------------   DMA   -----------------------------------*/
    /* When we use the Checksum offload feature, we need to enable the Store and Forward mode:
    the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum,
    if the checksum is OK the DMA can handle the frame otherwise the frame is dropped */
    ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable;
    ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
    ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;
    ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Enable;
    ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Enable;
    ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Disable;

    /* Configure Ethernet */
    ETH_RegInit( &ETH_InitStructure, PHY_ADDRESS );

    /* Enable the Ethernet Interrupt */
    ETH_DMAITConfig(ETH_DMA_IT_NIS |\
                ETH_DMA_IT_R |\
                ETH_DMA_IT_T |\
                ETH_DMA_IT_AIS |\
                ETH_DMA_IT_RBU,\
                ENABLE);
    /* Set the interrupt priority */
    PFIC_SetPriority(ETH_IRQn, 0x20);

    /* ETH send clock polarity and timing adjustment */
    RGMII_TXC_Delay(0, 4);

    /* Enable the RTL8211FS Interrupt */
    RTL8211FS_Interrupt_Init();

    /* Enable the GPIO Interrupt */
    GPIO_Interrupt_Init();
}

/*********************************************************************
 * @fn      MACRAW_Tx
 *
 * @brief   Ethernet sends data frames in chain mode.
 *
 * @param   buff   send buffer pointer
 *          len    Send data length
 *
 * @return  Send status.
 */
uint32_t MACRAW_Tx(uint8_t *buff, uint16_t len)
{
    /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
    if((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (uint32_t)RESET)
    {
        /* Return ERROR: OWN bit set */
        return ETH_ERROR;
    }
    /* Setting the Frame Length: bits[12:0] */
    DMATxDescToSet->ControlBufferSize = (len & ETH_DMATxDesc_TBS1);

    memcpy((uint8_t *)DMATxDescToSet->Buffer1Addr, buff, len);

    /* Setting the last segment and first segment bits (in this case a frame is transmitted in one descriptor) */
    DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;

    /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
    DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;

    /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
    /* Clear TBUS ETHERNET DMA flag */
    ETH->DMASR = ETH_DMASR_TBUS;
    /* Resume DMA transmission*/
    ETH->DMATPDR = 0;

    /* Update the ETHERNET DMA global Tx descriptor with next Tx descriptor */
    /* Chained Mode */
    /* Selects the next DMA Tx descriptor list for next buffer to send */
    DMATxDescToSet = (ETH_DMADESCTypeDef*) (DMATxDescToSet->Buffer2NextDescAddr);
    /* Return SUCCESS */
    return ETH_SUCCESS;
}

/*********************************************************************
 * @fn      ETH_Init
 *
 * @brief   Ethernet initialization.
 *
 * @return  none
 */
void ETH_Init( uint8_t *macAddr )
{
    ETH_Configuration( macAddr );
    ETH_DMATxDescChainInit(DMATxDscrTab, MACTxBuf, ETH_TXBUFNB);
    ETH_DMARxDescChainInit(DMARxDscrTab, MACRxBuf, ETH_RXBUFNB);
    PFIC_EnableIRQ(ETH_IRQn);
}

/******************************** endfile @ eth_driver ******************************/
