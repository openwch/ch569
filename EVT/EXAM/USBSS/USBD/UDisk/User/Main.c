/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 * @Note
 *  Example routine to emulate a simulate USB-disk Device,Use EMMC as storage medium.
*/

#include "CH56x_usb30.h"
#include "CH56x_common.h"
#include "CH56xusb30_LIB.h"
#include "CH56x_usb20.h"

/* Global Define */
#define	UART1_BAUD	115200

/* Global Variable */
UINT8V tx_lmp_port = 0;
UINT8V trans_err_flag = 0;
UINT8V link_sta = 0  ;
EMMC_PARAMETER  TF_EMMCParam;
extern UINT8V gcsw;
extern UINT32V emmcrst_gbf;
extern UINT8V RWsta_U2;
extern __attribute__ ((aligned(16))) UINT8  buf[512]   __attribute__((section(".DMADATA")));

void EMMC_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast" )));
void USBSS_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));
void LINK_IRQHandler (void)  __attribute__((interrupt("WCH-Interrupt-fast")));
void TMR0_IRQHandler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));

/*******************************************************************************
 * @fn      DebugInit
 *
 * @brief   Initializes the UART1 peripheral.
 *          baudrate: UART1 communication baud rate.
 *
 * @return  None
 */
void DebugInit(UINT32 baudrate)
{
	UINT32 x;
	UINT32 t = FREQ_SYS;
	x = 10 * t * 2 / 16 / baudrate;
	x = ( x + 5 ) / 10;
	R8_UART1_DIV = 1;
	R16_UART1_DL = x;
	R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
	R8_UART1_LCR = RB_LCR_WORD_SZ;
	R8_UART1_IER = RB_IER_TXD_EN;
	R32_PA_SMT |= (1<<8) |(1<<7);
	R32_PA_DIR |= (1<<8);

    R32_PA_SMT |= (1<<8) |(1<<7);
    R32_PA_DIR |= (1<<8);

}

/*******************************************************************************
 * @fn      EMMC_IO_init
 *
 * @brief   Emmc io initialization.
 *
 * @return  None
 */
void EMMC_IO_init(UINT8 a){
/* GPIO configuration */

    R32_PB_PU  |= bSDCMD;
    R32_PB_PU  |= (0x1f<<17);   //Data Line
    R32_PA_PU  |= (7<<0);

    R32_PB_DIR |= bSDCK;        //CLK Line
    if(1<a<6){
        R32_PA_DRV |= (7<<0);     // Drive Capacity
        R32_PB_DRV |= (0x1f<<17); // Drive Capacity
        R32_PB_DRV |= bSDCMD;     //Command Line
        R32_PB_DRV |= bSDCK;
    }
/* Controller Register */
    R8_EMMC_CONTROL = RB_EMMC_ALL_CLR
                    | RB_EMMC_RST_LGC;                // reset all register
    mDelaymS(1);
    if((a==0)||(a==1)||(a==6)||(a==7))
        R8_EMMC_CONTROL =  RB_EMMC_DMAEN ;             // Enable EMMCcard
    else{
        R8_EMMC_CONTROL =  RB_EMMC_DMAEN| RB_EMMC_NEGSMP;     // Enable EMMCcard
    }

    if(a < 4){
        R16_EMMC_CLK_DIV =  RB_EMMC_CLKOE|RB_EMMC_PHASEINV |LOWEMMCCLK;
    }
    else {
        R16_EMMC_CLK_DIV =  RB_EMMC_CLKOE | LOWEMMCCLK;
    }


/* Enable Interruption */
    R16_EMMC_INT_FG = 0xffff;
    R16_EMMC_INT_EN =
                    RB_EMMC_IE_FIFO_OV |          //Enable error Interruption
                    RB_EMMC_IE_TRANERR |
                    RB_EMMC_IE_DATTMO |
                    RB_EMMC_IE_REIDX_ER |
                    RB_EMMC_IE_RECRC_WR |
                    RB_EMMC_IE_RE_TMOUT;

/* Overtime */
    R8_EMMC_TIMEOUT = 15;   // calculating overtime
}


/*******************************************************************************
 * @fn      EMMC_function_init
 *
 * @brief   None
 *
 * @return  None
 */
UINT8 EMMC_function_init( UINT8 b){
    UINT8  sta;
    UINT32 i;
    EMMCResetIdle( &TF_EMMCParam );
    mDelaymS(30);
    printf("step - 1\n");
    //cmd1
    sta = EMMCReadOCR( &TF_EMMCParam );
    printf("step - 2\n");
    if(sta!=CMD_SUCCESS) return OP_FAILED;
    //cmd2
    sta = EMMCReadCID( &TF_EMMCParam );
    printf("step - 3\n");
    if(sta!=CMD_SUCCESS)    return OP_FAILED;
    //cmd3
    sta = EMMCSetRCA( &TF_EMMCParam );
    printf("step - 4\n");
    if(sta!=CMD_SUCCESS)    return OP_FAILED;
    //cmd9
    sta = EMMCReadCSD( &TF_EMMCParam );
    printf("step - 5\n");
    if(sta!=CMD_SUCCESS)    return OP_FAILED;
    //cmd7;
    sta = SelectEMMCCard( &TF_EMMCParam );
    printf("step - 6\n");
    if(sta!=CMD_SUCCESS)    return OP_FAILED;

    //cmd13
    sta = ReadEMMCStatus( &TF_EMMCParam );

    if(sta!=CMD_SUCCESS)    return OP_FAILED;
    printf("step - 7\n");

    if(TF_EMMCParam.EMMCSecNum == 0xFFF)
    {
        sta = EMMCCardReadEXCSD( &TF_EMMCParam, buf );
        if(sta!=OP_SUCCESS)
            return OP_FAILED;
        TF_EMMCParam.EMMCSecNum = *((PUINT32)&buf[212]);      // SEC_COUNT [215:212] MSB-LSB
    }
    printf("step - 8\n");

    //cmd6
    sta = EMMCSetBusWidth( &TF_EMMCParam, 2);
    if(sta!=CMD_SUCCESS)
        return OP_FAILED;
    R8_EMMC_CONTROL = (R8_EMMC_CONTROL&~RB_EMMC_LW_MASK) | bLW_OP_DAT8;     // 8line_mode

    printf("step - 8.5\n");
    while(1){
        UINT16 t;

        sta = ReadEMMCStatus( &TF_EMMCParam );    //cmd13
        mDelaymS(10);
        t=R16_EMMC_INT_FG;
        if(R16_EMMC_INT_FG){
           printf("CMD13 ERR:%04X\n",R16_EMMC_INT_FG);
           R16_EMMC_INT_FG=t;
        }
        else{
            printf("CMD13 stable\n");
            break;
        }
    }

    EMMCSetHighSpeed(&TF_EMMCParam);
    printf("step - 9\n");


    if(b < 4){
        R16_EMMC_CLK_DIV =  RB_EMMC_CLKMode| RB_EMMC_PHASEINV | RB_EMMC_CLKOE  | 9;
    }
    else {
        R16_EMMC_CLK_DIV =  RB_EMMC_CLKMode|RB_EMMC_CLKOE | 9;
    }

    while(1){
        UINT16 t;

        sta = ReadEMMCStatus( &TF_EMMCParam );    //cmd13
        mDelaymS(10);
        t=R16_EMMC_INT_FG;
        if(R16_EMMC_INT_FG){


           printf("CMD13 ERR:%04X\n",R16_EMMC_INT_FG);
           R16_EMMC_INT_FG=t;
        }
        else{
            printf("CMD13 stable\n");
            break;
        }
    }

    printf("step - 10\n");
    //ReadEXCSD
    if(TF_EMMCParam.EMMCSecNum == 0xFFF)
    {
        sta = EMMCCardReadEXCSD( &TF_EMMCParam, buf );
        if(sta!=OP_SUCCESS)
            return OP_FAILED;
        TF_EMMCParam.EMMCSecNum = *((PUINT32)&buf[212]);      // SEC_COUNT [215:212] MSB-LSB
    }

    return OP_SUCCESS;
}

/*******************************************************************************
 * @fn      emmc_init
 *
 * @brief   None
 *
 * @return  None
 */
void emmc_init(){
    UINT32 i,s;

    RWsta_U2 = 0xff;
    TF_EMMCParam.EMMCSecNum=0;
    while(!TF_EMMCParam.EMMCSecNum){
            i=0;
            for(i=0;i<8;i++)
            {
                printf("i:%d\n",i);
                EMMC_IO_init(i);
                s = EMMC_function_init( i );

                if(s != OP_SUCCESS)
                {
                    printf("Init Failed...\n");
                }
                else{
                    printf("Init succ,");
                    printf("EMMC Cap:%ld MB\n", TF_EMMCParam.EMMCSecNum/2048);
                    break;
                }
            }
        }

}

/*******************************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  None
 */
void main()
{
    UINT16 s;
    UINT8 i;

    SystemInit(FREQ_SYS);
	Delay_Init(FREQ_SYS);
    GPIOB_ResetBits( GPIO_Pin_12 );
    GPIOB_SetBits( GPIO_Pin_11 );
    GPIOB_ModeCfg( GPIO_Pin_12|GPIO_Pin_11, GPIO_Slowascent_PP_8mA );

    /* Configure serial port debugging */
	DebugInit(UART1_BAUD);
	printf("Udisk Program(120MHz) !\n");

	TF_EMMCParam.EMMCSecNum=0;
	mDelaymS(100);
	emmc_init();
	PFIC_EnableIRQ(EMMC_IRQn);

    Uinfo_init(  );
    R32_USB_CONTROL = 0;
	PFIC_EnableIRQ(USBSS_IRQn);
    PFIC_EnableIRQ(LINK_IRQn);
    PFIC_EnableIRQ(TMR0_IRQn);
    R8_TMR0_INTER_EN = 1;
    TMR0_TimerInit( 67000000 );
    USB30D_init(ENABLE);

	while(1)
	{
	    if(gcsw)
	    {
	        csw_U2();
	        gcsw=0;
	    }
	    if(emmcrst_gbf)
	    {
	        emmcrst_gbf=0;
	        emmc_init();
	    }
	}
}

/*******************************************************************************
 * @fn      TMR0_IRQHandler
 *
 * @brief   USB3.0 connection failure timeout processing
 *
 * @return  None
 */
void TMR0_IRQHandler()
{

    R8_TMR0_INTER_EN |= 1;
    PFIC_DisableIRQ(TMR0_IRQn);
    R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
    if(link_sta == 1 ){
        link_sta =0;
        PFIC_DisableIRQ(USBSS_IRQn);
        PFIC_DisableIRQ(LINK_IRQn);
        USB30D_init(DISABLE);
        return;
    }
    if(link_sta != 3){
        PFIC_DisableIRQ(USBSS_IRQn);
        PFIC_DisableIRQ(LINK_IRQn);
        USB30D_init(DISABLE);
        PRINT("USB2.0\n");
        R32_USB_CONTROL = 0;
        PFIC_EnableIRQ(USBHS_IRQn);
        USB20_dev_init(ENABLE);
    }
    link_sta=1;
    return;
}

/*******************************************************************************
 * @fn      USB30_BUS_RESET
 *
 * @brief   USB3.0 bus reset
 *
 * @return  None
 */
void USB30_BUS_RESET( )
{
    R8_SAFE_ACCESS_SIG = 0x57; // enable safe access mode
    R8_SAFE_ACCESS_SIG = 0xa8;
    R8_RST_WDOG_CTRL = 0x40 | RB_SOFTWARE_RESET;
}

/*******************************************************************************
 * @fn      LINK_IRQHandler
 *
 * @brief   USB3.0 Link Interrupt Handler.
 *
 * @return  None
 */
void LINK_IRQHandler (void)
{
    if(USBSS->LINK_INT_FLAG & LINK_Ux_EXIT_FLAG) // device enter U2
    {
        USBSS->LINK_CFG = CFG_EQ_EN | DEEMPH_CFG | TERM_EN;
        USB30_Switch_Powermode(POWER_MODE_0);
        USBSS->LINK_INT_FLAG = LINK_Ux_EXIT_FLAG;
    }
    if(USBSS->LINK_INT_FLAG & LINK_RDY_FLAG)    // POLLING SHAKE DONE
    {
        USBSS->LINK_INT_FLAG = LINK_RDY_FLAG;
        if(tx_lmp_port)                         // LMP, TX PORT_CAP & RX PORT_CAP
        {
            USBSS->LMP_TX_DATA0 = LINK_SPEED | PORT_CAP | LMP_HP;
            USBSS->LMP_TX_DATA1 = UP_STREAM | NUM_HP_BUF;
            USBSS->LMP_TX_DATA2 = 0x0;
            tx_lmp_port = 0;
        }
        /*Successful USB3.0 communication*/
        link_sta = 3;
        PFIC_DisableIRQ(TMR0_IRQn);
        R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
        R8_TMR0_INTER_EN = 0;
        PFIC_DisableIRQ(USBHS_IRQn);
        USB20_dev_init(DISABLE);
    }

    if(USBSS->LINK_INT_FLAG & LINK_INACT_FLAG)
    {
        USBSS->LINK_INT_FLAG = LINK_INACT_FLAG;
        USB30_Switch_Powermode(POWER_MODE_2);
    }
    if(USBSS->LINK_INT_FLAG & LINK_DISABLE_FLAG) // GO DISABLED
    {
        USBSS->LINK_INT_FLAG = LINK_DISABLE_FLAG;
        link_sta = 1;
        USB30D_init(DISABLE);
        PFIC_DisableIRQ(USBSS_IRQn);
        R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
        R8_TMR0_INTER_EN = 0;
        PFIC_DisableIRQ(TMR0_IRQn);
        PFIC_EnableIRQ(USBHS_IRQn);
        USB20_dev_init(ENABLE);
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
    if(USBSS->LINK_INT_FLAG & HOT_RESET_FLAG)  //The host may issue a hot reset. Pay attention to the endpoint configuration
    {
        USBSS->USB_CONTROL |= 1 << 31;
        USBSS->LINK_INT_FLAG = HOT_RESET_FLAG; // HOT RESET begin
        USBSS->UEP0_TX_CTRL = 0;
        USB30_IN_Set(ENDP_1, DISABLE, NRDY, 0, 0);
        USB30_OUT_Set(ENDP_2, NRDY, 0);
        USB30_Device_Setaddress(0);
        USBSS->LINK_CTRL &= ~TX_HOT_RESET; // HOT RESET end
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
 * @fn      USBSS_IRQHandler
 *
 * @brief   USB3.0 Interrupt Handler.
 *
 * @return  None
 */
void USBSS_IRQHandler (void)			//USBSS interrupt service
{
    USB30_IRQHandler();
}

/*******************************************************************************
 * @fn      EMMC_IRQHandler
 *
 * @brief   EMMC Interrupt Handler.
 *
 * @return  None
 */
void EMMC_IRQHandler(){
  UINT16 t;
  t=R16_EMMC_INT_FG;
  printf("itp:%04X\n",t);
  R16_EMMC_INT_FG = t;
  emmc_init();
}

