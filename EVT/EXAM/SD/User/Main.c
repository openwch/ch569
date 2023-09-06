/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : 
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 *SD routine
 * 
 */

#include "CH56x_common.h"
#include "SD.h"

#define  FREQ_SYS   80000000

#define UART1_BAUD  921600

EMMC_PARAMETER  TF_EMMCParam;
UINT8V trans_err_flag = 0;

__attribute__ ((aligned(16))) UINT8  test_data_buffer[5120]   __attribute__((section(".DMADATA")));
void EMMC_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast" )));

/***************************************************************
 * @fn        DebugInit
 * 
 * @brief     Initializes the UART1 peripheral.
 * 
 * @param     baudrate: UART1 communication baud rate.
 * 
 * @return    None
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

/*****************************************************************
 * @fn     EMMC_IO_init
 *
 * @brief  Init EMMC GPIO.
 *
 * @return None
 */
void EMMC_IO_init(UINT8 a)
{
    /* GPIO configuration */
    R32_PB_PU  |= bSDCMD;
    R32_PB_PU  |= (0x1f<<17);   //Data Line
    R32_PA_PU  |= (7<<0);

    R32_PB_DIR |= bSDCK;        //CLK Line
    if(1<a<6){
        R32_PA_DRV |= (7<<0);     // Drive Capacity
        R32_PB_DRV |= (0x1f<<17); // Drive Capacity
        R32_PB_DRV |= bSDCMD;       //Command Line
        R32_PB_DRV |= bSDCK;
    }

    /* Controller Register */
    R8_EMMC_CONTROL = RB_EMMC_ALL_CLR | RB_EMMC_RST_LGC;                // reset all register

    if((a==0)||(a==1)||(a==6)||(a==7))
        R8_EMMC_CONTROL =  RB_EMMC_DMAEN ;                              // Enable EMMCcard
    else{
        R8_EMMC_CONTROL =  RB_EMMC_DMAEN | RB_EMMC_NEGSMP;              // Enable EMMCcard
    }

    R8_EMMC_CONTROL = (R8_EMMC_CONTROL & (~RB_EMMC_LW_MASK) ) | bLW_OP_DAT0;   // 4line_mode

    if(a < 4){
        R16_EMMC_CLK_DIV =  RB_EMMC_CLKOE | RB_EMMC_PHASEINV | LOWEMMCCLK;
    }
    else {
        R16_EMMC_CLK_DIV =  RB_EMMC_CLKOE | LOWEMMCCLK;
    }

    /* Enable Interruption */
    R16_EMMC_INT_FG = 0xffff;
    R16_EMMC_INT_EN = RB_EMMC_IE_FIFO_OV |          //Enable error Interruption
                    RB_EMMC_IE_TRANERR |
                    RB_EMMC_IE_DATTMO |
                    RB_EMMC_IE_REIDX_ER |
                    RB_EMMC_IE_RECRC_WR |
                    RB_EMMC_IE_RE_TMOUT;

    /* Overtime */
    R8_EMMC_TIMEOUT = 14;   // calculating overtime
}

UINT8V gb_err_flag=0;

/*****************************************************************
 * @fn     EMMC_function_init
 *
 * @brief  Init EMMC function.
 *
 * @return OP_SUCCESS
 *         OP_FAILED
 */
UINT8 EMMC_function_init( UINT8 b){
    UINT8  sta;
    UINT32 i;

    EMMCResetIdle( &TF_EMMCParam );
    mDelaymS(30);

    EMMCResetIdle( &TF_EMMCParam );
    mDelaymS(30);

    for( i = 0; i < 3; i++ )
    {
        EMMCSendCmd(0x01AA,  RB_EMMC_CKIDX | RB_EMMC_CKCRC |RESP_TYPE_48  | EMMC_CMD8  );
        while(1)
        {
            sta = CheckCMDComp( &TF_EMMCParam );
            if( sta!= CMD_NULL  )
            {
                break;
            }
        }
        if(sta != CMD_SUCCESS)
            mDelaymS(30);
        else {
            goto eee;
        }
    }
    return(OP_FAILED);

    R16_EMMC_CLK_DIV |=  RB_EMMC_CLKMode;
eee:
    //acmd41
    sta = SDReadOCR( &TF_EMMCParam );
    if(sta!=CMD_SUCCESS) return OP_FAILED;
    //cmd2
    sta = EMMCReadCID( &TF_EMMCParam );
    if(sta!=CMD_SUCCESS)    return OP_FAILED;

    //cmd3
    sta = SDSetRCA( &TF_EMMCParam );

    if(sta!=CMD_SUCCESS)    return OP_FAILED;
    //cmd9
    sta = SDReadCSD( &TF_EMMCParam );

    if(sta!=CMD_SUCCESS)    return OP_FAILED;
    //cmd7;
    mDelaymS( 5 );
    sta = SelectEMMCCard( &TF_EMMCParam );

    if(sta!=CMD_SUCCESS)    return OP_FAILED;


    //cmd6
    sta = SDSetBusWidth( &TF_EMMCParam, 1);
    if(sta!=CMD_SUCCESS)
        return OP_FAILED;
    R8_EMMC_CONTROL = (R8_EMMC_CONTROL & (~RB_EMMC_LW_MASK) ) | bLW_OP_DAT4;


    sta = SD_ReadSCR(&TF_EMMCParam, test_data_buffer);
    if(sta!=CMD_SUCCESS)
        return OP_FAILED;


    if(b < 4){
        R16_EMMC_CLK_DIV =  RB_EMMC_CLKMode | RB_EMMC_PHASEINV | RB_EMMC_CLKOE  | 10;   //5 frequency division 96M clock
    }
    else {
        R16_EMMC_CLK_DIV =  RB_EMMC_CLKMode | RB_EMMC_CLKOE | 10;
    }

    return OP_SUCCESS;
}

/*****************************************************************
 * @fn     main
 * 
 * @brief  Main program.
 * 
 * @return None
 */
int main()
{
    UINT16 s;
    UINT8 i;

    SystemInit(FREQ_SYS);
    Delay_Init(FREQ_SYS);

/* Configure serial debugging */
    DebugInit(UART1_BAUD);
    printf("EMMC Program(120MHz) !\n");

    mDelaymS(10);
    for(i=0;i<8;i++)//IO Adaptive
    {
        printf("IO mode:%d\n",i);
        EMMC_IO_init(i);
        s = EMMC_function_init( i );
        if(s != OP_SUCCESS)
        {
            printf("Init Failed...\n");
            continue;
        }
        else{
            printf("INTI DOWN\n");
            break;
        }
    }
    while(s != OP_SUCCESS);
    PFIC_EnableIRQ(EMMC_IRQn);
    TF_EMMCParam.EMMCOpErr = 0;

    printf("EMMC LinkSatus:%x\r\n",TF_EMMCParam.EMMCLinkSatus);
    printf("EMMC CardSatus:%x\r\n",TF_EMMCParam.EMMCCardSatus);
    printf("EMMC VoltageMode:%x\r\n",TF_EMMCParam.EMMCVoltageMode);
    printf("EMMC SecSize:%d\r\n",TF_EMMCParam.EMMCSecSize);
    printf("EMMC SecNum:%x\r\n",TF_EMMCParam.EMMCSecNum);
    printf("EMMC OpErr:%x\r\n",TF_EMMCParam.EMMCOpErr);
    printf("EMMC BlockSize:%d B \n", TF_EMMCParam.EMMCSecSize);
    printf("EMMC Cap:%ldMB \n", TF_EMMCParam.EMMCSecNum/2048);


//============================================================
    printf("\n\nread one sec\n");
    i=SDCardReadOneSec( &TF_EMMCParam, test_data_buffer, 0 );

    for(s=0;s<512;s++)
    {
        if(!(s%16))
            printf("\n");
        printf("%02x ",test_data_buffer[s]);
    }
    printf("\n");

//============================================================
    printf("\n\nread one sec2:\n");
    SDCardReadOneSec( &TF_EMMCParam, test_data_buffer, 0 );
    for(s=0;s<512;s++)
    {
        if(!(s%16))
            printf("\n");
        printf("%02x ",test_data_buffer[s]);
    }
//============================================================

    printf("\n\nwrite %d sec:\n",i);
    for(s=0;s<512;s++)
    {
        test_data_buffer[s] =~test_data_buffer[s];
    }

    SDCardWriteONESec( &TF_EMMCParam , test_data_buffer, 0 );
//============================================================
    printf("\n\nread one sec:\n");
    for(s=0;s<512;s++)
    {
        test_data_buffer[s] = 0xff;
    }
    SDCardReadOneSec( &TF_EMMCParam, test_data_buffer, 0);
    for(s=0;s<512;s++)
    {
        if(!(s%16))
            printf("\n");
        printf("%02x ",test_data_buffer[s]);
    }
    printf("\n");

    while(1);
}


/**************************************************************
 * @fn        EMMC_IRQHandler
 *
 * @brief     EMMC Interrupt Handler.
 *
 * @brief     None
 *
 * @return    None
 */
void EMMC_IRQHandler(){

    UINT16 t;

    TF_EMMCParam.EMMCOpErr = 1;
    t= R16_EMMC_INT_FG;

    R16_EMMC_INT_FG =  t& (   RB_EMMC_IF_FIFO_OV
                              |RB_EMMC_IF_TRANERR
                              |RB_EMMC_IF_DATTMO
                              |RB_EMMC_IF_REIDX_ER
                              |RB_EMMC_IF_RECRC_WR
                              |RB_EMMC_IF_RE_TMOUT
                              );
    gb_err_flag=1;
}
