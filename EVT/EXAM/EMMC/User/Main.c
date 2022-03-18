/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"

#define  FREQ_SYS   80000000

void EMMC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


EMMC_PARAMETER	TF_EMMCParam;
__attribute__ ((aligned(16))) UINT8	Sendbuff[512*2]  __attribute__((section(".DMADATA")));
__attribute__ ((aligned(16))) UINT8	Recvbuff[512*2]  __attribute__((section(".DMADATA")));
UINT32  KeyValue[] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
UINT32  CountValue[] = {0x00000001, 0x00000002, 0x00000003, 0x00000300};

/*******************************************************************************
 * @fn      DebugInit
 *
 * @brief   Initializes the UART1 peripheral.
 *
 * @param   baudrate - UART1 communication baud rate.
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
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main()
{
	SystemInit(FREQ_SYS);
	Delay_Init(FREQ_SYS);

/* configuration of UART on printing */
	DebugInit(115200);
	PRINT("Start @ChipID=%02X\r\n", R8_CHIP_ID );
#if 1

    UINT8   s;
    UINT16  i;

	PFIC_EnableIRQ(EMMC_IRQn);
	mDelaymS(40);
    R32_EMMC_TRAN_MODE = 0;                              //ensure EMMC clock
	EMMCIO0Init();

	mDelaymS(10);
	s = EMMCCardConfig( &TF_EMMCParam );
	mDelaymS(250);
	PRINT("SDLinkSatus:%x\r\n",TF_EMMCParam.EMMCLinkSatus);
	PRINT("SDCardSatus:%x\r\n",TF_EMMCParam.EMMCCardSatus);
	PRINT("SDVoltageMode:%x\r\n",TF_EMMCParam.EMMCVoltageMode);
	PRINT("SDSecSize:%x\r\n",TF_EMMCParam.EMMCSecSize);
	PRINT("SDSecNum:%x\r\n",TF_EMMCParam.EMMCSecNum);
	PRINT("SDOpErr:%x\r\n",TF_EMMCParam.EMMCOpErr);

	if(s != OP_SUCCESS)
	{
		PRINT("Init Failed...\n");
	}
	else
	{
		PRINT("Init Success...\n");
		PRINT("BlockSize:%d B \n", TF_EMMCParam.EMMCSecSize);
		PRINT("Cap:%ld MB \n", TF_EMMCParam.EMMCSecNum/2048);

		PRINT("Test read and write...\n");

		printf("Single Sec Read:\n");
		s = EMMCCardReadOneSec( &TF_EMMCParam, Recvbuff, 0x50000 );
		if( s == OP_SUCCESS )
		{
			for(i=0; i<(TF_EMMCParam.EMMCSecSize)*3; i++){
				PRINT("%02x", Recvbuff[i]);
			}PRINT("\n");
		}

		PRINT("MulSec Write:\n");
		if(Recvbuff[0] != 0x55) i = 0x55;
		else                    i = 0xaa;
		memset(&Sendbuff[0], i, TF_EMMCParam.EMMCSecSize);
		memset(&Sendbuff[TF_EMMCParam.EMMCSecSize], i+1, TF_EMMCParam.EMMCSecSize);
		for(i=0; i<(TF_EMMCParam.EMMCSecSize); i++){
			PRINT("%02x ", Sendbuff[i]);
		}PRINT("\n");


		//R16_EMMC_CLK_DIV |= RB_EMMC_PHASEINV;       // Clock inverting


		i = 2;
		s = EMMCCardWriteMulSec( &TF_EMMCParam, &i, Sendbuff, 0x50000);
		if( s != OP_SUCCESS ){
			PRINT("MulSec Write Failed !!!\n");
		}

		PRINT("MulSec Read:\n");
		i = 4;
		s = EMMCCardReadMulSec( &TF_EMMCParam, &i, Recvbuff, 0x50000 );
		if( s == OP_SUCCESS ){
			for(i=0; i<(TF_EMMCParam.EMMCSecSize)*2; i++){
				PRINT("%02x ", Recvbuff[i]);
			}PRINT("\n");


			PRINT("Test End...\n");
		}
	}
#endif

#if 0
	UINT8   s=0;
	UINT32  i=0, blocknum=2;
//加解密初始化
	ECDC_Init(MODE_AES_ECB, ECDCCLK_240MHZ, KEYLENGTH_128BIT, KeyValue, NULL);
//	ECDC_Init(MODE_AES_CTR, ECDCCLK_240MHZ, KEYLENGTH_128BIT, KeyValue, CountValue);

//emmc初始化
	EMMCIO0Init();
	s = EMMCCardConfig( &TF_EMMCParam );
	mDelayuS(250);

	PRINT("EMMCSecSize:%d\r\n",TF_EMMCParam.EMMCSecSize);
	if(s != OP_SUCCESS)
	{
		PRINT("Init Failed...\n");
	}
	else
	{
		PRINT("Init Success...\n");
		for(i=0;i<128*blocknum;i++)     //一块-512字节|一个地址对应4字节
		{
			*(BA_RAMX+i)=i;
		}
//写密文
		PRINT("write plaintext:\n");
		PRINT("R16_ECEC_CTRL:%08x\n",R16_ECEC_CTRL);

		ECDC_Excute( RAM_TO_PERIPHERAL_ENCRY, MODE_BIG_ENDIAN );


		s = EMMCCardWriteMulSec(&TF_EMMCParam, &blocknum, BA_RAMX, 0x5000);
		if( s != OP_SUCCESS ){
			PRINT("MulSec Write Failed !!!\n");
		}
//读密文
		PRINT("read ciphertext:\n");
		PRINT("|ADRESS	VALUE\t|\n");
		s = EMMCCardReadMulSec( &TF_EMMCParam, &blocknum, BA_RAMX, 0x5000);
			if( s == OP_SUCCESS ){
				for(i=0; i<128*blocknum; i++){
					if(i%4==0)	printf("\n");
					PRINT("|%#x	%#x\t", (BA_RAMX+i),*(BA_RAMX+i));
				}PRINT("\n");
			}
//读明文
		PRINT("read plaintext:\n");
		PRINT("|ADRESS	VALUE\t|\n");
		PRINT("R16_ECEC_CTRL:%08x\n",R16_ECEC_CTRL);

		ECDC_Excute( PERIPHERAL_TO_RAM_DECRY, MODE_BIG_ENDIAN );

		s = EMMCCardReadMulSec(&TF_EMMCParam, &blocknum, BA_RAMX, 0x5000);
		if( s == OP_SUCCESS )
		{
			for(i=0; i<128*blocknum; i++){
				if(i%4==0)	printf("\n");
				PRINT("|%#x	%#x\t", (BA_RAMX+i),*(BA_RAMX+i));
			}PRINT("\n");
		}
	}
#endif

	while(1);
}

/*********************************************************************
 * @fn      EMMC_IRQHandler
 *
 * @brief   This function handles EMMC exception.
 *
 * @return  none
 */
void EMMC_IRQHandler(void)   
{
	if(R16_EMMC_INT_FG)                      //Error interuption
	{
		PRINT("e:%04x\n", R16_EMMC_INT_FG);
	}
}

