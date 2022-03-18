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

UINT32  KeyValue[] = {0x55acd4c5, 0x97d4570e, 0xb89464ba, 0xe4a0556b, 0x84af48fd, 0x51af5d2e, 0xadec514f, 0x9642cadd};
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

/*******************************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return None
 */
int main()
{  
    SystemInit(FREQ_SYS);
//    Delay_Init(FREQ_SYS);

/* 配置串口调试 */
	DebugInit(115200);
	PRINT("Start @ChipID=%02X\r\n", R8_CHIP_ID );

/* 单次寄存器*/
#if 0
//	ECDC_Init(MODE_AES_ECB, ECDCCLK_240MHZ, KEYLENGTH_128BIT, KeyValue, NULL);
//	ECDC_Init(MODE_AES_CTR, ECDCCLK_240MHZ, KEYLENGTH_128BIT, KeyValue, CountValue);
//	ECDC_Init(MODE_SM4_ECB, ECDCCLK_240MHZ, KEYLENGTH_128BIT, KeyValue, NULL);
	ECDC_Init(MODE_SM4_CTR, ECDCCLK_240MHZ, KEYLENGTH_128BIT, KeyValue, CountValue);

	UINT32 plaintext[4]={0x11112222, 0x33334444, 0x55556666, 0x77778888},ciphertext[4]={0};

/*打印加密后的密文*/
	PRINT("encryption:\n");
	ECDC_Excute(SINGLEREGISTER_ENCRY, MODE_BIG_ENDIAN);
	PRINT("%#X \n",R16_ECEC_CTRL);
	ECDC_SingleRegister( plaintext, ciphertext);
	for(UINT8 i=0; i<4; i++)
		printf("%08x\n",ciphertext[i]);
/*打印解密后的明文*/
	PRINT("decryption:\n");
	ECDC_Excute(SINGLEREGISTER_DECRY, MODE_BIG_ENDIAN);
	PRINT("%#X \n",R16_ECEC_CTRL);
	ECDC_SingleRegister(ciphertext, plaintext);
	for(UINT8 i=0; i<4; i++)
		PRINT("%08x\n",plaintext[i]);
#endif
/* RAMX */
#if 0
	ECDC_Init(MODE_AES_ECB, ECDCCLK_240MHZ, KEYLENGTH_128BIT, KeyValue, NULL);
//	ECDC_Init(MODE_AES_CTR, ECDCCLK_240MHZ, KEYLENGTH_128BIT, KeyValue, CountValue);
//	ECDC_Init(MODE_SM4_ECB, ECDCCLK_240MHZ, KEYLENGTH_128BIT, KeyValue, NULL);
//	ECDC_Init(MODE_SM4_CTR, ECDCCLK_240MHZ, KEYLENGTH_128BIT, KeyValue, CountValue);
/*写明文*/
	UINT32 i=0, len=64;
	for(i=0; i<len; i++)
		*(UINT32*)(0x20020000+i*4) = i;
/*打印加密后的密文*/
	PRINT("encryption:\n");
	ECDC_Excute(SELFDMA_ENCRY, MODE_BIG_ENDIAN);
	ECDC_SelfDMA((UINT32)BA_RAMX, len);
	for(i=0; i<len; i++)
		PRINT("|%#x	%#x\n",(UINT32*)(0x20020000+i*4),*(UINT32*)(0x20020000+i*4));
/*打印解密后的明文*/
	PRINT("decryption:\n");
	ECDC_Excute(SELFDMA_DECRY, MODE_BIG_ENDIAN);
	ECDC_SelfDMA((UINT32)BA_RAMX, len);
	for(i=0; i<len; i++)
		PRINT("|%#x	%#x\n",(UINT32*)(0x20020000+i*4),*(UINT32*)(0x20020000+i*4));
#endif
	while(1);
}





