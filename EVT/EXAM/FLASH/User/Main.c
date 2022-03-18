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
#include "ISPEM569.h"

#define FREQ_SYS    80000000

UINT8 my_buffer[1024];

/*******************************************************************************
 * @fn      DebugInit
 *
 * @brief   Initializes the UART1 peripheral.
 *
 * @param   baudrate: UART1 communication baud rate.
 *
 * @return  None
 */
void DebugInit(UINT32 baudrate)
{
    UINT32 x;
    UINT32 t = FREQ_SYS;

    x = 10 * t * 2 / 16 / baudrate;
    x = (x + 5) / 10;
    R8_UART1_DIV = 1;
    R16_UART1_DL = x;
    R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART1_LCR = RB_LCR_WORD_SZ;
    R8_UART1_IER = RB_IER_TXD_EN;
    R32_PA_SMT |= (1 << 8) | (1 << 7);
    R32_PA_DIR |= (1 << 8);
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

    DebugInit(115200);
    PRINT("Start @ChipID=%02X\r\n", R8_CHIP_ID);

    /* unique ID */
    UINT8 i = 0;
    GET_UNIQUE_ID(my_buffer);
    PRINT("chip id: ");
    for(i = 0; i < 8; i++)
        PRINT("%02x ", my_buffer[i]);
    PRINT("\n");

/* Test CodeFlash */
#if 1

    PRINT("Flash Erase:\n");
    i = FLASH_ROMA_ERASE(0X50000, 1024 * 4); //Erase 4K
    if(i)
    {
        PRINT("    ***Erase Error***\n");
    }
    else
    {
        PRINT("    ***erase ok***\n");
    }
    for(i = 0; i < 64; i++)
        my_buffer[i] = i;
    PRINT("Flash Write:\n");
    i = FLASH_ROMA_WRITE(0X50000, my_buffer, 64); //Write 64K
    if(i)
    {
        PRINT("    ***Write Error***\n");
    }
    else
    {
        PRINT("    ***write ok***\n");
    }
    PRINT("Flash Verify:\n");
    i = FLASH_ROMA_VERIFY(0X50000, my_buffer, 64);
    if(i)
    {
        PRINT("    ***Verify Error***\n");
    }
    else
    {
        PRINT("    ***verify ok***\n");
    }
    PRINT("\n");

#endif

    /* Test DataFlash */
#if 1

    PRINT("Eeprom Erase:\n");
    i = EEPROM_ERASE(0x6000, 1024 * 4);
    if(i)
    {
        PRINT("    ***Error***\n");
    }

    for(i = 0; i < 64; i++)
        my_buffer[i] = i;
    PRINT("Eeprom Write:\n");
    i = EEPROM_WRITE(0x6000, my_buffer, 64);
    if(i)
    {
        PRINT("    ***Error***\n");
    }

    PRINT("Eeprom Read:\n");
    EEPROM_READ(0x6000, my_buffer, 64);
    for(i = 0; i < 64; i++){
        if(i % 8 == 0)
            PRINT("\n");
        PRINT("%02x ", my_buffer[i]);
    }
    PRINT("\n");

#endif

    while(1);
}
