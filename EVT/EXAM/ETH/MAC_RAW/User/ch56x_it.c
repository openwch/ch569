/********************************** (C) COPYRIGHT *******************************
* File Name          : ch56x_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/11/21
* Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch56x_it.h"
#include "eth_driver.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void ETH_IRQHandler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));
void TMR0_IRQHandler(void)  __attribute__((interrupt("WCH-Interrupt-fast")));
void GPIO_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
    printf("HardFault_Handler\r\n");

    printf("mepc  :%08x\r\n", __get_MEPC());
    printf("mcause:%08x\r\n", __get_MCAUSE());
    printf("mtval :%08x\r\n", __get_MTVAL());
    while(1);
}


/*********************************************************************
 * @fn      ETH_IRQHandler
 *
 * @brief   This function handles ETH exception.
 *
 * @return  none
 */
void ETH_IRQHandler(void)
{
    ETH_ETHIsr();
}

/*********************************************************************
 * @fn      TMR0_IRQHandler
 *
 * @brief   This function handles TIM0 exception.
 *
 * @return  none
 */
void TMR0_IRQHandler(void)
{
    if(R8_TMR0_INT_FLAG & RB_TMR_IF_CYC_END)
    {
        R8_TMR0_INTER_EN = RB_TMR_IF_CYC_END;
        R8_TMR0_INT_FLAG =  RB_TMR_IF_CYC_END;
    }
}

/*********************************************************************
 * @fn      GPIO_IRQHandler
 *
 * @brief   This function handles GPIO exception.
 *
 * @return  none
 */
void GPIO_IRQHandler(void)
{
    if(GPIOB_15_ReadITFlagBit())
    {
        ETH_PHYLink();
    }
    GPIOB_15_ClearITFlagBit();
}
