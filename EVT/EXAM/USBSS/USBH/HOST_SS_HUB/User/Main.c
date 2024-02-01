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
 * This routine demonstrates CH569 as a USB host to operate HUB,and can enumerate the devices connected to HUB.
 * HUB that supports up to 4 ports.
 * Supports up to two levels of USB3.0 HUB.
 * Support for more levels of USB2.0 HUB.
*/

#include "CH56x_common.h"
#include "CH56xusb30h_LIB.h"
#include "CH56x_usb30h.h"
#include "CH56x_host_hs.h"
#include "hub.h"

/* Global Variable */
UINT8V U30_Check_Time = 0;


USB_HUB_Info ss_hub_info[SS_HUBNUM];
USB_HUB_Info hs_hub_info[HS_HUBNUM];
DEV_INFO_Typedef  thisUsbDev;
__attribute__ ((aligned(4))) UINT8  RxBuffer[1] ;      // IN, must even address
__attribute__ ((aligned(4))) UINT8  TxBuffer[1] ;      // OUT, must even address
__attribute__ ((aligned(16))) UINT8 pNTFS_BUF[512] __attribute__((section(".DMADATA")));

/* Function  */
void TMR0_IRQHandler( void ) __attribute__((interrupt("WCH-Interrupt-fast")));

/*******************************************************************************
 * @fn        TMR0_IRQHandler
 *
 * @brief     TMR0 handler.
 *
 * @return    None
 */
void TMR0_IRQHandler( void )
{
    if( R8_TMR0_INT_FLAG & RB_TMR_IF_CYC_END )
    {
        R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;
        if( gDeviceConnectstatus == USB_INT_CONNECT_U20 )
        {
             U30_Check_Time++;
             if( U30_Check_Time>= 5 ) //1 second timeout
             {
                 printf("u20_connect\n");
                 gDeviceConnectstatus = USB_INT_CONNECT;
                 U30_Check_Time = 0;
             }
        }
        ss_hub_info[0].time_out++;
        hs_hub_info[0].time_out++;
    }
}

/*******************************************************************************
 * @fn        DebugInit
 *
 * @brief     nitializes the UART1 peripheral.
 *
 * @param     baudrate - UART1 communication baud rate.
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
}

/*******************************************************************************
 * @fn        DebugInit
 *
 * @brief     nitializes the HUB variable.
 *
 * @param     none.
 *
 * @return    None
 */
void HUB_Init( void )
{
    memset( &ss_hub_info,0x00,sizeof(ss_hub_info) );    //Clear all variables
    ss_hub_info[0].devaddr = DEVICE_ADDR-1;             //Type of root HUB or other devices
    memset( &hs_hub_info,0x00,sizeof(ss_hub_info) );    //Clear all variables
    hs_hub_info[0].devaddr = DEVICE_ADDR;               //Type of root HUB or other devices
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main( )
{
    UINT8 s = 0;
    UINT8 depth = 0;
    UINT8 ss_rootnumofport;
    UINT8 hs_rootnumofport;

    SystemInit(FREQ_SYS);
    Delay_Init(FREQ_SYS);
    DebugInit(115200);
    PRINT("\n\nThis is USB3.0 host program(80MHz)\n");
    HUB_Init();
    PFIC_EnableIRQ(LINK_IRQn);           //enable USBSSH LINK global interrupt
    USB20Host_Init(ENABLE);              //USB2.0 initialization
    TMR0_TimerInit(FREQ_SYS);            //3.0link timeout timer initialization
    TMR0_ITCfg( ENABLE ,RB_TMR_IE_CYC_END);
    PFIC_EnableIRQ( TMR0_IRQn );
    Hub_LinkHead = InitHubLink();

    while(1)
    {
       mDelayuS(2);
       if( gDeviceConnectstatus == USB_INT_CONNECT )
       {
           if( gDeviceUsbType == USB_U30_SPEED )
           {
               printf("devaddr=%02x\n",ss_hub_info[depth].devaddr);
               USB30Host_Enum( depth,ss_hub_info[depth].devaddr ,0);
               if( ss_hub_info[depth].device_type == 0x09 )
               {
                   printf("U20_Init\n");
                   s = U20HOST_Enumerate( depth,pNTFS_BUF,hs_hub_info[depth].devaddr,0 );//enumerate USB2.0 HUB
               }
               if( ss_hub_info[depth].device_type == 0x09 )//The device under HUB starts processing
               {
                   ss_rootnumofport = ss_hub_info[depth].numofport;
                   hs_rootnumofport = hs_hub_info[depth].numofport;
                   while(1)
                   {
                         s = USBSS_HUB_Main_Process( depth ,ss_hub_info[depth].devaddr,0,0,ss_rootnumofport);
                         if( s == ERR_USB_DISCON )break;
                         s = USBHS_HUB_Main_Process( depth ,hs_hub_info[depth].devaddr,0,hs_rootnumofport);
                         if( s == ERR_USB_DISCON )break;
                  }
              }
           }
           else
           {
               s = U20HOST_Enumerate( depth,pNTFS_BUF,hs_hub_info[depth].devaddr,0);
               if(s != USB_INT_SUCCESS)
               {
                   printf("error:%02x\n",s);
               }
               if( hs_hub_info[depth].device_type == 0x09 )
               {
                   Global_Index = 0;
                   hs_rootnumofport = hs_hub_info[depth].numofport;
                   while(1)
                   {
                       s = USBHS_HUB_Main_Process( depth ,hs_hub_info[depth].devaddr,0,hs_rootnumofport);
                       if( s ==ERR_USB_DISCON )break;

                       if(! (R8_USB_MIS_ST&RB_USB_ATTACH) )
                       {
                           break;
                       }
                   }
               }
           }

           printf("end\n");
           while(gDeviceConnectstatus == USB_INT_CONNECT)   //Wait for the device to disconnect
           {
               mDelaymS( 500 );
               printf("wait device disconnect...\n");
               if( gDeviceUsbType == USB_U20_SPEED )
               {
                   R8_USB_INT_FG = RB_USB_IF_DETECT;
                   if(! (R8_USB_MIS_ST&RB_USB_ATTACH) )
                   {
                     printf("USB2.0 DEVICE DEATTACH !\n");
                     break;
                   }
               }
           }
           mDelaymS( 100 );
           mDelaymS( 100 );
           gDeviceConnectstatus = 0;
           gDeviceUsbType = 0;
           USB30HOST_Init(DISABLE,endpTXbuff,endpRXbuff);
           USB20Host_Init(ENABLE);                     //USB2.0 initialization
           printf("disconnect\n");
           mDelaymS(10);
           R8_SAFE_ACCESS_SIG = 0x57;                  //enable safe access mode
           R8_SAFE_ACCESS_SIG = 0xa8;
           R8_RST_WDOG_CTRL = 0x40 | RB_SOFTWARE_RESET;
           while(1);
       }
       else if( R8_USB_INT_FG & RB_USB_IF_DETECT )     //Connect the USB2.0 device first, and then initialize the USB3.0 device connection
       {
           R8_USB_INT_FG = RB_USB_IF_DETECT;
           if( R8_USB_MIS_ST & RB_USB_ATTACH )
           {
               printf("USB2.0 DEVICE ATTACH !\n");
               gDeviceUsbType = USB_U20_SPEED;
               gDeviceConnectstatus = USB_INT_CONNECT_U20;
               mDelaymS(100);
               USB30HOST_Init(ENABLE,endpTXbuff,endpRXbuff);
               USB20HOST_SetBusReset();
           }
           else
           {
               printf("USB2.0 DEVICE DEATTACH !\n");
           }
       }
    }
}


