/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : USB3.0 和 USB2.0主机例程
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#include "CH56x_common.h"
#include "string.h"
#include "stdio.h"
#include "CH56X_ENMU.H"
#include "CHRV3UFI.h"
#include "CH56x_UDISK.h"
#include "fat_process.h"
#include "type.h"
#include "usbss.h"
#include "usbhs.h"
#define	FREQ_SYS	80000000
#define	STDIO_BAUD_RATE	115200				//baudrate 1152000BPS
#define NO_DEFAULT_ACCESS_SECTOR			1

__attribute__ ((aligned(4))) UINT8  RxBuffer[1] ;      // IN, must even address
__attribute__ ((aligned(4))) UINT8  TxBuffer[1] ;      // OUT, must even address

UINT32  DMAaddr = Usb_Tx_DMAaddr;
UINT8 tx_lmp_port = 0;
UINT8 enum_ready = 0;
UINT8 disk_max_lun;
unsigned long NTFS_BUF[ 128 ];//4K
UINT8 *pNTFS_BUF;
UINT8  U30_Check_Time = 0;      //U30检测超时

void LINK_IRQHandler (void) __attribute__((interrupt()));
void TMR0_IRQHandler (void) __attribute__((interrupt()));

/*******************************************************************************
 * @fn       DebugInit
 *
 * @brief    Initializes the UART3 peripheral.
 *
 * @param    baudrate: UART3 communication baud rate.
 *
 * @return   None
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
 * @fn      TMR0_IRQHandler
 *
 * @return  None
 */
void TMR0_IRQHandler( void )
{
	if( R8_TMR0_INT_FLAG & RB_TMR_IF_CYC_END ){
		R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;
		if( gDeviceConnectstatus == USB_INT_CONNECT_U20 ){
			 U30_Check_Time++;
			 if( U30_Check_Time>= 5 ){			//1S的超时时间
				 printf("u20_connect\n");
				 gDeviceConnectstatus = USB_INT_CONNECT;
				 U30_Check_Time = 0;
			 }
		}
	}
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
	UINT8   s;
	UINT8   s1;
    UINT8   *pd,*pd1;
    UINT8   buf[8];
	UINT32	k,cc,i,temp32,data;
	UINT32  *p32_txdma = (UINT32 *)DMAaddr;

	pNTFS_BUF = NTFS_ADDR;
	k = 0;

	SystemInit(FREQ_SYS);
	Delay_Init(FREQ_SYS);
	DebugInit(STDIO_BAUD_RATE);
	printf("This is usb3.0 host program(120MHz) !\n");
	usbhs_init();
	PFIC_EnableIRQ( LINK_IRQn );

	TMR0_TimerInit(8000000);
    R8_TMR0_INTER_EN |= RB_TMR_IE_CYC_END;
    PFIC_EnableIRQ( TMR0_IRQn );                    //定时器0中断使能

	mDelayuS(2000);


	while(1)
	{
		 mDelayuS(2);
		if( gDeviceConnectstatus == USB_INT_CONNECT )
		{
			if( gDeviceUsbType == USB_U30_SPEED ){
				s = U30HSOT_Enumerate( pNTFS_BUF );
			}
			else{
				s = U20HSOT_Enumerate( pNTFS_BUF );
			}
			printf("Enumerate_status=%02x\n",s);
			s = MS_Init(pNTFS_BUF);
			printf("udisk_init=%02x\n",s);
			s = Fat_Init();
			printf("end\n");
			printf("wait_disconnect\n");
			while(gDeviceConnectstatus == USB_INT_CONNECT){		//等待设备断开
				mDelaymS( 100 );

				if( gDeviceUsbType == USB_U20_SPEED ){
					R8_USB_INT_FG = RB_USB_IF_DETECT;
					if( R8_USB_MIS_ST & RB_USB_ATTACH ){

					}
					else   break;
				}
			}
			gDeviceConnectstatus = 0;
			gDeviceUsbType = 0;
			usbssh_init_reset();
			usbhs_init();
			printf("disconnect\n");
		}
		else if( R8_USB_INT_FG & RB_USB_IF_DETECT )		//先进行USB2.0设备连接，之后在进行USB3.0设备连接初始化
		{
			R8_USB_INT_FG = RB_USB_IF_DETECT;
			if( R8_USB_MIS_ST & RB_USB_ATTACH )
			{
				printf("USB2.0 DEVICE ATTACH !\n");
				gDeviceUsbType = USB_U20_SPEED;
				gDeviceConnectstatus = USB_INT_CONNECT_U20;
				mDelaymS(100);
				usbssh_init();		//初始化USB3.0寄存器
				R8_UHOST_CTRL |= RB_UH_BUS_RESET;
				mDelaymS(200);
				R8_UHOST_CTRL &= ~RB_UH_BUS_RESET;
			}
			else
			{
				printf("USB2.0 DEVICE DEATTACH !\n");
			}
		}
	}	
}

/*******************************************************************************
 * @fn       LINK_IRQHandler
 *
 * @return   None
 */
void LINK_IRQHandler (void)			//USBSSH interrupt severice
{
	UINT32 temp;

	temp = USBSS->LINK_ERR_STATUS;
	if( USBSSH->LINK_INT_FLAG & LINK_RECOV_FLAG )
	{
		USBSSH->LINK_INT_FLAG = LINK_RECOV_FLAG;
		printf("link recovery, error status = %0x\n", temp);
	}
	if( USBSSH->LINK_INT_FLAG & LINK_INACT_FLAG )
	{

		USBSSH->LINK_INT_FLAG = LINK_INACT_FLAG;
		switch_pwr_mode(POWER_MODE_2);
		printf("link inactive, error status = %0x\n", temp>>16);
	}
	else if( USBSS->LINK_INT_FLAG & LINK_DISABLE_FLAG ) // GO DISABLED
	{
		USBSSH->LINK_INT_FLAG = LINK_DISABLE_FLAG;
		USBSS->LINK_CTRL = POWER_MODE_2;// GO RX DETECT
			printf("go disabled \n");
	}
	else if( USBSSH->LINK_INT_FLAG & LINK_RX_DET_FLAG )
	{
		USBSSH->LINK_INT_FLAG = LINK_RX_DET_FLAG;

		printf("link is det !\n\n");
		switch_pwr_mode(POWER_MODE_2);
	}
	else if( USBSSH->LINK_INT_FLAG & TERM_PRESENT_FLAG ) // term present , begin POLLING
	{
		USBSSH->LINK_INT_FLAG = TERM_PRESENT_FLAG;
		if( USBSS->LINK_STATUS & LINK_PRESENT )
		{
			switch_pwr_mode(POWER_MODE_2);
			USBSSH->LINK_CTRL |= POLLING_EN;
			printf("rx term present!\n\n");
		}
		else
		{
			USBSSH->LINK_INT_CTRL = 0;
			printf("link is disconnect !\n\n");
			gDeviceConnectstatus = USB_INT_DISCONNECT;
			gDeviceUsbType = 0;
		}
	}
	else if( USBSSH->LINK_INT_FLAG & LINK_TXEQ_FLAG ) // POLLING SHAKE DONE
	{
		tx_lmp_port = 1;
		USBSSH->LINK_INT_FLAG = LINK_TXEQ_FLAG;
		if( USBSS->LINK_STATUS & LINK_PRESENT )
		{
			while(USBSS->LINK_STATUS & LINK_RX_DETECT);
			switch_pwr_mode(POWER_MODE_0);
			printf("link is tx EQ !%0x\n\n", USBSS->LINK_STATUS);
		}
	}
	else if( USBSSH->LINK_INT_FLAG & LINK_RDY_FLAG ) // POLLING SHAKE DONE
	{
		USBSSH->LINK_INT_FLAG = LINK_RDY_FLAG;
		if( tx_lmp_port ) // LMP, TX PORT_CAP & RX PORT_CAP
		{
			tx_lmp_port = 0;
			usbssh_lmp_init();
			printf("lmp rtx successfully !\n\n");
			enum_ready = 1;
			gDeviceConnectstatus = USB_INT_CONNECT;
			gDeviceUsbType = USB_U30_SPEED;
		}

	}

}








