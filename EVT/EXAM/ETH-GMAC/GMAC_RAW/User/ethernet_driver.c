/********************************** (C) COPYRIGHT *******************************
* File Name          : ethernet_driver.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : CH565/569 千兆以太网链路层收发接口函数实现
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

/*-----------------------------------------------头文件包含----------------------------- -------------*/
#include "ethernet_driver.h"
#include "CH56x_common.h"
#include "ethernet_config.h"

#ifdef USE_USB125M/* 使用使用内部125MHz时钟 */
#include "usbss.h"
#endif

#ifdef USE_ETH_PHY_INTERRUPT
#include "PHY_interrupt.h"
#endif
/*-----------------------------------------------宏定义-----------------------------------------------*/


/*----------------------------------------------全局变量-----------------------------------------------*/
volatile  Globe_RxDes_status_t Globe_RxDes_status;/* 全局接收状态 */
extern UINT8  local_mac[];/* 本地MAC地址，定义在main.c中 */
volatile extern UINT8 enable_send;
volatile extern UINT8 rece_timeout_cnt;    /* 接收超时计时器 */

#ifdef USE_ETH_PHY_INTERRUPT
extern globe_eth_status_t globe_eth_status; /* 全局以太网状态 */
#endif

/*******************************************************************************
 * @fn      ETH_GPIO_Init
 *
 * @brief   ETH GPIO initialization
 *
 * @return  None
 */
void ETH_GPIO_Init(void)
{
#ifdef USE_RMII
//	(*((PUINT8V)0x40001000+0x12))&=~0x01;/* 使用RMII接口 */
	R8_PIN_ALTERNATE&=~RB_PIN_MII;/* 使用RMII接口 */
#endif
	/* 需要注意CH565/569的MII接口、SMI接口、125M时钟的输入和EMCO的输出管脚的方向  */
	/* PB13 RXDV / PB12 RXD0 / PB11 RXD1 / PB10 RXD2 / PB9 RXD3 / PB8 RXC  */
	/* PB7  ETXC / PB6 ETXEN / PB5 TXD0 / PB4 TXD1 / PB3 TXD2 / PB2 TXD3 /PB1 EMDCK / PB0 EMDIO*/
	/* PA16 ETCKI / PA20 EMCO / */
	R32_PA_DIR |=  bEMCO;/* PA20做MCO输出 */
	R32_PA_DIR &=~ bEMCI;/* PA16做ETCKI输入 */
	R32_PB_DIR |=  bMDCK|
#ifndef USE_RMII
			bETHT3|bETHT2|/* 使用RMII时只用两根数据线 */
#endif
			bETHT1|bETHT0|bETHTEN|bETHTC;/* RGMII的TXC、TXEN、TXD0/1/2/3和MDCK置为输出 */
	R32_PB_DIR &=~ bETHRC|
#ifndef USE_RMII
			bETHR3|bETHR2|/* 使用RMII时只用两根数据线 */
#endif
			bETHR1|bETHR0|bETHRDV|bMDIO;/* RGMII的RXC、RXDV、RXD0/1/2/3和MDIO置为输入 */
}

/*******************************************************************************
 * @fn       RGMII_TXC_Delay
 *
 * @brief    ETH send timing adjustment
 *
 * @param    clock_polarity:send clock polarity
 *           delay_time:delay time(unit : half second)
 *
 * @return   None
 */
void RGMII_TXC_Delay(UINT8 clock_polarity, UINT8 delay_time)
{
	if(clock_polarity)
		ETH->MACCR |= (UINT32)(1<<1);/* 发送时钟反向 */
	else
		ETH->MACCR &=~(UINT32)(1<<1);/* 发送时钟反向 */
	if(delay_time<=7)
		ETH->MACCR |= (UINT32)(delay_time<<29);
	else
		printf("Error:delay_time is out of range!\n");
}

/*******************************************************************************
 * @fn       ETH_Mem_Malloc
 *
 * @brief    ETH transceiver queue ram space application
 *
 * @return   Execution status
 */
UINT8 ETH_Mem_Malloc(void)
{
	UINT8* PBUF=ETH_queue_base;/* 地址映射 */
	/* 需要申请一个连续的地址空间，大小大概为((RX_Des_Num+TX_Des_Num)*16 +RX_Des_Num*RX_Buf_Size+TX_Buf_Size*TX_Des_Num)) */
	/* 实际的申请步骤在主函数开始时向编译器申请 */
	memset(PBUF,0,((RX_Des_Num+TX_Des_Num)*16 +RX_Des_Num*RX_Buf_Size+TX_Buf_Size*TX_Des_Num));/* 清零 */

	return 0;
}

/*******************************************************************************
 * @fn       ETH_buf_init
 *
 * @brief    ETH transceiver queue initialization
 *
 * @return   Execution status
 */
UINT8 ETH_buf_init(void)
{
#if 0  /*是否在初始化时打印出缓冲区和描述符列表的首地址  */
	printf("DMA_tx_des:0x%08x\n",pDMATxDscrTab);
	printf("DMA_Rx_des:0x%08x\n",pDMARxDscrTab);
	printf("DMA_Tx_BUF:0x%08x\n",pTx_Buff);
	printf("DMA_rx_BUF:0x%08x\n",pRx_Buff);
#endif
	ETH_MACAddressConfig(ETH_MAC_Address0,local_mac);         /* 设置本地MAC地址 */
	ETH_DMATxDescChainInit(pDMATxDscrTab, pTx_Buff,TX_Des_Num);/* 设置发送缓冲区和描述符 */
	ETH_DMARxDescChainInit(pDMARxDscrTab, pRx_Buff,RX_Des_Num);/* 设置接收缓冲区和描述符 */
	Globe_RxDes_status.LastRxDes=(ETH_DMADESCTypeDef*)pDMARxDscrTab;/* 待处理的帧地址赋初值 */

	return 0;
}

#ifdef USE_USB125M
void myusbssh_init ()	// USBSS host initial
{
	USBSSH->UH_TX_CTRL = 0;
	USBSSH->UH_RX_CTRL = 0;
	USBSSH->LINK_CFG = HP_PENDING | CFG_EQ_EN | DEEMPH_CFG | DOWN_FLAG;// downstream
	USBSSH->LINK_CTRL = POWER_MODE_2 | GO_DISABLED;		// change U3-PHY enter P2

	while( USBSSH->LINK_STATUS & LINK_BUSY ); 			// wait link_busy=0
	printf("power mode: P2 \n");
	USBSSH->USB_CONTROL = HOST_MODE | ITP_EN | INT_BUSY_EN | DMA_EN;// host_mode, release link reset
	USBSSH->UH_TX_CTRL = 0x0;
	USBSSH->USB_STATUS = USB_ACT_FLAG | USB_LMP_TX_FLAG | USB_LMP_RX_FLAG | USB_OV_FLAG;
//	USBSSH->UH_TX_DMA = Usb_Tx_DMAaddr; 					// set tx dma address
//	USBSSH->UH_RX_DMA = Usb_Rx_DMAaddr; 					// set rx dma address
	USBSSH->UEP_CFG = UH_T_EN | UH_R_EN;				// set HOST rx/tx enable
	printf("TERM ENABLE \n");
	USBSS->LINK_CFG |= TERM_EN; // term enable
	USBSS->LINK_INT_CTRL = LINK_DISABLE_IE | LINK_INACT_IE | LINK_RX_DET_IE | TERM_PRESENT_IE | LINK_TXEQ_IE | LINK_RDY_IE;

	USBSS->LINK_CTRL = POWER_MODE_2; // GO RX DETECT
}

/*******************************************************************************
 * @fn      switch_pwr_mode
 *
 * @return  None
 */
void myswitch_pwr_mode(UINT8 pwr_mode)
{
    UINT32 temp;
    temp = USBSS->LINK_CTRL;
    temp &= ~PM_MASK;
    temp |= pwr_mode;
    USBSS->LINK_CTRL = temp;
    while( USBSS->LINK_STATUS & LINK_BUSY );            // wait power mode switch done
}
#endif

/*******************************************************************************
 * @fn      ETH_uClock_enable
 *
 * @brief   ETH peripheral clock initialization
 *
 * @return  None
 */
void ETH_uClock_enable(void)
{
	// config ethernet clock frequency
#ifdef USE_USB125M
	myusbssh_init();
	myswitch_pwr_mode(POWER_MODE_2);
#endif
	R8_SAFE_ACCESS_SIG = 0x57; // enable safe access mode
	R8_SAFE_ACCESS_SIG = 0xa8;

	R8_CLK_MOD_AUX=RB_MCO_EN|    /* 使/能MCO频率输出 */
				  RB_SEL_25M;    /* 使能25M频率输出 */
#ifdef USE_RMII
	R8_CLK_MOD_AUX = R8_CLK_MOD_AUX & (~RB_SRC_125M_MSK) | RB_INT_125M_EN;
#else
#ifndef USE_USB125M
	R8_CLK_MOD_AUX = (R8_CLK_MOD_AUX&(~RB_SRC_125M_MSK))|RB_EXT_125M_EN;
#else
	R8_CLK_MOD_AUX = (R8_CLK_MOD_AUX&(~RB_SRC_125M_MSK))|RB_INT_125M_EN;
#endif
	/* TXC的来源除了内部和外部的125M，还可以使用RXC当做TXC，此时需要置MACCR[0]为高  */
#endif
	R8_SAFE_ACCESS_SIG = 0                   /* 退出安全模式 */;
}

/*******************************************************************************
 * @fn        rtl8211dn_Get_Speed
 *
 * @brief     Get PHY link speed
 *
 * @return    link speed
 */
Link_Speed_t rtl8211dn_Get_Speed(void)
{
	Link_Speed_t speed;
	UINT16 val;

	val=ETH_ReadPHYRegister(PHY_ADDRESS,0x11); //从的31号寄存器中读取网络速度和双工模式
	switch(val&0xc000)
	{
		case 0xc000:return ETH_disconnect;
		case 0x0000:return ETH_10M;
		case 0x4000:return ETH_100M;
		case 0x8000:return ETH_Gigabit;
		default:break;
	}
	return speed;
}

/*******************************************************************************
 * @fn       ENABLE_ETH_WAKEUP
 *
 * @brief    ENABLE ETH wakeup
 *
 * @return   None
 */
void ENABLE_ETH_WAKEUP(void)
{
	R8_SAFE_ACCESS_SIG = 0x57; // enable safe access mode
	R8_SAFE_ACCESS_SIG = 0xa8;
    R8_SLP_WAKE_CTRL |= RB_SLP_ETH_WAKE;
    R8_SAFE_ACCESS_SIG = 0;
}

/*******************************************************************************
 * @fn          ETH_MACDMA_Config
 *
 * @brief       Mac & DMA config initilization
 *
 * @return      Execution status
 */
UINT8 ETH_MACDMA_Config(void)
{
	unsigned char rval=100;
	ETH_InitTypeDef ETH_InitStructure; 
	
	ETH_uClock_enable();	                    /* 使能以太网时钟 */
	ETH_SoftwareReset();  						/* 软件重启网络 */
	mDelaymS(10);
	while ( (ETH_GetSoftwareResetStatus() == SET)&&(rval >= 0) )
	{
		PRINT("正在进行以太网软件复位.\n");
		mDelaymS(10);
		rval--;
		if(rval <= 0)
		{
			printf("软复位超时！\n");				/* 以太网软复位须有正常的125M时钟输入和RXC 125MHz时钟 */
			printf("*********Please cheak 125MHz clock and Rxc clock!*********\n");
			break;
		}
	}/* 等待软件重启网络完成，如果现实软复位超时，请检查EMCO是否有设置的波形输出和RXC是否有125MHz输出 */
	ETH->MACCR |= ETH_MACCR_RE;
	ETH_StructInit(&ETH_InitStructure); 	 	/* 初始化网络为默认值   */
	rval=ETH_Init(&ETH_InitStructure,PHY_ADDRESS);/* 配置ETH */
	if(rval==ETH_SUCCESS)//配置成功
	{
		ETH_DMAITConfig(ETH_DMA_IT_NIS
						|ETH_DMA_IT_T/* 使能发送完成中断 */
						|ETH_DMA_IT_R /* 使能接收完成中断 */
//						|ETH_DMA_IT_TBU,/* 使能发送描述符不可用中断 */
						,ENABLE );//使能以太网发送中断
	}
	else
	{
		printf("Error:ETH_MACDMA_Config failed!!!\n");
		return ETH_ERROR;
	}
		PFIC_EnableIRQ(ETH_IRQn);
//		printf("ETH_MACDMA_Config finished!\n");
		return rval;
}

/*******************************************************************************
 * @fn       ETH_MACDMA_reConfig
 *
 * @brief    Mac & DMA config reinitilization
 *
 * @return   Execution status
 */
UINT8 ETH_MACDMA_reConfig(void)
{
	unsigned char rval=100;
	ETH_InitTypeDef ETH_InitStructure;

	PFIC_DisableIRQ(ETH_IRQn);
	ETH->DMABMR |= ETH_DMABMR_SR;						/* 软件重启网络 */
	mDelaymS(10);
	while ( (ETH_GetSoftwareResetStatus() == SET)&&(rval >= 0) )
	{
		PRINT("正在进行以太网软件重启.\n");
		mDelaymS(10);
		rval--;
		if(rval <= 0)
		{
			printf("软复位超时！\n");				/* 以太网软复位须有正常的125M时钟输入和RXC 125MHz时钟 */
			printf("*********Please cheak 125MHz clock and Rxc clock!*********\n");
			break;
		}
	}/* 等待软件重启网络完成，如果现实软复位超时，请检查EMCO是否有设置的波形输出和RXC是否有125MHz输出 */
	ETH->MACCR |= ETH_MACCR_RE;
	ETH_StructInit(&ETH_InitStructure); 	 	/* 初始化网络为默认值   */
	rval=ETH_Init(&ETH_InitStructure,PHY_ADDRESS);/* 配置ETH */
	if(rval==ETH_SUCCESS)//配置成功
	{
		ETH_DMAITConfig(ETH_DMA_IT_NIS
						|ETH_DMA_IT_T/* 使能发送完成中断 */
						|ETH_DMA_IT_R /* 使能接收完成中断 */
//						|ETH_DMA_IT_TBU,/* 使能发送描述符不可用中断 */
						,ENABLE );//使能以太网发送中断
	}
	else
	{
		printf("Error:ETH_MACDMA_Config failed!!!\n");
		return ETH_ERROR;
	}
		PFIC_EnableIRQ(ETH_IRQn);
//		printf("ETH_MACDMA_reConfig finished!\n");

		ETH_buf_init();                /*缓冲区和描述赋初始化*/

		ETH->DMAOMR |= ETH_DMAOMR_SR;  /* 开启接收DMA */
		ETH->DMAOMR |= ETH_DMAOMR_TSF; /* 开启存储转发,在启用硬件填写检验和的时候，必须开启这一步。默认开启硬件填写校验和 */
		ETH->DMAOMR |= ETH_DMAOMR_ST;  /* 打开DMA发送  */
		mDelaymS(2);
		ETH->MACCR |= ETH_MACCR_TE;/* 开启Mac发送使能 */

		Check_TxDes();/* 发送描述符检查 */

		return rval;
}

/*******************************************************************************
 * @fn         Mac_init
 *
 * @brief      Eth phripheral initilization
 *
 * @return     Execution status
 */
UINT8 Mac_init(void)
{
	ETH_GPIO_Init();/* GPIO初始化 */
	if( ETH_Mem_Malloc() )/* 以太网缓冲区和描述符队列申请内存的函数 */
	{
		return 1;
	}
	if( ETH_MACDMA_Config()!=1 )   /* MAC的寄存器初始化 */
	{
		return 2;
	}
	ETH_buf_init();                /*缓冲区和描述赋初始化*/

	ETH->DMAOMR |= ETH_DMAOMR_SR;  /* 开启接收DMA */
	ETH->DMAOMR |= ETH_DMAOMR_TSF; /* 开启存储转发,在启用硬件填写检验和的时候，必须开启这一步。默认开启硬件填写校验和 */
	ETH->DMAOMR |= ETH_DMAOMR_ST;  /* 打开DMA发送  */
	mDelaymS(2);
	ETH->MACCR |= ETH_MACCR_TE;/* 开启Mac发送使能 */
	return 0;
}

/*******************************************************************************
 * @fn        ETH_init
 *
 * @brief     Eth initilization
 *
 * @return    None
 */
void ETH_init(void)
{
	UINT8 rc;
	rc=Mac_init();
	while(rc)
	{
		PRINT("MAC init failed.retrying...\n");
		mDelaymS(2);
		rc=Mac_init();
	}
}

/*******************************************************************************
 * @fn        ENABLE_PMT_INT
 *
 * @brief     Eth power manager interrupt initilization
 *
 * @return    None
 */
void ENABLE_PMT_INT(void)
{
	PFIC_EnableIRQ(PMT_IRQn);
}

/*******************************************************************************
 * @fn        Check_TxDes
 *
 * @brief     Check Transmit descriptor
 *
 * @return    None
 */
void Check_TxDes(void)
{
	if(DMATxDescToSet == (ETH_DMADESCTypeDef*)ETH->DMACHTDR)
		return;
	ETH_DMATxDescChainInit(pDMATxDscrTab, pTx_Buff,TX_Des_Num);/* 设置发送缓冲区和描述符 */
	DMATxDescToSet = (ETH_DMADESCTypeDef*)ETH->DMACHTDR;
}

/*******************************************************************************
 * @fn        ETH_IRQHandler
 *
 * @brief     Eth interrupt
 *
 * @return    None
 */
void ETH_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void ETH_IRQHandler(void)
{
#if 0 /* 跳过这一步可以加快中断函数完成的速度 */
	if(ETH->DMASR&ETH_DMA_IT_TBU)/* 有发送描述符不可用 */
	{
		ETH_DMAClearITPendingBit(ETH_DMA_IT_TBU);
	}
#endif
	/*-----------------发送完成中断----------------------*/
	if(ETH->DMASR&ETH_DMA_IT_T)/* 有一帧已经发送完成 */
	{
		ETH_DMAClearITPendingBit(ETH_DMA_IT_T);
//		printf("t\n");
		enable_send|=0x01;/* 重启发送 */
		rece_timeout_cnt=0;
	}
	/*-----------------接收完成中断----------------------*/
	if(ETH->DMASR&ETH_DMA_IT_R)/* 有一帧已经接收完成 */
	{
		while((DMARxDescToGet->Status&ETH_DMARxDesc_OWN)==0)/*判断描述符有没有读尽*/
		{
			ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
			DMARxDescToGet->Status|=ETH_DMARxDesc_OWN;                /*  */
			DMARxDescToGet=(ETH_DMADESCTypeDef*)(DMARxDescToGet->Buffer2NextDescAddr);
			Globe_RxDes_status.pengding_RxDes_cnt++;
		}
		/* 如果主循环的读取以太网帧函数速度不够快，会出现在两次读取的间隙，以太网接收帧超过最大接收
		 * 描述符数量（RX_Des_Num，20200704定为8个）的情形，因此为保证接收流程正常运转，应丢弃之
		 * 前接收到的包 */
		if(Globe_RxDes_status.pengding_RxDes_cnt>RX_Des_Num)
		{
			Globe_RxDes_status.pengding_RxDes_cnt=1;
			Globe_RxDes_status.package_loss_cnt++;
		}
//		printf("r\n");
		enable_send|=0x02;/* 重启发送 */
		rece_timeout_cnt=0;
	}
	/* 清除DMASR标志位 */
	ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
}

/*******************************************************************************
 * @fn      PMT_IRQHandler
 *
 * @brief   Eth power manager interrupt
 *
 * @return  None
 */
void PMT_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void PMT_IRQHandler(void)
{
	printf("ETH->MACPMTCSR:0x%08x\n",ETH->MACPMTCSR);
	printf("PMT_IRQHandler\n");
}

/*******************************************************************************
 * @fn           mac_send
 *
 * @brief        Eth send
 *
 * @param        length:length of data will be sent
 *
 * @return       Execution status
 */
UINT8 mac_send(uint32_t length)
{
	UINT8 rc;

	if((DMATxDescToSet->Status&ETH_DMATxDesc_OWN)!=(UINT32)RESET )/* 理论上不应该进入这个情况。若发生需要检查发送描述符 */
	{
		/* 进入这里的原因一般是在没有报发送完成中断的情况下就继续使用该发送描述符  */
		printf("Error:DMATxDescToSet can no t use!\n");
		printf("send:DMACHTDR:x%08x\n",ETH->DMACHTDR);
		printf("send:DMATxDescToSet:0x%08x\n",DMATxDescToSet);
//		printf("send:0x%08x:0x%08x\n",&(DMATxDescToSet->Status),DMATxDescToSet->Status);
//		printf("send:0x%08x:0x%08x\n",&(DMATxDescToSet->ControlBufferSize),DMATxDescToSet->ControlBufferSize);
//		printf("send:0x%08x:0x%08x\n",&(DMATxDescToSet->Buffer1Addr),DMATxDescToSet->Buffer1Addr);
//		printf("send:0x%08x:0x%08x\n",&(DMATxDescToSet->Buffer2NextDescAddr),DMATxDescToSet->Buffer2NextDescAddr);
		printf("send:DMASR:08%08x\n",ETH->DMASR);
		return 1;    /* 错误,OWN位被设置了 */
	}
	DMATxDescToSet->ControlBufferSize=(length&ETH_DMATxDesc_TBS1);        //设置帧长度,bits[12:0]
	DMATxDescToSet->Status=ETH_DMATxDesc_LS|ETH_DMATxDesc_FS|ETH_DMATxDesc_IC|ETH_DMATxDesc_CIC_TCPUDPICMP_Full;//设置最后一个和第一个位段置位(1个描述符传输一帧)，设置第二个缓冲区指向下一个描述符
//	DMATxDescToSet->Status|=ETH_DMATxDesc_DC;                              /* 不自动计算并填充CRC */
	DMATxDescToSet->Status|=ETH_DMATxDesc_OWN;                             //设置Tx描述符的OWN位,buffer重归ETH DMA

	/* 当前发送描述符的流转机制，分为两种，一是直接取第四个双字的值，这种方法虽然快，但是如果在CHTDR跑飞的情况下，有全盘飞掉的风险
	  * 二是检测是否是最后一个描述符，不是则直接向后移4个双字的位置。CH565/569的描述符列表是链式和环式相结合的方法排布的 */
#if 0
	DMATxDescToSet==(ETH_DMADESCTypeDef*)DMATxDescToSet->Buffer2NextDescAddr;
#else
	if(DMATxDescToSet>=((ETH_DMADESCTypeDef*)pDMATxDscrTab+(TX_Des_Num-1)))
		DMATxDescToSet = (ETH_DMADESCTypeDef*)(pDMATxDscrTab);
	else
		DMATxDescToSet++;
#endif
	ETH->DMASR=ETH_DMASR_TBUS;                                         /* 重置ETH DMA TBUS位 */
	ETH->DMATPDR=0;                                                    /* 恢复DMA发送 */

	return 0;
}

/*******************************************************************************
 * @fn        mac_rece
 *
 * @brief     Eth receive
 *
 * @param     ptr:pointer to the receive buffer
 *
 * @return    The length of received data
 */
UINT16 mac_rece(UINT8 **ptr)
{

	UINT16 frame_length;
	if(Globe_RxDes_status.pengding_RxDes_cnt==0)
	{
		frame_length=0;
		printf("Error:no package received!\n");
	}
	else
	{
		*ptr=(UINT8*)(Globe_RxDes_status.LastRxDes->Buffer1Addr);
		frame_length=((UINT16)(((Globe_RxDes_status.LastRxDes->Status)&0x3fff0000)>>16));
		(Globe_RxDes_status.LastRxDes) = (ETH_DMADESCTypeDef*)(Globe_RxDes_status.LastRxDes->Buffer2NextDescAddr);
		Globe_RxDes_status.pengding_RxDes_cnt--;
	}

	return frame_length;
}

/*******************************************************************************
 * @fn        ETH_StructInit
 *
 * @brief     Eth config struct initialization
 *
 * @param     ETH_InitStruct:pointer to the eth config struct
 *
 * @return    None
 */
void ETH_StructInit(ETH_InitTypeDef *ETH_InitStruct)
{
	  /*------------------------------------------   MAC默认初始化配置  ---------------------------------------------*/

	  /* 使能使能物理层自动协商模式 */
	  ETH_InitStruct->ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
	  /* 使能MAC看门狗 */
	  ETH_InitStruct->ETH_Watchdog = ETH_Watchdog_Enable;
	  /* 使能Jabber在半双工模式中，无意义，CH56X不支持半双工 */
	  ETH_InitStruct->ETH_Jabber = ETH_Jabber_Enable;
	  /* 设置帧间隔为96bit */
	  ETH_InitStruct->ETH_InterFrameGap = ETH_InterFrameGap_96Bit;
	  /* 半双工模式下使能载波侦听功能， */
	  ETH_InitStruct->ETH_CarrierSense = ETH_CarrierSense_Enable;
	  /* PHY层速度默认设为为1000Mbps */
	  ETH_InitStruct->ETH_Speed = ETH_Speed_1000M;
	  /* 半双工模式下允许接收 own frame,无意义，CH56X不支持半双工 */
	  ETH_InitStruct->ETH_ReceiveOwn = ETH_ReceiveOwn_Enable;
	  /* 关闭MII接口的反馈功能,无意义，CH56X不支持半双工  */
	  ETH_InitStruct->ETH_LoopbackMode = ETH_LoopbackMode_Disable;
	  /* 使用全双工模式,无意义，CH56X不支持半双工 */
	  ETH_InitStruct->ETH_Mode = ETH_Mode_FullDuplex;
	  /* 默认ipv4和TCP/UDP/ICMP的帧校验和卸载 */
	  ETH_InitStruct->ETH_ChecksumOffload = ETH_ChecksumOffload_Enable;
	  /* 开启半双工模式下的重试传输功能，无意义，CH56X不支持半双工 */
	  ETH_InitStruct->ETH_RetryTransmission = ETH_RetryTransmission_Enable;
	  /* 默认自动去除PDA/CRC功能*/
	  ETH_InitStruct->ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Enable;
	  /* 设置半双工模式下的最大重传回退事件10 slot，无意义，CH56X不支持半双工 */
	  ETH_InitStruct->ETH_BackOffLimit = ETH_BackOffLimit_10;
	  /* 关闭半双工模式下的延时检查功能，无意义，CH56X不支持半双工*/
	  ETH_InitStruct->ETH_DeferralCheck = ETH_DeferralCheck_Disable;
	  /* 默认接收所有帧 */
	  ETH_InitStruct->ETH_ReceiveAll = ETH_ReceiveAll_Enable;
	  /* 关闭MAC地址的源地址过滤功能 */
	  ETH_InitStruct->ETH_SourceAddrFilter = ETH_SourceAddrFilter_Disable;
	  /* Do not forward control frames that do not pass the address filtering */
	  ETH_InitStruct->ETH_PassControlFrames = ETH_PassControlFrames_BlockAll;
	  /* 禁止接收所有的广播帧 */
	  ETH_InitStruct->ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Disable;
	  /* 正常的远端地址过滤 */
	  ETH_InitStruct->ETH_DestinationAddrFilter = ETH_DestinationAddrFilter_Normal;
	  /* 关闭混合模式的地址过滤 */
	  ETH_InitStruct->ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
	  /* 对于组播地址使用完美地址过滤 */
	  ETH_InitStruct->ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
	  /* 对单播地址使用完美地址过滤 */
	  ETH_InitStruct->ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
	  /* 初始化HASH表的高位寄存器 */
	  ETH_InitStruct->ETH_HashTableHigh = 0x0;
	  /* 初始化HASH表的低位位寄存器 */
	  ETH_InitStruct->ETH_HashTableLow = 0x0;
	  /* 流控配置*/
	  ETH_InitStruct->ETH_PauseTime = 0x0;
	  ETH_InitStruct->ETH_ZeroQuantaPause = ETH_ZeroQuantaPause_Disable;
	  ETH_InitStruct->ETH_PauseLowThreshold = ETH_PauseLowThreshold_Minus4;
	  ETH_InitStruct->ETH_UnicastPauseFrameDetect = ETH_UnicastPauseFrameDetect_Disable;
	  ETH_InitStruct->ETH_ReceiveFlowControl = ETH_ReceiveFlowControl_Disable;
	  ETH_InitStruct->ETH_TransmitFlowControl = ETH_TransmitFlowControl_Disable;
	  /* VLANtag config (VLAN field not checked) */
	  ETH_InitStruct->ETH_VLANTagComparison = ETH_VLANTagComparison_16Bit;
	  ETH_InitStruct->ETH_VLANTagIdentifier = 0x0;

	  /*---------------------- DMA 初始化        -------------------------------*/

	  /* 关闭丢弃TCP/IP错误帧 */
	  ETH_InitStruct->ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Disable;
	  /* 开启接收数据的存储转发功能 */
	  ETH_InitStruct->ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
	  /* Flush received frame that created FIFO overflow */
	  ETH_InitStruct->ETH_FlushReceivedFrame = ETH_FlushReceivedFrame_Enable;
	  /* 开启发送模式的存储转发功能 */
	  ETH_InitStruct->ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;
	  /* 设置阈值模式下的发送FIFO的阈值为64字节 */
	  ETH_InitStruct->ETH_TransmitThresholdControl = ETH_TransmitThresholdControl_64Bytes;
	  /* 禁止转发错误帧 */
	  ETH_InitStruct->ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;
	  /* 不转发过小的好帧 */
	  ETH_InitStruct->ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;
	  /* 设置直通模式下的发送FIFO阈值为64字节 */
	  ETH_InitStruct->ETH_ReceiveThresholdControl = ETH_ReceiveThresholdControl_64Bytes;
	  /* 关闭处理第二帧数据 */
	  ETH_InitStruct->ETH_SecondFrameOperate = ETH_SecondFrameOperate_Disable;
	  /* 开启DMA传输的地址对齐功能 */
	  ETH_InitStruct->ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;
	  /* 开启固定突发功能 */
	  ETH_InitStruct->ETH_FixedBurst = ETH_FixedBurst_Enable;
	  /* DMA发送的最大突发长度为32 */
	  ETH_InitStruct->ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;
	  /* DMA接收的最大突发长度为32 */
	  ETH_InitStruct->ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;
	  /* DMA Ring mode skip length = 0 */
	  ETH_InitStruct->ETH_DescriptorSkipLength = 0x0;
	  /* Equal priority (round-robin) between transmit and receive DMA engines */
	  ETH_InitStruct->ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_1_1;

	  /* 个性化设置 */
	  ETH_InitStruct->ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;   		//开启网络自适应功能
	  ETH_InitStruct->ETH_LoopbackMode = ETH_LoopbackMode_Disable;				//关闭反馈
	  ETH_InitStruct->ETH_RetryTransmission = ETH_RetryTransmission_Disable; 		//关闭重传功能kp
	  ETH_InitStruct->ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable; 	//关闭自动去除PDA/CRC功能
	  ETH_InitStruct->ETH_ReceiveAll = ETH_ReceiveAll_Enable;						       //关闭接收所有的帧
	  ETH_InitStruct->ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;//允许接收所有广播帧
	  ETH_InitStruct->ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;			//关闭混合模式的地址过滤
	  ETH_InitStruct->ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;//对于组播地址使用完美地址过滤
	  ETH_InitStruct->ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;	//对单播地址使用完美地址过滤
	#ifdef CHECKSUM_BY_HARDWARE
		ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable; 			//开启ipv4和TCP/UDP/ICMP的帧校验和卸载
	#endif
		//当我们使用帧校验和卸载功能的时候，一定要使能存储转发模式,存储转发模式中要保证整个帧存储在FIFO中,
		//这样MAC能插入/识别出帧校验值,当真校验正确的时候DMA就可以处理帧,否则就丢弃掉该帧
		ETH_InitStruct->ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable; //开启丢弃TCP/IP错误帧
		ETH_InitStruct->ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;     //开启接收数据的存储转发模式
		ETH_InitStruct->ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;   //开启发送数据的存储转发模式
		ETH_InitStruct->ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Enable;     	//禁止转发错误帧
		ETH_InitStruct->ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Enable;	//不转发过小的好帧
		ETH_InitStruct->ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;  		//打开处理第二帧功能
		ETH_InitStruct->ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;  	//开启DMA传输的地址对齐功能
		ETH_InitStruct->ETH_FixedBurst = ETH_FixedBurst_Enable;            			//开启固定突发功能
		ETH_InitStruct->ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;     		//DMA发送的最大突发长度为32个节拍
		ETH_InitStruct->ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;			//DMA接收的最大突发长度为32个节拍
		ETH_InitStruct->ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;
}

/*******************************************************************************
 * @fn          ETH_Init
 *
 * @brief       Eth register & PHY register initialization
 *
 * @param       ETH_InitStruct:pointer to the eth config struct
 *              PHYAddress:PHY address
 *
 * @return      Execution status
 */
UINT32 ETH_Init(ETH_InitTypeDef *ETH_InitStruct, UINT16 PHYAddress)
{
  UINT32  tmpreg = 0;
  UINT16 RegValue = 0;
  __IO UINT32 i = 0;
  UINT32 hclk = 80000000;
  __IO UINT32 timeout = 0;
  /* MAC --------------------------*/

  /*-------------------------------- MAC Config ------------------------------*/
  /*---------------------- ETHERNET MACMIIAR Configuration -------------------*/
  /* Get the ETHERNET MACMIIAR value */
  tmpreg = ETH->MACMIIAR;
  /* Clear CSR Clock Range CR[2:0] bits */
  tmpreg &= MACMIIAR_CR_MASK;
  /* Get hclk frequency value */

/*__________注意___________注意______注意_______注意______注意_______注意_______注意_____注意______*/
  /* 修改系统主频需要同步修改这个值 */
  hclk=12000000;//得到系统时钟频率(单位:Hz)
  /* 这里获取系统主频的原因主要是为了确定SMI接口时钟线的分频因数，感觉影响不大。我觉得应该SMI时钟尽可能降低点。 */
//    tmpreg |= (UINT32)ETH_MACMIIAR_CR_Div42;/* 就固定选择42分频好了，80M时钟被42分频后频率就很低了 */
    tmpreg |= (UINT32)ETH_MACMIIAR_CR_Div62;
//    tmpreg |= (UINT32)ETH_MACMIIAR_CR_Div102;
  /* Write to ETHERNET MAC MIIAR: Configure the ETHERNET CSR Clock Range */
  ETH->MACMIIAR = (UINT32)tmpreg;
  /*-------------------- PHY initialization and configuration ----------------*/
  /* Put the PHY in reset mode */
  if(!(ETH_WritePHYRegister(PHYAddress, PHY_BCR, PHY_Reset)))
  {
    /* Return ERROR in case of write timeout */
	  printf("物理层复位失败！\n");
      return 0;
  }
  else
  {
//	  PRINT("物理层复位成功！\n");
  }

  /* Delay to assure PHY reset */
	mDelaymS(200);

  if(ETH_InitStruct->ETH_AutoNegotiation != ETH_AutoNegotiation_Disable)
  {
#ifndef FIBER_1000M
    /* We wait for linked status... */
//    do
//    {
//      timeout++;
//      RegValue=ETH_ReadPHYRegister(PHYAddress, PHY_BSR) ;
////      printf("RegValue:0x%04x\n",RegValue);
//    } while (!(RegValue & PHY_Linked_Status) && (timeout < PHY_READ_TO));
//
//    /* Return ERROR in case of timeout */
//    if(timeout >= PHY_READ_TO)
//    {
//    	printf("READ PHY_BSR TIMEOUT!\n");
//    	return ETH_ERROR;
//    }

    /* Reset Timeout counter */
    timeout = 0;
    /* Enable Auto-Negotiation */
    if(!(ETH_WritePHYRegister(PHYAddress, PHY_BCR, PHY_AutoNegotiation)))
    {
      /* Return ERROR in case of write timeout */
    	printf("WRITE PHY_AutoNegotiation  TIMEOUT!\n");
    	return ETH_ERROR;
    }

    /* Wait until the auto-negotiation will be completed */
    do
    {
      timeout++;
    } while (!(ETH_ReadPHYRegister(PHYAddress, PHY_BSR) & PHY_AutoNego_Complete) && (timeout < (UINT32)PHY_READ_TO));

    /* Return ERROR in case of timeout */
    if(timeout == PHY_READ_TO)
    {
    	PRINT("AutoNego timeout!\n");
    	return ETH_ERROR;
    }

    /* Reset Timeout counter */
    timeout = 0;
#endif
    /* Read the result of the auto-negotiation */
    timeout=100;
    do
    {
    	RegValue = ETH_ReadPHYRegister(PHYAddress, PHY_BMSR);
    	mDelaymS(2);/* 等待连接上 */
        timeout--;
        if(timeout<=0)
        {
        	printf("Wait phy linking timeout!\n");
        	break;
        }
    }
    while((RegValue & 0x0004)==0);
    if(RegValue & 0x0004)
    {
    	PRINT("PHY UTP/fiber linked.PHY_BMSR:0x%04x\n",RegValue);
#ifdef USE_ETH_PHY_INTERRUPT
    	globe_eth_status.phy_link_status=link_ok;
#endif
    }
#ifdef USE_RTL8211FS
#ifndef FIBER_1000M
    ETH_WritePHYRegister(PHYAddress, 31,0x0a43 );
    RegValue = ETH_ReadPHYRegister(PHYAddress, 26);
    PRINT("PHY_SR:0x%04x\n",RegValue);
    if( RegValue & 0x0008 )
    {
    	ETH_InitStruct->ETH_Mode = ETH_Mode_FullDuplex;
    	PRINT("full duplex.\n");
#ifdef USE_ETH_PHY_INTERRUPT
    	globe_eth_status.phy_link_duples=full_duples;
#endif
    }
    else
    {
    	ETH_InitStruct->ETH_Mode = ETH_Mode_HalfDuplex;
    	PRINT("half duplex!\n");
#ifdef USE_ETH_PHY_INTERRUPT
    	globe_eth_status.phy_link_duples=half_duples;
#endif
    }
    if(( RegValue & 0x0030 ) == 0x0000)
    {
    	ETH_InitStruct->ETH_Speed = ETH_Speed_10M;
    	PRINT("Link speed:10Mbps.\n");
#ifdef USE_ETH_PHY_INTERRUPT
    	globe_eth_status.phy_link_speed=linked_10M;
#endif
    }
    else if(( RegValue & 0x0030 ) == 0x0010)
    {
    	ETH_InitStruct->ETH_Speed = ETH_Speed_100M;
    	PRINT("Link speed:100Mbps.\n");
#ifdef USE_ETH_PHY_INTERRUPT
    	globe_eth_status.phy_link_speed=linked_100M;
#endif
    }
    else if(( RegValue & 0x0030 ) == 0x0020)
    {
    	ETH_InitStruct->ETH_Speed = ETH_Speed_1000M;
    	PRINT("Link speed:1000Mbps.\n");
#ifdef USE_ETH_PHY_INTERRUPT
    	globe_eth_status.phy_link_speed=linked_1000M;
#endif
    }
    else
    {
    	printf("Do not support peer device.\nPlease link with 1000Base_T/100Base-TX/10Base-T device and then restart.\n");
    	while(1);
    }
    /* 使能RTL8211FS的TXC内部延时1.5纳秒 */
    ETH_WritePHYRegister(PHYAddress, 31,0x0d08);
    RegValue = ETH_ReadPHYRegister(PHYAddress, 17);
//    printf("page d08 reg 17 is %04x\n",RegValue);
    ETH_WritePHYRegister(PHYAddress, 17,RegValue | 0x0100);
    RegValue = ETH_ReadPHYRegister(PHYAddress, 17);
//    printf("page d08 reg 17 is %04x\n",RegValue);

//    ETH_WritePHYRegister(PHYAddress, 31,0xd08 );
//    RegValue = ETH_ReadPHYRegister(PHYAddress, 0x10);
//    PRINT("SGMII ANAR:0x%04x\n",RegValue);
#else/* fiber模式下，手动指定MAC速度 */
    ETH_InitStruct->ETH_Mode = ETH_Mode_FullDuplex;
    ETH_InitStruct->ETH_Speed = ETH_Speed_1000M;
#endif
#endif
#ifdef USE_RTL8211DN
#ifndef FIBER_1000M
    RegValue = ETH_ReadPHYRegister(PHYAddress, 0x11);
    PRINT("PHY_SR:0x%04x\n",RegValue);
    if( RegValue & 0x2000 )
    {
    	ETH_InitStruct->ETH_Mode = ETH_Mode_FullDuplex;
    	PRINT("full duplex.\n");
    }
    else
    {
    	ETH_InitStruct->ETH_Mode = ETH_Mode_HalfDuplex;
    	PRINT("half duplex!\n");
    }
    if(( RegValue & 0xC000 ) == 0x0000)
    {
    	ETH_InitStruct->ETH_Speed = ETH_Speed_10M;
    	PRINT("Link speed:10Mbps.\n");
    }
    else if(( RegValue & 0xC000 ) == 0x4000)
    {
    	ETH_InitStruct->ETH_Speed = ETH_Speed_100M;
    	PRINT("Link speed:100Mbps.\n");
    }
    else if(( RegValue & 0xC000 ) == 0x8000)
    {
    	ETH_InitStruct->ETH_Speed = ETH_Speed_1000M;
    	PRINT("Link speed:1000Mbps.\n");
    }
    else
    {
    	printf("Do not support peer device.\nPlease link with 1000Base_T/100Base-TX/10Base-T device and then restart.\n");
    	while(1);
    }

#else
    /* fiber模式下，手动指定MAC速度 */
    ETH_InitStruct->ETH_Mode = ETH_Mode_FullDuplex;
    ETH_InitStruct->ETH_Speed = ETH_Speed_1000M;
#endif
#endif

  }
  /*------------------------ ETHERNET MACCR Configuration --------------------*/
  /* Get the ETHERNET MACCR value */
  tmpreg = ETH->MACCR;
  /* Clear WD, PCE, PS, TE and RE bits */
  tmpreg &= MACCR_CLEAR_MASK;
  /* Set the WD bit according to ETH_Watchdog value */
  /* Set the JD: bit according to ETH_Jabber value */
  /* Set the IFG bit according to ETH_InterFrameGap value */
  /* Set the DCRS bit according to ETH_CarrierSense value */
  /* Set the FES bit according to ETH_Speed value */
  /* Set the DO bit according to ETH_ReceiveOwn value */
  /* Set the LM bit according to ETH_LoopbackMode value */
  /* Set the DM bit according to ETH_Mode value */
  /* Set the IPCO bit according to ETH_ChecksumOffload value */
  /* Set the DR bit according to ETH_RetryTransmission value */
  /* Set the ACS bit according to ETH_AutomaticPadCRCStrip value */
  /* Set the BL bit according to ETH_BackOffLimit value */
  /* Set the DC bit according to ETH_DeferralCheck value */
  tmpreg |= (UINT32)(ETH_InitStruct->ETH_Watchdog |
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
  /* Write to ETHERNET MACCR */
  ETH->MACCR = (UINT32)tmpreg;
  ETH->MACCR|=(1<<16);/* 关闭载波侦听 */
  /*----------------------- ETHERNET MACFFR Configuration --------------------*/
  /* Set the RA bit according to ETH_ReceiveAll value */
  /* Set the SAF and SAIF bits according to ETH_SourceAddrFilter value */
  /* Set the PCF bit according to ETH_PassControlFrames value */
  /* Set the DBF bit according to ETH_BroadcastFramesReception value */
  /* Set the DAIF bit according to ETH_DestinationAddrFilter value */
  /* Set the PR bit according to ETH_PromiscuousMode value */
  /* Set the PM, HMC and HPF bits according to ETH_MulticastFramesFilter value */
  /* Set the HUC and HPF bits according to ETH_UnicastFramesFilter value */
  /* Write to ETHERNET MACFFR */
  ETH->MACFFR = (UINT32)(ETH_InitStruct->ETH_ReceiveAll |
                          ETH_InitStruct->ETH_SourceAddrFilter |
                          ETH_InitStruct->ETH_PassControlFrames |
                          ETH_InitStruct->ETH_BroadcastFramesReception |
                          ETH_InitStruct->ETH_DestinationAddrFilter |
                          ETH_InitStruct->ETH_PromiscuousMode |
                          ETH_InitStruct->ETH_MulticastFramesFilter |
                          ETH_InitStruct->ETH_UnicastFramesFilter);
  /*--------------- ETHERNET MACHTHR and MACHTLR Configuration ---------------*/
  /* Write to ETHERNET MACHTHR */
  ETH->MACHTHR = (UINT32)ETH_InitStruct->ETH_HashTableHigh;
  /* Write to ETHERNET MACHTLR */
  ETH->MACHTLR = (UINT32)ETH_InitStruct->ETH_HashTableLow;
  /*----------------------- ETHERNET MACFCR Configuration --------------------*/
  /* Get the ETHERNET MACFCR value */
  tmpreg = ETH->MACFCR;
  /* Clear xx bits */
  tmpreg &= MACFCR_CLEAR_MASK;

  /* Set the PT bit according to ETH_PauseTime value */
  /* Set the DZPQ bit according to ETH_ZeroQuantaPause value */
  /* Set the PLT bit according to ETH_PauseLowThreshold value */
  /* Set the UP bit according to ETH_UnicastPauseFrameDetect value */
  /* Set the RFE bit according to ETH_ReceiveFlowControl value */
  /* Set the TFE bit according to ETH_TransmitFlowControl value */
  tmpreg |= (UINT32)((ETH_InitStruct->ETH_PauseTime << 16) |
                   ETH_InitStruct->ETH_ZeroQuantaPause |
                   ETH_InitStruct->ETH_PauseLowThreshold |
                   ETH_InitStruct->ETH_UnicastPauseFrameDetect |
                   ETH_InitStruct->ETH_ReceiveFlowControl |
                   ETH_InitStruct->ETH_TransmitFlowControl);
  /* Write to ETHERNET MACFCR */
  ETH->MACFCR = (UINT32)tmpreg;
  /*----------------------- ETHERNET MACVLANTR Configuration -----------------*/
  /* Set the ETV bit according to ETH_VLANTagComparison value */
  /* Set the VL bit according to ETH_VLANTagIdentifier value */
  ETH->MACVLANTR = (UINT32)(ETH_InitStruct->ETH_VLANTagComparison |
                             ETH_InitStruct->ETH_VLANTagIdentifier);

  /*-------------------------------- DMA Config ------------------------------*/
  /*----------------------- ETHERNET DMAOMR Configuration --------------------*/
  /* Get the ETHERNET DMAOMR value */
  tmpreg = ETH->DMAOMR;
  /* Clear xx bits */
//  tmpreg &= DMAOMR_CLEAR_MASK;

  /* Set the DT bit according to ETH_DropTCPIPChecksumErrorFrame value */
  /* Set the RSF bit according to ETH_ReceiveStoreForward value */
  /* Set the DFF bit according to ETH_FlushReceivedFrame value */
  /* Set the TSF bit according to ETH_TransmitStoreForward value */
  /* Set the TTC bit according to ETH_TransmitThresholdControl value */
  /* Set the FEF bit according to ETH_ForwardErrorFrames value */
  /* Set the FUF bit according to ETH_ForwardUndersizedGoodFrames value */
  /* Set the RTC bit according to ETH_ReceiveThresholdControl value */
  /* Set the OSF bit according to ETH_SecondFrameOperate value */
  tmpreg |= (UINT32)(ETH_InitStruct->ETH_DropTCPIPChecksumErrorFrame |
                  ETH_InitStruct->ETH_ReceiveStoreForward |
                  ETH_InitStruct->ETH_FlushReceivedFrame |
                  ETH_InitStruct->ETH_TransmitStoreForward |
                  ETH_InitStruct->ETH_TransmitThresholdControl |
                  ETH_InitStruct->ETH_ForwardErrorFrames |
                  ETH_InitStruct->ETH_ForwardUndersizedGoodFrames |
                  ETH_InitStruct->ETH_ReceiveThresholdControl |
                  ETH_InitStruct->ETH_SecondFrameOperate);
  /* Write to ETHERNET DMAOMR */
  ETH->DMAOMR = (UINT32)tmpreg;

  /*----------------------- ETHERNET DMABMR Configuration --------------------*/
  /* Set the AAL bit according to ETH_AddressAlignedBeats value */
  /* Set the FB bit according to ETH_FixedBurst value */
  /* Set the RPBL and 4*PBL bits according to ETH_RxDMABurstLength value */
  /* Set the PBL and 4*PBL bits according to ETH_TxDMABurstLength value */
  /* Set the DSL bit according to ETH_DesciptorSkipLength value */
  /* Set the PR and DA bits according to ETH_DMAArbitration value */
  ETH->DMABMR = (UINT32)(ETH_InitStruct->ETH_AddressAlignedBeats |
                          ETH_InitStruct->ETH_FixedBurst |
                          ETH_InitStruct->ETH_RxDMABurstLength | /* !! if 4xPBL is selected for Tx or Rx it is applied for the other */
                          ETH_InitStruct->ETH_TxDMABurstLength |
                         (ETH_InitStruct->ETH_DescriptorSkipLength << 2) |
                          ETH_InitStruct->ETH_DMAArbitration |
                          ETH_DMABMR_USP); /* Enable use of separate PBL for Rx and Tx */

  #ifdef USE_ENHANCED_DMA_DESCRIPTORS
    /* Enable the Enhanced DMA descriptors */
    ETH->DMABMR |= ETH_DMABMR_EDE;
  #endif /* USE_ENHANCED_DMA_DESCRIPTORS */

  /* Return Ethernet configuration success */
  return ETH_SUCCESS;
}

/*******************************************************************************
 * @fn         ETH_MACAddressConfig
 *
 * @brief     Mac address initialization
 *
 * @param     MacAddr - Mac address register
 *            PHYAddress - pointer to the Mac address string
 *
 * @return     None
 */
void ETH_MACAddressConfig(UINT32 MacAddr, UINT8 *Addr)
{
	UINT32 tmpreg;

  /* Calculate the selected MAC address high register */
  tmpreg = ((UINT32)Addr[5] << 8) | (UINT32)Addr[4]; //计算出所选择的MAC地址的高位寄存器值
																												 //即ETH_MACA0HR的16位值
  /* Load the selected MAC address high register */
  (*(__IO UINT32 *) (ETH_MAC_ADDR_HBASE + MacAddr)) = tmpreg; //将计算出的高位值写入ETH_MAC0HR中
  /* Calculate the selected MAC address low register */
	//计算出所选择的MAC地址的第位寄存器值,即ETH_MACA0LR的16位值
  tmpreg = ((UINT32)Addr[3] << 24) | ((UINT32)Addr[2] << 16) | ((UINT32)Addr[1] << 8) | Addr[0];

  /* Load the selected MAC address low register */
  (*(__IO UINT32 *) (ETH_MAC_ADDR_LBASE + MacAddr)) = tmpreg;//将计算出的低位值写入ETH_MAC0LR中
}

/*******************************************************************************
 * @fn          ETH_DMARxDescChainInit
 *
 * @brief       Receive desciptor iniliazation
 *
 * @param       DMARxDescTab:pointer to the receive desciptor table
 *              RxBuff:pointer to the receive buffer (receive queue)
 *              RxBuffCount:Number of receive desciptor or receive queue
 *
 * @return      None
 */
void ETH_DMARxDescChainInit(ETH_DMADESCTypeDef *DMARxDescTab, UINT8 *RxBuff, UINT32 RxBuffCount)
{
  UINT32 i ,j= 0;
  UINT32* p;
  ETH_DMADESCTypeDef *DMARxDesc;

  DMARxDescToGet = DMARxDescTab;

  for(i=0; i < RxBuffCount; i++)
  {
    DMARxDesc = DMARxDescTab+i;
    DMARxDesc->Status = ETH_DMARxDesc_OWN;
    DMARxDesc->ControlBufferSize =  (UINT32)ETH_RX_BUF_SIZE;
    DMARxDesc->Buffer1Addr = (UINT32)(&RxBuff[i*ETH_RX_BUF_SIZE]);
    if(i >= (RxBuffCount-1))
    {
      DMARxDesc->Buffer2NextDescAddr = (UINT32)(DMARxDescTab);
      DMARxDesc->Status |= ETH_DMARxDesc_RER;/* 置RER给软件用 */
    }
    else
    {
      DMARxDesc->Buffer2NextDescAddr = (UINT32)(DMARxDescTab+i+1);
    }
#ifdef Enable_IEEE_1588
    DMARxDesc->Buffer1Addr_BK = DMARxDesc->Buffer1Addr;
    DMARxDesc->NextDescAddr_BK = DMARxDesc->Buffer2NextDescAddr;
#endif
    p=(void*)DMARxDesc;
#if 0
    printf("第%d个接收描述符:\n",i);
    for(j=0;j<(sizeof(ETH_DMADESCTypeDef))/4;j++)
    printf("addr:0x%08x:0x%08x.\n",(p+j),*(UINT32*)(p+j));
#endif
  }
  ETH->DMARDLAR = (UINT32) DMARxDescTab;
}

/*******************************************************************************
 * @fn        ETH_DMATxDescChainInit
 *
 * @brief     transmit desciptor iniliazation
 *
 * @param     DMARxDescTab:pointer to the transmit desciptor table
 *            RxBuff:pointer to the transmit buffer (transmit queue)
 *            RxBuffCount:Number of transmit desciptor or transmit queue
 *
 * @return    None
 */
void ETH_DMATxDescChainInit(ETH_DMADESCTypeDef *DMATxDescTab, UINT8 *TxBuff, UINT32 TxBuffCount)
{
  UINT32 i = 0, buf_addr;
  UINT8 *p;
  ETH_DMADESCTypeDef *DMATxDesc;

  DMATxDescToSet = DMATxDescTab;                               /* 当前描述符指针指向发送描述符表首地址 */

  /* 注意CH565的描述符的的独特之处 */
  for(i=0; i < TxBuffCount; i++)
  {
    DMATxDesc = DMATxDescTab + i;
    DMATxDesc->Buffer1Addr = (UINT32)TxBuff+i*ETH_TX_BUF_SIZE;/* 写缓冲区地址 */                                         /* 第i个发送队列的地址 */
//    DMATxDesc->Status |= ETH_DMATxDesc_OWN;                       /* 初始化时不释放发送描述符给DMA */
    DMATxDesc->Status |= ETH_DMATxDesc_LS|ETH_DMATxDesc_FS|ETH_DMATxDesc_CIC_TCPUDPICMP_Full;         /* 写状态字，描述符的第一个32位字 */
    DMATxDesc->ControlBufferSize|=ETH_TX_BUF_SIZE;  /* 写入第一个缓冲区大小并置上 */

    if(i>=TxBuffCount-1)
    {
    	DMATxDesc->Buffer2NextDescAddr = (UINT32) DMATxDescTab;   /* 指向首地址，成环 */
    	DMATxDesc->Status |= ETH_DMATxDesc_TER; /* 置TER位 */
    }
    else
    	DMATxDesc->Buffer2NextDescAddr = (UINT32)(DMATxDescTab+i+1);
#ifdef Enable_IEEE_1588
    DMATxDesc->Status |= ETH_DMATxDesc_TTSE;/* 使能时间戳 */
    DMATxDesc->Buffer1Addr_BK = DMATxDesc->Buffer1Addr;/* 写备用缓冲区地址 */
    DMATxDesc->NextDescAddr_BK = DMATxDesc->Buffer2NextDescAddr;/* 写备用下一描述赋符地址 */
#endif
  }
#if 0 /* 是否开启初始化后打印出全部描述符内容 */
  for(i=0;i<TxBuffCount;i++)
  {
#ifndef Enable_IEEE_1588
  printf("init Tx_des_addr:0x%08x:0x%08x 0x%08x 0x%08x 0x%08x\n",
#else
  printf("init Tx_des_addr:0x%08x:\n 0x%08x\n 0x%08x\n 0x%08x\n 0x%08x\n 0x%08x\n 0x%08x\n 0x%08x\n 0x%08x\n",
#endif
		   (UINT32) ((UINT8*)DMATxDescToSet+i*sizeof(ETH_DMADESCTypeDef)),\
		  *(UINT32*)((UINT8*)DMATxDescToSet+0x00+i*sizeof(ETH_DMADESCTypeDef)),\
		  *(UINT32*)((UINT8*)DMATxDescToSet+0x04+i*sizeof(ETH_DMADESCTypeDef)),\
		  *(UINT32*)((UINT8*)DMATxDescToSet+0x08+i*sizeof(ETH_DMADESCTypeDef)),\
		  *(UINT32*)((UINT8*)DMATxDescToSet+0x0c+i*sizeof(ETH_DMADESCTypeDef))
#ifdef Enable_IEEE_1588
		 ,*(UINT32*)((UINT8*)DMATxDescToSet+0x10+i*sizeof(ETH_DMADESCTypeDef)),\
		  *(UINT32*)((UINT8*)DMATxDescToSet+0x14+i*sizeof(ETH_DMADESCTypeDef)),\
		  *(UINT32*)((UINT8*)DMATxDescToSet+0x18+i*sizeof(ETH_DMADESCTypeDef)),\
		  *(UINT32*)((UINT8*)DMATxDescToSet+0x1c+i*sizeof(ETH_DMADESCTypeDef))
#endif
		  );
  }
#endif
//  printf("INIT:TDLAR: 0x%08x\n",DMATxDescTab);
  ETH->DMATDLAR = (UINT32)DMATxDescTab;                    /* 设置发送描述符列表的基地址 */
//  printf("INIT:TDLAR: 0x%08X\n",ETH->DMATDLAR);
}

/*******************************************************************************
 * @fn       ETH_SoftwareReset
 *
 * @brief    ETH software reset
 *
 * @return   None
 */
void ETH_SoftwareReset(void)
{
  /* Set the SWR bit: resets all MAC subsystem internal registers and logic */
  /* After reset all the registers holds their respective reset values */
  ETH->DMABMR |= ETH_DMABMR_SR;
}

/*******************************************************************************
 * @fn       ETH_GetSoftwareResetStatus
 *
 * @brief    Get software reset status
 *
 * @return   FlagStatus:software reset status
 */
FlagStatus ETH_GetSoftwareResetStatus(void)
{
  FlagStatus bitstatus = RESET;
  if((ETH->DMABMR & ETH_DMABMR_SR) != 0)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*******************************************************************************
 * @fn         ETH_DMAITConfig
 *
 * @brief      Configuration DMA interrupt
 *
 * @param      ETH_DMA_IT:Type of DMA interrupt
 *             NewState:Enabe DMA interrupt or Disable DMA interrupt
 *
 * @return     None
 */
void ETH_DMAITConfig(UINT32 ETH_DMA_IT, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the selected ETHERNET DMA interrupts */
    ETH->DMAIER |= ETH_DMA_IT;
  }
  else
  {
    /* Disable the selected ETHERNET DMA interrupts */
    ETH->DMAIER &=(~(UINT32)ETH_DMA_IT);
  }
}

/*******************************************************************************
 * @fn        ETH_DMAClearITPendingBit
 *
 * @brief     Clear DMA interupt flag
 *
 * @param     ETH_DMA_IT:Type of DMA interrupt
 *
 * @return     None
 */
void ETH_DMAClearITPendingBit(UINT32 ETH_DMA_IT)
{
  /* Clear the selected ETHERNET DMA IT */
  ETH->DMASR = (UINT32) ETH_DMA_IT;
}

/*******************************************************************************
 * @fn        ETH_ReadPHYRegister
 *
 * @brief     Read PHY register
 *
 * @param     PHYAddress:PHY address
 *            PHYReg:PHY register address
 *
 * @return    Value of PHY register
 */
UINT16 ETH_ReadPHYRegister(UINT16 PHYAddress, UINT16 PHYReg)
{
  UINT32 tmpreg = 0;
__IO UINT32 timeout = 0;
  /* Check the parameters */

  /* Get the ETHERNET MACMIIAR value */
  tmpreg = ETH->MACMIIAR;
  /* Keep only the CSR Clock Range CR[2:0] bits value */
  tmpreg &= ~MACMIIAR_CR_MASK;
  /* Prepare the MII address register value */
  tmpreg |=(((UINT32)PHYAddress<<11) & ETH_MACMIIAR_PA); /* Set the PHY device address */
  tmpreg |=(((UINT32)PHYReg<<6) & ETH_MACMIIAR_MR);      /* Set the PHY register address */
  tmpreg &= ~ETH_MACMIIAR_MW;                              /* Set the read mode */
  tmpreg |= ETH_MACMIIAR_MB;                               /* Set the MII Busy bit */
  /* Write the result value into the MII Address register */
  ETH->MACMIIAR = tmpreg;
  /* Check for the Busy flag */
  do
  {
    timeout++;
    tmpreg = ETH->MACMIIAR;
  } while ((tmpreg & ETH_MACMIIAR_MB) && (timeout < (UINT32)PHY_READ_TO));
  /* Return ERROR in case of timeout */
  if(timeout == PHY_READ_TO)
  {
    return (UINT16)ETH_ERROR;
  }

  /* Return data register value */
  return (UINT16)(ETH->MACMIIDR);
}

/*******************************************************************************
 * @fn          ETH_WritePHYRegister
 *
 * @brief       Write PHY register
 *
 * @param       PHYAddress:PHY address
 *              PHYReg:PHY register address
 *              PHYValue:Value will be written of PHY register
 *
 * @return      Execution status
 */
UINT32 ETH_WritePHYRegister(UINT16 PHYAddress, UINT16 PHYReg, UINT16 PHYValue)
{
  UINT32 tmpreg = 0;
  __IO UINT32 timeout = 0;

  /* Get the ETHERNET MACMIIAR value */
  tmpreg = ETH->MACMIIAR;
  /* Keep only the CSR Clock Range CR[2:0] bits value */
  tmpreg &= ~MACMIIAR_CR_MASK;
  /* Prepare the MII register address value */
  tmpreg |=(((UINT32)PHYAddress<<11) & ETH_MACMIIAR_PA); /* Set the PHY device address */
  tmpreg |=(((UINT32)PHYReg<<6) & ETH_MACMIIAR_MR);      /* Set the PHY register address */
  tmpreg |= ETH_MACMIIAR_MW;                               /* Set the write mode */
  tmpreg |= ETH_MACMIIAR_MB;                               /* Set the MII Busy bit */
  /* Give the value to the MII data register */
  ETH->MACMIIDR = PHYValue;
  /* Write the result value into the MII Address register */
  ETH->MACMIIAR = tmpreg;
  /* Check for the Busy flag */
  do
  {
    timeout++;
    tmpreg = ETH->MACMIIAR;
  } while ((tmpreg & ETH_MACMIIAR_MB) && (timeout < (UINT32)PHY_WRITE_TO));
  /* Return ERROR in case of timeout */
  if(timeout == PHY_WRITE_TO)
  {
    return ETH_ERROR;
  }

  /* Return SUCCESS */
  return ETH_SUCCESS;
}

/*--------------------------file end-------------------------------*/
