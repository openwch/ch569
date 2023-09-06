/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/05/31
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
/*
 *@Note
IPRaw_PING example, demonstrate the PING function.
 */
#include "string.h"
#include "eth_driver.h"
#include "PING.h"

uint8_t MACAddr[6];                                    //MAC address
uint8_t IPAddr[4] = { 192, 168, 1, 10 };               //IP address
uint8_t GWIPAddr[4] = { 192, 168, 1, 1 };              //Gateway IP address
uint8_t IPMask[4] = { 255, 255, 255, 0 };              //subnet mask
uint8_t DESIP[4] = { 192, 168, 1, 100 };               //destination IP address
uint8_t IPRawProto  = 1;

uint8_t SocketId;                                      //socket id
uint8_t SocketRecvBuf[RECE_BUF_LEN];                   //socket receive buffer
uint8_t MyBuf[RECE_BUF_LEN];

/*********************************************************************
 * @fn      mStopIfError
 *
 * @brief   check if error.
 *
 * @param   iError - error constants.
 *
 * @return  none
 */
void mStopIfError(uint8_t iError)
{
    if (iError == WCHNET_ERR_SUCCESS)
        return;
    printf("Error: %02x\r\n", (uint16_t)iError);
}

/* *****************************************************************************
 * @fn     TMR0_Init
 *
 * @brief  Initializes TIM0.
 *
 * @param  NULL
 *
 * @return NULL
 */
void TMR0_Init()
{
    R32_TMR0_CNT_END = FREQ_SYS / 100;
    R8_TMR0_CTRL_MOD = RB_TMR_MODE_IN;
    R8_TMR0_INTER_EN |= RB_TMR_IE_CYC_END;
    R8_TMR0_CTRL_MOD = RB_TMR_COUNT_EN;
    PFIC_SetPriority(TMR0_IRQn, 0xa0);
    PFIC_EnableIRQ(TMR0_IRQn);
}

/*******************************************************************************
 * @fn     DebugInit
 *
 * @brief  Initializes the UART1 peripheral.
 *
 * @param  baudrate: UART1 communication baud rate.
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
 * @fn      WCHNET_CreateIPRawSocket
 *
 * @brief   Create raw IP Socket
 *
 * @return  none
 */
void WCHNET_CreateIPRawSocket(void)
{
   uint8_t i;
   SOCK_INF TmpSocketInf;

   memset((void *)&TmpSocketInf,0,sizeof(SOCK_INF));
   memcpy((void *)TmpSocketInf.IPAddr,DESIP,4);
   TmpSocketInf.SourPort = IPRawProto;                       //In IPRAW mode, SourPort is the protocol type
   TmpSocketInf.ProtoType = PROTO_TYPE_IP_RAW;
   TmpSocketInf.RecvStartPoint = (uint32_t)SocketRecvBuf;
   TmpSocketInf.RecvBufLen = RECE_BUF_LEN ;
   i = WCHNET_SocketCreat(&SocketId,&TmpSocketInf);
   mStopIfError(i);
}

/*********************************************************************
 * @fn      WCHNET_HandleSockInt
 *
 * @brief   Socket Interrupt Handle
 *
 * @param   socketid - socket id.
 *          intstat - interrupt status
 *
 * @return  none
 */
void WCHNET_HandleSockInt(uint8_t socketid,uint8_t intstat)
{
    uint32_t len;

    if(intstat & SINT_STAT_RECV)                           //receive data
    {
        len = WCHNET_SocketRecvLen(socketid, NULL);         //get socket buffer data length
        WCHNET_SocketRecv(socketid, MyBuf, &len);            //Read the data of the receive buffer into MyBuf
        WCHNET_ICMPRecvData( len, MyBuf );
    }
    if(intstat & SINT_STAT_CONNECT)                        //connect successfully
    {
        /***/
    }
    if(intstat & SINT_STAT_DISCONNECT)                     //disconnect
    {
        /***/
    }
    if(intstat & SINT_STAT_TIM_OUT)                        //timeout disconnect
    {
        /***/
    }
}

/*********************************************************************
 * @fn      WCHNET_HandleGlobalInt
 *
 * @brief   Global Interrupt Handle
 *
 * @return  none
 */
void WCHNET_HandleGlobalInt(void)
{
    uint8_t intstat;
    uint16_t i;
    uint8_t socketint;

    intstat = WCHNET_GetGlobalInt();                              //get global interrupt flag
    if (intstat & GINT_STAT_UNREACH)                              //Unreachable interrupt
    {
        printf("GINT_STAT_UNREACH\r\n");
    }
    if (intstat & GINT_STAT_IP_CONFLI)                            //IP conflict
    {
        printf("GINT_STAT_IP_CONFLI\r\n");
    }
    if (intstat & GINT_STAT_PHY_CHANGE)                           //PHY status change
    {
        i = WCHNET_GetPHYStatus();
        if(i & PHY_Linked_Status){
            ICMPSuc = ICMP_SOKE_CON;
            printf("PHY Link Success\r\n");
        }
    }
    if (intstat & GINT_STAT_SOCKET) {                             //socket related interrupt
        for (i = 0; i < WCHNET_MAX_SOCKET_NUM; i++) {
            socketint = WCHNET_GetSocketInt(i);
            if (socketint)
                WCHNET_HandleSockInt(i, socketint);
        }
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program
 *
 * @return  none
 */
int main(void)
{
    uint8_t i;

    SystemInit(FREQ_SYS);
    Delay_Init(FREQ_SYS);
    DebugInit(115200);                                      //USART initialize
    printf("IPRaw_PING Test\r\n");
    printf("net version:%x\n", WCHNET_GetVer());
    if ( WCHNET_LIB_VER != WCHNET_GetVer()) {
        printf("version error.\n");
    }
    WCHNET_GetMacAddr(MACAddr);                             //get the chip MAC address
    printf("mac addr:");
    for(i = 0; i < 6; i++) 
        printf("%x ",MACAddr[i]);
    printf("\n");
    TMR0_Init();
    i = ETH_LibInit(IPAddr, GWIPAddr, IPMask, MACAddr);     //Ethernet library initialize
    mStopIfError(i);
    if (i == WCHNET_ERR_SUCCESS)
        printf("WCHNET_LibInit Success\r\n");
    WCHNET_CreateIPRawSocket();                             //create IPRAW socket
    InitParameter( );
    InitPING( );

    while(1)
    {
        /*Ethernet library main task function,
         * which needs to be called cyclically*/
        WCHNET_MainTask();
        /*Query the Ethernet global interrupt,
         * if there is an interrupt, call the global interrupt handler*/
        if(WCHNET_QueryGlobalInt())
        {
            WCHNET_HandleGlobalInt();
        }
        WCHNET_PINGCmd();
    }
}
