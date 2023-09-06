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
DNS example ,demonstrate that DHCP automatically obtains
an IP address and then requests domain name resolution.
 */
#include "string.h"
#include "eth_driver.h"

uint8_t MACAddr[6];                                      //MAC address
uint8_t IPAddr[4]   = {0, 0, 0, 0};                      //IP address
uint8_t GWIPAddr[4] = {0, 0, 0, 0};                      //Gateway IP address
uint8_t IPMask[4]   = {0, 0, 0, 0};                      //subnet mask
uint8_t DESIP[4]    = {255, 255, 255, 255};              //destination IP address
uint16_t DnsPort = 53;
__attribute__((__aligned__(4))) uint8_t  RemoteIp[4];

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
 * @fn      WCHNET_HandleSockInt
 *
 * @brief   Socket Interrupt Handle
 *
 * @param   socketid - socket id.
 *          intstat - interrupt status
 *
 * @return  none
 */
void WCHNET_HandleSockInt(uint8_t socketid, uint8_t intstat)
{
    if (intstat & SINT_STAT_RECV)                              //receive data
    {
    }
    if (intstat & SINT_STAT_CONNECT)                           //connect successfully
    {
        printf("TCP Connect Success\r\n");
    }
    if (intstat & SINT_STAT_DISCONNECT)                        //disconnect
    {
        printf("TCP Disconnect\r\n");
    }
    if (intstat & SINT_STAT_TIM_OUT)                           //timeout disconnect
    {
        printf("TCP Timeout\r\n");
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
        if (i & PHY_Linked_Status)
            printf("PHY Link Success\r\n");
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
 * @fn      WCHNET_DNSCallBack
 *
 * @brief   DNSCallBack
 *
 * @return  none
 */
void WCHNET_DNSCallBack(const char *name, uint8_t *ipaddr, void *callback_arg)
{
    if(ipaddr == NULL)
    {
        printf("DNS Fail\r\n");
        return;
    }
    printf("Host Name = %s\r\n", name);
    printf("IP= %d.%d.%d.%d\r\n", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
    if(callback_arg != NULL)
    {
        printf("callback_arg = %02x\r\n", (*(uint8_t *)callback_arg));
    }
    WCHNET_DNSStop();                                                          //stop DNS,and release socket
}

/*********************************************************************
 * @fn      WCHNET_DHCPCallBack
 *
 * @brief   DHCPCallBack
 *
 * @param   status - status returned by DHCP
 *          arg - Data returned by DHCP
 *
 * @return  DHCP status
 */
uint8_t WCHNET_DHCPCallBack(uint8_t status, void *arg)
{
    uint8_t *p;

    if(!status)
    {
        p = arg;
        printf("DHCP Success\r\n");
        memcpy(IPAddr, p, 4);
        memcpy(GWIPAddr, &p[4], 4);
        memcpy(IPMask, &p[8], 4);
        printf("IPAddr = %d.%d.%d.%d \r\n", (uint16_t)IPAddr[0], (uint16_t)IPAddr[1],
               (uint16_t)IPAddr[2], (uint16_t)IPAddr[3]);
        printf("GWIPAddr = %d.%d.%d.%d \r\n", (uint16_t)GWIPAddr[0], (uint16_t)GWIPAddr[1],
               (uint16_t)GWIPAddr[2], (uint16_t)GWIPAddr[3]);
        printf("IPAddr = %d.%d.%d.%d \r\n", (uint16_t)IPMask[0], (uint16_t)IPMask[1],
               (uint16_t)IPMask[2], (uint16_t)IPMask[3]);
        printf("DNS1: %d.%d.%d.%d \r\n", p[12], p[13], p[14], p[15]);            //DNS server address provided by the router
        printf("DNS2: %d.%d.%d.%d \r\n", p[16], p[17], p[18], p[19]);

        WCHNET_InitDNS(&p[12], DnsPort);                                         //Set DNS server IP address, and DNS server port is 53
        WCHNET_HostNameGetIp("www.wch.cn", RemoteIp, WCHNET_DNSCallBack, NULL);  //Start DNS
        return ETH_SUCCESS;
    }
    else
    {
        printf("DHCP Fail %02x \r\n", status);
        return ETH_ERROR;
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
    printf("DNS Test\r\n");
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
    WCHNET_DHCPSetHostname("WCHNET");                       //Configure DHCP host name
    i = ETH_LibInit(IPAddr, GWIPAddr, IPMask, MACAddr);     //Ethernet library initialize
    mStopIfError(i);
    if (i == WCHNET_ERR_SUCCESS)
        printf("WCHNET_LibInit Success\r\n");
    WCHNET_DHCPStart(WCHNET_DHCPCallBack);                  //Start DHCP

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
    }
}
