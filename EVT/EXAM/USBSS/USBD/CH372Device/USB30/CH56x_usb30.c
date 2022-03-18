/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30.c
* Author             : WCH
* Version            : V1.1
* Date               : 2020/12/31
* Description 		 :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56xUSB30_LIB.H"
#include "CH56x_usb20.h"
#include "CH56x_usb30.h"
/*
 * 此例程为USB3.0 device使用例程
 * 主机可向1号端点突发4包数据后然后从1号端点取走4包数据
 * 主机可连续向2号端点下传数据 同时也可连续从2号端点取数据
 * */
/* Global define */
/* Global Variable */
UINT8V        tx_lmp_port = 0;
UINT8V        link_sta = 0;
static UINT32 SetupLen = 0;
static UINT8  SetupReqCode = 0;
static PUINT8 pDescr;

__attribute__((aligned(16))) UINT8 endp0RTbuff[512] __attribute__((section(".DMADATA")));  //端点0数据收发缓冲区
__attribute__((aligned(16))) UINT8 endp1RTbuff[4096] __attribute__((section(".DMADATA"))); //端点1数据收发缓冲区
__attribute__((aligned(16))) UINT8 endp2RTbuff[4096] __attribute__((section(".DMADATA"))); //端点2数据收发缓冲区
__attribute__((aligned(16))) UINT8 endp3RTbuff[4096] __attribute__((section(".DMADATA"))); //端点3数据收发缓冲区
__attribute__((aligned(16))) UINT8 endp4RTbuff[4096] __attribute__((section(".DMADATA"))); //端点4数据收发缓冲区
__attribute__((aligned(16))) UINT8 endp5RTbuff[4096] __attribute__((section(".DMADATA"))); //端点5数据收发缓冲区
__attribute__((aligned(16))) UINT8 endp6RTbuff[4096] __attribute__((section(".DMADATA"))); //端点6数据收发缓冲区
__attribute__((aligned(16))) UINT8 endp7RTbuff[4096] __attribute__((section(".DMADATA"))); //端点7数据收发缓冲区

/*超速设备描述符*/
const UINT8 SS_DeviceDescriptor[] =
    {
        0x12, // bLength
        0x01, // DEVICE descriptor type
        0x00, // 3.00
        0x03,
        0xff, // device class
        0x80, // device sub-class
        0x55, // vendor specific protocol
        0x09, // max packet size 512B
        0x86, // vendor id-0x1A86(qinheng)
        0x1A,
        0x37, // product id 0x8080
        0x55,
        0x00, //bcdDevice 0x0011
        0x01,
        0x01, // manufacturer index string
        0x02, // product index string
        0x03, // serial number index string
        0x01  // number of configurations
};

/*超速配置描述符*/
const UINT8 SS_ConfigDescriptor[] =
    {
        0x09, // length of this descriptor
        0x02, // CONFIGURATION (2)
        0xc8, // total length includes endpoint descriptors (should be 1 more than last address)
        0x00, // total length high byte
        0x01, // number of interfaces
        0x01, // configuration value for this one
        0x00, // configuration - string is here, 0 means no string
        0x80, // attributes - bus powered, no wakeup
        0x64, // max power - 800 ma is 100 (64 hex)
              //interface_descriptor
        0x09, // length of the interface descriptor
        0x04, // INTERFACE (4)
        0x00, // Zero based index 0f this interface
        0x00, // Alternate setting value (?)
        0x0e, // Number of endpoints (not counting 0)
        0xff, // Interface class, ff is vendor specific
        0xff, // Interface sub-class
        0xff, // Interface protocol
        0x00, // Index to string descriptor for this interface
              //Endpoint 1 Descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x81, // endpoint direction (80 is in) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp1_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP1_IN_BURST_LEVEL - 1, // max burst size
        0x00,                         // no stream
        0x00,
        0x00,
        //endp1_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x01, // endpoint direction (00 is out) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp1_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP1_OUT_BURST_LEVEL - 1, // max burst size
        0x00,                          // no stream
        0x00,
        0x00,
        //endp2_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x82, // endpoint direction (80 is in) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp2_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP2_IN_BURST_LEVEL - 1, // max burst size
        0x00,                         // no stream
        0x00,
        0x00,
        //endp2_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x02, // endpoint direction (00 is out) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp2_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP2_OUT_BURST_LEVEL - 1, // max burst size
        0x00,                          // no stream
        0x00,
        0x00,
        //endp3_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x83, // endpoint direction (80 is in) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp3_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP3_IN_BURST_LEVEL - 1, // max burst size
        0x00,                         // no stream
        0x00,
        0x00,
        //endp3_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x03, // endpoint direction (00 is out) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp3_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP3_OUT_BURST_LEVEL - 1, // max burst size
        0x00,                          // no stream
        0x00,
        0x00,
        //endp4_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x84, // endpoint direction (80 is in) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp4_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP4_IN_BURST_LEVEL - 1, // max burst size
        0x00,                         // no stream
        0x00,
        0x00,
        //endp4_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x04, // endpoint direction (00 is out) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp4_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP4_OUT_BURST_LEVEL - 1, // max burst size
        0x00,                          // no stream
        0x00,
        0x00,
        //endp5_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x85, // endpoint direction (80 is in) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp5_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP5_IN_BURST_LEVEL - 1, // max burst size
        0x00,                         // no stream
        0x00,
        0x00,
        //endp5_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x05, // endpoint direction (00 is out) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp5_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP5_OUT_BURST_LEVEL - 1, // max burst size
        0x00,                          // no stream
        0x00,
        0x00,
        //endp6_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x86, // endpoint direction (80 is in) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp6_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP6_IN_BURST_LEVEL - 1, // max burst size
        0x00,                         // no stream
        0x00,
        0x00,
        //endp6_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x06, // endpoint direction (00 is out) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp6_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP6_OUT_BURST_LEVEL - 1, // max burst size
        0x00,                          // no stream
        0x00,
        0x00,
        //endp7_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x87, // endpoint direction (80 is in) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp7_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP7_IN_BURST_LEVEL - 1, // max burst size
        0x00,                         // no stream
        0x00,
        0x00,
        //endp7_descriptor
        0x07, // length of this endpoint descriptor
        0x05, // ENDPOINT (5)
        0x07, // endpoint direction (00 is out) and address
        0x02, // transfer type - 00 = control, 01 = iso, 10 = bulk, 11 = int
        0x00, // max packet size - 1024 bytes
        0x04, // max packet size - high
        0x00, // polling interval in milliseconds (1 for iso)
              //endp7_compansion_desc
        0x06, // length of this endpoint compansion descriptor
        0x30,
        DEF_ENDP7_OUT_BURST_LEVEL - 1, // max burst size
        0x00,                          // no stream
        0x00,
        0x00};

// ===== String Descriptor Lang ID=====
// ====================================//

const UINT8 StringLangID[] =
    {
        0x04, // this descriptor length
        0x03, // descriptor type
        0x09, // Language ID 0 low byte
        0x04  // Language ID 0 high byte
};

// ====================================
// =====   String Descriptor 1    =====
// ====================================
const UINT8 StringVendor[] =
    {
        0x08, // length of this descriptor
        0x03,
        'W',
        0x00,
        'C',
        0x00,
        'H',
        0x00};

// ====================================
// =====   String Descriptor 2    =====
// ====================================
const UINT8 StringProduct[] =
    {
        38,         //该描述符的长度为38字节
        0x03,       //字符串描述符的类型编码为0x03
        0x57, 0x00, //W
        0x43, 0x00, //C
        0x48, 0x00, //H
        0x20, 0x00, //
        0x55, 0x00, //U
        0x53, 0x00, //S
        0x42, 0x00, //B
        0x33, 0x00, //3
        0x2e, 0x00, //.
        0x30, 0x00, //0
        0x20, 0x00, //
        0x44, 0x00, //D
        0x45, 0x00, //E
        0x56, 0x00, //V
        0x49, 0x00, //I
        0x43, 0x00, //C
        0x45, 0x00, //E
        0x20, 0x00};

// ====================================
// =====   String Descriptor 3    =====
// ====================================
UINT8 StringSerial[] =
    {
        0x16, // length of this descriptor
        0x03,
        '0',
        0x00,
        '1',
        0x00,
        '2',
        0x00,
        '3',
        0x00,
        '4',
        0x00,
        '5',
        0x00,
        '6',
        0x00,
        '7',
        0x00,
        '8',
        0x00,
        '9',
        0x00,
};

const UINT8 OSStringDescriptor[] =
    {
        0x12, // length of this descriptor
        0x03,
        'M',
        0x00,
        'S',
        0x00,
        'F',
        0x00,
        'T',
        0x00,
        '1',
        0x00,
        '0',
        0x00,
        '0',
        0x00,
        0x01,
        0x00};

const UINT8 BOSDescriptor[] =
    {
        0x05, // length of this descriptor
        0x0f, // CONFIGURATION (2)
        0x16, // total length includes endpoint descriptors (should be 1 more than last address)
        0x00, // total length high byte
        0x02, // number of device cap

        //dev_cap_descriptor1
        0x07,
        0x10, // DEVICE CAPABILITY type
        0x02, // USB2.0 EXTENSION
        0x06,
        0x00,
        0x00,
        0x00,

        //dev_cap_descriptor2
        0x0a, // length of this descriptor
        0x10, // DEVICE CAPABILITY type
        0x03, // superspeed usb device capability
        0x00, //
        0x0e, // ss/hs/fs
        0x00,
        0x01, // the lowest speed is full speed
        0x0a, // u1 exit latency is 10us
        0xff, // u1 exit latency is 8us
        0x07};

const UINT8 MSOS20DescriptorSet[] =
    {
        // Microsoft OS 2.0 Descriptor Set Header
        0x0A, 0x00,             // wLength - 10 bytes
        0x00, 0x00,             // MSOS20_SET_HEADER_DESCRIPTOR
        0x00, 0x00, 0x03, 0x06, // dwWindowsVersion 每 0x06030000 for Windows Blue
        0x48, 0x00,             // wTotalLength 每 72 bytes
                    // Microsoft OS 2.0 Registry Value Feature Descriptor
        0x3E, 0x00,             // wLength - 62 bytes
        0x04, 0x00,             // wDescriptorType 每 4 for Registry Property
        0x04, 0x00,             // wPropertyDataType - 4 for REG_DWORD
        0x30, 0x00,             // wPropertyNameLength 每 48 bytes
        0x53, 0x00, 0x65, 0x00, // Property Name - ※SelectiveSuspendEnabled§
        0x6C, 0x00, 0x65, 0x00,
        0x63, 0x00, 0x74, 0x00,
        0x69, 0x00, 0x76, 0x00,
        0x65, 0x00, 0x53, 0x00,
        0x75, 0x00, 0x73, 0x00,
        0x70, 0x00, 0x65, 0x00,
        0x6E, 0x00, 0x64, 0x00,
        0x45, 0x00, 0x6E, 0x00,
        0x61, 0x00, 0x62, 0x00,
        0x6C, 0x00, 0x65, 0x00,
        0x64, 0x00, 0x00, 0x00,
        0x04, 0x00,            // wPropertyDataLength 每 4 bytes
        0x01, 0x00, 0x00, 0x00 // PropertyData - 0x00000001
};

const UINT8 PropertyHeader[] =
    {
        0x8e, 0x00, 0x00, 0x00, 0x00, 01, 05, 00, 01, 00,
        0x84, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x00, 0x00,
        0x28, 0x00,
        0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00, 0x49, 0x00, 0x6e,
        0x00, 0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x66, 0x00, 0x61, 0x00, 0x63, 0x00, 0x65, 0x00, 0x47, 0x00, 0x55, 0x00, 0x49, 0x00, 0x44, 0x00, 0x00, 0x00,

        0x4e, 0x00, 0x00, 0x00,
        0x7b, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00, 0x35, 0x00, 0x36, 0x00, 0x37, 0x00, 0x38, 0x00,
        0x2d, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00,
        0x2d, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00,
        0x2d, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00, 0x2d, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00,
        0x34, 0x00, 0x35, 0x00, 0x36, 0x00, 0x37, 0x00, 0x38, 0x00, 0x39, 0x00, 0x41, 0x00, 0x42, 0x00, 0x43, 0x00,
        0x7d, 0x00, 0x00, 0x00};

const UINT8 CompactId[] =
    {
        0x28, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x57, 0x49, 0x4e, 0x55, 0x53, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

UINT8 GetStatus[] =
    {
        0x01, 0x00};

/*******************************************************************************
 * @fn      USB30_BUS_RESET
 *
 * @brief   USB3.0总线复位，代码中选择复位方式，建议采用只复位USB的方式，如果连接有问题，可以采用复位单片机的方式
 *
 * @return   None
 */
void USB30_BUS_RESET()
{
    USB30D_init(DISABLE); //USB3.0初始化
    mDelaymS(20);
    USB30D_init(ENABLE); //USB3.0初始化
}

/*******************************************************************************
 * @fn      USB30D_init
 *
 * @brief   USB3.0设备初始化
 *
 * @return   None
 */
void USB30D_init(FunctionalState sta)
{
    UINT16 i, s;
    if(sta)
    {
        s = USB30_Device_Init();
        if(s)
        {
            printf("Init err\n");
            while(1)
                ;
        }
        USBSS->UEP_CFG = EP0_R_EN | EP0_T_EN | EP1_R_EN | EP1_T_EN | EP2_R_EN | EP2_T_EN | EP3_R_EN | EP3_T_EN | EP4_R_EN | EP4_T_EN | EP5_R_EN | EP5_T_EN | EP6_R_EN | EP6_T_EN | EP7_R_EN | EP7_T_EN; // set end point rx/tx enable

        USBSS->UEP0_DMA = (UINT32)(UINT8 *)endp0RTbuff;
        USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)endp1RTbuff;
        USBSS->UEP2_TX_DMA = (UINT32)(UINT8 *)endp2RTbuff;
        USBSS->UEP3_TX_DMA = (UINT32)(UINT8 *)endp3RTbuff;
        USBSS->UEP4_TX_DMA = (UINT32)(UINT8 *)endp4RTbuff;
        USBSS->UEP5_TX_DMA = (UINT32)(UINT8 *)endp5RTbuff;
        USBSS->UEP6_TX_DMA = (UINT32)(UINT8 *)endp6RTbuff;
        USBSS->UEP7_TX_DMA = (UINT32)(UINT8 *)endp7RTbuff;

        USBSS->UEP1_RX_DMA = (UINT32)(UINT8 *)endp1RTbuff;
        USBSS->UEP2_RX_DMA = (UINT32)(UINT8 *)endp2RTbuff;
        USBSS->UEP3_RX_DMA = (UINT32)(UINT8 *)endp3RTbuff;
        USBSS->UEP4_RX_DMA = (UINT32)(UINT8 *)endp4RTbuff;
        USBSS->UEP5_RX_DMA = (UINT32)(UINT8 *)endp5RTbuff;
        USBSS->UEP6_RX_DMA = (UINT32)(UINT8 *)endp6RTbuff;
        USBSS->UEP7_RX_DMA = (UINT32)(UINT8 *)endp7RTbuff;

        USB30_OUT_Set(ENDP_1, ACK, DEF_ENDP1_OUT_BURST_LEVEL); // endpoint1 receive setting
        USB30_OUT_Set(ENDP_2, ACK, DEF_ENDP2_OUT_BURST_LEVEL);
        USB30_OUT_Set(ENDP_3, ACK, DEF_ENDP3_OUT_BURST_LEVEL);
        USB30_OUT_Set(ENDP_4, ACK, DEF_ENDP4_OUT_BURST_LEVEL);
        USB30_OUT_Set(ENDP_5, ACK, DEF_ENDP5_OUT_BURST_LEVEL);
        USB30_OUT_Set(ENDP_6, ACK, DEF_ENDP6_OUT_BURST_LEVEL);
        USB30_OUT_Set(ENDP_7, ACK, DEF_ENDP7_OUT_BURST_LEVEL);

        USB30_IN_Set(ENDP_2, ENABLE, ACK, DEF_ENDP2_IN_BURST_LEVEL, 1024); // endpoint2 send setting
    }
    else
    {
        USB30_Switch_Powermode(POWER_MODE_2);
        USBSS->LINK_CFG = PIPE_RESET | LFPS_RX_PD;
        USBSS->LINK_CTRL = GO_DISABLED | POWER_MODE_3;
        USBSS->LINK_INT_CTRL = 0;
        USBSS->USB_CONTROL = USB_FORCE_RST | USB_ALL_CLR;
    }
}

/*******************************************************************************
 * @fn      USB30_NonStandardReq
 *
 * @brief   USB3.0非标准请求处理函数
 *
 * @return   长度
 */
UINT16 USB30_NonStandardReq()
{
    UINT8 endp_dir;

    SetupReqCode = UsbSetupBuf->bRequest;
    SetupLen = UsbSetupBuf->wLength;
    endp_dir = UsbSetupBuf->bRequestType & 0x80;
    UINT16 len = 0;
#if 0
    printf("NS:%02x %02x %02x %02x %02x %02x %02x %02x\n", endp0RTbuff[0], endp0RTbuff[1],
            endp0RTbuff[2], endp0RTbuff[3], endp0RTbuff[4], endp0RTbuff[5],
            endp0RTbuff[6], endp0RTbuff[7]);
#endif
    switch(SetupReqCode)
    {
            //     case 0x01:
            //         switch(UsbSetupBuf->wIndex.bw.bb1)
            //         {
            //             case 0x04:
            //                if(SetupLen>SIZE_CompactId) SetupLen  = SIZE_CompactId;
            //                pDescr = (PUINT8)CompactId;
            //                break;
            //             case 0x06:
            //                 break;
            //             case 0x09://
            //                 break;
            //             case 0x07:
            //               if(SetupLen>SIZE_MSOS20DescriptorSet) SetupLen  = SIZE_MSOS20DescriptorSet;
            //               pDescr = (PUINT8)MSOS20DescriptorSet;
            //                break;
            //            case 0x08:
            //                break;
            //            case 0xc0:
            //                break;
            //            default:
            //                SetupReqCode = INVALID_REQ_CODE;
            //                return USB_DESCR_UNSUPPORTED;
            //                break;
            //            }
            //            break;
        case 0x02: //用户定义命令
            switch(UsbSetupBuf->wIndex.bw.bb1)
            {
                case 0x05:
                    if(SetupLen > SIZE_PropertyHeader)
                        SetupLen = SIZE_PropertyHeader;
                    pDescr = (PUINT8)PropertyHeader;
                    break;
                default:
                    SetupReqCode = INVALID_REQ_CODE;
                    return USB_DESCR_UNSUPPORTED;
                    break;
            }
            break;
        default:
            //        printf("stall\n");
            SetupReqCode = INVALID_REQ_CODE;
            return USB_DESCR_UNSUPPORTED;
            break;
    }
    len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen; // 本次传输长度
    if(endp_dir)
    {
        memcpy(endp0RTbuff, pDescr, len); // device  /* 加载上传数据 */
        pDescr += len;
    }
    SetupLen -= len;
    return len;
}

/*******************************************************************************
 * @fn      USB30_StandardReq
 *
 * @brief   USB3.0标准请求
 *
 * @return   长度
 */
UINT16 USB30_StandardReq()
{
    SetupReqCode = UsbSetupBuf->bRequest;
    SetupLen = UsbSetupBuf->wLength;
    UINT16 len = 0;
#if 0
    printf("S:%02x %02x %02x %02x %02x %02x %02x %02x\n", endp0RTbuff[0], endp0RTbuff[1],
            endp0RTbuff[2], endp0RTbuff[3], endp0RTbuff[4], endp0RTbuff[5],
            endp0RTbuff[6], endp0RTbuff[7]);
#endif
    switch(SetupReqCode)
    {
        case USB_GET_DESCRIPTOR:
            switch(UsbSetupBuf->wValue.bw.bb0)
            {
                case USB_DESCR_TYP_DEVICE:
                    if(SetupLen > SIZE_DEVICE_DESC)
                        SetupLen = SIZE_DEVICE_DESC;
                    pDescr = (PUINT8)SS_DeviceDescriptor;
                    break;
                case USB_DESCR_TYP_CONFIG:
                    if(SetupLen > SIZE_CONFIG_DESC)
                        SetupLen = SIZE_CONFIG_DESC;
                    pDescr = (PUINT8)SS_ConfigDescriptor;
                    break;
                case USB_DESCR_TYP_BOS:
                    if(SetupLen > SIZE_BOS_DESC)
                        SetupLen = SIZE_BOS_DESC;
                    pDescr = (PUINT8)BOSDescriptor;
                    break;
                case USB_DESCR_TYP_STRING:
                    switch(UsbSetupBuf->wValue.bw.bb1)
                    {
                        case USB_DESCR_LANGID_STRING:
                            if(SetupLen > SIZE_STRING_LANGID)
                                SetupLen = SIZE_STRING_LANGID;
                            pDescr = (PUINT8)StringLangID;
                            break;
                        case USB_DESCR_VENDOR_STRING:
                            if(SetupLen > SIZE_STRING_VENDOR)
                                SetupLen = SIZE_STRING_VENDOR;
                            pDescr = (PUINT8)StringVendor;
                            break;
                        case USB_DESCR_PRODUCT_STRING:
                            if(SetupLen > SIZE_STRING_PRODUCT)
                                SetupLen = SIZE_STRING_PRODUCT;
                            pDescr = (PUINT8)StringProduct;
                            break;
                        case USB_DESCR_SERIAL_STRING:
                            if(SetupLen > SIZE_STRING_SERIAL)
                                SetupLen = SIZE_STRING_SERIAL;
                            pDescr = (PUINT8)StringSerial;
                            break;
                        case USB_DESCR_OS_STRING:
                            if(SetupLen > SIZE_STRING_OS)
                                SetupLen = SIZE_STRING_OS;
                            pDescr = (PUINT8)OSStringDescriptor;
                            break;
                        default:
                            len = USB_DESCR_UNSUPPORTED;     //不支持的描述符
                            SetupReqCode = INVALID_REQ_CODE; //无效的请求码
                            break;
                    }
                    break;
                default:
                    len = USB_DESCR_UNSUPPORTED; //不支持的描述符
                    SetupReqCode = INVALID_REQ_CODE;
                    break;
            }
            len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen; //本次传输长度
            memcpy(endp0RTbuff, pDescr, len);                           // device  /*加载上传数据 */
            SetupLen -= len;
            pDescr += len;
            break;
        case USB_SET_ADDRESS:
            SetupLen = UsbSetupBuf->wValue.bw.bb1; // 暂存USB设备地址
            break;
        case 0x31:
            SetupLen = UsbSetupBuf->wValue.bw.bb1; // 暂存USB设备地址
            break;
        case 0x30:
            break;
        case USB_SET_CONFIGURATION:
            break;
        case USB_GET_STATUS:
            len = 2;
            endp0RTbuff[0] = 0x01;
            endp0RTbuff[1] = 0x00;
            SetupLen = 0;
            break;
        case USB_CLEAR_FEATURE:
            break;
        case USB_SET_FEATURE:
            break;
        case USB_SET_INTERFACE:
            break;
        default:
            len = USB_DESCR_UNSUPPORTED; //返回stall，不支持的命令
            SetupReqCode = INVALID_REQ_CODE;
            //        printf(" stall \n");
            break;
    }
    return len;
}

/*******************************************************************************
 * @fn      EP0_IN_Callback
 *
 * @brief   USB3.0端点0IN事务回调
 *
 * @return   发送长度
 */
UINT16 EP0_IN_Callback(void)
{
    UINT16 len = 0;
    switch(SetupReqCode)
    {
        case USB_GET_DESCRIPTOR:
            len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen;
            memcpy(endp0RTbuff, pDescr, len);
            SetupLen -= len;
            pDescr += len;
            break;
    }
    return len;
}

/*******************************************************************************
 * @fn      EP0_OUT_Callback
 *
 * @brief   USB3.0端点0OUT回调
 *
 * @return   长度
 */
UINT16 EP0_OUT_Callback(void)
{
    UINT16 len;
    return len;
}

/*******************************************************************************
 * @fn      USB30_Setup_Status
 *
 * @brief   USB3.0控制传输状态阶段回调
 *
 * @return   None
 */
void USB30_Setup_Status(void)
{
    switch(SetupReqCode)
    {
        case USB_SET_ADDRESS:
            USB30_Device_Setaddress(SetupLen); // SET ADDRESS
            break;
        case 0x31:
            break;
    }
}

/*******************************************************************************
 * @fn      USBSS_IRQHandler
 *
 * @brief   USB3.0 Interrupt Handler.
 *
 * @return   None
 */
void USBSS_IRQHandler(void) //USBSS interrupt service
{
    USB30_IRQHandler();
}

/*******************************************************************************
 * @fn      TMR0_IRQHandler
 *
 * @brief   USB3.0连接失败超时处理
 *
 * @return   None
 */
void TMR0_IRQHandler()
{
    R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;

    if(link_sta == 1)
    {
        link_sta = 0;
        PFIC_DisableIRQ(USBSS_IRQn);
        PFIC_DisableIRQ(LINK_IRQn);
        USB30D_init(DISABLE);
        PRINT("USB3.0 disable\n");
        return;
    }
    if(link_sta != 3)
    {
        PFIC_DisableIRQ(USBSS_IRQn);
        PFIC_DisableIRQ(LINK_IRQn);
        USB30D_init(DISABLE);
        //        PRINT("USB2.0\n");
        R32_USB_CONTROL = 0;
        PFIC_EnableIRQ(USBHS_IRQn);
        USB20_Device_Init(ENABLE);
    }
    link_sta = 1;
    R8_TMR0_INTER_EN = 0;
    PFIC_DisableIRQ(TMR0_IRQn);
    R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
}

/*******************************************************************************
 * @fn      LINK_IRQHandler
 *
 * @brief   USB3.0 Link Interrupt Handler.
 *
 * @return   None
 */
void LINK_IRQHandler() //USBSS link interrupt service
{
    if(USBSS->LINK_INT_FLAG & LINK_Ux_EXIT_FLAG) // device enter U2
    {
        USBSS->LINK_CFG = CFG_EQ_EN | DEEMPH_CFG | TERM_EN;
        USB30_Switch_Powermode(POWER_MODE_0);
        USBSS->LINK_INT_FLAG = LINK_Ux_EXIT_FLAG;
    }
    if(USBSS->LINK_INT_FLAG & LINK_RDY_FLAG) // POLLING SHAKE DONE
    {
        USBSS->LINK_INT_FLAG = LINK_RDY_FLAG;
        if(tx_lmp_port) // LMP, TX PORT_CAP & RX PORT_CAP
        {
            USBSS->LMP_TX_DATA0 = LINK_SPEED | PORT_CAP | LMP_HP;
            USBSS->LMP_TX_DATA1 = UP_STREAM | NUM_HP_BUF;
            USBSS->LMP_TX_DATA2 = 0x0;
            tx_lmp_port = 0;
        }
        //成功USB3.0通讯
        link_sta = 3;
        PFIC_DisableIRQ(TMR0_IRQn);
        R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
        R8_TMR0_INTER_EN = 0;
        PFIC_DisableIRQ(USBHS_IRQn);
        USB20_Device_Init(DISABLE);
    }

    if(USBSS->LINK_INT_FLAG & LINK_INACT_FLAG)
    {
        USBSS->LINK_INT_FLAG = LINK_INACT_FLAG;
        USB30_Switch_Powermode(POWER_MODE_2);
    }
    if(USBSS->LINK_INT_FLAG & LINK_DISABLE_FLAG) // GO DISABLED
    {
        USBSS->LINK_INT_FLAG = LINK_DISABLE_FLAG;
        link_sta = 1;
        USB30D_init(DISABLE);
        PFIC_DisableIRQ(USBSS_IRQn);
        R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
        R8_TMR0_INTER_EN = 0;
        PFIC_DisableIRQ(TMR0_IRQn);
        PFIC_EnableIRQ(USBHS_IRQn);
        USB20_Device_Init(ENABLE);
    }
    if(USBSS->LINK_INT_FLAG & LINK_RX_DET_FLAG)
    {
        USBSS->LINK_INT_FLAG = LINK_RX_DET_FLAG;
        USB30_Switch_Powermode(POWER_MODE_2);
    }
    if(USBSS->LINK_INT_FLAG & TERM_PRESENT_FLAG) // term present , begin POLLING
    {
        USBSS->LINK_INT_FLAG = TERM_PRESENT_FLAG;
        if(USBSS->LINK_STATUS & LINK_PRESENT)
        {
            USB30_Switch_Powermode(POWER_MODE_2);
            USBSS->LINK_CTRL |= POLLING_EN;
        }
        else
        {
            USBSS->LINK_INT_CTRL = 0;
            mDelayuS(2000);
            USB30_BUS_RESET();
        }
    }
    if(USBSS->LINK_INT_FLAG & LINK_TXEQ_FLAG) // POLLING SHAKE DONE
    {
        tx_lmp_port = 1;
        USBSS->LINK_INT_FLAG = LINK_TXEQ_FLAG;
        USB30_Switch_Powermode(POWER_MODE_0);
    }
    if(USBSS->LINK_INT_FLAG & WARM_RESET_FLAG)
    {
        USBSS->LINK_INT_FLAG = WARM_RESET_FLAG;
        USB30_Switch_Powermode(POWER_MODE_2);
        USB30_BUS_RESET();
        USB30_Device_Setaddress(0);
        USBSS->LINK_CTRL |= TX_WARM_RESET;
        while(USBSS->LINK_STATUS & RX_WARM_RESET)
            ;
        USBSS->LINK_CTRL &= ~TX_WARM_RESET;
        mDelayuS(2);
        //        printf("warm reset\n");
    }
    if(USBSS->LINK_INT_FLAG & HOT_RESET_FLAG) //主机可能会下发hot reset,需注意端点的配置
    {
        USBSS->USB_CONTROL |= 1 << 31;
        USBSS->LINK_INT_FLAG = HOT_RESET_FLAG; // HOT RESET begin
        USBSS->UEP0_TX_CTRL = 0;
        USB30_IN_Set(ENDP_1, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_2, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_3, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_4, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_5, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_6, DISABLE, NRDY, 0, 0);
        USB30_IN_Set(ENDP_7, DISABLE, NRDY, 0, 0);
        USB30_OUT_Set(ENDP_1, NRDY, 0);
        USB30_OUT_Set(ENDP_2, NRDY, 0);
        USB30_OUT_Set(ENDP_3, NRDY, 0);
        USB30_OUT_Set(ENDP_4, NRDY, 0);
        USB30_OUT_Set(ENDP_5, NRDY, 0);
        USB30_OUT_Set(ENDP_6, NRDY, 0);
        USB30_OUT_Set(ENDP_7, NRDY, 0);

        USB30_Device_Setaddress(0);
        USBSS->LINK_CTRL &= ~TX_HOT_RESET; // HOT RESET end
    }
    if(USBSS->LINK_INT_FLAG & LINK_GO_U1_FLAG) // device enter U1
    {
        USB30_Switch_Powermode(POWER_MODE_1);
        USBSS->LINK_INT_FLAG = LINK_GO_U1_FLAG;
    }
    if(USBSS->LINK_INT_FLAG & LINK_GO_U2_FLAG) // device enter U2
    {
        USB30_Switch_Powermode(POWER_MODE_2);
        USBSS->LINK_INT_FLAG = LINK_GO_U2_FLAG;
    }
    if(USBSS->LINK_INT_FLAG & LINK_GO_U3_FLAG) // device enter U2
    {
        USB30_Switch_Powermode(POWER_MODE_2);
        USBSS->LINK_INT_FLAG = LINK_GO_U3_FLAG;
    }
}
/***************Endpointn IN Transaction Processing*******************/
/*******************************************************************************
 * @fn      EP1_IN_Callback
 *
 * @brief   USB3.0 endpoint1 in callback.
 *
 * @return   None
 */
void EP1_IN_Callback(void)
{
    UINT8 nump;
    nump = USB30_IN_Nump(ENDP_1); //nump: 剩余待发送包数量
    if(nump == 0)
    {                                                             //全部发完
        USBSS->UEP1_TX_DMA = (UINT32)(UINT8 *)endp1RTbuff;        //突发传输 DMA地址偏移 需重重置
        USB30_IN_ClearIT(ENDP_1);                                 //清除端点状态 仅保留包序列号
        USB30_OUT_Set(ENDP_1, ACK, DEF_ENDP1_OUT_BURST_LEVEL);    //设置端点能够接收4包
        USB30_Send_ERDY(ENDP_1 | OUT, DEF_ENDP1_OUT_BURST_LEVEL); //通知主机发4包
    }

    //还剩一包未发完; 在突发过程中主机可能一次不能取走全部的数据包 因此要判断当前剩余包数量 通知主机还有几包未取,end of burst位需写enable
    else if(nump == 1)
    {
        USB30_IN_ClearIT(ENDP_1);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_1, ENABLE, ACK, 1, 1024); //能够发送1包
        USB30_Send_ERDY(ENDP_1 | IN, 1);
    }
    else if(nump == 2)
    {
        USB30_IN_ClearIT(ENDP_1);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_1, ENABLE, ACK, 2, 1024); //能够发送2包
        USB30_Send_ERDY(ENDP_1 | IN, 2);
    }
    else if(nump == 3)
    {
        USB30_IN_ClearIT(ENDP_1);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_1, ENABLE, ACK, 3, 1024); //能够发送3包
        USB30_Send_ERDY(ENDP_1 | IN, 3);
    }
}

/*******************************************************************************
 * @fn      EP2_IN_Callback
 *
 * @brief   USB3.0 endpoint2 in callback.
 *
 * @return   None
 */
void EP2_IN_Callback(void)
{
    UINT8 nump;
    nump = USB30_IN_Nump(ENDP_2); //nump 剩余待发送包数量
    if(nump == 0)
    {                                                                      //全部发完
        USBSS->UEP2_TX_DMA = (UINT32)(UINT8 *)endp2RTbuff;                 //突发传输 DMA地址偏移 需重重置
        USB30_IN_ClearIT(ENDP_2);                                          //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_2, ENABLE, ACK, DEF_ENDP2_IN_BURST_LEVEL, 1024); //设置端点能够发送4包
        USB30_Send_ERDY(ENDP_2 | IN, DEF_ENDP2_IN_BURST_LEVEL);            //通知主机取4包
    }
    else if(nump == 1)
    {
        USB30_IN_ClearIT(ENDP_2);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_2, ENABLE, ACK, 1, 1024); //能够发送1包
        USB30_Send_ERDY(ENDP_2 | IN, 1);
    }
    else if(nump == 2)
    {
        USB30_IN_ClearIT(ENDP_2);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_2, ENABLE, ACK, 2, 1024); //能够发送2包
        USB30_Send_ERDY(ENDP_2 | IN, 2);
    }
    else if(nump == 3)
    {
        USB30_IN_ClearIT(ENDP_2);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_2, ENABLE, ACK, 3, 1024); //能够发送3包
        USB30_Send_ERDY(ENDP_2 | IN, 3);
    }
}

/*******************************************************************************
 * @fn      EP3_IN_Callback
 *
 * @brief   USB3.0 endpoint3 in callback.
 *
 * @return   None
 */
void EP3_IN_Callback(void)
{
    UINT8 nump;
    nump = USB30_IN_Nump(ENDP_3); //nump 剩余待发送包数量
    if(nump == 0)
    {
        USBSS->UEP3_TX_DMA = (UINT32)(UINT8 *)endp3RTbuff;        //突发传输 DMA地址偏移 需重重置
        USB30_IN_ClearIT(ENDP_3);                                 //清除端点状态 仅保留包序列号
        USB30_OUT_Set(ENDP_3, ACK, DEF_ENDP3_OUT_BURST_LEVEL);    //能够接收4包
        USB30_Send_ERDY(ENDP_3 | OUT, DEF_ENDP3_OUT_BURST_LEVEL); //通知主机发4包
    }
    else if(nump == 1)
    {
        USB30_IN_ClearIT(ENDP_3);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_3, ENABLE, ACK, 1, 1024); //能够发送1包
        USB30_Send_ERDY(ENDP_3 | IN, 1);
    }
    else if(nump == 2)
    {
        USB30_IN_ClearIT(ENDP_3);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_3, ENABLE, ACK, 2, 1024); //能够发送2包
        USB30_Send_ERDY(ENDP_3 | IN, 2);
    }
    else if(nump == 3)
    {
        USB30_IN_ClearIT(ENDP_3);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_3, ENABLE, ACK, 3, 1024); //能够发送3包
        USB30_Send_ERDY(ENDP_3 | IN, 3);
    }
}

/*******************************************************************************
 * @fn      EP4_IN_Callback
 *
 * @brief   USB3.0 endpoint4 in callback.
 *
 * @return   None
 */
void EP4_IN_Callback(void)
{
    UINT8 nump;
    nump = USB30_IN_Nump(ENDP_4); //s: 剩余待发送包数量
    if(nump == 0)
    {                                                             //全部发完
        USBSS->UEP4_TX_DMA = (UINT32)(UINT8 *)endp4RTbuff;        //突发传输 DMA地址偏移 需重重置
        USB30_IN_ClearIT(ENDP_4);                                 //清除端点状态 仅保留包序列号
        USB30_OUT_Set(ENDP_4, ACK, DEF_ENDP4_OUT_BURST_LEVEL);    //设置能够接收4包
        USB30_Send_ERDY(ENDP_4 | OUT, DEF_ENDP4_OUT_BURST_LEVEL); //通知主机发4包
    }
    else if(nump == 1) //还剩一包没发完
    {
        USB30_IN_ClearIT(ENDP_4);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_4, ENABLE, ACK, 1, 1024); //能够发送1包
        USB30_Send_ERDY(ENDP_4 | IN, 1);
    }
    else if(nump == 2)
    {
        USB30_IN_ClearIT(ENDP_4);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_4, ENABLE, ACK, 2, 1024); //能够发送2包
        USB30_Send_ERDY(ENDP_4 | IN, 2);
    }
    else if(nump == 3)
    {
        USB30_IN_ClearIT(ENDP_4);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_4, ENABLE, ACK, 3, 1024); //能够发送3包
        USB30_Send_ERDY(ENDP_4 | IN, 3);
    }
}
/*******************************************************************************
 * @fn      EP5_IN_Callback
 *
 * @brief   USB3.0 endpoint5 in callback.
 *
 * @return   None
 */
void EP5_IN_Callback(void)
{
    UINT8 nump;
    nump = USB30_IN_Nump(ENDP_5); //nump 剩余待发送包数量
    if(nump == 0)
    {                                                             //全部发完
        USBSS->UEP5_TX_DMA = (UINT32)(UINT8 *)endp5RTbuff;        //突发传输 DMA地址偏移 需重重置
        USB30_IN_ClearIT(ENDP_5);                                 //清除端点状态 仅保留包序列号
        USB30_OUT_Set(ENDP_5, ACK, DEF_ENDP5_OUT_BURST_LEVEL);    //设置能够接收4包
        USB30_Send_ERDY(ENDP_5 | OUT, DEF_ENDP5_OUT_BURST_LEVEL); //通知主机发4包
    }
    else if(nump == 1) //还剩一包未发完
    {
        USB30_IN_ClearIT(ENDP_5);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_5, ENABLE, ACK, 1, 1024); //能够发送1包
        USB30_Send_ERDY(ENDP_5 | IN, 1);
    }
    else if(nump == 2)
    {
        USB30_IN_ClearIT(ENDP_5);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_5, ENABLE, ACK, 2, 1024); //能够发送2包
        USB30_Send_ERDY(ENDP_5 | IN, 2);
    }
    else if(nump == 3)
    {
        USB30_IN_ClearIT(ENDP_5);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_5, ENABLE, ACK, 3, 1024); //能够发送3包
        USB30_Send_ERDY(ENDP_5 | IN, 3);
    }
}
/*******************************************************************************
 * @fn      EP6_IN_Callback
 *
 * @brief   USB3.0 endpoint6 in callback.
 *
 * @return   None
 */
void EP6_IN_Callback(void)
{
    UINT8 nump;
    nump = USB30_IN_Nump(ENDP_6); //nump 剩余待发送包数量
    if(nump == 0)
    {                                                             //全部发完
        USBSS->UEP6_TX_DMA = (UINT32)(UINT8 *)endp6RTbuff;        //突发传输 DMA地址偏移 需重重置
        USB30_IN_ClearIT(ENDP_6);                                 //清除端点状态 仅保留包序列号
        USB30_OUT_Set(ENDP_6, ACK, DEF_ENDP6_OUT_BURST_LEVEL);    //设置能够接收4包
        USB30_Send_ERDY(ENDP_6 | OUT, DEF_ENDP6_OUT_BURST_LEVEL); //通知主机发4包
    }
    else if(nump == 1) //还剩一包没发完
    {
        USB30_IN_ClearIT(ENDP_6);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_6, ENABLE, ACK, 1, 1024); //能够发送4包
        USB30_Send_ERDY(ENDP_6 | IN, 1);
    }
    else if(nump == 2)
    {
        USB30_IN_ClearIT(ENDP_6);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_6, ENABLE, ACK, 2, 1024); //能够发送4包
        USB30_Send_ERDY(ENDP_6 | IN, 2);
    }
    else if(nump == 3)
    {
        USB30_IN_ClearIT(ENDP_6);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_6, ENABLE, ACK, 3, 1024); //能够发送4包
        USB30_Send_ERDY(ENDP_6 | IN, 3);
    }
}
/*******************************************************************************
 * @fn      EP7_IN_Callback
 *
 * @brief   USB3.0 endpoint7 in callback.
 *
 * @return   None
 */
void EP7_IN_Callback(void)
{
    UINT8 nump;
    nump = USB30_IN_Nump(ENDP_7); //nump:剩余待发送包数量
    if(nump == 0)
    {                                                             //全部发完
        USBSS->UEP7_TX_DMA = (UINT32)(UINT8 *)endp7RTbuff;        //突发传输 DMA地址偏移 需重重置
        USB30_IN_ClearIT(ENDP_7);                                 //清除端点状态 仅保留包序列号
        USB30_OUT_Set(ENDP_7, ACK, DEF_ENDP7_OUT_BURST_LEVEL);    //设置能够接收4包
        USB30_Send_ERDY(ENDP_7 | OUT, DEF_ENDP7_OUT_BURST_LEVEL); //通知主机发4包
    }
    else if(nump == 1) //还剩一包未发完
    {
        USB30_IN_ClearIT(ENDP_7);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_7, ENABLE, ACK, 1, 1024); //能够发送1包
        USB30_Send_ERDY(ENDP_7 | IN, 1);
    }
    else if(nump == 2)
    {
        USB30_IN_ClearIT(ENDP_7);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_7, ENABLE, ACK, 2, 1024); //能够发送2包
        USB30_Send_ERDY(ENDP_7 | IN, 2);
    }
    else if(nump == 3)
    {
        USB30_IN_ClearIT(ENDP_7);                   //清除端点状态 仅保留包序列号
        USB30_IN_Set(ENDP_7, ENABLE, ACK, 3, 1024); //能够发送3包
        USB30_Send_ERDY(ENDP_7 | IN, 3);
    }
}
/***************Endpointn OUT Transaction Processing*******************/

/*******************************************************************************
 * @fn      EP1_OUT_Callback
 *
 * @brief   USB3.0 endpoint1 out callback.
 *
 * @return   None
 */
void EP1_OUT_Callback(void)
{
    UINT16 rx_len, i;
    UINT8  nump;
    UINT8  status;
    USB30_OUT_Status(ENDP_1, &nump, &rx_len, &status); //获取接收到的包数量   rxlen 为最后一包的包长度
    if(nump == 0)
    {
        USB30_OUT_ClearIT(ENDP_1);                                         //清除端点所有状态 仅保留包序列
        USBSS->UEP1_RX_DMA = (UINT32)(UINT8 *)endp1RTbuff;                 //突发模式下由于地址自动偏移 地址需重置
        USB30_IN_Set(ENDP_1, ENABLE, ACK, DEF_ENDP1_IN_BURST_LEVEL, 1024); //能够1号端点能发送4包
        USB30_Send_ERDY(ENDP_1 | IN, DEF_ENDP1_IN_BURST_LEVEL);            //通知主机取4包
    }
    else if(nump == 1) //还可以接收一包
    {
        USB30_OUT_ClearIT(ENDP_1);
        USB30_OUT_Set(ENDP_1, ACK, 1);    //能够接收一包
        USB30_Send_ERDY(ENDP_1 | OUT, 1); //通知主机下发1包
    }
    else if(nump == 2)
    {
        USB30_OUT_ClearIT(1);
        USB30_OUT_Set(ENDP_1, ACK, 2);    //能够接收2包
        USB30_Send_ERDY(ENDP_1 | OUT, 2); //通知主机下发2包
    }
    else if(nump == 3)
    {
        USB30_OUT_ClearIT(1);
        USB30_OUT_Set(ENDP_1, ACK, 3);    //能够接收3包
        USB30_Send_ERDY(ENDP_1 | OUT, 3); //通知主机下发3包
    }
}

/*******************************************************************************
 * @fn      EP2_OUT_Callback
 *
 * @brief   USB3.0 endpoint2 out callback.
 *
 * @return   None
 */
void EP2_OUT_Callback(void)
{
    UINT16 rx_len, i;
    UINT8  nump;
    UINT8  status;
    UINT32 recv_index;
    USB30_OUT_Status(ENDP_2, &nump, &rx_len, &status); //获取接收到的包数量
    if(nump == 0)                                      //全部接收完了
    {
        USB30_OUT_ClearIT(ENDP_2);                                //清除端点所有状态 仅保留包序列
        USBSS->UEP2_RX_DMA = (UINT32)(UINT8 *)endp2RTbuff;        //突发模式下由于地址自动偏移 地址需重置
        USB30_OUT_Set(ENDP_2, ACK, DEF_ENDP2_OUT_BURST_LEVEL);    //设置端点能够接收4包
        USB30_Send_ERDY(ENDP_2 | OUT, DEF_ENDP2_OUT_BURST_LEVEL); //通知主机下发4包
    }
    else if(nump == 1)
    {
        USB30_OUT_ClearIT(ENDP_2);
        USB30_OUT_Set(ENDP_2, ACK, 1);    //还能够接收一包
        USB30_Send_ERDY(ENDP_2 | OUT, 1); //通知主机下发一包
    }
    else if(nump == 2)
    {
        USB30_OUT_ClearIT(ENDP_2);
        USB30_OUT_Set(ENDP_2, ACK, 2);    //还能够接收2包
        USB30_Send_ERDY(ENDP_2 | OUT, 2); //通知主机下发2包
    }
    else if(nump == 3)
    {
        USB30_OUT_ClearIT(ENDP_2);
        USB30_OUT_Set(ENDP_2, ACK, 3);    //还能够接收3包
        USB30_Send_ERDY(ENDP_2 | OUT, 3); //通知主机下发3包
    }
}
/*******************************************************************************
 * @fn      EP3_OUT_Callback
 *
 * @brief   USB3.0 endpoint3 out callback.
 *
 * @return   None
 */
void EP3_OUT_Callback(void)
{
    UINT16 rx_len, i;
    UINT8  nump;
    UINT8  status;
    USB30_OUT_Status(ENDP_3, &nump, &rx_len, &status); //获取接收到的包数量
    if(nump == 0)
    {
        USB30_OUT_ClearIT(ENDP_3);                                         //清除端点所有状态 仅保留包序列
        USBSS->UEP3_RX_DMA = (UINT32)(UINT8 *)endp3RTbuff;                 //突发模式下由于地址自动偏移 地址需重置
        USB30_IN_Set(ENDP_3, ENABLE, ACK, DEF_ENDP3_IN_BURST_LEVEL, 1024); //设置能够发送4包
        USB30_Send_ERDY(ENDP_3 | IN, DEF_ENDP3_IN_BURST_LEVEL);            //通知主机取4包
    }
    else if(nump == 1)
    {
        USB30_OUT_ClearIT(ENDP_3);
        USB30_OUT_Set(ENDP_3, ACK, 1);    //能够接收一包
        USB30_Send_ERDY(ENDP_3 | OUT, 1); //通知主机下发一包
    }
    else if(nump == 2)
    {
        USB30_OUT_ClearIT(ENDP_3);
        USB30_OUT_Set(ENDP_3, ACK, 2);    //能够接收一包
        USB30_Send_ERDY(ENDP_3 | OUT, 2); //通知主机下发一包
    }
    else if(nump == 3)
    {
        USB30_OUT_ClearIT(ENDP_3);
        USB30_OUT_Set(ENDP_3, ACK, 3);    //能够接收一包
        USB30_Send_ERDY(ENDP_3 | OUT, 3); //通知主机下发一包
    }
}
/*******************************************************************************
 * @fn      EP4_OUT_Callback
 *
 * @brief   USB3.0 endpoint4 out callback.
 *
 * @return   None
 */
void EP4_OUT_Callback(void)
{
    UINT16 rx_len, i;
    UINT8  nump;
    UINT8  status;
    USB30_OUT_Status(ENDP_4, &nump, &rx_len, &status); //获取接收到的包数量
    if(nump == 0)
    {
        USB30_OUT_ClearIT(ENDP_4);                                         //清除端点所有状态 仅保留包序列
        USBSS->UEP4_RX_DMA = (UINT32)(UINT8 *)endp4RTbuff;                 //突发模式下由于地址自动偏移 地址需重置
        USB30_IN_Set(ENDP_4, ENABLE, ACK, DEF_ENDP4_IN_BURST_LEVEL, 1024); //能够发送4包
        USB30_Send_ERDY(ENDP_4 | IN, DEF_ENDP4_IN_BURST_LEVEL);
    }
    else if(nump == 1)
    {
        USB30_OUT_ClearIT(ENDP_4);
        USB30_OUT_Set(ENDP_4, ACK, 1);    //能够接收一包
        USB30_Send_ERDY(ENDP_4 | OUT, 1); //通知主机下发一包
    }
    else if(nump == 2)
    {
        USB30_OUT_ClearIT(ENDP_4);
        USB30_OUT_Set(ENDP_4, ACK, 2);    //能够接收2包
        USB30_Send_ERDY(ENDP_4 | OUT, 2); //通知主机下发2包
    }
    else if(nump == 3)
    {
        USB30_OUT_ClearIT(ENDP_4);
        USB30_OUT_Set(ENDP_4, ACK, 3);    //能够接收3包
        USB30_Send_ERDY(ENDP_4 | OUT, 3); //通知主机下发3包
    }
}
/*******************************************************************************
 * @fn      EP5_OUT_Callback
 *
 * @brief   USB3.0 endpoint5 out callback.
 *
 * @return   None
 */
void EP5_OUT_Callback(void)
{
    UINT16 rx_len, i;
    UINT8  nump;
    UINT8  status;
    USB30_OUT_Status(ENDP_5, &nump, &rx_len, &status); //获取接收到的包数量
    if(nump == 0)
    {
        USB30_OUT_ClearIT(ENDP_5);                                         //清除端点所有状态 仅保留包序列
        USBSS->UEP5_RX_DMA = (UINT32)(UINT8 *)endp5RTbuff;                 //突发模式下由于地址自动偏移 地址需重置
        USB30_IN_Set(ENDP_5, ENABLE, ACK, DEF_ENDP5_IN_BURST_LEVEL, 1024); //能够发送4包
        USB30_Send_ERDY(ENDP_5 | IN, DEF_ENDP5_IN_BURST_LEVEL);
    }
    else if(nump == 1)
    {
        USB30_OUT_ClearIT(ENDP_5);
        USB30_OUT_Set(ENDP_5, ACK, 1);    //能够接收一包
        USB30_Send_ERDY(ENDP_5 | OUT, 1); //通知主机下发一包
    }
    else if(nump == 2)
    {
        USB30_OUT_ClearIT(ENDP_5);
        USB30_OUT_Set(ENDP_5, ACK, 2);    //能够接收一包
        USB30_Send_ERDY(ENDP_5 | OUT, 2); //通知主机下发一包
    }
    else if(nump == 3)
    {
        USB30_OUT_ClearIT(ENDP_5);
        USB30_OUT_Set(ENDP_5, ACK, 3);    //能够接收一包
        USB30_Send_ERDY(ENDP_5 | OUT, 3); //通知主机下发一包
    }
}
/*******************************************************************************
 * @fn      EP6_OUT_Callback
 *
 * @brief   USB3.0 endpoint6 out callback.
 *
 * @return   None
 */
void EP6_OUT_Callback(void)
{
    UINT16 rx_len, i;
    UINT8  nump;
    UINT8  status;
    USB30_OUT_Status(ENDP_6, &nump, &rx_len, &status); //获取接收到的包数量
    if(nump == 0)
    {
        USB30_OUT_ClearIT(ENDP_6);                                         //清除端点所有状态 仅保留包序列
        USBSS->UEP6_RX_DMA = (UINT32)(UINT8 *)endp6RTbuff;                 //突发模式下由于地址自动偏移 地址需重置
        USB30_IN_Set(ENDP_6, ENABLE, ACK, DEF_ENDP6_IN_BURST_LEVEL, 1024); //能够发送4包
        USB30_Send_ERDY(ENDP_6 | IN, DEF_ENDP6_IN_BURST_LEVEL);
    }
    else if(nump == 1)
    {
        USB30_OUT_ClearIT(ENDP_6);
        USB30_OUT_Set(ENDP_6, ACK, 1);    //能够接收一包
        USB30_Send_ERDY(ENDP_6 | OUT, 1); //通知主机下发一包
    }
    else if(nump == 2)
    {
        USB30_OUT_ClearIT(ENDP_6);
        USB30_OUT_Set(ENDP_6, ACK, 2);    //能够接收2包
        USB30_Send_ERDY(ENDP_6 | OUT, 2); //通知主机下发2包
    }
    else if(nump == 3)
    {
        USB30_OUT_ClearIT(ENDP_6);
        USB30_OUT_Set(ENDP_6, ACK, 3);    //能够接收3包
        USB30_Send_ERDY(ENDP_6 | OUT, 3); //通知主机下发3包
    }
}
/*******************************************************************************
 * @fn      EP7_OUT_Callback
 *
 * @brief   USB3.0 endpoint7 out callback.
 *
 * @return   None
 */
void EP7_OUT_Callback(void)
{
    UINT16 rx_len, i;
    UINT8  nump;
    UINT8  status;
    USB30_OUT_Status(ENDP_7, &nump, &rx_len, &status); //获取接收到的包数量
    if(nump == 0)
    {
        USB30_OUT_ClearIT(ENDP_7);                                         //清除端点所有状态 仅保留包序列
        USBSS->UEP7_RX_DMA = (UINT32)(UINT8 *)endp7RTbuff;                 //突发模式下由于地址自动偏移 地址需重置
        USB30_IN_Set(ENDP_7, ENABLE, ACK, DEF_ENDP7_IN_BURST_LEVEL, 1024); //能够发送4包
        USB30_Send_ERDY(ENDP_7 | IN, DEF_ENDP7_IN_BURST_LEVEL);
    }
    else if(nump == 1)
    {
        USB30_OUT_ClearIT(ENDP_7);
        USB30_OUT_Set(ENDP_7, ACK, 1);    //能够接收一包
        USB30_Send_ERDY(ENDP_7 | OUT, 1); //通知主机下发一包
    }
    else if(nump == 2)
    {
        USB30_OUT_ClearIT(ENDP_7);
        USB30_OUT_Set(ENDP_7, ACK, 2);    //能够接收2包
        USB30_Send_ERDY(ENDP_7 | OUT, 2); //通知主机下发2包
    }
    else if(nump == 3)
    {
        USB30_OUT_ClearIT(ENDP_7);
        USB30_OUT_Set(ENDP_7, ACK, 3);    //能够接收3包
        USB30_Send_ERDY(ENDP_7 | OUT, 3); //通知主机下发3包
    }
}

/*******************************************************************************
 * @fn      USB30_ITP_Callback
 *
 * @brief   USB3.0 ITP回调函数
 *
 * @return   None
 */
void USB30_ITP_Callback(UINT32 ITPCounter)
{
}
