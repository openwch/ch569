/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30h_lib.h
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef HOST_CH56X_USB30H_LIB_H_
#define HOST_CH56X_USB30H_LIB_H_

#include "CH56x_common.h"


// link CFG
#define TERM_EN                 (1<<1)
#define PIPE_RESET              (1<<3)
#define LFPS_RX_PD              (1<<5)
#define CFG_EQ_EN               (1<<6)
#define DEEMPH_CFG              (1<<8)

#define POWER_MODE_0            0
#define POWER_MODE_1            1
#define POWER_MODE_2            2
#define POWER_MODE_3            3

#define LINK_PRESENT            (1<<0)
#define RX_WARM_RESET           ((UINT32)1<<1)

#define GO_DISABLED             (1<<4)
#define LINK_TXEQ               (1<<6)
#define GO_RX_DET               (1<<7)

#define POLLING_EN              (1<<12)

#define TX_HOT_RESET            ((UINT32)1<<16)
#define RX_HOT_RESET            ((UINT32)1<<24)

#define TX_WARM_RESET           ((UINT32)1<<8)
#define TX_Ux_EXIT              ((UINT32)1<<9)
// link int flag
#define LINK_RDY_FLAG           (1<<0)
#define LINK_RECOV_FLAG         (1<<1)
#define LINK_INACT_FLAG         (1<<2)
#define LINK_DISABLE_FLAG       (1<<3)
#define LINK_GO_U3_FLAG         (1<<4)
#define LINK_GO_U2_FLAG         (1<<5)
#define LINK_GO_U1_FLAG         (1<<6)
#define LINK_GO_U0_FLAG         (1<<7)
#define LINK_U3_WAKE_FLAG       (1<<8)
#define LINK_Ux_REJECT_FLAG     (1<<9)
#define TERM_PRESENT_FLAG       (1<<10)
#define LINK_TXEQ_FLAG          (1<<11)
#define LINK_Ux_EXIT_FLAG       (1<<12)
#define WARM_RESET_FLAG         (1<<13)
#define U3_WAKEUP_FLAG          (1<<14)
#define HOT_RESET_FLAG          (1<<15)
#define LINK_RX_DET_FLAG        (1<<20)

// LMP
#define LMP_HP                  0
#define LMP_SUBTYPE_MASK        (0xf<<5)
#define SET_LINK_FUNC           (0x1<<5)
#define U2_INACT_TOUT           (0x2<<5)
#define VENDOR_TEST             (0x3<<5)
#define PORT_CAP                (0x4<<5)
#define PORT_CFG                (0x5<<5)
#define PORT_CFG_RES            (0x6<<5)

#define LINK_SPEED              (1<<9)

#define NUM_HP_BUF              (4<<0)
#define DOWN_STREAM             (1<<16)
#define UP_STREAM               (2<<16)
#define TIE_BRK                 (1<<20)

// link define
#define DOWN_FLAG       (1<<0)
#define TERM_EN         (1<<1)
#define DEVICE_PHY      (1<<2)
#define PIPE_RESET      (1<<3)
#define COMPLIANCE_EN   (1<<4)
#define LFPS_RX_PD      (1<<5)
#define CFG_EQ_EN       (1<<6)
#define TX_SWING        (1<<7)
#define DEEMPH_CFG      (1<<8)
#define LPM_EN          (1<<12)
#define U1_ALLOW        (1<<16)
#define U2_ALLOW        (1<<17)
#define HP_PENDING      (2<<19)

#define LINK_PRESENT    (1<<0)
#define RX_WARM_RESET   ((UINT32)1<<1)
#define LINK_BUSY       (1<<2)
#define LINK_READY      (1<<3)

#define LINK_RX_DETECT  (1<<7)

// link int en
#define LINK_RDY_IE             (1<<0)
#define LINK_RECOV_IE           (1<<1)
#define LINK_INACT_IE           (1<<2)
#define LINK_DISABLE_IE         (1<<3)
#define LINK_GO_U3_IE           (1<<4)
#define LINK_GO_U2_IE           (1<<5)
#define LINK_GO_U1_IE           (1<<6)
#define LINK_GO_U0_IE           (1<<7)
#define LINK_U3_WAKE_IE         (1<<8)
#define LINK_Ux_REJECT_IE       (1<<9)
#define TERM_PRESENT_IE         (1<<10)
#define LINK_TXEQ_IE            (1<<11)
#define LINK_Ux_EXIT_IE         (1<<12)
#define WARM_RESET_IE           (1<<13)
#define U3_WAKEUP_IE            (1<<14)
#define HOT_RESET_IE            (1<<15)
#define LINK_RX_DET_IE          (1<<20)
#define LINK_RX_DET_FLAG        (1<<20)

//// USB CONTROL
//#define DMA_EN              (1<<0)
//#define USB_ALL_CLR         (1<<1)
//#define USB_FORCE_RST       (1<<2)
//#define INT_BUSY_EN         (1<<3)
//#define DIR_ABORT           (1<<4)
//#define SETUP_FLOW_EN       (1<<5)    // if enable, data stage/status stage will begin after send ERDY
//#define ITP_EN              (1<<6)
//#define HOST_MODE           (1<<7)
//
//#define USB_ACT_FLAG        0x01
//#define USB_LMP_RX_FLAG     0x02
//#define USB_LMP_TX_FLAG     0x04
//#define USB_ITP_FLAG        0x08
//#define USB_OV_FLAG         0x10
//#define USB_ERDY_FLAG       0x20

// USB CONTROL
#define DMA_EN              1<<0
#define USB_ALL_CLR         1<<1
#define USB_FORCE_RST       1<<2
#define INT_BUSY_EN         1<<3
#define DIR_ABORT           1<<4
#define SETUP_FLOW_EN       1<<5    // if enable, data stage/status stage will begin after send ERDY
#define ITP_EN              1<<6
#define HOST_MODE           1<<7

#define USB_ACT_IE          0x01<<16
#define USB_LMP_RX_IE       0x02<<16
#define USB_LMP_TX_IE       0x04<<16
#define USB_ITP_IE          0x08<<16
#define USB_OV_IE           0x10<<16
#define USB_ERDY_IE         0x20<<16

#define USB_ACT_FLAG        0x01
#define USB_LMP_RX_FLAG     0x02
#define USB_LMP_TX_FLAG     0x04
#define USB_ITP_FLAG        0x08
#define USB_OV_FLAG         0x10
#define USB_ERDY_FLAG       0x20

#define USB_INT_EP_MASK     0x7<<8
#define USB_INT_RES_MASK    0x3<<14
#define USB_RES_ACK         0x0<<14
#define USB_RES_ERDY        0x1<<14
#define USB_RES_NRDY        0x2<<14
#define USB_RES_STALL       0x3<<14

#define USB_SEQ_MASK        0x1f<<21

// #define USB_INT_ST_MASK  0xf<<16
// #define USB_SEQ_MATCH        0x1<<16

#define USB_TX_LPF          0x1<<28

#define USB_STATUS_IS       0x1<<29
#define USB_SETUP_IS        0x1<<30
#define USB_TX_DIR          0x1<<12


#define USB_ERDY_ENDP_MASK  0xf<<4
#define USB_ERDY_NUMP_MASK  0x1f<<8
// #define USB_ITP_PREAGE       0x1<<31


#define EP_INT_FLAG         1<<31

#define ERDY_NUM_MASK       0x1f<<24

#define EP_RES_MASK         0x3<<26
#define EP_RES_NRDY         0x0<<26
#define EP_RES_ACK          0x1<<26
#define EP_RES_STALL        0x2<<26

#define USB_NUMP_MASK       0x1f<<16
#define USB_NUMP_1          0x1<<16
#define USB_NUMP_2          0x2<<16
#define USB_NUMP_3          0x3<<16
#define USB_NUMP_4          0x4<<16
#define USB_NUMP_4          0x4<<16
#define USB_NUMP_5          0x5<<16
#define USB_NUMP_6          0x6<<16

#define EP0_R_EN            1<<0
#define EP1_R_EN            1<<1
#define EP2_R_EN            1<<2
#define EP3_R_EN            1<<3
#define EP4_R_EN            1<<4
#define EP5_R_EN            1<<5
#define EP6_R_EN            1<<6
#define EP7_R_EN            1<<7

#define EP0_T_EN            1<<8
#define EP1_T_EN            1<<9
#define EP2_T_EN            1<<10
#define EP3_T_EN            1<<11
#define EP4_T_EN            1<<12
#define EP5_T_EN            1<<13
#define EP6_T_EN            1<<14
#define EP7_T_EN            1<<15

#define EP1_R_ISO           1<<17
#define EP2_R_ISO           1<<18
#define EP3_R_ISO           1<<19
#define EP4_R_ISO           1<<20
#define EP5_R_ISO           1<<21
#define EP6_R_ISO           1<<22
#define EP7_R_ISO           1<<23

#define EP1_T_ISO           1<<25
#define EP2_T_ISO           1<<26
#define EP3_T_ISO           1<<27
#define EP4_T_ISO           1<<28
#define EP5_T_ISO           1<<29
#define EP6_T_ISO           1<<30
#define EP7_T_ISO           1<<31

#define TX_ERDY_ACT         1<<0
#define TX_ERDY_DIR         1<<1
#define TX_ERDY_ENDP_0      0<<2
#define TX_ERDY_ENDP_1      1<<2
#define TX_ERDY_ENDP_2      2<<2
#define TX_ERDY_ENDP_3      3<<2
#define TX_ERDY_ENDP_4      4<<2
#define TX_ERDY_ENDP_5      5<<2
#define TX_ERDY_ENDP_6      6<<2
#define TX_ERDY_ENDP_7      7<<2

#define TX_ERDY_NUMP_0      0<<6
#define TX_ERDY_NUMP_1      1<<6
#define TX_ERDY_NUMP_2      2<<6
#define TX_ERDY_NUMP_3      3<<6
#define TX_ERDY_NUMP_4      4<<6
#define TX_ERDY_NUMP_5      5<<6
#define TX_ERDY_NUMP_6      6<<6
#define TX_ERDY_NUMP_7      7<<6

// host
#define UH_T_EN             (1<<1)
#define UH_R_EN             (1<<9)


#define UH_R_ISO            1<<17
#define UH_T_ISO            1<<25

#define UH_TX_SETUP         0x1<<30
#define UH_TX_STATUS        0x1<<29
#define UH_TX_LPF           0x1<<28

#define UH_RTX_VALID        (0x1<<26)

#define UH_RX_EOB           0x1<<16
#define UH_ISO_RX_ERR       0x1<<17

/*******************************************************************************
 * @fn        USB30HOST_Init
 *
 * @brief     USB3.0 host initialized
 *
 * @param     sta - ENABLE    DISABLE
 *            endpTXbuff - Host send buffer
 *            endpRXbuff - Host rceive buffer
 *
 * @return    None
 */
extern void USB30HOST_Init (FunctionalState sta,PUINT8 endpTXbuff , PUINT8 endpRXbuff);
/*******************************************************************************
 * @fn     USB30H_Lmp_Init
 *
 * @brief  USB30 lmp initialization
 *
 * @return   None
 */
extern void USB30H_Lmp_Init(void);

/*******************************************************************************
 * @fn    USB30H_Send_Setup
 *
 * @brief  send SETUP
 *
 * @param  SETUP packet length
 *
 * @return  0 - success
 *          1 - timeout
 */
extern UINT8 USB30H_Send_Setup( UINT32 tx_len );

/*******************************************************************************
 * @fn     USB30H_Send_Status
 *
 * @brief  Control transmission status phase status package
 *
 * @param  tx_len - send length
 *
 * @return   None
 */
extern UINT8 USB30H_Send_Status(void);

/*******************************************************************************
 * @fn     USB30H_Erdy_Status
 *
 * @brief  Control transmission status phase status package
 *
 * @param  nump - Return the number of packages ready for transmission
 *                endp  - Endpoint number of data transmission preparation
 *                endp_num -  Endpoint number
 *                tx_len -  packet length
 *
 * @return   1 - received ERDY       0 - Not received ERDY
 */
extern UINT8 USB30H_Erdy_Status( PUINT8 nump, PUINT8 endp  );

/*******************************************************************************
 * @fn     USB30H_IN_Data
 *
 * @brief  Get data from device endpoint
 *
 * @param  seq_num  - Packet sequence
 *         packet_num  - Number of packages
 *         endp_num - Endpoint number
 *         tx_len   - Package length
 *
 * @return   bit0~15£º
 *                  bit15 - EOB/LPF
 *                  bit12~13 -    1-ACK   2-NRDY   3-STALL   4-error
 *                  bit0~11 -  packet length
 */
extern UINT16 USB30H_IN_Data( UINT8 seq_num, PUINT8 packet_num, UINT8 endp_num );

/*******************************************************************************
 * @fn     USB30H_OUT_Data
 *
 * @brief  Send data to device endpoint
 *
 * @param  seq_num  - Packet sequence
 *         packet_num  - Number of packages
 *         endp_num - Endpoint number
 *         tx_len   - Package length
 *
 * @return   status 1-ACK   2-NRDY   3-STALL   4-error
 */
extern UINT8 USB30H_OUT_Data( UINT8 seq_num, UINT8 packet_num, UINT8 endp_num, UINT32 tx_len );

/*******************************************************************************
 * @fn    USB30H_Set_Address
 *
 * @brief  Set the destination address sent by the host
 *
 * @param   address - address
 *
 * @return   None
 */
extern void USB30H_Set_Address( UINT32 address );

/*******************************************************************************
 * @fn    USB30H_Switch_Powermode
 *
 * @brief  Switch USBSS power mode
 *
 * @param  pwr_mode - power mode
 *
 * @return   None
 */
extern void USB30H_Switch_Powermode( UINT32 pwr_mode );


#endif
