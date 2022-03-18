/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30h_lib.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/12/31
* Description        : USB3.0
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
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

// USB CONTROL
#define DMA_EN              (1<<0)
#define USB_ALL_CLR         (1<<1)
#define USB_FORCE_RST       (1<<2)
#define INT_BUSY_EN         (1<<3)
#define DIR_ABORT           (1<<4)
#define SETUP_FLOW_EN       (1<<5)    // if enable, data stage/status stage will begin after send ERDY
#define ITP_EN              (1<<6)
#define HOST_MODE           (1<<7)

#define USB_ACT_FLAG        0x01
#define USB_LMP_RX_FLAG     0x02
#define USB_LMP_TX_FLAG     0x04
#define USB_ITP_FLAG        0x08
#define USB_OV_FLAG         0x10
#define USB_ERDY_FLAG       0x20

// host
#define UH_T_EN             (1<<1)
#define UH_R_EN             (1<<9)

/*******************************************************************************
 * @fn     USB30H_Lmp_Init
 *
 * @brief  USB30初始化
 *
 * @return   None
 */
extern void USB30H_Lmp_Init(void);

/*******************************************************************************
 * @fn    USB30H_Send_Setup
 *
 * @brief  发送SETUP包
 *
 * @param  SETUP包长度
 *
 * @return   None
 */
extern void USB30H_Send_Setup( UINT32 tx_len );

/*******************************************************************************
 * @fn     USB30H_Send_Status
 *
 * @brief  控制传输状态阶段状态包
 *
 * @param  tx_len - 发送长度
 *
 * @return   None
 */
extern void USB30H_Send_Status(void);

/*******************************************************************************
 * @fn     USB30H_Erdy_Status
 *
 * @brief  控制传输状态阶段状态包
 *
 * @param  nump - 返回设备准备传输包数量
 *                endp  - 数据传输准备完成端点号
 *                endp_num - 端点号
 *                tx_len - 包长度
 *
 * @return   1 - 收到ERDY       0 - 未收到ERDY
 */
extern UINT8 USB30H_Erdy_Status( PUINT8 nump, PUINT8 endp  );

/*******************************************************************************
 * @fn     USB30H_IN_Data
 *
 * @brief  从设备端点获取数据
 *
 * @param  seq_num - 包序列
 *                packet_num  - 包数量
 *                endp_num - 端点号
 *                tx_len - 包长度
 *
 * @return   bit0~15定义：
 *                  bit15 - EOB/LPF标志
 *                  bit12~13 - 应答状态   1-ACK   2-NRDY   3-STALL   4-错误不应该出现
 *                  bit0~11 - 传输包长度（突发对应最后一包的长度）
 */
extern UINT16 USB30H_IN_Data( UINT8 seq_num, PUINT8 packet_num, UINT8 endp_num );

/*******************************************************************************
 * @fn     USB30H_OUT_Data
 *
 * @brief  向设备端点发送数据
 *
 * @param  seq_num - 包序列
 *                packet_num  - 包数量
 *                endp_num - 端点号
 *                tx_len - 包长度
 *
 * @return   返回设备应答状态1-ACK   2-NRDY   3-STALL   4-错误不应该出现
 */
extern UINT8 USB30H_OUT_Data( UINT8 seq_num, UINT8 packet_num, UINT8 endp_num, UINT32 tx_len );

/*******************************************************************************
 * @fn    USB30H_Set_Address
 *
 * @brief  设置主机发送的目标地址
 *
 * @param   address - 地址
 *
 * @return   None
 */
extern void USB30H_Set_Address( UINT32 address );

/*******************************************************************************
 * @fn    USB30H_Switch_Powermode
 *
 * @brief  切换USBSS电源模式
 *
 * @param  pwr_mode - 电源模式
 *
 * @return   None
 */
extern void USB30H_Switch_Powermode( UINT32 pwr_mode );


#endif
