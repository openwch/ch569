/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56X_UDISK.h
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "CH56x_common.h"
#include "CH56xusb30h_LIB.h"
#include "CH56x_usb30h.h"
#include "CH56x_host_hs.h"
/******************************************************************************/

#define USB_BO_CBW_SIG			  0x43425355									/* Command block CBW identification flag'USBC' */
#define USB_BO_CSW_SIG			  0x53425355									/* Command block CSW identification flag'USBC' */

#define USB_BO_CBW_SIZE			  0x1F											/* Total length of command block CBW */
#define USB_BO_CSW_SIZE			  0x0D											/* Total length of command status block CSW */
#define USB_BO_DATA_IN			  0x80
#define USB_BO_DATA_OUT			  0x00


typedef union _BULK_ONLY_CMD
{
	struct
	{
		UINT32 mCBW_Sig;
		UINT32 mCBW_Tag;
		UINT32 mCBW_DataLen;													/* Input: data transmission length */
		UINT8  mCBW_Flag;														/* Input: transmission direction and other signs */
		UINT8  mCBW_LUN;
		UINT8  mCBW_CB_Len;														/* Input: length of command block, valid values are 1 to 16 */
		UINT8  mCBW_CB_Buf[16];													/* Input: Command block, the buffer is up to 16 bytes */
	} mCBW;																		/* Command block of BulkOnly protocol, input CBW structure */
	struct
	{
		UINT32 mCSW_Sig;
		UINT32 mCSW_Tag;
		UINT32 mCSW_Residue;													/* Return: Remaining data length */
		UINT8  mCSW_Status;														/* Return: command execution result status */
	} mCSW;																		/* Command status block of BulkOnly protocol, output CSW structure */
} BULK_ONLY_CMD;


/******************************************************************************/
extern UINT8  gDiskMaxLun;                                                             /* Maximum logical unit number of disk */
extern UINT8  gDiskCurLun;                                                             /* Current operating logical unit number of the disk */
extern UINT32 gDiskCapability;                                                         /* Total disk capacity */
extern UINT32 gDiskPerSecSize;                                                         /* Disk sector size */
extern UINT8  gDiskBulkInEp;                                                           /* IN endpoint address of USB mass storage device */
extern UINT8  gDiskBulkInTog;                                                          /* USB mass transfer synchronization flag : 0-31 */
extern UINT8  gDiskBulkOutEp;                                                          /* OUT endpoint address of USB mass storage device */
extern UINT8  gDiskBulkOutTog;                                                         /* USB mass transfer synchronization flag : 0-31 */
extern UINT16 gDiskBulkInEpSize;                                                       /* Maximum packet size of IN endpoint of USB mass storage device */
extern UINT16 gDiskBulkOutEpSize;                                                      /* The maximum packet size of the OUT endpoint of the USB mass storage device */
extern UINT8  gDiskInterfNumber;                                                       /* Interface number of USB mass storage device */
extern UINT8V gDeviceConnectstatus;                                                    /* USB connection status */
extern UINT8  gDeviceUsbType;                                                          /* 01--USB2.0&1.1  02--USB3.0*/

#define	DEFAULT_MAX_OPERATE_SIZE      	8*1024					/***Default maximum packet size for current operation***/
#define	MAX_DATA_ADDR	0x20020000
extern UINT8V U30_TIME_OUT;
extern UINT8V tx_lmp_port;
extern UINT8V Hot_ret_flag;
extern __attribute__ ((aligned(16))) UINT8 pNTFS_BUF[512] __attribute__((section(".DMADATA")));
extern UINT8 gUdisk_flag;
extern UINT8 gUdisk_flag1;
extern UINT16 gUdisk_delay;

extern UINT8 MS_U30HOST_CofDescrAnalyse( UINT8 *pbuf );
extern UINT8 MS_Init(  UINT8 *pbuf );
extern UINT8 MS_ReadSector( UINT32 StartLba, UINT16 SectCount, PUINT8 DataBuf );
extern UINT8 MS_WriteSector( UINT32 StartLba, UINT8 SectCount, PUINT8 DataBuf );


extern UINT8 CHRV3BulkOnlyCmd( UINT8 *DataBuf );
extern UINT8 MS_U20HOST_BulkOutHandle( UINT8 *pDatBuf, UINT32 *pSize );
extern UINT8 MS_U20HOST_BulkInHandle( UINT8 *pDatBuf, UINT32 *pSize );
extern UINT8 MS_U20HOST_CofDescrAnalyse( UINT8 *pbuf );
extern UINT8 Hot_Reset( UINT8 *pdata );



