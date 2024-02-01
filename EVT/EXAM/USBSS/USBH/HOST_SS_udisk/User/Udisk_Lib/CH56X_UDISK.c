/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56X_UDISK.c
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "string.h"
#include "stdio.h"
#include "CH56x_common.h"
#include "CH56xusb30h_LIB.h"
#include "CH56x_usb30h.h"
#include "CH56x_host_hs.h"
#include "CH56X_UDISK.h"
#include "CHRV3UFI.h"


/* Global Define */
#define U30_MAX_PACKSIZE        1024
#define  TIME_OUT_VALUE         1000

/* Global Variable */
UINT8  gDiskMaxLun;				    											/* Maximum logical unit number of disk */
UINT8  gDiskCurLun;	    														/* Current operating logical unit number of the disk */
UINT32 gDiskCapability;		    												/* Total disk capacity */
UINT32 gDiskPerSecSize;	    													/* Disk sector size */
UINT8  gDiskBulkInEp;															/* IN endpoint address of USB mass storage device */
UINT8  gDiskBulkInTog;															/* USB mass transfer synchronization flag : 0-31 */
UINT8  gDiskBulkOutEp;															/* OUT endpoint address of USB mass storage device */
UINT8  gDiskBulkOutTog;															/* USB mass transfer synchronization flag : 0-31 */
UINT16 gDiskBulkInEpSize;  	    												/* Maximum packet size of IN endpoint of USB mass storage device */
UINT16 gDiskBulkOutEpSize;  													/* The maximum packet size of the OUT endpoint of the USB mass storage device */
UINT8  gDiskInterfNumber;														/* Interface number of USB mass storage device */
UINT8V gDeviceConnectstatus;                                                    /* USB connection status */
UINT8  gDeviceUsbType = 0;                                                      /* 01--USB2.0&1.1  02--USB3.0*/
UINT8  gUdisk_flag = 0;
UINT8V Hot_ret_flag;
UINT16 U20_ENDP_SIZE;

__attribute__ ((aligned(16))) BULK_ONLY_CMD   mBOC  __attribute__((section(".DMADATA")));

typedef struct _USB_ENDPOINT_DESCRIPTOR_U30 /*Endpoint descriptor*/
{
    UINT8  bLength;
    UINT8  bDescriptorType;
    UINT8  bEndpointAddress;
    UINT8  bmAttributes;
    UINT8  wMaxPacketSizeL;
    UINT8  wMaxPacketSizeH;
    UINT8  bInterval;

    UINT8  bLength1;                //3.0 EndpointCompanion descriptor
    UINT8  bDescriptorType1;
    UINT8  bMaxBurst1;
    UINT8  bmAttributes1;
    UINT8  wBytesPerInterval_L;
    UINT8  wBytesPerInterval_H;
}USB_ENDP_DESCR_U30, *PUSB_ENDP_DESCR_U30;

typedef struct _USB_CONFIG_DESCRIPTOR_LONG_U30
{
    USB_CFG_DESCR  cfg_descr;
    USB_ITF_DESCR  itf_descr;
    USB_ENDP_DESCR_U30 endp_descr[2];
}USB_CFG_DESCR_LONG_U30, *PUSB_CFG_DESCR_LONG_U30;


UINT8 MS_Init_Hotrst(  UINT8 *pbuf );
/*******************************************************************************
* Function Name  : MS_GetMaxLun
* Description    : USBThe host obtains the maximum logical unit number
* Input          : NULL
* Output         : NULL
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 MS_GetMaxLun( void )
{
	UINT8  status = 0;
	UINT8  buf[ 1 ];
	UINT8  setup_buf[8];
	UINT16 s;
	/* Populate the SETUP command package */
	setup_buf[0] = 0xA1;
	setup_buf[1] = 0xFE;
	setup_buf[2] = 0x00;
	setup_buf[3] = 0x00;
	setup_buf[4] = 0x00;
	setup_buf[5] = 0x00;
	setup_buf[6] = 0x01;
	setup_buf[7] = 0x00;

	gDiskMaxLun = 0x00;
    if( gDeviceUsbType == USB_U30_SPEED )
    {
        memcpy( endpTXbuff , setup_buf , 8);
        s = USB30H_Send_Setup( 8 );
        if( s )
        {
            return USB_CH56XUSBTIMEOUT;
        }
        s = USB30HOST_CtrlTransaciton(endpRXbuff);
        if( s == 0)
        {
            return USB_CH56XUSBTIMEOUT;
        }

        s = USB30H_Send_Status();
        if( s )
        {
            return USB_CH56XUSBTIMEOUT;
        }
        status = USB_INT_SUCCESS;
    }
    else if( gDeviceUsbType == USB_U20_SPEED )
    {
        status = USB20HOST_CtrlTransfer(setup_buf, buf, NULL );
    }

    if( status == USB_INT_SUCCESS )
    {

    }
    else if( status == USB_INT_DISK_ERR )
    {
        if( gDeviceUsbType == USB_U30_SPEED )
        {
            status = USB30HOST_ClearEndpStall(0x00);
        }
        else{
            status = USB20HOST_ClearEndpStall( 0x00 );                  /* claer endpoint0 */
        }
    }
    if( status == USB_INT_SUCCESS )return 0x00;
    return 0x00;
}


/*******************************************************************************
* Function Name  : MS_ResetErrorBOC
* Description    : USB host reset USB disk
* Input          : NULL
* Output         : NULL
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 MS_ResetErrorBOC( void )
{
	UINT8  status;
	UINT8 setup_buf[8];
	setup_buf[0] = 0x21;
	setup_buf[1] = 0xFF;
	setup_buf[2] = 0x00;
	setup_buf[3] = 0x00;
	setup_buf[4] = gDiskInterfNumber;
	setup_buf[5] = 0x00;
	setup_buf[6] = 0x00;
	setup_buf[7] = 0x00;
	if( gDeviceUsbType == USB_U30_SPEED )
	{
	    USB30HOST_CtrlTransaciton(setup_buf);
	    status = USB_INT_SUCCESS;
	}
	else
	{
		status = USB20HOST_CtrlTransfer(  setup_buf, NULL, NULL  );
	}
	if( status == USB_INT_SUCCESS )
	{
		if( gDeviceUsbType == USB_U30_SPEED )
		{
            status = USB30HOST_ClearEndpStall( 0x80 | gDiskBulkInEp );

		}
		else
		{
			status = USB20HOST_ClearEndpStall( 0x80 | gDiskBulkInEp );
		}
		if( status != USB_INT_SUCCESS )
		{
			return( status );
		}
		if( gDeviceUsbType == USB_U30_SPEED )
		{
            status  =  USB30HOST_ClearEndpStall( gDiskBulkOutEp );

		}
		else
		{
			status = USB20HOST_ClearEndpStall( gDiskBulkOutEp );
		}
	}
	return( status );
}

/*******************************************************************************
* Function Name  : MS_U30HOST_CofDescrAnalyse
* Description    : USB Host analysis Mass Storage Device Configuration Descriptor
* Input          :
*                  *pbuf------Configuration descriptors to be analyzed
* Output         : None
* Return         : USB_OPERATE_SUCCESS----Mass Storage Device;
*                  USB_OPERATE_ERROR---other device
*******************************************************************************/
UINT8 MS_U30HOST_CofDescrAnalyse( UINT8 *pbuf )
{
	UINT16 i;

	/* Analyze Configuration Descriptor */
	if ( ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> itf_descr.bInterfaceClass != 0x08 )
	{
		return( USB_OPERATE_ERROR );  									 	    /* If it is not a USB storage device, return */
	}
	gDiskInterfNumber = ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> itf_descr.bInterfaceNumber;
	gDiskBulkInEp  = 0;
	gDiskBulkOutEp = 0;
	for( i = 0; i < 2; i ++ )   												/* Analyze only the first two endpoints */
	{
		if( ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> endp_descr[ i ].bLength == 0x07
			&& ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> endp_descr[ i ].bDescriptorType == 0x05
			&& ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> endp_descr[ i ].bmAttributes == 2 )
		{
			if( ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> endp_descr[ i ].bEndpointAddress & 0x80 )
			{
				/* Bulk IN endpoint */
				gDiskBulkInEp = ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> endp_descr[ i ].bEndpointAddress & 0x0F;   /* IN endpoint */
				gDiskBulkInEpSize = ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> endp_descr[ i ].wMaxPacketSizeH;
				gDiskBulkInEpSize = gDiskBulkInEpSize << 8;
				gDiskBulkInEpSize += ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> endp_descr[ i ].wMaxPacketSizeL;
				gDiskBulkInTog = 0;
			}
			else
			{
				gDiskBulkOutEp = ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> endp_descr[ i ].bEndpointAddress & 0x0F;  /* OUT endpoint */
				gDiskBulkOutEpSize = ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> endp_descr[ i ].wMaxPacketSizeH;
				gDiskBulkOutEpSize = gDiskBulkOutEpSize << 8;
				gDiskBulkOutEpSize += ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> endp_descr[ i ].wMaxPacketSizeL;
				gDiskBulkOutTog = 0;
			}
		}
	}
	if( ( (PUSB_CFG_DESCR_LONG_U30)pbuf ) -> itf_descr.bInterfaceClass != 0x08 || gDiskBulkInEp == 0 || gDiskBulkOutEp == 0 )/* Not a USB storage device, not supported */
	{
		return( USB_OPERATE_ERROR ); 	  									    /* Terminate the operation and return directly */
	}
	gDiskCurLun = 0x00;															/* Clear the logical unit number of the current operation */
	return( USB_OPERATE_SUCCESS );
}

/*******************************************************************************
* Function Name  : MS_U20HOST_CofDescrAnalyse
* Description    : USB Host analysis Mass Storage Device Configuration Descriptor
* Input          :
*                  *pbuf------Configuration descriptors to be analyzed
* Output         : None
* Return         : USB_OPERATE_SUCCESS----Mass Storage Device;
*                  USB_OPERATE_ERROR---other device
*******************************************************************************/
UINT8 MS_U20HOST_CofDescrAnalyse( UINT8 *pbuf )
{
	UINT16 i;

	/* Analyze Configuration Descriptor */
	if ( ( (PUSB_CFG_DESCR_LONG)pbuf ) -> itf_descr.bInterfaceClass != 0x08 )
	{
		return( USB_OPERATE_ERROR );
	}
	gDiskInterfNumber = ( (PUSB_CFG_DESCR_LONG)pbuf ) -> itf_descr.bInterfaceNumber;
	gDiskBulkInEp  = 0;
	gDiskBulkOutEp = 0;
	for( i = 0; i < 2; i ++ )
	{
		if( ( (PUSB_CFG_DESCR_LONG)pbuf ) -> endp_descr[ i ].bLength == 0x07
			&& ( (PUSB_CFG_DESCR_LONG)pbuf ) -> endp_descr[ i ].bDescriptorType == 0x05
			&& ( (PUSB_CFG_DESCR_LONG)pbuf ) -> endp_descr[ i ].bmAttributes == 2 )
		{
			if( ( (PUSB_CFG_DESCR_LONG)pbuf ) -> endp_descr[ i ].bEndpointAddress & 0x80 )
			{
				gDiskBulkInEp = ( (PUSB_CFG_DESCR_LONG)pbuf ) -> endp_descr[ i ].bEndpointAddress & 0x0F;
				gDiskBulkInEpSize = ( (PUSB_CFG_DESCR_LONG)pbuf ) -> endp_descr[ i ].wMaxPacketSizeH;
				gDiskBulkInEpSize = gDiskBulkInEpSize << 8;
				gDiskBulkInEpSize += ( (PUSB_CFG_DESCR_LONG)pbuf ) -> endp_descr[ i ].wMaxPacketSizeL;
				gDiskBulkInTog = 0;
			}
			else
			{
				gDiskBulkOutEp = ( (PUSB_CFG_DESCR_LONG)pbuf ) -> endp_descr[ i ].bEndpointAddress & 0x0F;
				gDiskBulkOutEpSize = ( (PUSB_CFG_DESCR_LONG)pbuf ) -> endp_descr[ i ].wMaxPacketSizeH;
				gDiskBulkOutEpSize = gDiskBulkOutEpSize << 8;
				gDiskBulkOutEpSize += ( (PUSB_CFG_DESCR_LONG)pbuf ) -> endp_descr[ i ].wMaxPacketSizeL;
				gDiskBulkOutTog = 0;
			}
		}
	}
	if( ( (PUSB_CFG_DESCR_LONG)pbuf ) -> itf_descr.bInterfaceClass != 0x08 || gDiskBulkInEp == 0 || gDiskBulkOutEp == 0 )
	{
		return( USB_OPERATE_ERROR );
	}

	U20_ENDP_SIZE = gDiskBulkOutEpSize;

	gDiskCurLun = 0x00;
	return( USB_OPERATE_SUCCESS );
}

/*******************************************************************************
* Function Name  : MS_U30HOST_BulkInHandle
* Description    : USB Host performs batch transfer processing
* Input          :
*                  *pDatBuf---Data buffer, buffer address must be 4-byte aligned
*				   *pSize-----Send and receive data size  maximum 1024 byte
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 MS_U30HOST_BulkInHandle( UINT8 *pDatBuf, UINT32 *pSize )
{
	UINT16 status = 0;
	UINT16 len,total_len;
	UINT8 packnum = 1;
	total_len = 0;

	while(1)
	{

	    packnum = 1;
		if( gDeviceConnectstatus == USB_INT_DISCONNECT )return USB_INT_DISCONNECT;
		if( *pSize >= U30_MAX_PACKSIZE )
		{
			len = U30_MAX_PACKSIZE;
		}
		else
		{
			len = *pSize;
		}
        USBSSH->UH_RX_DMA = (UINT32)(UINT8 *)endpRXbuff;
		len = USB30HOST_INTransaction(gDiskBulkInTog ,&packnum , gDiskBulkInEp , &status);
		memcpy(pDatBuf,endpRXbuff,len);
		if( status == 0x3000 )
		{
		    return USB_INT_DISK_ERR;
		}

		gDiskBulkInTog++;
        gDiskBulkInTog &= 0x1f;
		*pSize -= len;
		pDatBuf+=len;
		total_len+=len;
		if( (*pSize == 0) || (len<U30_MAX_PACKSIZE))
		{
			*pSize = total_len;
			break;
		}
	}
	if( gDeviceConnectstatus == USB_INT_DISCONNECT )return USB_INT_DISCONNECT;
	return USB_INT_SUCCESS;
}

/*******************************************************************************
* Function Name  : MS_U30HOST_BulkOutHandle
* Description    : USB Host performs batch transfer processing
* Input          :
*                  *pDatBuf---Data buffer, buffer address must be 4-byte aligned
*				   *pSize-----Send and receive data size  maximum 1024 byte
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 MS_U30HOST_BulkOutHandle( UINT8 *pDatBuf, UINT32 *pSize )
{
	UINT16 len;
	UINT8* p;
	UINT8 count = 0;
    UINT32 timeoutcount = 0;
	while(1)
	{
		if( gDeviceConnectstatus == USB_INT_DISCONNECT )return USB_INT_DISCONNECT;

		if( *pSize >= U30_MAX_PACKSIZE )
		{
			len = U30_MAX_PACKSIZE;
		}
		else
		{
			len = *pSize;
		}

		p = (UINT8 *)endpTXbuff;
		memcpy(p , pDatBuf , *pSize);

		do
		{
		    count = USB30HOST_OUTTransaction(gDiskBulkOutTog , 1 , gDiskBulkOutEp,len);
            if( timeoutcount >= 10000)
            {
                return USB_CH56XUSBTIMEOUT;
            }
            timeoutcount ++;
        }
		while (count);


		*pSize -= len;
		pDatBuf+=len;
		gDiskBulkOutTog++;
		if( *pSize == 0 )break;

	}
	return USB_INT_SUCCESS;
}

/*******************************************************************************
* Function Name  : Hot_Reset
* Description    : None
* Input          :
*                  *pdata---data buffer
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 Hot_Reset( UINT8 *pdata )
{
    UINT8 s,count;
    UINT32 time_count = 0;
    count = 0;
    if( gUdisk_flag )
    {
hot_reset_ag:
        printf("hot_reset\n");
        Hot_ret_flag = 1;

        USB30H_Switch_Powermode(POWER_MODE_2);
        USBSSH->LINK_CTRL |= (1<<8) ;
        mDelaymS( 80 );
        USBSSH->LINK_CTRL &= ~(1<<8) ;
        tx_lmp_port = 0;
        while( Hot_ret_flag == 1 )
        {
            if( gDeviceConnectstatus == USB_INT_DISCONNECT )return USB_INT_DISCONNECT;
            time_count++;
            if( time_count>=0x1ffff )
            {
                break;
            }
        }
        mDelaymS( 20*(count+1) );
        Hot_ret_flag = 0;
        s = USB30HSOT_Enumerate_Hotrst( pdata );
        if( gDeviceConnectstatus == USB_INT_DISCONNECT )return USB_INT_DISCONNECT;
        if( s == USB_INT_DISCONNECT )return s;

        s = MS_Init_Hotrst( pdata );
        if( s == USB_INT_DISCONNECT )return s;
        if( gDeviceConnectstatus == USB_INT_DISCONNECT )return USB_INT_DISCONNECT;
        if( s!= 0x00 )
        {
            count++;
            if( count<3 )
            {
                mDelaymS(count*10+1);
                goto hot_reset_ag;
            }
        }
    }
    else
    {
        hot_reset_ag1:
            printf("hot_reset1\n");
            Hot_ret_flag = 1;

            USB30H_Switch_Powermode(POWER_MODE_2);
            printf("warn_rst\n");
            USBSSH->LINK_CTRL |= (1<<8) ;
            mDelaymS( 80 );
            USBSSH->LINK_CTRL &= ~(1<<8) ;
            tx_lmp_port = 0;
            while( Hot_ret_flag == 1 )
            {
                time_count++;
                if( gDeviceConnectstatus == USB_INT_DISCONNECT )return USB_INT_DISCONNECT;
                if( time_count>=0x1ffff )
                {
                    break;
                }
            }
            mDelaymS( 20*(count+1) );
            Hot_ret_flag = 0;
            if( gDeviceConnectstatus == USB_INT_DISCONNECT )return USB_INT_DISCONNECT;
            USB30Host_Enum();
            if( gDeviceConnectstatus == USB_INT_DISCONNECT )return USB_INT_DISCONNECT;

            s = MS_Init( pdata );
            if( gDeviceConnectstatus == USB_INT_DISCONNECT )return USB_INT_DISCONNECT;
            if( s == USB_INT_DISCONNECT )return s;
            if( s!= 0x00 ){
                count++;
                if( count<3 )
                {
                    mDelaymS(count*10);
                    goto hot_reset_ag1;
                }
            }
            mDelaymS(500);
    }
    return s;
}

/*******************************************************************************
* Function Name  : MS_ScsiCmd_Process
* Description    : Execute command processing based on BulkOnly protocol
*                  Since the maximum 20K data can be transmitted at one time,
*                  no cyclic processing is performed.
*                  Note: The data buffer should be aligned with the 4K boundary.
*                  If it is not aligned, the size of the buffer itself must be.
*                  It is 4K larger than the actual size to read and write.
* Input          :
*                  *DataBuf----Input and output data buffer
* Output         : None
* Return         : Return to execution status
*******************************************************************************/
UINT8 MS_ScsiCmd_Process( UINT8 *DataBuf )
{
	UINT8  status;
	UINT8  *p,s;
	UINT32 len,i;

	p = DataBuf;
	mBOC.mCBW.mCBW_Sig = USB_BO_CBW_SIG;
	mBOC.mCBW.mCBW_Tag = 0x05630563;
	mBOC.mCBW.mCBW_LUN = gDiskCurLun;			   							    /* Operation current logical unit number */
	len = USB_BO_CBW_SIZE;
	if( gDeviceUsbType == USB_U30_SPEED )
	{
		status = MS_U30HOST_BulkOutHandle( (UINT8 *)&mBOC.mCBW, &len );
		for (i = 0; i < 50; ++i)
		{
            mDelayuS(1);
        }
	}
	else
	{
		status = MS_U20HOST_BulkOutHandle( (UINT8 *)&mBOC.mCBW, &len );
	}
	if( status == USB_INT_DISCONNECT )											/* If the device is disconnected, return directly */
	{
		return( status );
	}
	if( status != USB_INT_SUCCESS )
	{
	    if( gDeviceUsbType == USB_U30_SPEED )
	    {                                  /*Directly return when timeout occurs when USB3.0 is modified*/
            if( status!= USB_INT_SUCCESS )
            {
                return status;                        /*Direct return to reduce time*/
            }
        }
		status = MS_ResetErrorBOC( );

		if( gDeviceUsbType == USB_U30_SPEED )
		{
		    if(gDeviceConnectstatus == USB_INT_DISCONNECT)return USB_INT_DISCONNECT;
		}
		if( status == USB_INT_DISCONNECT )
		{
			return( status );
		}
		else if( status != 0x14 )return ( status );
		/* Send CBW packet again */
		len = USB_BO_CBW_SIZE;
		if( gDeviceUsbType == USB_U30_SPEED )
		{

			status = MS_U30HOST_BulkOutHandle( (UINT8 *)&mBOC.mCBW, &len );
		}
		else
		{
			status = MS_U20HOST_BulkOutHandle( (UINT8 *)&mBOC.mCBW, &len );
		}
		if( status == USB_INT_DISCONNECT )
		{
			return( status );
		}
		if( status != USB_INT_SUCCESS )
		{
	        if( gDeviceUsbType == USB_U30_SPEED )
	        {
	            if( status!= USB_INT_SUCCESS )return status;
	        }
			s = MS_ResetErrorBOC( );
	        printf("boc_status1=%02x\n",status);
			return( s );
		}
	}

	if( ( mBOC.mCBW.mCBW_DataLen > 0 ) && ( mBOC.mCBW.mCBW_Flag == USB_BO_DATA_IN ) )
	{

		/* If there is data to be uploaded, send IN token for data reading */
//		if( mBOC.mCBW.mCBW_DataLen > DEFAULT_MAX_OPERATE_SIZE )
//		{
//			return( USB_PARAMETER_ERROR );										/* Parameter error */
//		}
		/* Send upload IN */
		len = mBOC.mCBW.mCBW_DataLen;
		if( gDeviceUsbType == USB_U30_SPEED )
		{
			status = MS_U30HOST_BulkInHandle( p, &len );
		}
		else
		{
			status = MS_U20HOST_BulkInHandle( p, &len );
		}

		if( status == USB_INT_DISCONNECT )
		{
			return( status );
		}
		if( status != USB_INT_SUCCESS )
		{
			if( status == USB_INT_DISK_ERR )						           /* Return STALL error */
			{
                if( gDeviceUsbType == USB_U30_SPEED )
                {
                    status = USB30HOST_ClearEndpStall(  0x80 | gDiskBulkInEp );
                }
                else
                {
                    status = USB20HOST_ClearEndpStall( 0x80 | gDiskBulkInEp );
                }
                if( status == USB_INT_DISCONNECT )
                {
                    return( status );
                }
                else if( status == 0x14 )
                {
                    gDiskBulkInTog = 0;
                }
                else
                {
                    gDiskBulkInTog = 0;
                }
			}
			else return status;
		}
	}
	else if( ( mBOC.mCBW.mCBW_DataLen > 0 ) && ( mBOC.mCBW.mCBW_Flag == USB_BO_DATA_OUT ) )
	{

		/* If there is data to be downloaded, send the OUT token for data reading */
		if( mBOC.mCBW.mCBW_DataLen > DEFAULT_MAX_OPERATE_SIZE )
		{
			return( USB_PARAMETER_ERROR );
		}
		/* Send download OUT */
		len = mBOC.mCBW.mCBW_DataLen;
		if( gDeviceUsbType == USB_U30_SPEED )
		{
			status = MS_U30HOST_BulkOutHandle( p, &len );
		}
		else
		{
			status = MS_U20HOST_BulkOutHandle( p, &len );
		}
		if( status == USB_INT_DISCONNECT )
		{
			return( status );
		}
		if( status != USB_INT_SUCCESS )
		{
			if( status == USB_INT_DISK_ERR )
			{
				if( gDeviceUsbType == USB_U30_SPEED )
				{
				    status = USB30HOST_ClearEndpStall( gDiskBulkOutEp );
				}
				else
				{
					status = USB20HOST_ClearEndpStall( gDiskBulkOutEp );
				}
				if( status == USB_INT_DISCONNECT )
				{
					return( status );
				}
			}
		}
	}

	/* 4¡¢Send CSW package */
	len = 0;

	if( gDeviceUsbType == USB_U30_SPEED )
	{
	    status = MS_U30HOST_BulkInHandle( (UINT8 *)&mBOC.mCSW, &len );
	}
	else
	{
		status = MS_U20HOST_BulkInHandle( (UINT8 *)&mBOC.mCSW, &len );
	}
	p = (UINT8 *)&mBOC.mCSW;
#ifdef  MY_DEBUG_PRINTF
	for( i=0;i!=len;i++ ){
		printf("%02x ",*p++);
	}
	printf("len=%d\n",len);
#endif

	if( status == USB_INT_SUCCESS )
	{
		if( len != USB_BO_CSW_SIZE )											/* Judge whether the length is 13 bytes */
		{
			return( USB_INT_DISK_ERR );
		}
		if( mBOC.mCSW.mCSW_Status == 0 )
		{
			return( USB_OPERATE_SUCCESS );
		}
		else if( mBOC.mCSW.mCSW_Status >= 2 )
		{
			return( USB_INT_DISK_ERR );
		}
		else
		{
			return( USB_INT_DISK_ERR1 );  										/* Disk operation error */
		}
	}
	else if( status == USB_INT_DISCONNECT )
	{
		return( status );
	}
	else
	{
		/* Judge which step is wrong */
		if( (status == USB_INT_DISK_ERR))
		{
			if( gDeviceUsbType == USB_U30_SPEED )
			{
			    status = USB30HOST_ClearEndpStall( 0x80 | gDiskBulkInEp );
				gDiskBulkInTog = 0;
			}
			else
			{
				status = USB20HOST_ClearEndpStall(  0x80 | gDiskBulkInEp );
			}
			if( status == USB_INT_DISCONNECT )
			{
				return( status );
			}
		}
		return status;
	}
	status = USB_OPERATE_SUCCESS;
	return( status );
}

/*******************************************************************************
* Function Name  : MS_RequestSense
* Description    : USB Host checks disk error status
* Input          :
*                  *pbuf------data buffer
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 MS_RequestSense( UINT8 *pbuf )
{
	mDelaymS( 10 );
	if( gDeviceConnectstatus == USB_INT_DISCONNECT )return USB_INT_DISCONNECT;
	mBOC.mCBW.mCBW_DataLen 	   = 0x00000012;
	mBOC.mCBW.mCBW_Flag 	   = 0x80;
	mBOC.mCBW.mCBW_CB_Len 	   = 0x06;
	mBOC.mCBW.mCBW_CB_Buf[ 0 ] = 0x03;
	mBOC.mCBW.mCBW_CB_Buf[ 1 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 2 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 3 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 4 ] = 0x12;
	mBOC.mCBW.mCBW_CB_Buf[ 5 ] = 0x00;
	return( MS_ScsiCmd_Process( pbuf ) );
}

/*******************************************************************************
* Function Name  : MS_DiskInquiry
* Description    : USB Host obtains disk characteristics
* Input          :
*                  *pbuf------data buffer
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 MS_DiskInquiry(  UINT8 *pbuf )
{
	UINT8 s,retry;
	for( retry = 0;retry!=3;retry++ )
	{
	    mDelaymS( 100 * (retry+1) );
        mBOC.mCBW.mCBW_DataLen 	   = 0x00000024;
        mBOC.mCBW.mCBW_Flag 	   = 0x80;
        mBOC.mCBW.mCBW_CB_Len 	   = 0x06;
        mBOC.mCBW.mCBW_CB_Buf[ 0 ] = 0x12;
        mBOC.mCBW.mCBW_CB_Buf[ 1 ] = 0x00;
        mBOC.mCBW.mCBW_CB_Buf[ 2 ] = 0x00;
        mBOC.mCBW.mCBW_CB_Buf[ 3 ] = 0x00;
        mBOC.mCBW.mCBW_CB_Buf[ 4 ] = 0x24;
        mBOC.mCBW.mCBW_CB_Buf[ 5 ] = 0x00;
        s = MS_ScsiCmd_Process( pbuf );
        if( s == USB_OPERATE_SUCCESS )
        {
            break;
        }
	}
	return( s );
}

/*******************************************************************************
* Function Name  : MS_DiskInquiry
* Description    : USB Host obtains disk characteristics
* Input          :
*                  *pbuf------data buffer
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 MS_DiskInquiry_1(  UINT8 *pbuf )
{
    UINT8 s;
    mBOC.mCBW.mCBW_DataLen     = 0x00000024;
    mBOC.mCBW.mCBW_Flag        = 0x80;
    mBOC.mCBW.mCBW_CB_Len      = 0x06;
    mBOC.mCBW.mCBW_CB_Buf[ 0 ] = 0x12;
    mBOC.mCBW.mCBW_CB_Buf[ 1 ] = 0x00;
    mBOC.mCBW.mCBW_CB_Buf[ 2 ] = 0x00;
    mBOC.mCBW.mCBW_CB_Buf[ 3 ] = 0x00;
    mBOC.mCBW.mCBW_CB_Buf[ 4 ] = 0x24;
    mBOC.mCBW.mCBW_CB_Buf[ 5 ] = 0x00;
    s = MS_ScsiCmd_Process( pbuf );
    return( s );
}

/*******************************************************************************
* Function Name  : MS_DiskCapacity
* Description    : USBHost obtains disk capacity
* Input          :
*                  *pbuf------data buffer
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 MS_DiskCapacity( UINT8 *pbuf )
{
	mBOC.mCBW.mCBW_DataLen     = 0x00000008;
	mBOC.mCBW.mCBW_Flag 	   = 0x80;
	mBOC.mCBW.mCBW_CB_Len      = 10;
	mBOC.mCBW.mCBW_CB_Buf[ 0 ] = 0x25;
	mBOC.mCBW.mCBW_CB_Buf[ 1 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 2 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 3 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 4 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 5 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 6 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 7 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 8 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 9 ] = 0x00;
	return( MS_ScsiCmd_Process( pbuf ) );
}

/*******************************************************************************
* Function Name  : MS_DiskTestReady
* Description    : USB The host tests whether the disk is ready
* Input          : *Device----USB device currently operating
*                  *pbuf------data buffer
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 MS_DiskTestReady(  UINT8 *pbuf )
{
	mBOC.mCBW.mCBW_DataLen 	   = 0x00;
	mBOC.mCBW.mCBW_Flag 	   = 0x00;
	mBOC.mCBW.mCBW_CB_Len      = 0x06;
	mBOC.mCBW.mCBW_CB_Buf[ 0 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 1 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 2 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 3 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 4 ] = 0x00;
	mBOC.mCBW.mCBW_CB_Buf[ 5 ] = 0x00;
	return( MS_ScsiCmd_Process( pbuf ) );
}

/*******************************************************************************
* Function Name  : MS_ReadSector
* Description    : Read data from disk in sectors
* Input          :
*                  StartLba----Start sector number
*                  SectCount---Number of sectors to read
*                  DataBuf-----data buffer
* Output         : None
* Return         : Execution status
*******************************************************************************/
UINT8 MS_ReadSector( UINT32 StartLba, UINT16 SectCount, PUINT8 DataBuf )
{
    UINT8  err, s;
    UINT32 len;

    len = SectCount * gDiskPerSecSize;                                          /* Calculate total read length */

    for( err = 0; err < 3; err ++ )                                             /* Error retry */
    {
        mBOC.mCBW.mCBW_DataLen = len;
        mBOC.mCBW.mCBW_Flag = 0x80;
        mBOC.mCBW.mCBW_CB_Len = 10;
        mBOC.mCBW.mCBW_CB_Buf[ 0 ] = 0x28;
        mBOC.mCBW.mCBW_CB_Buf[ 1 ] = 0x00;
        mBOC.mCBW.mCBW_CB_Buf[ 2 ] = (UINT8)( StartLba >> 24 );
        mBOC.mCBW.mCBW_CB_Buf[ 3 ] = (UINT8)( StartLba >> 16 );
        mBOC.mCBW.mCBW_CB_Buf[ 4 ] = (UINT8)( StartLba >> 8 );
        mBOC.mCBW.mCBW_CB_Buf[ 5 ] = (UINT8)( StartLba );
        mBOC.mCBW.mCBW_CB_Buf[ 6 ] = 0x00;
        mBOC.mCBW.mCBW_CB_Buf[ 7 ] = 0x00;
        mBOC.mCBW.mCBW_CB_Buf[ 8 ] = SectCount;
        mBOC.mCBW.mCBW_CB_Buf[ 9 ] = 0x00;

        s = MS_ScsiCmd_Process( DataBuf );

        if( s == USB_OPERATE_SUCCESS )
        {
            return( USB_OPERATE_SUCCESS );
        }
        if( s == USB_INT_DISCONNECT || s == USB_INT_CONNECT || s == USB_INT_DISK_ERR1)    /* The USB device disconnection event is detected.*/
        {                                                                       /* The disk has been disconnected or just reinserted */
            return( s );
        }
        s = MS_RequestSense( DataBuf );
        if( s == USB_INT_DISCONNECT || s == USB_INT_CONNECT  )
        {
            return( s );
        }
    }
    return( s );
}

/*******************************************************************************
* Function Name  : MS_WriteSector
* Description    : Write data to disk in sectors
* Input          : *Device-----USB device currently operating
*                  StartLba----Start sector number
*                  SectCount---Number of sectors to write
*                  DataBuf-----data buffer
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 MS_WriteSector( UINT32 StartLba, UINT8 SectCount, PUINT8 DataBuf )
{
	UINT8  err, s;
	UINT32 len;

#ifdef  MY_DEBUG_PRINTF
	printf( "MS_WriteSector:...\n" );
#endif
	len = SectCount * gDiskPerSecSize;		  									/* Calculate total read length*/
	if( len > DEFAULT_MAX_OPERATE_SIZE )
	{
		return( USB_PARAMETER_ERROR );
	}
	for( err = 0; err < 3; err++ )
	{
		mBOC.mCBW.mCBW_DataLen = len;
		mBOC.mCBW.mCBW_Flag = 0x00;
		mBOC.mCBW.mCBW_CB_Len = 10;
		mBOC.mCBW.mCBW_CB_Buf[ 0 ] = 0x2A;
		mBOC.mCBW.mCBW_CB_Buf[ 1 ] = 0x00;
		mBOC.mCBW.mCBW_CB_Buf[ 2 ] = (UINT8)( StartLba >> 24 );
		mBOC.mCBW.mCBW_CB_Buf[ 3 ] = (UINT8)( StartLba >> 16 );
		mBOC.mCBW.mCBW_CB_Buf[ 4 ] = (UINT8)( StartLba >> 8 );
		mBOC.mCBW.mCBW_CB_Buf[ 5 ] = (UINT8)( StartLba );
		mBOC.mCBW.mCBW_CB_Buf[ 6 ] = 0x00;
		mBOC.mCBW.mCBW_CB_Buf[ 7 ] = 0x00;
		mBOC.mCBW.mCBW_CB_Buf[ 8 ] = SectCount;
		mBOC.mCBW.mCBW_CB_Buf[ 9 ] = 0x00;

		s = MS_ScsiCmd_Process( DataBuf );
		if( s == USB_OPERATE_SUCCESS )
		{
			return( USB_OPERATE_SUCCESS );
		}
		if( s == USB_INT_DISCONNECT || s == USB_INT_CONNECT )
		{
			return( s );
		}
		s = MS_RequestSense( DataBuf );
		if( s == USB_INT_DISCONNECT || s == USB_INT_CONNECT  )
		{
			return( s );
		}
	}
	return( s );
}

/*******************************************************************************
* Function Name  : MS_Init
* Description    : USB Mass storage device initialization
* Input          :
*                  *pbuf------data buffer
* Output         : None
* Return         : USB_OPERATE_SUCCESS---Initialization succeeded;
*                  USB_OPERATE_ERROR--Initialization error;
*******************************************************************************/
UINT8 MS_Init(  UINT8 *pbuf )
{
	UINT8  count, status;
	/*******************************Get the maximum logical unit number********************************/
	status = MS_GetMaxLun(  );

	if ( status != USB_OPERATE_SUCCESS )
	{
		gDiskMaxLun = 0x00;											 			/* Some USB flash drives may not support */
	}
	printf("gDiskMaxLun=%02x\n",gDiskMaxLun);
	mDelaymS( 10 );

	/*******************************Get USB disk information(INQUIRY)******************************/
	/* Determine whether the current logical unit is CD-ROM */
	gDiskCurLun = 0x00;
	for( count = 0; count < gDiskMaxLun + 1; count++ )
	{
		status = MS_DiskInquiry( pbuf );  								        /* Get Disk Properties */
		printf( "Disk Inquiry:...%02x\n",status );

		if ( status != USB_OPERATE_SUCCESS )
		{
			if( status == USB_INT_DISCONNECT )
			{
				return( status );
			}
			else if( (status&0x20) != USB_PID_NAK )
			{
			    return( USB_OPERATE_ERROR );
			}
		}

		if( ( gDiskMaxLun == 0x00 ) && ( *( pbuf + 0 ) == 0x05 ) )
		{
			return( USB_OPERATE_ERROR );
		}
		if( *( pbuf + 0 ) == 0x05 )
		{
			gDiskCurLun++;
			mDelaymS( 3 );
			continue;
		}
		else
		{
			break;
		}
	}

	/*******************************Get USB flash disk capacity***************************************/
	for( count = 0; count < 5; count++ )
	{
		/* Note: For some USB flash disks, after the first error,
		 * the second operation must wait long enough before operation, otherwise it will always returnNAK */
		mDelaymS( 200 );
		mDelaymS( 100 * count +1 );
		status = MS_DiskCapacity( pbuf );  							/* Get disk capacity*/
		printf( "Disk Capacity:...=%02x\n",status );
		if ( status != USB_OPERATE_SUCCESS )
		{
			if( status == USB_INT_DISCONNECT )
			{
				return( status );
			}
			MS_RequestSense( pbuf );
		}
		else
		{
			/* Save the current sector size */
			gDiskPerSecSize = ( ( (UINT32)( *( pbuf + 4 ) ) ) << 24 );
			gDiskPerSecSize |= ( ( (UINT32)( *( pbuf + 5 ) ) ) << 16 );
			gDiskPerSecSize |= ( ( (UINT32)( *( pbuf + 6 ) ) ) << 8 ) + ( *( pbuf + 7 ) );

			/* Save the current number of sectors */
			gDiskCapability = ( ( (UINT32)( *( pbuf + 0 ) ) ) << 24 );
			gDiskCapability |= ( ( (UINT32)( *( pbuf + 1 ) ) ) << 16 );
			gDiskCapability |= ( ( (UINT32)( *( pbuf + 2 ) ) ) << 8 ) + ( *( pbuf + 3 ) );

			if( gDiskPerSecSize <= 512 ){
			    gDiskPerSecSize = 512;
			}
			if( gDiskPerSecSize >=2048 ){
			    gDiskPerSecSize = 512;
			}
			printf("gDiskPerSecSize: %08lx\n",(UINT32)gDiskPerSecSize);
			printf("gDiskCapability: %08lx\n",gDiskCapability);

			break;
		}
	}

	/*******************************Test whether the USB disk is ready*********************************/
	for( count = 0; count < 5; count ++ )
	{
		mDelaymS( 50 );

		status = MS_DiskTestReady( pbuf );
		if ( status != USB_OPERATE_SUCCESS )
		{
			if( status == USB_INT_DISCONNECT )
			{
				return( status );
			}
			MS_RequestSense( pbuf );
		}
		else
		{
			break;
		}
	}
	return( USB_OPERATE_SUCCESS );
}

/*******************************************************************************
* Function Name  : MS_Init
* Description    : USBMass storage device initialization
* Input          :
*                  *pbuf------data buffer
* Output         : None
* Return         : USB_OPERATE_SUCCESS---Initialization succeeded;
*                  USB_OPERATE_ERROR--Initialization error;
*******************************************************************************/
UINT8 MS_Init_Hotrst(  UINT8 *pbuf )
{
    UINT8  count, status;

    /*******************************Get USB disk information(INQUIRY)******************************/
    gDiskCurLun = 0x00;
    for( count = 0; count < gDiskMaxLun + 1; count++ )
    {
        status = MS_DiskInquiry_1( pbuf );
        if ( status != USB_OPERATE_SUCCESS )
        {
            if( status == USB_INT_DISCONNECT )
            {
                return( status );
            }
            else if( (status&0x20) != USB_PID_NAK )
            {
                return( USB_OPERATE_ERROR );
            }
            else return status;
        }

        if( ( gDiskMaxLun == 0x00 ) && ( *( pbuf + 0 ) == 0x05 ) )
        {
            return( USB_OPERATE_ERROR );
        }
        if( *( pbuf + 0 ) == 0x05 )
        {
            gDiskCurLun++;
            mDelaymS( 3 );
            continue;
        }
        else
        {
            break;
        }
    }

    return( USB_OPERATE_SUCCESS );
}

/*******************************************************************************
* Function Name  : MS_U20HOST_Bulk_Handle
* Description    : USB Host performs batch transfer
* Input          :
*                  EndpNum----Endpoint number
*                  *SeqNum-----0-31
*                  PacketNum---1
*                  *pDatBuf---Data buffer, buffer address must be 4-byte aligned
*                  *pSize-----Send and receive data size maximum 512 byte
*                  Type-------Transport type(IN/OUT)
* Output         : None
* Return         : Returns the current command execution status
*
*******************************************************************************/
UINT8 MS_U20HOST_Bulk_Handle(  UINT8 EndpNum,UINT8 tog, UINT8 *pDatBuf, UINT16 *pSize, UINT8 Pid )
{
	UINT8 s = 0;
	UINT8 *p;
	if( Pid == USB_PID_OUT )
	{			//send data
		if( (UINT32)(UINT8 *)pDatBuf < (UINT32)(UINT8 *)MAX_DATA_ADDR )
		{
			p = (UINT8 *)endpTXbuff;
			R32_UH_TX_DMA = (UINT32)(UINT8 *)endpTXbuff;
			R32_UH_RX_DMA = (UINT32)(UINT8 *)endpRXbuff;
			memcpy( p,pDatBuf,*pSize );
		}
		else
		{
			R32_UH_TX_DMA = (UINT32)(UINT8 *)pDatBuf;
			R32_UH_RX_DMA = (UINT32)(UINT8 *)endpRXbuff;
		}
		R16_UH_TX_LEN = *pSize;
	    s = USB20HOST_Transact( USB_PID_OUT << 4 | EndpNum, tog, 1000000 );          //out data,200mS timeout
	}
	else if( Pid == USB_PID_IN )
	{		//recive data
		if( (UINT32)(UINT8 *)pDatBuf < (UINT32)(UINT8 *)MAX_DATA_ADDR )
		{
			R32_UH_TX_DMA = (UINT32)(UINT8 *)endpTXbuff;
			R32_UH_RX_DMA = (UINT32)(UINT8 *)endpRXbuff;
		}
		else
		{
			R32_UH_TX_DMA = (UINT32)(UINT8 *)endpTXbuff;
			R32_UH_RX_DMA = (UINT32)(UINT8 *)pDatBuf;
		}
	    s = USB20HOST_Transact( USB_PID_IN << 4 | EndpNum, tog, 1000000 );          //in data,200mS timeout
	    if( s == ERR_SUCCESS1 )
	    {
	    	*pSize = R16_USB_RX_LEN;

	    	if( (UINT32)(UINT8 *)pDatBuf < (UINT32)(UINT8 *)MAX_DATA_ADDR )
	    	{
				p = (UINT8 *)endpRXbuff;
				memcpy( pDatBuf,p,*pSize );
	    	}
	    }
	    else if( s == 0x2e )return USB_INT_DISK_ERR;
	}
	return s;
}

/*******************************************************************************
* Function Name  : MS_U20HOST_BulkOutHandle
* Description    : USBHost performs batch transfer
* Input          :
*                  *pDatBuf---Data buffer, buffer address must be 4-byte aligned
*				   *pSize-----Send and receive data size maximum 512 byte
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 MS_U20HOST_BulkOutHandle( UINT8 *pDatBuf, UINT32 *pSize )
{
	UINT8 status;
	UINT16 len;
	while(1)
	{
		if( *pSize >= U20_ENDP_SIZE )
		{
			len = U20_ENDP_SIZE;
		}
		else
		{
			len = *pSize;
		}
		status = MS_U20HOST_Bulk_Handle( gDiskBulkOutEp,gDiskBulkOutTog,pDatBuf,&len,USB_PID_OUT );
		if( status != USB_INT_SUCCESS )return status;
		*pSize -= len;
		pDatBuf+=len;
		gDiskBulkOutTog^=0x01;
		if( *pSize == 0 )break;
	}
	return USB_INT_SUCCESS;
}

/*******************************************************************************
* Function Name  : MS_U20HOST_BulkInHandle
* Description    : USB Host performs batch transfer
* Input          :
*                  *pDatBuf---Data buffer, buffer address must be 4-byte aligned
*				   *pSize-----Send and receive data size maximum 512 byte
* Output         : None
* Return         : Returns the current command execution status
*******************************************************************************/
UINT8 MS_U20HOST_BulkInHandle( UINT8 *pDatBuf, UINT32 *pSize )
{
	UINT8 status;
	UINT16 len,total_len;
	total_len = 0;
	while(1)
	{
		if( *pSize >= U20_ENDP_SIZE )
		{
			len = U20_ENDP_SIZE;
		}
		else
		{
			len = *pSize;
		}
		status = MS_U20HOST_Bulk_Handle( gDiskBulkInEp,gDiskBulkInTog,pDatBuf,&len,USB_PID_IN );
		if( status != USB_INT_SUCCESS )return status;
		gDiskBulkInTog^= 0x01;
		*pSize -= len;
		pDatBuf+=len;
		total_len+=len;
		if( (*pSize == 0) || (len<U20_ENDP_SIZE))
		{
			*pSize = total_len;
			break;
		}
	}
	return USB_INT_SUCCESS;
}
