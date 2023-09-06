/********************************** (C) COPYRIGHT *******************************
* File Name          : SD.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/


#ifndef __SD_H__
#define __SD_H__
#include "CH56xSFR.h"
#include "CH56x_emmc.h"


UINT8 SDReadOCR( PSD_PARAMETER pEMMCPara );
UINT8 SDSetRCA( PSD_PARAMETER pEMMCPara );
UINT8 SDReadCSD( PSD_PARAMETER pEMMCPara );
UINT8 SDSetBusWidth(PSD_PARAMETER pEMMCPara, UINT8 bus_mode);
UINT8 SD_ReadSCR(PSD_PARAMETER pEMMCPara, PUINT8 pRdatbuf);
UINT8 SDCardReadOneSec( PSD_PARAMETER pEMMCPara, PUINT8 pRdatbuf, UINT32 Lbaaddr );
UINT8 SDCardWriteONESec( PSD_PARAMETER pEMMCPara,  PUINT8 pWdatbuf, UINT32 Lbaaddr );

#endif
