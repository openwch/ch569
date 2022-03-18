/********************************** (C) COPYRIGHT *******************************
* File Name          : usb30_callback.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 : 
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56xUSB30_LIB.H"
#include "usb30_porp.h"

void  ITP_Callback(UINT32 ITPCounter)
{

}
/***************Endpointn IN Transaction Processing*******************/
void EP1_IN_Callback()
{
	endp1_Hander();
}

void EP2_IN_Callback()
{

}

void EP3_IN_Callback()
{

}
void EP4_IN_Callback()
{
}
void EP5_IN_Callback()
{

}
void EP6_IN_Callback()
{

}
void EP7_IN_Callback()
{

}

/***************Endpointn OUT Transaction Processing*******************/
void EP1_OUT_Callback()
{

}

void EP2_OUT_Callback()
{

}
void EP3_OUT_Callback()
{

}
void EP4_OUT_Callback()
{

}
void EP5_OUT_Callback()
{

}
void EP6_OUT_Callback()
{

}
void EP7_OUT_Callback()
{

}

