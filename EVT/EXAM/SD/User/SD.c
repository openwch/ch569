/********************************** (C) COPYRIGHT *******************************
* File Name          : SD.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 
*******************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/


#include "SD.h"
#include "CH56x_common.h"

/***************************************************************
 * @fn        SDReadOCR
 *
 * @brief     when SD waiting status,do the OCR analysis
 *
 * @param     pEMMCPara
 *
 * @return    CMD_SUCCESS
 *            OP_FAILED
 */
UINT8 SDReadOCR( PSD_PARAMETER pEMMCPara )
{
	UINT8  i;
	UINT32 cmd_arg_val;
	UINT16 cmd_set_val;
	UINT8  sta = 0;
	UINT32 cmd_rsp_val;                 //command returned value

	for(i=0; i<100; i++)
	{
		cmd_arg_val = 0;                //request switching voltage
		cmd_set_val = RB_EMMC_CKIDX |   //ACK's index
		              RB_EMMC_CKCRC |   //CRC
					  RESP_TYPE_48  |   //ACK type
					  55;               //command's index this moment
		EMMCSendCmd(cmd_arg_val,  cmd_set_val);
		mDelayuS(2);
		while(1)
		{
			sta = CheckCMDComp( pEMMCPara );
			if( sta!= CMD_NULL ) 	break;
		}
        if(sta == CMD_FAILED)
        {
            mDelaymS(20);
            continue;
        }
        cmd_arg_val = 0x40FF8000;       //Request voltage switching, 0x41FF8000-->Request voltage switching
        cmd_set_val = 0 |               //The command index of the verification response
                      0 |               //Check the CRC of the response
                      RESP_TYPE_48 |    //expected response type
                      41;               //The index number of the currently sent command
        EMMCSendCmd(cmd_arg_val,  cmd_set_val);
        mDelayuS(2);
        while(1)
        {
            sta = CheckCMDComp( pEMMCPara );
            if( sta!= CMD_NULL )    break;
        }
        if(sta == CMD_SUCCESS)
        {
            cmd_rsp_val = R32_EMMC_RESPONSE3 ;
            if(cmd_rsp_val & (1<<31))                                       // Card initialization complete
            {
                if(cmd_rsp_val & (1<<30))
                    pEMMCPara->EMMCType = EMMCIO_HIGH_CAPACITY_SD_CARD;     //high capacity card
                else
                    pEMMCPara->EMMCType = EMMCIO_CAPACITY_SD_CARD_V2_0;     //Standard Capacity Card

                if(cmd_rsp_val & (1<<24))                                   //Support cut voltage
                {
                    printf(" support low vol..\n ");
                }
                break;
            }
        }
        mDelaymS(20);
	}
	if(i == 100)		return OP_FAILED;

	return sta;
}

/***************************************************************
 * @fn        SDSetRCA
 *
 * @brief     Assign relative address to deviceARC   16bit
 *
 * @param     pEMMCPara
 *
 * @return    OP_SUCCESS
 *            OP_FAILED
 */
UINT8 SDSetRCA( PSD_PARAMETER pEMMCPara )
{
	UINT32 cmd_arg_val;
	UINT16 cmd_set_val;
	UINT8  sta;

	cmd_arg_val = 0;
	cmd_set_val = RB_EMMC_CKIDX |
				  RB_EMMC_CKCRC |
				  RESP_TYPE_48  |
				  EMMC_CMD3;
	EMMCSendCmd(cmd_arg_val,cmd_set_val);
	while(1)
	{
		sta = CheckCMDComp( pEMMCPara );
		if( sta != CMD_NULL ) break;
	}
	if(sta == CMD_SUCCESS)
	{
		pEMMCPara->EMMC_RCA = R32_EMMC_RESPONSE3 >> 16;
	}
	return sta;
}

/***************************************************************
 * @fn        SDReadCSD
 *
 * @brief     Acquire 128bit CSD parameter and get it analyzed
 *
 * @param     pEMMCPara
 *
 * @return    OP_SUCCESS
 *            OP_FAILED
 */
UINT8 SDReadCSD( PSD_PARAMETER pEMMCPara )
{
	UINT32 cmd_arg_val;
	UINT16 cmd_set_val;
	UINT8  sta;
	UINT32 disk_block_num = 0;

	cmd_arg_val = pEMMCPara->EMMC_RCA<<16;
	cmd_set_val = 0 |
				  0 |
				  RESP_TYPE_136 |
				  EMMC_CMD9;
	EMMCSendCmd(cmd_arg_val, cmd_set_val);
	while(1)
	{
		sta = CheckCMDComp( pEMMCPara );
		if( sta != CMD_NULL ) break;
	}

	if(sta == CMD_SUCCESS)
	{
		pEMMCPara->EMMC_CSD[0] = R32_EMMC_RESPONSE0;
		pEMMCPara->EMMC_CSD[1] = R32_EMMC_RESPONSE1;
		pEMMCPara->EMMC_CSD[2] = R32_EMMC_RESPONSE2;
		pEMMCPara->EMMC_CSD[3] = R32_EMMC_RESPONSE3;

		if(pEMMCPara->EMMC_CSD[3]>>30)   //High Capacity and Extended Capacity Cards
        {
		    pEMMCPara->EMMCType = EMMCIO_HIGH_CAPACITY_SD_CARD;
            /* C_Size memory capacity = (C_SIZE+1)*512K byte */
            disk_block_num = ( (((pEMMCPara->EMMC_CSD[2]&0x0ff)<<16) | ((pEMMCPara->EMMC_CSD[1]&0xffff0000)>>16)) + 1 ) << 10;
        }
        else //dard card
        {
            pEMMCPara->EMMCType = EMMCIO_CAPACITY_SD_CARD_V2_0;
            /* memory capacity = BLOCKNR*BLOCK_LEN = (C_SIZE+1)<<(C_SIZE_MULT+2)<<(READ_BL_LEN) */
            disk_block_num = ( (((pEMMCPara->EMMC_CSD[2]&0x3ff)<<2) | (pEMMCPara->EMMC_CSD[1]>>30)) + 1 );
            disk_block_num = ( (disk_block_num) << (((pEMMCPara->EMMC_CSD[1]>>15)&0x07) + 2));
        }

	}
	pEMMCPara->EMMCSecNum = disk_block_num;                             //total number of sectors
    pEMMCPara->EMMCSecSize = 1<<((pEMMCPara->EMMC_CSD[2]>>16)&0x000f);  //sector size
	return sta;
}

/***************************************************************
 * @fn        SDSetBusWidth
 *
 * @brief     Set bus width.
 *
 * @param     pEMMCPara
 *            bus_mode
 *
 * @return    OP_SUCCESS
 *            OP_FAILED
 */
UINT8 SDSetBusWidth(PSD_PARAMETER pEMMCPara, UINT8 bus_mode)
{
	UINT32 cmd_arg_val;
	UINT16 cmd_set_val;
	UINT8  sta;

    cmd_arg_val = (pEMMCPara->EMMC_RCA)<<16;
    cmd_set_val = RB_EMMC_CKIDX |            //The command index of the verification response
                  RB_EMMC_CKCRC |            //Check the CRC of the response
                  RESP_TYPE_48  |            //expected response type
                  55;                        //The index number of the currently sent command
    EMMCSendCmd(cmd_arg_val,cmd_set_val);
    while(1)
    {
        sta = CheckCMDComp( pEMMCPara );
        if( sta != CMD_NULL ) break;
    }
    if(sta == CMD_SUCCESS)
    {
        if(bus_mode == 0)
            cmd_arg_val = 0x0;               //single wire interface
        else
            cmd_arg_val = 0x2;               //4-wire interface

        cmd_set_val = RB_EMMC_CKIDX |        //The command index of the verification response
                      RB_EMMC_CKCRC |        //Check the CRC of the response
                      RESP_TYPE_48  |        //expected response type
                      EMMC_CMD6;             //The index number of the currently sent command
        EMMCSendCmd(cmd_arg_val, cmd_set_val);
        while(1)
        {
            sta = CheckCMDComp( pEMMCPara );
            if( sta != CMD_NULL ) break;
        }
    }
	return sta;
}

/***************************************************************
 * @fn        SD_ReadSCR
 *
 * @brief     Read SCR
 *
 * @param     pEMMCPara
 *            pRdatbuf
 *
 * @return    OP_SUCCESS
 *            OP_FAILED
 */
UINT8 SD_ReadSCR(PSD_PARAMETER pEMMCPara, PUINT8 pRdatbuf)
{
    UINT32 cmd_arg_val;
    UINT16 cmd_set_val,t;
    UINT8  sta;

    cmd_arg_val = (pEMMCPara->EMMC_RCA)<<16;
    cmd_set_val = RB_EMMC_CKIDX |               //The command index of the verification response
                  RB_EMMC_CKCRC |               //CRC of the verification response
                  RESP_TYPE_48  |               //expected response type
                  55;                           //The index number of the currently sent command
    EMMCSendCmd(cmd_arg_val,cmd_set_val);
    while(1)
    {
        sta = CheckCMDComp( pEMMCPara );
        if( sta != CMD_NULL )
            break;
    }

    if(sta == CMD_SUCCESS)
    {
        R32_EMMC_DMA_BEG1 = (UINT32)pRdatbuf;
        R32_EMMC_TRAN_MODE = 0;
        R32_EMMC_BLOCK_CFG = 8 << 16 | 1;        //The number of data bytes received per block | Set the number of blocks to receive and start the transfer

        /* Send CMD51 */
        cmd_arg_val = 0;
        cmd_set_val = RB_EMMC_CKIDX |            //The command index of the verification response
                      RB_EMMC_CKCRC |            // CRC of the verification response
                      RESP_TYPE_48  |            // expected response type
                      51;                        // The index number of the currently sent command
        EMMCSendCmd(  cmd_arg_val, cmd_set_val );
        while(1)
        {
            sta = CheckCMDComp( pEMMCPara );
            if( sta != CMD_NULL )
                break;
        }
        while( 1 )
        {
            if( R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE )
            {
                break;
            }
        }
        t=R16_EMMC_INT_FG;
        R16_EMMC_INT_FG = t;
    }

    return sta;
}

/***************************************************************
 * @fn        SDCardReadOneSec
 *
 * @brief     Read single section
 *
 * @param     pEMMCPara
 *            pRdatbuf
 *            Lbaaddr
 *
 * @return    OP_SUCCESS
 *            OP_FAILED
 */
UINT8 SDCardReadOneSec( PSD_PARAMETER pEMMCPara, PUINT8 pRdatbuf, UINT32 Lbaaddr )
{
	UINT32 cmd_arg_val;
	UINT16 cmd_set_val;

	if(Lbaaddr > (pEMMCPara->EMMCSecNum))
	    return  OP_INVALID_ADD;

    while(!(R32_EMMC_STATUS & (1<<17)));

    R32_EMMC_DMA_BEG1 = (UINT32)pRdatbuf;
    R32_EMMC_TRAN_MODE = 0;
    R32_EMMC_BLOCK_CFG = (pEMMCPara->EMMCSecSize)<<16 | 1;

	cmd_arg_val = Lbaaddr;
	cmd_set_val = RB_EMMC_CKIDX |
				  RB_EMMC_CKCRC |
				  RESP_TYPE_48  |
				  EMMC_CMD17;  //read single block

	EMMCSendCmd(cmd_arg_val, cmd_set_val);

	/*R32_EMMC_TRAN_MODE |= RB_EMMC_GAP_STOP*/
	while(1)
	{
		if(R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE)
		{
		    R16_EMMC_INT_FG = RB_EMMC_IF_CMDDONE|RB_EMMC_IF_TRANDONE;
		    break;
		}
	}
	return 	OP_SUCCESS;
}

/***************************************************************
 * @fn        SDCardWriteONESec
 *
 * @brief     write ONE sections
 *
 * @param     pSDPara
 *            pReqnum
 *            pWdatbuf
 *            Lbaaddr
 *
 * @return    OP_SUCCESS
 *            OP_FAILED
 */
UINT8 SDCardWriteONESec( PSD_PARAMETER pEMMCPara,  PUINT8 pWdatbuf, UINT32 Lbaaddr )
{
	UINT32 cmd_arg_val;
	UINT16 cmd_set_val;
	UINT8  sta;

	if(Lbaaddr > (pEMMCPara->EMMCSecNum))
	    return  OP_INVALID_ADD;

    while(!(R32_EMMC_STATUS & (1<<17)));

    //cmd25
	cmd_arg_val = Lbaaddr;
	cmd_set_val = RB_EMMC_CKIDX |
				  RB_EMMC_CKCRC |
				  RESP_TYPE_48  |
				  EMMC_CMD25;
	EMMCSendCmd(cmd_arg_val, cmd_set_val);
	while(1)
	{
		sta = CheckCMDComp( pEMMCPara );
		if( sta != CMD_NULL )
		    break;
	}

	if( sta == CMD_FAILED )
	{	//The operation failed
	    return OP_FAILED;
	}

	//DAT
	R32_EMMC_TRAN_MODE = RB_EMMC_DMA_DIR|(1<<6);
	R32_EMMC_DMA_BEG1 = (UINT32)pWdatbuf;
	R32_EMMC_BLOCK_CFG = (pEMMCPara->EMMCSecSize)<<16 | 1;

	while(1)
	{
		if(R16_EMMC_INT_FG & RB_EMMC_IF_BKGAP)
		{
			R16_EMMC_INT_FG = RB_EMMC_IF_BKGAP;
		}
		else if(R16_EMMC_INT_FG & RB_EMMC_IF_TRANDONE)
		{
			R16_EMMC_INT_FG = RB_EMMC_IF_TRANDONE | RB_EMMC_IF_CMDDONE;
			//cmd12
			cmd_arg_val = 0;
			cmd_set_val = RB_EMMC_CKIDX |
						  RB_EMMC_CKCRC |
						  RESP_TYPE_R1b |
						  EMMC_CMD12;
			EMMCSendCmd(cmd_arg_val, cmd_set_val);
			break;
		}
		if( pEMMCPara->EMMCOpErr )
		    return CMD_FAILED;
	}
	while(1)
	{
		sta = CheckCMDComp( pEMMCPara );
		if( sta != CMD_NULL ) break;
	}
	R16_EMMC_INT_FG = RB_EMMC_IF_CMDDONE;
	return 	sta;
}
