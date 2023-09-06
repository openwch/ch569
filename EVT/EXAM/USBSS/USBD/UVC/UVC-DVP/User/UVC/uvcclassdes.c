/********************************** (C) COPYRIGHT *******************************
* File Name          : uvcDvp_Classdes.h
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "UVCLIB.H"

#define NoError         0x00
#define NotReady        0x01
#define WrongState      0x02
#define Power           0x03
#define OutOfRange      0x04
#define InvalidUnit     0x05
#define InvalidControl  0x06
#define InvalidRequest  0x07
#define InvalidValueWithinRange 0x08
#define Unknown         0xFF

typedef struct
{
    UINT8 info[1];
    UINT8 len[2];   //Used by the expansion unit
    UINT8 min[10];
    UINT8 max[10];
    UINT8 res[10];
    UINT8 def[10];
    UINT8 cur[10];
    UINT8 set[10];
}Unit_t;

Unit_t VAICSID[]={//ID=0,send VideoControl Interface Control Selectors
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//VC _CONTROL_ UNDEFINED
        {
                0,{0x01,0x00},
                {0},{0},{0},{0},{0x10},{0}
        },//VC_VIDEO_POWER_MODE_CONTROL
        {
                0,{0x01,0x00},
                {0},{0},{0},{0},{NoError},{0}
        },//VC_REQUEST_ERROR_CODE_CONTROL
        {}//Reserved


};
Unit_t CameraID[]={//ID 01
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//CT_CONTROL_UNDEFINED 0
        {
                0x03,{0x01,0x00},
                {0},{0},{0},{0},{0},{0}

        },//CT_SCANNING_MODE_CONTROL 1
        {
                0x03,{0x01,0x00},
                {0},{0},{0},{0},{0x06},{0}
        },//CT_AE_MODE_CONTROL 2
        {
                0x3,{0x01,0x00},
                {0x00},{0x01},{0x01},{0x00},{0},{0}
        },//CT_AE_PRIORITY_CONTROL 3
        {
                0x0f,{0x04,0x00},
                {0x32,0x00,0x00,0x00},{0x10,0x27,0x00,0x00},{0x01,0x00,0x00,0x00},{0xa6,0x00,0x00,0x00},{0},{0}
        },//CT_EXPOSURE_TIME_ABSOLUTE_CONTROL 4
        {
                0x00,{0x00,0x00},
                {0},{0},{0},{0},{0},{0}
        },//CT_EXPOSURE_TIME_RELATIVE_CONTROL 5
        {
                0x0f,{0x02,0x00},
                {0x00,0x00},{0xff,0x03},{0x01,0x00},{0x44,0x00},{0x44,0x00},{0}
        },//CT_FOCUS_ABSOLUTE_CONTROL 6
        {
                0x00,{0x00,0x00},
                {0},{0},{0},{0},{0},{0}
        },//CT_FOCUS_RELATIVE_CONTROL 7
        {
                0,{0x01},
                {0},{0},{0},{0},{0x01},{0}
        },//CT_FOCUS_AUTO_CONTROL 8
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//CT_IRIS_ABSOLUTE_CONTROL 9
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//CT_IRIS_RELATIVE_CONTROL a
        {
                0x03,{0x02,0x00},
                {0x00,0x00},{0x03,0x00},{0x01,0x00},{0x00,0x00},{0},{0}
        },//CT_ZOOM_ABSOLUTE_CONTROL b
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//CT_ZOOM_RELATIVE_CONTROL c
        {
                0x03,{0x08,0x00},
                {0x00, 0x1f, 0xff, 0xff,  0x40, 0x57, 0xff, 0xff },
                {0x00, 0xe1, 0x00, 0x00,  0xc0, 0xa8, 0x00, 0x00},
                {0x10, 0x0e, 0x00, 0x00,  0x10, 0x0e, 0x00, 0x00},{0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00},{0},{0}
        },//CT_PANTILT_ABSOLUTE_CONTROL d
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//CT_PANTILT_RELATIVE_CONTROL e
        {
                0x03,{0x02,0x00},
                {0},{0x03,0x00},{0x01,0x00},{0},{0},{0}
        },//CT_ROLL_ABSOLUTE_CONTROL f
        {
                0x03,{0x02,0x00},
                {0x00, 0x00},{0x03, 0x00},{0x01, 0x00},{0x00, 0x00},{0},{0}
        },//CT_ROLL_RELATIVE_CONTROL
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//CT_PRIVACY_CONTROL
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//CT_FOCUS_SIMPLE_CONTROL
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//CT_WINDOW_CONTROL
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        }//CT_REGION_OF_INTEREST_CONTROL
};

Unit_t ProcessID[]={//ID 02
        {
                0,{0x00,0x00},
                {0},{0},{0},{0},{0},{0}
        },//PU_CONTROL_UNDEFINED 0
        {
                0x03,{0x02,0x00},
                {0x00, 0x00},{0x02, 0x00},{0x01, 0x00},{0x00, 0x00},{0},{0}
        },//PU_BACKLIGHT_COMPENSATION_CONTROL 1
        {
                0x03,{0x02,0x00},
                {0xc0, 0xff},{0x40, 0x00 },{0x01, 0x00},{0x00, 0x00},{0},{0}
        },//PU_BRIGHTNESS_CONTROL 2
        {
                0x03,{0x02,0x00},
                {0x00, 0x00},{0x64, 0x00},{0x01, 0x00 },{0x32, 0x00},{0},{0}
        },//PU_CONTRAST_CONTROL 3
        {
                0,{0x00,0x00},
                {0},{0},{0},{0},{0},{0}
        },//PU_GAIN_CONTROL 4
        {
                0x03,{0x01,0x00},
                {0x00},{0x02},{0x01},{0x01},{0x01},{0}
        },//PU_POWER_LINE_FREQUENCY_CONTROL 5
        {
                0x03,{0x02,0x00},
                {0x4c, 0xff},{ 0xb4, 0x00},{0x01, 0x00 },{0x00, 0x00},{0},{0}
        },//PU_HUE_CONTROL 6
        {
                0x03,{0x02,0x00},
                {0x00, 0x00},{0x64, 0x00},{0x01, 0x00},{0x40, 0x00},{0},{0}
        },//PU_SATURATION_CONTROL 7
        {
                0x03,{0x02,0x00},
                {0x00, 0x00},{0x64, 0x00},{0x01, 0x00},{0x32, 0x00},{0},{0}
        },//PU_SHARPNESS_CONTROL 8
        {
                0x03,{0},
                {0x64, 0x00},{0xf4, 0x01},{0x01, 0x00},{0x2c, 0x01},{0},{0}
        },//PU_GAMMA_CONTROL 9
        {
                0x0f,{0x02,0x00},
               {0xf0, 0x0a},{0x64, 0x19},{0x0a, 0x00},{0xf8, 0x11},{0},{0}
        },//PU_WHITE_BALANCE_TEMPERATURE_CONTROL a
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//PU_WHITE_BALANCE_COMPONENT_CONTROL
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//PU_DIGITAL_MULTIPLIER_CONTROL
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//PU_HUE_AUTO_CONTROL
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//PU_ANALOG_VIDEO_STANDARD_CONTROL
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        },//PU_ANALOG_LOCK_STATUS_CONTROL
        {
                0,{0},
                {0},{0},{0},{0},{0},{0}
        }//PU_CONTRAST_AUTO_CONTROL

};
Unit_t ExtensionID1[]={//ID 04
        {0},{0},{0},{0},{0},{0},{0},{0},{0},{0},
        {
                0x03,{0x08,0x00},
                {0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00},
                {0xff, 0xff, 0xff, 0xff,  0xff, 0xff, 0xff, 0xff },
                {0x01, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00},{0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00},{0},{0}
        },//a
        {
                0x03,{0x08,0x00},
                {0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00},
                {0xff, 0xff, 0xff, 0xff,  0xff, 0xff, 0xff, 0xff },
                {0x01, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00},{0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00},{0},{0}
        }//b
};
Unit_t ExtensionID2[]={//ID 06
        {0},
        {
                0x03,{0x02,0x00},
                {0x00, 0x00},{0xff, 0xff},{0x00, 0x00},{0x01, 0x00},{0},{0}
        },
        {
                0x03,{0x02,0x00},
                {0xfa, 0xff},{0x06, 0x00},{0x40, 0x00},{0x00, 0x00},{0},{0}
        },
        {
                0x01,{0x02,0x00},
                {0x00, 0x00},{0x00, 0x00},{0x00, 0x00},{0x00, 0x00},{0},{0}
        },
        {
                0x03,{0x01,0x00},
                {0x00, 0x00},{0x00, 0x00},{0x00, 0x00},{0x00, 0x00},{0},{0}
        },
        {
                0x03,{0x01,0x00},
               {0x00, 0x00},{0x00, 0x00},{0x00, 0x00},{0x00, 0x00},{0},{0}
        },
        {
                0x03,{0x01,0x00},
                {0x00, 0x00},{0x00, 0x00},{0x00, 0x00},{0x00, 0x00},{0},{0}
        },{0},{0},{0},{0},{0},
        {0},{0},{0},{0},{0},{0},{0},
        {
                0x03,{0x01,0x00},
               {0x00, 0x00},{0x01, 0x00},{0x01, 0x00},{0x00, 0x00},{0},{0}
        },
        {
                0x03,{0x0a,0x00},
                {0},{0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  0x09, 0x00 },{0},{0},{0},{0}
        },
        {
                0x01,{0x04,0x00},
                {0},{0},{0},{0},{0},{0}
        },
        {
                0x03,{0x01,0x00},
                {0x00, 0x00},{0x01, 0x00},{0x01, 0x00},{0x00, 0x00},{0},{0}
        },
        {
                0x03,{0x01,0x00},
               {0x01, 0x00},{0x03, 0x00},{0x02, 0x00},{0x01, 0x00},{0},{0}
        },
        {
                0x03,{0x01,0x00},
                {0x01, 0x00},{0x00, 0x00},{0x01, 0x00},{0x01, 0x00},{0},{0}
        }
};

/*******************************************************************************
 * @fn      UVC_NonStandardReq
 *
 * @brief   Nonstandard request processing function
 *
 * @return  Length
 */
UINT16 UVC_NonStandardReq(UINT8 **pDescr){
    if(UsbSetupBuf->bRequestType==0xA1){
        switch(UsbSetupBuf->wIndexH){//UsbSetupBuf->wValueH
                    case 00:
                        switch(UsbSetupBuf->wIndexL){
                            case 00://Only send request to the VideoControl interface (interface ID 00)
                                switch(UsbSetupBuf->bRequest){
                                    case 0x81://GET CUR
                                                *pDescr = (PUINT8)&VAICSID[UsbSetupBuf->wValueH].cur;
                                                break;
                                }
                                break;
                            case 01://
                                switch(UsbSetupBuf->bRequest){
                                        case 0x81://GET CUR
                                        case 0x82:
                                        case 0x83:
                                                *pDescr = (PUINT8)&Get_Curr;
                                                break;
                                }
                                break;
                            }
                        break;
                    case 01:
                        switch(UsbSetupBuf->bRequest){//LEN->INFO->MIN->MAX->RES->DEF
                            case 0x81://GET CUR
                                *pDescr = (PUINT8)&CameraID[UsbSetupBuf->wValueH].cur;
                                break;
                            case 0x82://GET MIN
                                *pDescr = (PUINT8)&CameraID[UsbSetupBuf->wValueH].min;
                                break;
                            case 0x83://GET MAX
                                *pDescr = (PUINT8)&CameraID[UsbSetupBuf->wValueH].max;
                                break;
                            case 0x84://GET RES
                                *pDescr = (PUINT8)&CameraID[UsbSetupBuf->wValueH].res;
                                break;
                            case 0x85://GET LEN
                                *pDescr = (PUINT8)&CameraID[UsbSetupBuf->wValueH].len;
                                break;
                            case 0x86:// GET INFO
                                *pDescr = (PUINT8)&CameraID[UsbSetupBuf->wValueH].info;
                                break;
                            case 0x87://GET DEF
                                *pDescr = (PUINT8)&CameraID[UsbSetupBuf->wValueH].def;
                                break;
                        }
                        break;
                    case 02:
                        switch(UsbSetupBuf->bRequest){//LEN->INFO->MIN->MAX->RES->DEF
                            case 0x81://GET CUR
                                *pDescr = (PUINT8)&ProcessID[UsbSetupBuf->wValueH].cur;
                                break;
                            case 0x82://GET MIN
                                *pDescr = (PUINT8)&ProcessID[UsbSetupBuf->wValueH].min;
                                break;
                            case 0x83://GET MAX
                                *pDescr = (PUINT8)&ProcessID[UsbSetupBuf->wValueH].max;
                                break;
                            case 0x84://GET RES
                                *pDescr = (PUINT8)&ProcessID[UsbSetupBuf->wValueH].res;
                                break;
                            case 0x85://GET LEN
                                *pDescr = (PUINT8)&ProcessID[UsbSetupBuf->wValueH].len;
                                break;
                            case 0x86:// GET INFO
                                *pDescr = (PUINT8)&ProcessID[UsbSetupBuf->wValueH].info;
                                break;
                            case 0x87://GET DEF
                                *pDescr = (PUINT8)&ProcessID[UsbSetupBuf->wValueH].def;
                                break;
                        }
                        break;
                    case 04:
                        switch(UsbSetupBuf->bRequest){//LEN->INFO->MIN->MAX->RES->DEF
                            case 0x81://GET CUR
                                *pDescr = (PUINT8)&ExtensionID1[UsbSetupBuf->wValueH].cur;
                                break;
                            case 0x82://GET MIN
                                *pDescr = (PUINT8)&ExtensionID1[UsbSetupBuf->wValueH].min;
                                break;
                            case 0x83://GET MAX
                                *pDescr = (PUINT8)&ExtensionID1[UsbSetupBuf->wValueH].max;
                                break;
                            case 0x84://GET RES
                                *pDescr = (PUINT8)&ExtensionID1[UsbSetupBuf->wValueH].res;
                                break;
                            case 0x85://GET LEN
                                *pDescr = (PUINT8)&ExtensionID1[UsbSetupBuf->wValueH].len;
                                break;
                            case 0x86:// GET INFO
                                *pDescr = (PUINT8)&ExtensionID1[UsbSetupBuf->wValueH].info;
                                break;
                            case 0x87://GET DEF
                                *pDescr = (PUINT8)&ExtensionID1[UsbSetupBuf->wValueH].def;
                                break;
                        }
                        break;
                    case 06:
                        switch(UsbSetupBuf->bRequest){//LEN->INFO->MIN->MAX->RES->DEF
                            case 0x81://GET CUR
                                *pDescr = (PUINT8)&ExtensionID2[UsbSetupBuf->wValueH].cur;
                                break;
                            case 0x82://GET MIN
                                *pDescr = (PUINT8)&ExtensionID2[UsbSetupBuf->wValueH].min;
                                break;
                            case 0x83://GET MAX
                                *pDescr = (PUINT8)&ExtensionID2[UsbSetupBuf->wValueH].max;
                                break;
                            case 0x84://GET RES
                                *pDescr = (PUINT8)&ExtensionID2[UsbSetupBuf->wValueH].res;
                                break;
                            case 0x85://GET LEN
                                *pDescr = (PUINT8)&ExtensionID2[UsbSetupBuf->wValueH].len;
                                break;
                            case 0x86:// GET INFO
                                *pDescr = (PUINT8)&ExtensionID2[UsbSetupBuf->wValueH].info;
                                break;
                            case 0x87://GET DEF
                                *pDescr = (PUINT8)&ExtensionID2[UsbSetupBuf->wValueH].def;
                                break;
                        }
                        break;
                    }

        if(UsbSetupBuf->wIndexH ==0x01 &&((UsbSetupBuf->wValueH==0x14) ||(UsbSetupBuf->wValueH==0x09)))
        {
                VAICSID[2].cur[0]=(UINT8)InvalidControl;
                return 0xFFFF;
        }
    }
    return 0;
}

/*******************************************************************************
 * @fn      clearError
 *
 * @brief   clear error
 *
 * @return  None
 */
void ClearError( void )
{
    VAICSID[UsbSetupBuf->wValueH].cur[0]=NoError;
}
