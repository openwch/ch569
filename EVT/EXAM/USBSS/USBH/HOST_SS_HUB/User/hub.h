#ifndef __HUB_H
#define __HUB_H
#define    HUB_ERR_SCUESS       0x02
#define    HUB_ERR_CONNECT      0x01
#define    HUB_ERR_DISCONNECT   0x00
#define    HUB_CS_NAK           0x2a
#define    DEVICE_ADDR          0x08                                        /* U30 device address*/

#define MAX_HUBNUMPORT 4

 typedef struct  __attribute__((packed)) _HUB_Endp_Info {
     UINT8 num;          // Endpoint number
     UINT8 endptype;     // Endpoint Type:1-IOS, 2-BULK, 3-INT
     UINT8 tog;
     UINT8 HighTransNum; // Number of transactions within a microframe(<=3) USB2.0
     UINT16 endp_size;   // Endpoint size
 } HUB_Endp_Info, *PHUB_Enpd_Info;


 typedef struct  __attribute__((packed)) _HUB_Port_Info {
     UINT8 status;          //0x01:Device connection
     UINT8 speed;           // 0-fullspeed, 1-highspeed, 2-lowspeed,3-sspeed
     UINT8 port_num;        // Endpoint number
     UINT8 addr;            //HUB The following device address
     UINT8 devicetype;      //Device type
     UINT8 endpnum;         //Number of endpoints
     UINT8 portpchangefield;//Port state change
     HUB_Endp_Info portEndp[8];
 } HUB_Port_Info, *PHUB_Port_Info;

 typedef struct  __attribute__((packed)) _USB_HUB_Info {
     UINT8 status;      // 0-disconnect  1-connect  2-config
     UINT8 speed;       // 0-fullspeed, 1-highspeed, 2-lowspeed,3-sspeed
     UINT8 devaddr;     //hub address
     UINT8 endpsize;    //HUB Endpoint size
     UINT8 numofport;   //hub Number of ports
     UINT8 device_type; //Device type
     UINT8 time_out;
     HUB_Endp_Info rootEndp[2];
     HUB_Port_Info portD[MAX_HUBNUMPORT];
 } USB_HUB_Info, *PUSB_HUB_Info;

 typedef struct  __attribute__((packed)) _USB_HUB_SaveData {

     UINT8 Depth;
     UINT8 UpLevelPort;
     UINT8 CurrentPort;
     USB_HUB_Info HUB_Info;

 } USB_HUB_SaveData;

 //HUB structure of nodes in a data linked list
 typedef struct __attribute__((packed)) _Link_HUBSaveData
 {
     USB_HUB_SaveData HUB_SaveData;
     struct _Link_HUBSaveData* next;
 }Link_HUBSaveData;

 extern Link_HUBSaveData* Hub_LinkHead;
 Link_HUBSaveData* InitHubLink(void);

 void AssignHubData(USB_HUB_SaveData *hubdata ,UINT8 depth,UINT8 uplevelport,UINT8 currentport,USB_HUB_Info hub_info);
 UINT8 SearchHubData(Link_HUBSaveData* p, USB_HUB_SaveData *HUB_SaveData);
 UINT8 InsertHubData(Link_HUBSaveData* p, USB_HUB_SaveData HUB_SaveData);
 UINT8 DeleteHubData(Link_HUBSaveData* p, USB_HUB_SaveData HUB_SaveData);
 UINT8 ModifyHubData(Link_HUBSaveData* p, USB_HUB_SaveData oldhubdata, USB_HUB_SaveData newhubdata);
 UINT8  Hublink_judgingstructures( USB_HUB_SaveData hubdata );
 void Hublink_finesubstructures( USB_HUB_SaveData hubdata );
 extern UINT8 AddressNum[127];
 UINT8 USB_SetAddressNumber(void);
 UINT8 USB_DelAddressNumber(UINT8 addr);

 extern __attribute__ ((aligned(16))) UINT8 pNTFS_BUF[512] __attribute__((section(".DMADATA")));


#define     PORT_DISCONNECT         0x00        //The device is disconnected or not connected
#define     PORT_CONNECTION         0x01        //Device connection
#define     PORT_INIT               0x02        //Device connection, enumerated
#define     PORT_INIT_OK            0x03        //Device connection, enumerated, class command initialization completed

#define     PORT_RESET                  4         //Reset the device under the port
#define     PORT_POWER                  8         //HUB power on
#define     C_PORT_CONNECTION           16        //Port change status
#define     C_PORT_ENABLE               17        //device enable
#define     C_PORT_SUSPEND              18        //Device Pending
#define     C_PORT_OVER_CURRENT         19        //Overcurrent state clearing
#define     C_PORT_RESET                20        //Port change status
#define     C_BH_PORT_RESET             29        //Port change status
#define     C_PORT_LINK_STATE           25        //Port change status
#define     C_PORT_CONFIG_ERROR         26        //Port change status
#define     PORT_REMOTE_WAKE_MASK       27

/*HUB The following device speed types*/
#define     PORT_FULL_SPEED     0X00           //full speed
#define     PORT_HIGH_SPEED     0X01           //high speed
#define     PORT_LOW_SPEED      0X02           //low speed
#define     PORT_SS_SPEED       0X03           //super speed

typedef enum
{
    SS_HUBNUM = 5,
    HS_HUBNUM = 7,
}HUBNUM;



extern USB_HUB_Info ss_hub_info[SS_HUBNUM];
extern USB_HUB_Info hs_hub_info[HS_HUBNUM];
extern UINT8 U30HOST_Hub_Connect_Process( UINT8 depth );
extern void U30HOST_Hub_Device_Process( UINT8 depth,UINT8 port );
extern UINT8 U20HOST_Hub_Connect_Process( UINT8 depth );
extern UINT8 U20USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT32 timeout );


#endif
