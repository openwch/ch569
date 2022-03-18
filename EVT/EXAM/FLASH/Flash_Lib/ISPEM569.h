/********************************** (C) COPYRIGHT *******************************
* File Name          : ISPEM569.h
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description     : for the chip only USER code area, no BOOT
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
*******************************************************************************/

/* V1.1 FlashROM library for BOOT */
/* 1. for the chip only USER code area, no BOOT */
/* 2. for the target in BOOT area on the chip divided into USER code area and BOOT area */
/* 用于没有引导区、只有用户代码区的情况，可以在用户代码中被调用（IAP，擦写自身）。
   或用于具有用户代码区和引导区的芯片、操作目标为引导代码区的情况，只能在引导代码中被调用（更新自身） */

/* Flash-ROM feature:
     for store program code, support block erasing, dword and page writing, dword verifying, unit for Length is byte,
     minimal quantity for write or verify is one dword (4-bytes),
     256 bytes/page for writing, FLASH_ROM_WRITE support one dword or more dword writing, but multiple of 256 is the best,
     4KB (4096 bytes) bytes/block for erasing, so multiple of 4096 is the best */

/* Data-Flash(EEPROM) feature:
     for store data, support block erasing, byte and page writing, byte reading,
     minimal quantity for write or read is one byte,
     256 bytes/page for writing, EEPROM_WRITE support one byte or more byte writing, but multiple of 256 is the best,
     4KB (4096 bytes) bytes/block for erasing, so multiple of 4096 is the best */

#ifndef EEPROM_PAGE_SIZE
#define EEPROM_PAGE_SIZE    256                       // Flash-ROM & Data-Flash page size for writing
#define EEPROM_BLOCK_SIZE   4096                      // Flash-ROM & Data-Flash block size for erasing
#define EEPROM_MIN_ER_SIZE  EEPROM_BLOCK_SIZE         // Flash-ROM & Data-Flash minimal size for erasing
#define EEPROM_MIN_WR_SIZE  1                         // Data-Flash minimal size for writing
#define EEPROM_MAX_SIZE     0x8000                    // Data-Flash maximum size, 32KB
#endif
#ifndef FLASH_MIN_WR_SIZE
#define FLASH_MIN_WR_SIZE   4                         // Flash-ROM minimal size for writing
#endif
#ifndef FLASH_ROM_MAX_SIZE
#define FLASH_ROM_MAX_SIZE  0x070000                  // Flash-ROM maximum program size, 448KB
#endif

extern VOID GET_UNIQUE_ID( PVOID Buffer );  // get 64 bit unique ID

extern VOID FLASH_ROM_PWR_DOWN( VOID );  // power-down FlashROM

extern VOID FLASH_ROM_PWR_UP( VOID );  // power-up FlashROM

extern VOID EEPROM_READ( UINT32 StartAddr, PVOID Buffer, UINT32 Length );  // read Data-Flash data block

extern UINT8 EEPROM_ERASE( UINT32 StartAddr, UINT32 Length );  // erase Data-Flash block, minimal block is 4KB, return 0 if success

extern UINT8 EEPROM_WRITE( UINT32 StartAddr, PVOID Buffer, UINT32 Length );  // write Data-Flash data block, return 0 if success

extern UINT8 FLASH_ROMA_ERASE( UINT32 StartAddr, UINT32 Length );  // erase FlashROM block, minimal block is 4KB, return 0 if success

extern UINT8 FLASH_ROMA_WRITE( UINT32 StartAddr, PVOID Buffer, UINT32 Length );  // write FlashROM data block, minimal block is dword, return 0 if success

extern UINT32 FLASH_ROMA_VERIFY( UINT32 StartAddr, PVOID Buffer, UINT32 Length );  // verify FlashROM data block, minimal block is dword, return 0 if success

extern UINT8 FLASH_ROMA_READ( UINT32 StartAddr, PVOID Buffer, UINT32 Length );  // read FlashROM data block, minimal block is dword, return 0 if success

extern UINT8 FLASH_ROMA_LOCK( UINT8 LockFlag );  // unlock FlashROM block, return 0 if success
/* LockFlag: not care */
