/** \file
*
*  Header file for QOneBootloader.c.
*/

#ifndef _QONE_BOOTLOADER_H_
#define _QONE_BOOTLOADER_H_

/* Includes: */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#include "QuarkOne.h"

#include "Descriptors.h"

#include <LUFA/Drivers/Misc/RingBuffer.h>
#include <LUFA/Drivers/Peripheral/Serial.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

/* Macros: */


/* Function Prototypes: */
void SetupHardware(void);

void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);

// USB to Serial (ESP-01) Stuff: ----------------------------------------------

static void USBToUSART_Task(void);



// Bootloader Stuff: ----------------------------------------------------------

#include "sp_driver.h"
#include <avr/eeprom.h>

/* Preprocessor Checks: */
#if !defined(__OPTIMIZE_SIZE__)
//#error This bootloader requires that it be optimized for size, not speed, to fit into the target device. Change optimization settings and try again.
#endif


/* Macros: */
/** Version major of the CDC bootloader. */
#define BOOTLOADER_VERSION_MAJOR     0x01

/** Version minor of the CDC bootloader. */
#define BOOTLOADER_VERSION_MINOR     0x00

/** Hardware version major of the CDC bootloader. */
#define BOOTLOADER_HWVERSION_MAJOR   0x01

/** Hardware version minor of the CDC bootloader. */
#define BOOTLOADER_HWVERSION_MINOR   0x00

/** Eight character bootloader firmware identifier reported to the host when requested. */
#define SOFTWARE_IDENTIFIER          "QONECDC"

/** Magic bootloader key to unlock forced application start mode. */
#define MAGIC_BOOT_KEY               0xDC42


#define BOOTLOADER_RX_EP	CDC1_RX_EPADDR
#define BOOTLOADER_TX_EP	CDC1_TX_EPADDR

/* Enums: */
/** Possible memory types that can be addressed via the bootloader. */
enum AVR109_Memories
{
	MEMORY_TYPE_FLASH  = 'F',
	MEMORY_TYPE_EEPROM = 'E',
	MEMORY_TYPE_USERSIG = 'U',
	MEMORY_TYPE_PRODSIG = 'P'
};

/** Possible commands that can be issued to the bootloader. */
enum AVR109_Commands
{
	AVR109_COMMAND_Sync                     = 27,
	AVR109_COMMAND_ReadEEPROM               = 'd',
	AVR109_COMMAND_WriteEEPROM              = 'D',
	AVR109_COMMAND_ReadFLASHWord            = 'R',
	AVR109_COMMAND_WriteFlashPage           = 'm',
	AVR109_COMMAND_FillFlashPageWordLow     = 'c',
	AVR109_COMMAND_FillFlashPageWordHigh    = 'C',
	AVR109_COMMAND_GetBlockWriteSupport     = 'b',
	AVR109_COMMAND_BlockWrite               = 'B',
	AVR109_COMMAND_BlockRead                = 'g',
	AVR109_COMMAND_ReadExtendedFuses        = 'Q',
	AVR109_COMMAND_ReadHighFuses            = 'N',
	AVR109_COMMAND_ReadLowFuses             = 'F',
	AVR109_COMMAND_ReadLockbits             = 'r',
	AVR109_COMMAND_WriteLockbits            = 'l',
	AVR109_COMMAND_EraseFLASH               = 'e',
	AVR109_COMMAND_ReadSignature            = 's',
	AVR109_COMMAND_ReadBootloaderSWVersion  = 'V',
	AVR109_COMMAND_ReadBootloaderHWVersion  = 'v',
	AVR109_COMMAND_ReadBootloaderIdentifier = 'S',
	AVR109_COMMAND_ReadBootloaderInterface  = 'p',
	AVR109_COMMAND_SetCurrentAddress        = 'A',
	AVR109_COMMAND_SetExtendedCurrentAddress = 'H',
	AVR109_COMMAND_ReadAutoAddressIncrement = 'a',
	AVR109_COMMAND_ReadPartCode             = 't',
	AVR109_COMMAND_EnterProgrammingMode     = 'P',
	AVR109_COMMAND_LeaveProgrammingMode     = 'L',
	AVR109_COMMAND_SelectDeviceType         = 'T',
	AVR109_COMMAND_SetLED                   = 'x',
	AVR109_COMMAND_ClearLED                 = 'y',
	AVR109_COMMAND_ExitBootloader           = 'E',
};

/* Function Prototypes: */
static void BootloaderCDC_Task(void);

static void    ReadWriteMemoryBlock(const uint8_t Command);

static uint8_t FetchNextCommandByte(void);
static void    WriteNextResponseByte(const uint8_t Response);


#endif

