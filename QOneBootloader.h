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
void StartApplication(void);

void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);

// USB to Serial (ESP-01) Stuff: ----------------------------------------------

static void USBToUSART_Task(void);



// Bootloader Stuff: ----------------------------------------------------------

#include "sp_driver.h"
//#include <avr/eeprom.h>
#include "eeprom_driver.h"

/* Preprocessor Checks: */
#if !defined(__OPTIMIZE_SIZE__)
//#error This bootloader requires that it be optimized for size, not speed, to fit into the target device. Change optimization settings and try again.
#endif


/* Macros: */
/** Version major of the CDC bootloader. */
#define BOOTLOADER_VERSION_MAJOR     0x01

/** Version minor of the CDC bootloader. */
#define BOOTLOADER_VERSION_MINOR     0x00

/** Eight character bootloader firmware identifier reported to the host when requested. */
#define SOFTWARE_IDENTIFIER          "QOUSBBL"



#define BOOTLOADER_RX_EP	CDC1_RX_EPADDR
#define BOOTLOADER_TX_EP	CDC1_TX_EPADDR

// Variables:

/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
static RingBuffer_t USBtoUSART_Buffer;

/** Underlying data buffer for \ref USBtoUSART_Buffer, where the stored bytes are located. */
static uint8_t      USBtoUSART_Buffer_Data[128];

/** Circular buffer to hold data from the serial port before it is sent to the host. */
static RingBuffer_t USARTtoUSB_Buffer;

/** Underlying data buffer for \ref USARTtoUSB_Buffer, where the stored bytes are located. */
static uint8_t      USARTtoUSB_Buffer_Data[128];

/** Current address counter. This stores the current address of the FLASH or EEPROM as set by the host,
*  and is used when reading or writing to the AVRs memory (either FLASH or EEPROM depending on the issued
*  command.)
*/
static uint32_t CurrAddress = 0; //stored as byte address

static uint16_t TempWord;

/** Flag to indicate if the bootloader should be running, or should exit and allow the application code to run
*  via a watchdog reset. When cleared the bootloader will exit, starting the watchdog and entering an infinite
*  loop until the AVR restarts and the application runs.
*/
static bool RunBootloader = true;
static bool LEDFlicker = false;

//bootloaderSpice will init to zero, but after a software reboot it will not 
//reinitialize. If the value is set to BOOTLOADER_SPICE then we should run the BL.
#define BOOTLOADER_SPICE 0x66
#define BOOTLOADER_SPICE_EEPROM_PAGE 0x00
#define BOOTLOADER_SPICE_EEPROM_BYTE 0x00
static uint8_t bootloaderSpice = 0; //This should only be modified by the Get, Set and Clear functions, below.

#define EXT_RESET_TIMEOUT_VALUE 15 //Timer ticks every 50ms. 15*50 = 750ms
volatile uint16_t resetTimeout = 0;

static uint8_t LEDPulseCount = 0;


/* Enums: */
/** Possible memory types that can be addressed via the bootloader. */
enum AVR911_Memories
{
	MEMORY_TYPE_FLASH  = 'F',
	MEMORY_TYPE_EEPROM = 'E',
	MEMORY_TYPE_USERSIG = 'U',
	MEMORY_TYPE_PRODSIG = 'P'
};

////////////////////////////////
/*          COMMANDS          */
////////////////////////////////
// 'a'     = Check auto-increment status
// 'A'     = Set address, two parameters: <high byte>, <low byte>
// 'e'     = Erase Application Section and EEPROM
// 'b'     = Check block load support, returns BLOCKSIZE (2 bytes)
// 'B'     = Start block load, three parameters: block size (<high byte>,<low byte>),memtype
// 'g'     = Start block read, three parameters: block size (<high byte>,<low byte>),memtype
// 'R'     = Read program memory, returns high byte then low byte of flash word
// 'c'     = Write program memory, one parameter: low byte, returns '\r'
// 'C'     = Write program memory, one parameter: high byte, returns '\r'
// 'm'     = Write page, returns '?' if page is protected, returns '\r' if done
// 'D'     = Write EEPROM, one parameter: byte to write
// 'd'     = Read EEPROM, returns one byte
// 'l'     = Write lock bits, returns '\r'
// 'r'     = Read lock bits
// 'F'     = Read low fuse bits
// 'N'     = Read high fuse bits
// 'Q'     = Read extended fuse bits
// 'P'     = Enter and leave programming mode, returns '\r'
// 'L'     = Enter and leave programming mode, returns '\r'
// 'E'     = Exit bootloader, returns '\r', jumps to 0x0000
// 'p'     = Get programmer type, returns 'S'
// 't'     = Return supported device codes, returns PARTCODE and 0
// 'x'     = Turn on LED0, returns '\r'
// 'y'     = Turn off LED0, returns '\r'
// 'T'     = Set device type, one parameter: device byte, returns '\r'
// 'S'     = Returns Xmega_Bootloader
// 'V'     = Returns version number
// 's'     = Return signature bytes, returns 3 bytes (sig3, sig2, sig1)
// 0x1b    = ESC
// Unknown = '?'

enum AVR911_Commands
{
	COMMAND_Escape                   = 0x1b,
	COMMAND_ReadEEPROM               = 'd',
	COMMAND_WriteEEPROM              = 'D',
	COMMAND_ReadFLASHWord            = 'R',
	COMMAND_WriteFlashPage           = 'm',
	COMMAND_FillFlashPageWordLow     = 'c',
	COMMAND_FillFlashPageWordHigh    = 'C',
	COMMAND_GetBlockWriteSupport     = 'b',
	COMMAND_BlockWrite               = 'B',
	COMMAND_BlockRead                = 'g',
	COMMAND_ReadExtendedFuses        = 'Q',
	COMMAND_ReadHighFuses            = 'N',
	COMMAND_ReadLowFuses             = 'F',
	COMMAND_ReadLockbits             = 'r',
	COMMAND_WriteLockbits            = 'l',
	COMMAND_EraseFLASH               = 'e',
	COMMAND_ReadSignature            = 's',
	COMMAND_ReadBootloaderSWVersion  = 'V',
	COMMAND_ReadBootloaderHWVersion  = 'v',
	COMMAND_ReadBootloaderIdentifier = 'S',
	COMMAND_ReadBootloaderInterface  = 'p',
	COMMAND_SetCurrentAddress        = 'A',
	COMMAND_ReadAutoAddressIncrement = 'a',
	COMMAND_ReadPartCode             = 't',
	COMMAND_EnterProgrammingMode     = 'P',
	COMMAND_LeaveProgrammingMode     = 'L',
	COMMAND_SelectDeviceType         = 'T',
	COMMAND_SetLED                   = 'x',
	COMMAND_ClearLED                 = 'y',
	COMMAND_ExitBootloader           = 'E',
};

enum AVR911_Responses
{
	RESPONSE_OKAY              = '\r',
	RESPONSE_YES               = 'Y',
//	RESPONSE_NO                = 'N', //Not used
	RESPONSE_UNKNOWN           = '?'
};

/* Function Prototypes: */
static void BootloaderCDC_Task(void);

//static void    ReadWriteMemoryBlock(const uint8_t Command);

static uint8_t BlockLoad(uint16_t size, uint8_t mem, uint32_t *address);
static void BlockRead(uint16_t size, uint8_t mem, uint32_t *address);

static uint8_t FetchNextCommandByte(void);
static void    WriteNextResponseByte(const uint8_t Response);

static inline void GetBootloaderSpice(void) ATTR_ALWAYS_INLINE;
static inline void SetBootloaderSpice(void) ATTR_ALWAYS_INLINE;
static inline void ClearBootloaderSpice(void) ATTR_ALWAYS_INLINE;

static void ResetIntoBootloader(void); 
static void LaunchBootloader(void);
static void LaunchApplication(void);

#endif

