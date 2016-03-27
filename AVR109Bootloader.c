/** \file
*
*	Main source file for the CDC class bootloader. This file contains the complete bootloader logic.
*	Based on xboot and the LUFA CRC bootloader.
*/

#define  INCLUDE_FROM_BOOTLOADERCDC_C
#include "AVR109Bootloader.h"



/** Current address counter. This stores the current address of the FLASH or EEPROM as set by the host,
*  and is used when reading or writing to the AVRs memory (either FLASH or EEPROM depending on the issued
*  command.)
*/
static uint32_t CurrAddress; //stored as byte address

static uint16_t TempWord;

/** Flag to indicate if the bootloader should be running, or should exit and allow the application code to run
*  via a watchdog reset. When cleared the bootloader will exit, starting the watchdog and entering an infinite
*  loop until the AVR restarts and the application runs.
*/
static uint8_t RunBootloader = 1;

/** Magic lock for forced application start. If the HWBE fuse is programmed and BOOTRST is unprogrammed, the bootloader
*  will start if the /HWB line of the AVR is held low and the system is reset. However, if the /HWB line is still held
*  low when the application attempts to start via a watchdog reset, the bootloader will re-start. If set to the value
*  \ref MAGIC_BOOT_KEY the special init function \ref Application_Jump_Check() will force the application to start.
*/
static uint16_t MagicBootKey = 0;






/** Reads or writes a block of EEPROM or FLASH memory to or from the appropriate CDC data endpoint, depending
*  on the AVR109 protocol command issued.
*
*  \param[in] Command  Single character AVR109 protocol command indicating what memory operation to perform
*/
static void ReadWriteMemoryBlock(const uint8_t Command)
{
	uint16_t BlockSize;
	char     MemoryType;

	BlockSize  = (FetchNextCommandByte() << 8);
	BlockSize |=  FetchNextCommandByte();

	MemoryType =  FetchNextCommandByte();

	if ((MemoryType != MEMORY_TYPE_FLASH)
	//&& (MemoryType != MEMORY_TYPE_EEPROM) //MC: EEPROM not supported for now
	&& (MemoryType != MEMORY_TYPE_USERSIG)
	&& (MemoryType != MEMORY_TYPE_PRODSIG))
	{
		/* Send error byte back to the host */
		WriteNextResponseByte('?');

		return;
	}

	/* Check if command is to read a memory block */
	if (Command == AVR109_COMMAND_BlockRead)
	{
		//if (MemoryType == MEMORY_TYPE_EEPROM)
		//{
		//uint8_t buffer[SPM_PAGESIZE];
		//eeprom_read_block((void*)&buffer, (const void*)CurrAddress, BlockSize);
		//CurrAddress += BlockSize;
		//}
		//else
		//{
		while (BlockSize--)
		{
			if (MemoryType == MEMORY_TYPE_FLASH)
			{
				WriteNextResponseByte(SP_ReadByte(CurrAddress));
			}
			else if (MemoryType == MEMORY_TYPE_PRODSIG)
			{
				/* Read the next EEPROM byte into the endpoint */
				WriteNextResponseByte(SP_ReadCalibrationByte(CurrAddress));
			}
			else if (MemoryType == MEMORY_TYPE_USERSIG)
			{
				/* Read the next EEPROM byte into the endpoint */
				WriteNextResponseByte(SP_ReadUserSignatureByte(CurrAddress));
			}
			
			
			if (MemoryType == MEMORY_TYPE_EEPROM)
			{
				WriteNextResponseByte(eeprom_read_byte((uint8_t*)&CurrAddress));
			}
			else
			{
				SP_WaitForSPM(); //must have done a flash operation
			}
			
			CurrAddress++;
		}
		//}
	}
	else //BlockWrite
	{
		// Fill up the buffer and then burn the whole page:
		uint8_t buffer[SPM_PAGESIZE];
		
		for (int i = 0; i < SPM_PAGESIZE; ++i)
		{
			if (i < BlockSize)
			buffer[i] = FetchNextCommandByte();
		}

		if (MemoryType == MEMORY_TYPE_EEPROM)
		{
			eeprom_write_block((const void*)&buffer, (void*)&CurrAddress, BlockSize);
		}
		else if (MemoryType == MEMORY_TYPE_FLASH)
		{
			SP_LoadFlashPage((const uint8_t*)&buffer);
			SP_EraseWriteApplicationPage(CurrAddress);
			SP_WaitForSPM();
		}
		else if (MemoryType == MEMORY_TYPE_USERSIG)
		{
			SP_LoadFlashPage((const uint8_t*)&buffer);
			SP_EraseUserSignatureRow();
			SP_WaitForSPM();
			SP_WriteUserSignatureRow();
			SP_WaitForSPM();
		}
		else
		{
			WriteNextResponseByte('?');
		}

		CurrAddress += BlockSize;

		/* Send response byte back to the host */
		WriteNextResponseByte('\r');
	}
}


/** Retrieves the next byte from the host in the CDC data OUT endpoint, and clears the endpoint bank if needed
*  to allow reception of the next data packet from the host.
*
*  \return Next received byte from the host in the CDC data OUT endpoint
*/
static uint8_t FetchNextCommandByte(void)
{
	/* Select the OUT endpoint so that the next data byte can be read */
	Endpoint_SelectEndpoint(BOOTLOADER_RX_EP);

	/* If OUT endpoint empty, clear it and wait for the next packet from the host */
	while (!(Endpoint_IsReadWriteAllowed()))
	{
		Endpoint_ClearOUT();

		while (!(Endpoint_IsOUTReceived()))
		{
			if (USB_DeviceState == DEVICE_STATE_Unattached)
			return 0;
		}
	}

	/* Fetch the next byte from the OUT endpoint */
	return Endpoint_Read_8();
}

/** Writes the next response byte to the CDC data IN endpoint, and sends the endpoint back if needed to free up the
*  bank when full ready for the next byte in the packet to the host.
*
*  \param[in] Response  Next response byte to send to the host
*/
static void WriteNextResponseByte(const uint8_t Response)
{
	/* Select the IN endpoint so that the next data byte can be written */
	Endpoint_SelectEndpoint(BOOTLOADER_TX_EP);

	/* If IN endpoint full, clear it and wait until ready for the next packet to the host */
	if (!(Endpoint_IsReadWriteAllowed()))
	{
		Endpoint_ClearIN();

		while (!(Endpoint_IsINReady()))
		{
			if (USB_DeviceState == DEVICE_STATE_Unattached)
			return;
		}
	}

	/* Write the next byte to the IN endpoint */
	Endpoint_Write_8(Response);
}

/** Task to read in AVR109 commands from the CDC data OUT endpoint, process them, perform the required actions
*  and send the appropriate response back to the host.
*/
static void BootloaderCDC_Task(void)
{
	/* Select the OUT endpoint */
	Endpoint_SelectEndpoint(BOOTLOADER_RX_EP);

	/* Check if endpoint has a command in it sent from the host */
	if (!(Endpoint_IsOUTReceived()))
	return;

	/* Read in the bootloader command (first byte sent from host) */
	uint8_t Command = FetchNextCommandByte();

	if (Command == AVR109_COMMAND_ExitBootloader)
	{
		RunBootloader = 0;

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if ((Command == AVR109_COMMAND_SetLED) || (Command == AVR109_COMMAND_ClearLED) ||
	(Command == AVR109_COMMAND_SelectDeviceType))
	{
		FetchNextCommandByte();

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if ((Command == AVR109_COMMAND_EnterProgrammingMode) || (Command == AVR109_COMMAND_LeaveProgrammingMode))
	{
		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_ReadPartCode)
	{
		WriteNextResponseByte(123); //TODO
		WriteNextResponseByte(0x00);
	}
	else if (Command == AVR109_COMMAND_ReadAutoAddressIncrement)
	{
		/* Indicate auto-address increment is supported */
		WriteNextResponseByte('Y');
	}
	else if (Command == AVR109_COMMAND_SetCurrentAddress)
	{
		/* Set the current address to that given by the host (translate 16-bit word address to byte address) */
		CurrAddress   = (FetchNextCommandByte() << 9);
		CurrAddress  |= (FetchNextCommandByte() << 1);

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_SetExtendedCurrentAddress)
	{
		/* Set the current address to that given by the host (translate 24-bit word address to byte address) */
		CurrAddress   = (FetchNextCommandByte() << 17);
		CurrAddress  |= (FetchNextCommandByte() << 9);
		CurrAddress  |= (FetchNextCommandByte() << 1);

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_ReadBootloaderInterface)
	{
		/* Indicate serial programmer back to the host */
		WriteNextResponseByte('S');
	}
	else if (Command == AVR109_COMMAND_ReadBootloaderIdentifier)
	{
		/* Write the 7-byte software identifier to the endpoint */
		for (uint8_t CurrByte = 0; CurrByte < 7; CurrByte++)
		WriteNextResponseByte(SOFTWARE_IDENTIFIER[CurrByte]);
	}
	else if (Command == AVR109_COMMAND_ReadBootloaderSWVersion)
	{
		WriteNextResponseByte('0' + BOOTLOADER_VERSION_MAJOR);
		WriteNextResponseByte('0' + BOOTLOADER_VERSION_MINOR);
	}
	else if (Command == AVR109_COMMAND_ReadSignature)
	{
		WriteNextResponseByte(SIGNATURE_2);
		WriteNextResponseByte(SIGNATURE_1);
		WriteNextResponseByte(SIGNATURE_0);
	}
	else if (Command == AVR109_COMMAND_EraseFLASH)
	{
		SP_EraseApplicationSection();
		SP_WaitForSPM();
		
		//EEPROM:
		//while (NVM.STATUS & NVM_NVMBUSY_bm) { };
		//NVM.CMD = NVM_CMD_ERASE_EEPROM_gc;
		//NVM_EXEC_WRAPPER();

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	#if !defined(NO_LOCK_BYTE_WRITE_SUPPORT)
	else if (Command == AVR109_COMMAND_WriteLockbits)
	{
		/* Set the lock bits to those given by the host */
		SP_WriteLockBits(FetchNextCommandByte());

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	#endif
	else if (Command == AVR109_COMMAND_ReadLockbits)
	{
		WriteNextResponseByte(SP_ReadLockBits());
	}
	else if (Command == AVR109_COMMAND_ReadLowFuses)
	{
		WriteNextResponseByte(SP_ReadFuseByte(0));
	}
	else if (Command == AVR109_COMMAND_ReadHighFuses)
	{
		WriteNextResponseByte(SP_ReadFuseByte(1));
	}
	else if (Command == AVR109_COMMAND_ReadExtendedFuses)
	{
		WriteNextResponseByte(SP_ReadFuseByte(2));
	}
	#if !defined(NO_BLOCK_SUPPORT)
	else if (Command == AVR109_COMMAND_GetBlockWriteSupport)
	{
		WriteNextResponseByte('Y');

		/* Send block size to the host */
		WriteNextResponseByte((SPM_PAGESIZE >> 8) & 0xFF);
		WriteNextResponseByte(SPM_PAGESIZE & 0xFF);
	}
	else if ((Command == AVR109_COMMAND_BlockWrite) || (Command == AVR109_COMMAND_BlockRead))
	{
		/* Delegate the block write/read to a separate function for clarity */
		ReadWriteMemoryBlock(Command);
	}
	#endif
	#if !defined(NO_FLASH_BYTE_SUPPORT)
	else if (Command == AVR109_COMMAND_FillFlashPageWordHigh)
	{
		TempWord |= FetchNextCommandByte() << 8;

		SP_LoadFlashWord(CurrAddress, TempWord);

		CurrAddress += 2;

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_FillFlashPageWordLow)
	{
		TempWord = FetchNextCommandByte();

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_WriteFlashPage)
	{
		if (CurrAddress >= APP_SECTION_SIZE)
		{
			// don't allow bootloader overwrite
			WriteNextResponseByte('?');
		}
		else
		{
			SP_WriteApplicationPage(CurrAddress);
			WriteNextResponseByte('\r');
		}
	}
	else if (Command == AVR109_COMMAND_ReadFLASHWord)
	{
		uint16_t w = SP_ReadWord(CurrAddress);
		
		WriteNextResponseByte(w >> 8);
		WriteNextResponseByte(w);
		
		CurrAddress += 2;
	}
	#endif
	#if !defined(NO_EEPROM_BYTE_SUPPORT)
	else if (Command == AVR109_COMMAND_WriteEEPROM)
	{
		/* Read the byte from the endpoint and write it to the EEPROM */
		eeprom_write_byte((uint8_t*)((intptr_t)(CurrAddress >> 1)), FetchNextCommandByte());

		/* Increment the address after use */
		CurrAddress += 2;

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_ReadEEPROM)
	{
		/* Read the EEPROM byte and write it to the endpoint */
		WriteNextResponseByte(eeprom_read_byte((uint8_t*)((intptr_t)(CurrAddress >> 1))));

		/* Increment the address after use */
		CurrAddress += 2;
	}
	#endif
	else if (Command != AVR109_COMMAND_Sync)
	{
		/* Unknown (non-sync) command, return fail code */
		WriteNextResponseByte('?');
	}
	

	/* Select the IN endpoint */
	Endpoint_SelectEndpoint(BOOTLOADER_TX_EP);

	/* Remember if the endpoint is completely full before clearing it */
	bool IsEndpointFull = !(Endpoint_IsReadWriteAllowed());

	/* Send the endpoint data to the host */
	Endpoint_ClearIN();

	/* If a full endpoint's worth of data was sent, we need to send an empty packet afterwards to signal end of transfer */
	if (IsEndpointFull)
	{
		while (!(Endpoint_IsINReady()))
		{
			if (USB_DeviceState == DEVICE_STATE_Unattached)
			return;
		}

		Endpoint_ClearIN();
	}

	/* Wait until the data has been sent to the host */
	while (!(Endpoint_IsINReady()))
	{
		if (USB_DeviceState == DEVICE_STATE_Unattached)
		return;
	}

	/* Select the OUT endpoint */
	Endpoint_SelectEndpoint(BOOTLOADER_RX_EP);

	/* Acknowledge the command from the host */
	Endpoint_ClearOUT();
}

