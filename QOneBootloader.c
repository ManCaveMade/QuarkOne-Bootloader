/*
Quark One Bootloader
Copyright (C) Mitchell A. Cox, 2016

mitch [at] enox [dot] co [dot] za
*/

/** \file
*
*  Main source file for the Quark One Bootloader.
*	Based on LUFA Dual CDC Demo.
*/

#include "QOneBootloader.h"

/** LUFA CDC Class driver interface configuration and state information. This structure is
*  passed to all CDC Class driver functions, so that multiple instances of the same class
*  within a device can be differentiated from one another. This is for the first CDC interface,
*	which is the AVR109 CDC bootloader.
*/
USB_ClassInfo_CDC_Device_t AVR109_CDC_Interface =
{
	.Config =
	{
		.ControlInterfaceNumber   = INTERFACE_ID_CDC1_CCI,
		.DataINEndpoint           =
		{
			.Address          = CDC1_TX_EPADDR,
			.Size             = CDC_TXRX_EPSIZE,
			.Banks            = 1,
		},
		.DataOUTEndpoint =
		{
			.Address          = CDC1_RX_EPADDR,
			.Size             = CDC_TXRX_EPSIZE,
			.Banks            = 1,
		},
		.NotificationEndpoint =
		{
			.Address          = CDC1_NOTIFICATION_EPADDR,
			.Size             = CDC_NOTIFICATION_EPSIZE,
			.Banks            = 1,
		},
	},
};

/** LUFA CDC Class driver interface configuration and state information. This structure is
*  passed to all CDC Class driver functions, so that multiple instances of the same class
*  within a device can be differentiated from one another. This is for the second CDC interface,
*	which is a transparent bridge to the UART that the ESP-01 is connected to.
*/
USB_ClassInfo_CDC_Device_t ESP_CDC_Interface =
{
	.Config =
	{
		.ControlInterfaceNumber   = INTERFACE_ID_CDC2_CCI,
		.DataINEndpoint           =
		{
			.Address          = CDC2_TX_EPADDR,
			.Size             = CDC_TXRX_EPSIZE,
			.Banks            = 1,
		},
		.DataOUTEndpoint =
		{
			.Address          = CDC2_RX_EPADDR,
			.Size             = CDC_TXRX_EPSIZE,
			.Banks            = 1,
		},
		.NotificationEndpoint =
		{
			.Address          = CDC2_NOTIFICATION_EPADDR,
			.Size             = CDC_NOTIFICATION_EPSIZE,
			.Banks            = 1,
		},

	},
};

//USART_t USART_Interface =
//{

//};

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
//static uint16_t MagicBootKey = 0;


int main(void)
{
	SetupHardware();

	RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
	RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));

	GlobalInterruptEnable();

	uint16_t flicker = 0;

	while (RunBootloader)
	{
		BootloaderCDC_Task();
		CDC_Device_USBTask(&AVR109_CDC_Interface);
		
		USBToUSART_Task();
		CDC_Device_USBTask(&ESP_CDC_Interface);
		
		USB_USBTask();
		
		/*flicker++;
		if (flicker >= 1000)
		{
			flicker = 0;
		}
		if (flicker < 100)
		{
			QuarkOneSetLEDOn();
		}
		else
		{
			QuarkOneSetLEDOff();
		}*/
	}
	
	//if we get here, reboot to the application
	
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;

	QuarkOnePinSetup();

	/* Hardware Initialization */
	Serial_Init(&QuarkOne_ESP_USART, 115200, false, true); //MC: modified for interrupts

	//QuarkOneSetLEDOn();
	for (uint8_t i = 0; i < 10; ++i)
	{
		Delay_MS(100);
		QuarkOneSetLEDToggle();
	}
	QuarkOneSetLEDOff();

	//MC: The DTR signals mess with the mode at connect for some reason.
	//QuarkOneSetESP_FlashMode();
	QuarkOneSetESP_NormalMode();

	//gpio_set_pin_low(QuarkOne_LED);
	USB_Init();
}



/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	//QuarkOneSetLEDOn();
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	//QuarkOneSetLEDOff();
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	CDC_Device_ConfigureEndpoints(&AVR109_CDC_Interface);
	CDC_Device_ConfigureEndpoints(&ESP_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&AVR109_CDC_Interface);
	CDC_Device_ProcessControlRequest(&ESP_CDC_Interface);
}

/*
* -------------- USB to USART Bridge for ESP-01 ------------------------------
*/

/** CDC class driver callback function the processing of changes to the virtual
*  control lines sent from the host..
*
*  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
*/
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo)
{
	if ((CDCInterfaceInfo == &ESP_CDC_Interface))
	{
		//DTR -> GPIO0
		if ((CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR))
		{
			//QuarkOne_ESP_GPIO0_2_PORT.OUTSET = QuarkOne_ESP_GPIO0;
			
			//RTS -> RST
			if ((CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_RTS))
			{
				QuarkOneSetESP_FlashMode();
				//CDC_Device_SendString(&ESP_CDC_Interface, "Quark One ESP Flash Mode\n");

				//QuarkOne_ESP_CH_PD_RST_PORT.OUTSET = QuarkOne_ESP_RST;
			}
			//else
			//{
			//QuarkOne_ESP_CH_PD_RST_PORT.OUTCLR = QuarkOne_ESP_RST;
			//QuarkOneSetESP_NormalMode(); //reboot to go back to normal
			//}
		}
		//else
		//{
		//QuarkOne_ESP_GPIO0_2_PORT.OUTCLR = QuarkOne_ESP_GPIO0;
		//}
	}

}

/** Event handler for the CDC Class driver Line Encoding Changed event.
*
*  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
*/
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	if (CDCInterfaceInfo == &ESP_CDC_Interface)
	{
		// we can change things like the baud rate, etc. to match the usb cdc settings
		// For now, we only keep the baud rate 'synced'
		//Serial_Init(&QuarkOne_ESP_USART, CDCInterfaceInfo->State.LineEncoding.BaudRateBPS, false, true);
		
		//Toggle the ESP-01 reset so that it detects the change
		/*if (QuarkOne_ESP_CH_PD_RST_PORT.IN & QuarkOne_ESP_RST)
		{
		QuarkOne_ESP_CH_PD_RST_PORT.OUTCLR = QuarkOne_ESP_RST;
		Delay_MS(1);
		QuarkOne_ESP_CH_PD_RST_PORT.OUTSET = QuarkOne_ESP_RST;
		}*/
	}
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
*  for later transmission to the host.
*/

ISR(QuarkOne_ESP_USART_RX_vect)
{
	uint8_t ReceivedByte = (&QuarkOne_ESP_USART)->DATA;
	//
	if ((USB_DeviceState == DEVICE_STATE_Configured) && !(RingBuffer_IsFull(&USARTtoUSB_Buffer)))
	{
		//QuarkOneSetLEDOn();
		RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
		//CDC_Device_SendByte(&ESP_CDC_Interface, ReceivedByte);
		//QuarkOneSetLEDOff();
	}
}

/*
* Called when the RX data register is empty (all chars have been transmitted). We need to load up some more characters if we have any.
*/
ISR(QuarkOne_ESP_USART_TX_DRE_vect)
{
	/* Load the next few bytes from the USART transmit buffer into the USART if transmit buffer space is available */
	while ((Serial_IsSendReady(&QuarkOne_ESP_USART) && !(RingBuffer_IsEmpty(&USBtoUSART_Buffer))))
	{
		QuarkOneSetLEDOn();
		Serial_SendByte(&QuarkOne_ESP_USART, RingBuffer_Remove(&USBtoUSART_Buffer));
		QuarkOneSetLEDOff();
	}
}

static void USBToUSART_Task(void)
{
	//USB to USART
	//Do as many bytes as possible (probably less than 16 at a time so errors shouldn't happen)
	uint16_t bytesReceived = CDC_Device_BytesReceived(&ESP_CDC_Interface);
	
	while ((bytesReceived--) && !(RingBuffer_IsFull(&USBtoUSART_Buffer)))
	{
		RingBuffer_Insert(&USBtoUSART_Buffer, CDC_Device_ReceiveByte(&ESP_CDC_Interface));
	}

	/*if (RingBuffer_IsFull(&USBtoUSART_Buffer))
	{
	CDC_Device_SendString(&ESP_CDC_Interface, "Buffer overrun in USB to USART!\n");
	//QuarkOneSetLEDOn(); //error
	}*/


	//USART to USB
	uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
	if (BufferCount)
	{
		//Endpoint_SelectEndpoint(ESP_CDC_Interface.Config.DataINEndpoint.Address);

		//CDC_Device_SendString(&ESP_CDC_Interface, "Sending!\n");

		/* Check if a packet is already enqueued to the host - if so, we shouldn't try to send more data
		* until it completes as there is a chance nothing is listening and a lengthy timeout could occur */
		//if (Endpoint_IsINReady())
		//{
		/* Never send more than one bank size less one byte to the host at a time, so that we don't block
		* while a Zero Length Packet (ZLP) to terminate the transfer is sent if the host isn't listening */
		uint8_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));

		/* Read bytes from the USART receive buffer into the USB IN endpoint */
		while (BytesToSend--)
		{
			// Try to send the next byte of data to the host, abort if there is an error, without dequeuing
			CDC_Device_SendByte(&ESP_CDC_Interface,	RingBuffer_Remove(&USARTtoUSB_Buffer));
		}
		//
		//}
	}
	
	
}


/*
* --------------- Bootloader --------------------------------------------------
*/

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
	int16_t ReceivedByte = CDC_Device_ReceiveByte(&AVR109_CDC_Interface);
	
	if (ReceivedByte != -1)
	{
		return (uint8_t)ReceivedByte;
	}
	
	return 0;
	
	//Stuff below is without CDC driver
	/* Select the OUT endpoint so that the next data byte can be read */
	//Endpoint_SelectEndpoint(BOOTLOADER_RX_EP);

	/* If OUT endpoint empty, clear it and wait for the next packet from the host */
	//while (!(Endpoint_IsReadWriteAllowed()))
	//{
	//	Endpoint_ClearOUT();

	//	while (!(Endpoint_IsOUTReceived()))
	//	{
	//		if (USB_DeviceState == DEVICE_STATE_Unattached)
	//		return 0;
	//	}
	//}

	/* Fetch the next byte from the OUT endpoint */
	//return Endpoint_Read_8();
}

/** Writes the next response byte to the CDC data IN endpoint, and sends the endpoint back if needed to free up the
*  bank when full ready for the next byte in the packet to the host.
*
*  \param[in] Response  Next response byte to send to the host
*/
static void WriteNextResponseByte(const uint8_t Response)
{
	/* Select the IN endpoint so that the next data byte can be written */
	//Endpoint_SelectEndpoint(BOOTLOADER_TX_EP);
	//Endpoint_SelectEndpoint(AVR109_CDC_Interface.Config.DataINEndpoint.Address);

	//if (Endpoint_IsINReady())
	//{
	CDC_Device_SendByte(&AVR109_CDC_Interface, Response);
	//}

	/* If IN endpoint full, clear it and wait until ready for the next packet to the host */
	//if (!(Endpoint_IsReadWriteAllowed()))
	//{
	//	Endpoint_ClearIN();

	//	while (!(Endpoint_IsINReady()))
	//	{
	//		if (USB_DeviceState == DEVICE_STATE_Unattached)
	//		return;
	//	}
	//}

	/* Write the next byte to the IN endpoint */
	//Endpoint_Write_8(Response);
}

/** Task to read in AVR109 commands from the CDC data OUT endpoint, process them, perform the required actions
*  and send the appropriate response back to the host.
*/
static void BootloaderCDC_Task(void)
{
	/* Select the OUT endpoint */
	//Endpoint_SelectEndpoint(BOOTLOADER_RX_EP);

	/* Check if endpoint has a command in it sent from the host */
	//if (!(Endpoint_IsOUTReceived()))
	//return;

	/* Read in the bootloader command (first byte sent from host) */
	uint8_t Command = FetchNextCommandByte();

	if (Command == 0)
	{
		return;
	}
	

	if (Command == AVR109_COMMAND_ExitBootloader)
	{
		RunBootloader = 0;

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
	}
	else if (Command == AVR109_COMMAND_SetLED)
	{
		QuarkOneSetLEDOn();
	}
	else if (Command == AVR109_COMMAND_ClearLED)
	{
		QuarkOneSetLEDOff();
	}
	else if (Command == AVR109_COMMAND_SelectDeviceType)
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
		QuarkOneSetLEDOn();
		SP_EraseApplicationSection();
		SP_WaitForSPM();
		
		//EEPROM:
		//while (NVM.STATUS & NVM_NVMBUSY_bm) { };
		//NVM.CMD = NVM_CMD_ERASE_EEPROM_gc;
		//NVM_EXEC_WRAPPER();

		/* Send confirmation byte back to the host */
		WriteNextResponseByte('\r');
		QuarkOneSetLEDOff();
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
			QuarkOneSetLEDOn();
			SP_WriteApplicationPage(CurrAddress);
			WriteNextResponseByte('\r');
			QuarkOneSetLEDOff();
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
	else
	{
		WriteNextResponseByte('?');
	}
	

	/* Select the IN endpoint */
	//Endpoint_SelectEndpoint(BOOTLOADER_TX_EP);

	/* Remember if the endpoint is completely full before clearing it */
	//bool IsEndpointFull = !(Endpoint_IsReadWriteAllowed());

	/* Send the endpoint data to the host */
	//Endpoint_ClearIN();

	/* If a full endpoint's worth of data was sent, we need to send an empty packet afterwards to signal end of transfer */
	//if (IsEndpointFull)
	//{
	//	while (!(Endpoint_IsINReady()))
	//	{
	//		if (USB_DeviceState == DEVICE_STATE_Unattached)
	//		return;
	//	}

	//	Endpoint_ClearIN();
	//}

	/* Wait until the data has been sent to the host */
	//while (!(Endpoint_IsINReady()))
	//{
	//	if (USB_DeviceState == DEVICE_STATE_Unattached)
	//	return;
	//}

	/* Select the OUT endpoint */
	//Endpoint_SelectEndpoint(BOOTLOADER_RX_EP);

	/* Acknowledge the command from the host */
	//Endpoint_ClearOUT();
}