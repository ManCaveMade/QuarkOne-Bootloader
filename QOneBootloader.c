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
USB_ClassInfo_CDC_Device_t CDC_Interface =
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




int main(void)
{
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);
	
	//Make a decision about whether to go into the bootloader or not
	
	//Go into the bootloader if there was a software reset and the spice is there
	//Go into bootloader if flash is unprogrammed
	//Otherwise continue to the application
	
	//SP_WaitForSPM();
	if ((pgm_read_word(0) == 0xFFFF)
	|| ((RST.STATUS & RST_SRF_bm) && (bootloaderSpice == BOOTLOADER_SPICE))
	|| (RST.STATUS & RST_EXTRF_bm)
	|| true)
	{
		DoBootloader();
	}
	//RST.STATUS = RST_SRF_bm | RST_PDIRF_bm | RST_WDRF_bm | RST_BORF_bm | RST_EXTRF_bm | RST_PORF_bm; //clear the flags
		
	//Bootloader is finished, so reboot to the app
	RST.STATUS = RST_SRF_bm;
	bootloaderSpice = 0;
	EIND = 0x00;
	void (*reset_vect)( void ) = 0x000000;
	reset_vect();
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	CCP = CCP_IOREG_gc;
	PMIC.CTRL = PMIC_IVSEL_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;

	QuarkOnePinSetup();
	
	/* Further Hardware Initialization which is only done if we are going into the bootloader */
	Serial_Init(&QuarkOne_ESP_USART, 115200, false, true); //MC: modified for interrupts

	for (uint8_t i = 0; i < 5; ++i)
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

static void DoBootloader(void)
{
	RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
	RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));
	
	SetupHardware();

	GlobalInterruptEnable();

	uint16_t flicker = 0;

	while (RunBootloader)
	{
		BootloaderCDC_Task();
		CDC_Device_USBTask(&CDC_Interface);
		
		USBToUSART_Task();
		CDC_Device_USBTask(&ESP_CDC_Interface);
		
		USB_USBTask();
		
		flicker++;
		if (flicker >= 1000)
		{
			flicker = 0;
		}
		if (flicker < 50)
		{
			QuarkOneSetLEDOn();
		}
		else
		{
			QuarkOneSetLEDOff();
		}
	}
}

static void ResetIntoBootloader(void)
{
	GlobalInterruptDisable();
	PMIC.CTRL = 0;
	bootloaderSpice = BOOTLOADER_SPICE;
	CCP = CCP_IOREG_gc;
	RST.CTRL = RST_SWRST_bm;
	while(1); //wait while the chip reboots
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
	CDC_Device_ConfigureEndpoints(&CDC_Interface);
	CDC_Device_ConfigureEndpoints(&ESP_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&CDC_Interface);
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
			//RTS -> RST
			if ((CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_RTS))
			{
				QuarkOneSetESP_FlashMode();
			}
		}
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

static uint8_t BlockLoad(unsigned int size, uint8_t mem, uint32_t *address) {
	uint16_t data;
	uint32_t tempaddress;

	if(mem == MEMORY_TYPE_EEPROM)
	{
		uint8_t pageAddr, byteAddr, value;
		uint8_t buffer[SPM_PAGESIZE];

		EEPROM_FlushBuffer();
		// disable mapping of EEPROM into data space (enable IO mapped access)
		EEPROM_DisableMapping();

		// Fill buffer first, as EEPROM is too slow to copy with UART speed
		for(tempaddress=0; tempaddress < size; tempaddress++)
		{
			buffer[tempaddress] = FetchNextCommandByte();
		}
		// Then program the EEPROM
		for( tempaddress=0; tempaddress < size; tempaddress++)
		{
			pageAddr = (uint8_t)( (*address) / EEPROM_PAGE_SIZE);
			byteAddr = (uint8_t)( (*address) & (EEPROM_PAGE_SIZE - 1));
			value = buffer[tempaddress];

			EEPROM_WriteByte(pageAddr, byteAddr, value);

			(*address)++;
		}
		return RESPONSE_OKAY;
	}
	else if (mem == MEMORY_TYPE_FLASH)
	{
		// NOTE: For flash programming, 'address' is given in words.
		(*address) <<= 1;
		tempaddress = (*address);

		do
		{
			data = FetchNextCommandByte();
			data |= (FetchNextCommandByte() << 8);

			SP_LoadFlashWord(*address, data);

			(*address)+=2;
			size -= 2;
		}
		while(size);

		SP_WriteApplicationPage(tempaddress);

		SP_WaitForSPM();
		(*address) >>= 1;
		return RESPONSE_OKAY;
	}
	else
	{
		return RESPONSE_UNKNOWN;
	}
}

static void BlockRead(uint16_t size, uint8_t mem, uint32_t *address) {
	// EEPROM memory type.

	if (mem == MEMORY_TYPE_EEPROM)
	{                                           // Read EEPROM
		uint8_t byteAddr, pageAddr;
		EEPROM_DisableMapping();
		EEPROM_FlushBuffer();

		do
		{
			pageAddr = (uint8_t)(*address / EEPROM_PAGE_SIZE);
			byteAddr = (uint8_t)(*address & (EEPROM_PAGE_SIZE - 1));

			WriteNextResponseByte( EEPROM_ReadByte( pageAddr, byteAddr ) );
			(*address)++;                                     // Select next EEPROM byte
			size--;                                           // Decrease number of bytes to read
		}
		while (size);                                       // Repeat until all block has been read
	}
	else if(mem == MEMORY_TYPE_FLASH)
	{
		(*address) <<= 1;

		do
		{
			WriteNextResponseByte( SP_ReadByte( *address) );
			WriteNextResponseByte( SP_ReadByte( (*address)+1) );

			(*address) += 2;
			size -= 2;
		}
		while (size);

		(*address) >>= 1;
	}
}


static uint8_t FetchNextCommandByte(void)
{
	int16_t ReceivedByte = CDC_Device_ReceiveByte(&CDC_Interface);
	
	if (ReceivedByte != -1)
	{
		return (uint8_t)ReceivedByte;
	}
	
	return 0;
}


static void WriteNextResponseByte(const uint8_t Response)
{
	CDC_Device_SendByte(&CDC_Interface, Response);
}

/*
* Task to read bytes in on the xmega cdc interface and process them for the bootloader.
*/
static void BootloaderCDC_Task(void)
{
	uint8_t Command = FetchNextCommandByte();

	if (Command == 0)
	{
		return;
	}
	
	if (Command == COMMAND_ExitBootloader)
	{
		WriteNextResponseByte(RESPONSE_OKAY);
		SP_WaitForSPM();
		RunBootloader = 0;
	}
	else if (Command == COMMAND_SetLED)
	{
		QuarkOneSetLEDOn();
	}
	else if (Command == COMMAND_ClearLED)
	{
		QuarkOneSetLEDOff();
	}
	else if (Command == COMMAND_SelectDeviceType)
	{
		FetchNextCommandByte();

		/* Send confirmation byte back to the host */
		WriteNextResponseByte(RESPONSE_OKAY);
	}
	else if ((Command == COMMAND_EnterProgrammingMode) || (Command == COMMAND_LeaveProgrammingMode))
	{
		WriteNextResponseByte(RESPONSE_OKAY);
	}
	else if (Command == COMMAND_ReadPartCode)
	{
		WriteNextResponseByte(0xFA);
		WriteNextResponseByte(0);
	}
	else if (Command == COMMAND_ReadAutoAddressIncrement)
	{
		WriteNextResponseByte(RESPONSE_YES);
	}
	else if (Command == COMMAND_SetCurrentAddress)
	{
		CurrAddress = (FetchNextCommandByte() << 8) | FetchNextCommandByte();
		WriteNextResponseByte(RESPONSE_OKAY);
	}
	else if (Command == COMMAND_ReadBootloaderInterface)
	{
		WriteNextResponseByte('S');
	}
	else if (Command == COMMAND_ReadBootloaderIdentifier)
	{
		/* Write the 7-byte software identifier to the endpoint */
		for (uint8_t CurrByte = 0; CurrByte < 7; CurrByte++)
		WriteNextResponseByte(SOFTWARE_IDENTIFIER[CurrByte]);
	}
	else if (Command == COMMAND_ReadBootloaderSWVersion)
	{
		WriteNextResponseByte(BOOTLOADER_VERSION_MAJOR);
		WriteNextResponseByte(BOOTLOADER_VERSION_MINOR);
	}
	else if (Command == COMMAND_ReadSignature)
	{
		WriteNextResponseByte(SIGNATURE_2);
		WriteNextResponseByte(SIGNATURE_1);
		WriteNextResponseByte(SIGNATURE_0);
	}
	else if (Command == COMMAND_EraseFLASH)
	{
		//MC TODO: This should work instead of a loop...
		//SP_EraseApplicationSection();
		//SP_WaitForSPM();
		
		for(CurrAddress = 0; CurrAddress < APP_SECTION_END; CurrAddress += SPM_PAGESIZE) {
			SP_WaitForSPM();
			SP_EraseApplicationPage( CurrAddress ); // Byte address, not word address
		}

		//EEPROM_LoadPage(&val);                        // Write random values to the page buffer
		//EEPROM_EraseAll();

		WriteNextResponseByte(RESPONSE_OKAY);
	}
	else if (Command == COMMAND_WriteLockbits)
	{
		SP_WaitForSPM();
		SP_WriteLockBits(FetchNextCommandByte());
		WriteNextResponseByte(RESPONSE_OKAY);
	}
	else if (Command == COMMAND_ReadLockbits)
	{
		SP_WaitForSPM();
		WriteNextResponseByte(SP_ReadLockBits());
	}
	else if (Command == COMMAND_ReadLowFuses)
	{
		SP_WaitForSPM();
		WriteNextResponseByte(SP_ReadFuseByte(0));
	}
	else if (Command == COMMAND_ReadHighFuses)
	{
		SP_WaitForSPM();
		WriteNextResponseByte(SP_ReadFuseByte(1));
	}
	else if (Command == COMMAND_ReadExtendedFuses)
	{
		SP_WaitForSPM();
		WriteNextResponseByte(SP_ReadFuseByte(2));
	}

	else if (Command == COMMAND_GetBlockWriteSupport)
	{
		WriteNextResponseByte(RESPONSE_YES);
		// Send block size to the host
		WriteNextResponseByte((SPM_PAGESIZE >> 8) & 0xFF);
		WriteNextResponseByte(SPM_PAGESIZE & 0xFF);
	}
	else if (Command == COMMAND_BlockWrite)
	{
		TempWord = (FetchNextCommandByte() << 8) | FetchNextCommandByte();
		uint8_t val = FetchNextCommandByte();
		/* Delegate the block write/read to a separate function for clarity */
		//ReadWriteMemoryBlock(Command);
		WriteNextResponseByte( BlockLoad(TempWord, val, &CurrAddress) );
	}
	else if (Command == COMMAND_BlockRead)
	{
		TempWord = (FetchNextCommandByte() << 8) | FetchNextCommandByte();
		uint8_t val = FetchNextCommandByte();
		/* Delegate the block write/read to a separate function for clarity */
		//ReadWriteMemoryBlock(Command);
		BlockRead(TempWord, val, &CurrAddress);
	}
	else if (Command == COMMAND_FillFlashPageWordHigh)
	{
		TempWord |= FetchNextCommandByte() << 8;
		
		SP_WaitForSPM();
		SP_LoadFlashWord((CurrAddress << 1), TempWord);
		CurrAddress++;
		
		WriteNextResponseByte(RESPONSE_OKAY);
	}
	else if (Command == COMMAND_FillFlashPageWordLow)
	{
		TempWord = FetchNextCommandByte();
		WriteNextResponseByte(RESPONSE_OKAY);
	}
	else if (Command == COMMAND_WriteFlashPage)
	{
		if (CurrAddress >= (FLASHEND >> 1))
		{
			WriteNextResponseByte(RESPONSE_UNKNOWN);
		}
		else
		{
			QuarkOneSetLEDOn();
			SP_WaitForSPM();
			SP_WriteApplicationPage(CurrAddress << 1);
			WriteNextResponseByte(RESPONSE_OKAY);
			QuarkOneSetLEDOff();
		}
	}
	else if (Command == COMMAND_ReadFLASHWord)
	{
		SP_WaitForSPM();
		WriteNextResponseByte(SP_ReadByte( (CurrAddress << 1) + 1));
		WriteNextResponseByte(SP_ReadByte( (CurrAddress << 1) + 0));
		CurrAddress++;
	}
	else if (Command == COMMAND_WriteEEPROM)
	{
		EEPROM_WriteByte( (uint8_t)(CurrAddress / EEPROM_PAGE_SIZE), (uint8_t)(CurrAddress & (EEPROM_PAGE_SIZE - 1)), FetchNextCommandByte() );
		CurrAddress++;
	}
	else if (Command == COMMAND_ReadEEPROM)
	{
		WriteNextResponseByte( EEPROM_ReadByte( (uint8_t)(CurrAddress / EEPROM_PAGE_SIZE), (uint8_t)(CurrAddress & (EEPROM_PAGE_SIZE - 1)) ) );
		CurrAddress++;
	}
	else if (Command != COMMAND_Escape)
	{
		/* Unknown (non-sync) command, return fail code */
		WriteNextResponseByte(RESPONSE_UNKNOWN);
	}
}