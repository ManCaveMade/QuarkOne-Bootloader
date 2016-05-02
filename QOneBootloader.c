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

/** LUFA CDC Class driver interface configuration and state information. This structure is
*  passed to all CDC Class driver functions, so that multiple instances of the same class
*  within a device can be differentiated from one another. This is for the second CDC interface,
*	which is a transparent bridge to the UART that the ESP-01 is connected to.
*/
USB_ClassInfo_CDC_Device_t ESP_CDC_Interface =
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

//USART_t USART_Interface =
//{

//};

static inline void GetBootloaderSpice(void)
{
	EEPROM_EnableMapping();
	EEPROM_WaitForNVM();
	bootloaderSpice = EEPROM(BOOTLOADER_SPICE_EEPROM_PAGE, BOOTLOADER_SPICE_EEPROM_BYTE);
}

static inline void SetBootloaderSpice(void)
{
	if (bootloaderSpice == BOOTLOADER_SPICE)
	return;
	
	EEPROM_EnableMapping();
	EEPROM_WaitForNVM();
	EEPROM(BOOTLOADER_SPICE_EEPROM_PAGE, BOOTLOADER_SPICE_EEPROM_BYTE) = BOOTLOADER_SPICE;
	EEPROM_AtomicWritePage(BOOTLOADER_SPICE_EEPROM_PAGE);
	bootloaderSpice = BOOTLOADER_SPICE;
}

static inline void ClearBootloaderSpice(void)
{
	if (bootloaderSpice != BOOTLOADER_SPICE)
	return;
	
	EEPROM_EnableMapping();
	EEPROM_WaitForNVM();
	EEPROM(BOOTLOADER_SPICE_EEPROM_PAGE, BOOTLOADER_SPICE_EEPROM_BYTE) = 0;
	EEPROM_AtomicWritePage(BOOTLOADER_SPICE_EEPROM_PAGE);
	bootloaderSpice = 0;
}


int main(void)
{
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);
	
	CCP = CCP_IOREG_gc;
	PMIC.CTRL = PMIC_IVSEL_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	
	QuarkOnePinSetup();
	
	GetBootloaderSpice();
	

	//Make a decision about whether to go into the bootloader or not:
	
	//Must go into bootloader if:
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//Go into the bootloader if there was a reset and the spice is there
	//Go into bootloader if flash is unprogrammed
	
	//Otherwise:
	//^^^^^^^^^^
	//Start the timer (Same as LilyPad Caterina: 750ms) after which we launch the app
	//If there is a second external reset within this time, launch to bootloader
	
	//QuarkOneSetLEDOn();
	
	TCC0.PER = 1563; //~50ms
	TCC0.INTCTRLA = ( TCC0.INTCTRLA & ~TC0_OVFINTLVL_gm ) | TC_OVFINTLVL_LO_gc;
	TCC0.CTRLA = ( TCC0.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1024_gc; //31250 Hz
	
	//LaunchBootloader();
	
	bool appPresent = (pgm_read_word(0) == 0xFFFF);
	appPresent = true;
	
	if (appPresent)
	{
		if (RST.STATUS & RST_PORF_bm) //launch app immediately on power-on-reset
		{
			RST.STATUS |= RST_PORF_bm;
			LaunchApplication();
		}
		else if (bootloaderSpice == BOOTLOADER_SPICE) //if the spice is there, launch bootloader (regardless of the reset source).
		{
			ClearBootloaderSpice();
			LaunchBootloader();
		}
		else if (RST.STATUS & RST_EXTRF_bm) //external reset
		{
			QuarkOneSetLEDOn();
			RST.STATUS |= RST_EXTRF_bm;
			
			//Wait for 750ms in case there is another external reset, otherwise continue to the application
			GlobalInterruptEnable(); //so that we can use the timer interrupt to track time
			
			SetBootloaderSpice();
			RunBootloader = true;
			
			while (RunBootloader) //this will be interrupted by another reset
			{
				if (resetTimeout > EXT_RESET_TIMEOUT_VALUE)
				{
					RunBootloader = false;
				}
			}
			
			//If we get to this point then there was no second reset
		}
	}
	else
	{
		LaunchBootloader();
	}
	
	//Bootloader is finished, so reboot to the app, if there isnt one then a reset will be needed lol
	LaunchApplication();
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Further Hardware Initialization which is only done if we are going into the bootloader */
	Serial_Init(&QuarkOne_ESP_USART, 115200, false, true); //MC: modified for interrupts

	/*for (uint8_t i = 0; i < 5; ++i)
	{
	Delay_MS(100);
	QuarkOneSetLEDToggle();
	}*/
	QuarkOneSetLEDOff();

	//MC: The DTR signals mess with the mode at connect for some reason.
	//QuarkOneSetESP_FlashMode();
	QuarkOneSetESP_NormalMode();

	//gpio_set_pin_low(QuarkOne_LED);
	USB_Init();
}

ISR(TCC0_OVF_vect)
{
	resetTimeout++;
	
	//Sexy LED flashing
	if (LEDFlicker)
	{
		LEDPulseCount++;
		if ((LEDPulseCount == 1) || (LEDPulseCount == 4))
		{
			QuarkOneSetLEDOn();
		}
		else
		{
			QuarkOneSetLEDOff();
		}
		if (LEDPulseCount >= 20) //1s
		LEDPulseCount = 0;
	}
}


static void LaunchBootloader(void)
{
	GlobalInterruptDisable();
	
	RunBootloader = true;
	
	RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
	RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));
	
	SetupHardware();

	GlobalInterruptEnable();

	LEDFlicker = true;

	while (RunBootloader)
	{
		BootloaderCDC_Task();
		CDC_Device_USBTask(&CDC_Interface);
		
		USBToUSART_Task();
		CDC_Device_USBTask(&ESP_CDC_Interface);
		
		USB_USBTask();
	}
	
	SP_WaitForSPM();
	//SP_LockSPM();
	
	LaunchApplication();
}

static void LaunchApplication(void)
{
	GlobalInterruptDisable();
	ClearBootloaderSpice();
	
	//Undo everything so that the new app has a fresh start
	//We could also do a WDT reset or a soft reset (MC: Can't manage it right now :P )
	CCP = CCP_IOREG_gc;
	PMIC.CTRL = 0;
	TCC0.INTCTRLA = 0;
	TCC0.CTRLA = 0;
	Serial_Disable(&QuarkOne_ESP_USART);
	USB_Disable();
	
	asm("jmp 0x0000");
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
		//QuarkOneSetLEDOn();
		Serial_SendByte(&QuarkOne_ESP_USART, RingBuffer_Remove(&USBtoUSART_Buffer));
		//QuarkOneSetLEDOff();
	}
}

static void USBToUSART_Task(void)
{
	//USB to USART
	
	//Endpoint_SelectEndpoint(ESP_CDC_Interface.Config.DataINEndpoint.Address);
	
	//Do as many bytes as possible
	uint16_t bytesReceived = CDC_Device_BytesReceived(&ESP_CDC_Interface);
	
	while ((bytesReceived--) && !(RingBuffer_IsFull(&USBtoUSART_Buffer)))
	{
		RingBuffer_Insert(&USBtoUSART_Buffer, Endpoint_Read_8());
	}

	//USART to USB
	uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
	//uint8_t error = false;
	
	if (BufferCount)
	{
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
			if (CDC_Device_SendByte(&ESP_CDC_Interface,	RingBuffer_Peek(&USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
			{
				break;
			}
			
			RingBuffer_Remove(&USARTtoUSB_Buffer);
		}
		
		//CDC_Device_Flush(&ESP_CDC_Interface);
		//}
	}
	
	
}


/*
* --------------- Bootloader --------------------------------------------------
*/

static uint8_t BlockLoad(uint16_t size, uint8_t mem, uint32_t *address) {
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
	else if ((mem == MEMORY_TYPE_FLASH) || (mem == MEMORY_TYPE_USERSIG))
	{
		// NOTE: For flash programming, 'address' is given in words.
		(*address) <<= 1;
		tempaddress = (*address);

		SP_WaitForSPM();

		do
		{
			data = FetchNextCommandByte();
			data |= (FetchNextCommandByte() << 8);

			SP_LoadFlashWord(*address, data);
			SP_WaitForSPM();

			(*address)+=2;
			size -= 2;
		}
		while(size);


		if (mem == MEMORY_TYPE_FLASH)
		{
			if (ChipErased == true)
			{
				SP_WriteApplicationPage(tempaddress); //avrdude doesnt always erase chip first
			}
			else
			{
				SP_EraseWriteApplicationPage(tempaddress);
			}
		}
		else if (mem == MEMORY_TYPE_USERSIG)
		{
			//MC: Not tested.
			SP_EraseUserSignatureRow();
			SP_WaitForSPM();
			SP_WriteUserSignatureRow();
			SP_WaitForSPM();
		}
		
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
	else if ((mem == MEMORY_TYPE_FLASH) || (mem == MEMORY_TYPE_USERSIG) || (mem == MEMORY_TYPE_PRODSIG))
	{
		(*address) <<= 1;
		
		do
		{
			if (mem == MEMORY_TYPE_FLASH)
			{
				TempWord = SP_ReadByte(*address);
			}
			else if (mem == MEMORY_TYPE_USERSIG)
			{
				TempWord = SP_ReadUserSignatureByte(*address);
			}
			else if (mem == MEMORY_TYPE_PRODSIG)
			{
				TempWord = SP_ReadCalibrationByte(*address);
			}
			
			//SP_WaitForSPM();
			WriteNextResponseByte(TempWord & 0xFF);
			//WriteNextResponseByte((TempWord >> 8) & 0xFF);
			//WriteNextResponseByte( SP_ReadByte( *address) );
			//WriteNextResponseByte( SP_ReadByte( (*address)+1) );

			//(*address) += 2;
			//size -= 2;
			(*address)++;
			size--;
			
			//Stop when we get to the bootloader section
			if (*address >= (BOOT_SECTION_START >> 1))
			size = 0;
		}
		while (size);

		(*address) >>= 1;
	}
}


static uint8_t FetchNextCommandByte(void)
{
	int16_t ReceivedByte = CDC_Device_ReceiveByte(&CDC_Interface);
	
	return ReceivedByte < 0 ? 0 : (uint8_t)ReceivedByte;
	
	/*
	if (ReceivedByte != -1)
	{
	return (uint8_t)ReceivedByte;
	}
	
	return 0; */
}


static inline void WriteNextResponseByte(const uint8_t Response)
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
	else if (Command == COMMAND_EraseFLASH) //Note: This command is not necessarily called by avrdude unless -e is used.
	{
		SP_WaitForSPM();
		SP_EraseApplicationSection();
		SP_WaitForSPM();
		
		/*for(CurrAddress = 0; CurrAddress < APP_SECTION_END; CurrAddress += SPM_PAGESIZE) {
		SP_WaitForSPM();
		SP_EraseApplicationPage( CurrAddress ); // Byte address, not word address
		SP_WaitForSPM();
		}*/

		EEPROM_LoadPage(&Command);                        // Write random values to the page buffer
		EEPROM_EraseAll();

		ChipErased = true;

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
		uint8_t val = FetchNextCommandByte(); //memory type
		/* Delegate the block write/read to a separate function for clarity */
		//ReadWriteMemoryBlock(Command);
		BlockRead(TempWord, val, &CurrAddress);
	}
	/*else if (Command == COMMAND_FillFlashPageWordHigh)	//Flash Byte Support (Optional)
	{
	TempWord |= FetchNextCommandByte() << 8;
	
	SP_WaitForSPM();
	SP_LoadFlashWord((CurrAddress << 1), TempWord);
	CurrAddress++;

	WriteNextResponseByte(RESPONSE_OKAY);
	}
	else if (Command == COMMAND_FillFlashPageWordLow) //Flash Byte Support (Optional)
	{
	TempWord = FetchNextCommandByte();
	WriteNextResponseByte(RESPONSE_OKAY);
	}
	else if (Command == COMMAND_WriteFlashPage) //Flash Byte Support (Optional)
	{
	if (CurrAddress >= (APP_SECTION_END >> 1))
	{
	WriteNextResponseByte(RESPONSE_UNKNOWN);
	}
	else
	{
	SP_WaitForSPM();
	SP_WriteApplicationPage(CurrAddress << 1);
	WriteNextResponseByte(RESPONSE_OKAY);
	}
	}
	else if (Command == COMMAND_ReadFLASHWord) //Flash Byte Support (Optional)
	{
	SP_WaitForSPM();
	WriteNextResponseByte(SP_ReadByte( (CurrAddress << 1) + 1));
	SP_WaitForSPM();
	WriteNextResponseByte(SP_ReadByte( (CurrAddress << 1) + 0));
	CurrAddress++;
	}*/
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