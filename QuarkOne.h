/*
 * QuarkOne.h
 *
 * Defines the Quark One built in peripherals and pin descriptions.
 *
 *  Author: Mitchell A. Cox
 */ 


#ifndef QUARKONE_H_
#define QUARKONE_H_

#include <avr/io.h>
#include <LUFA/Common/Common.h>
//#include <util/delay.h>

// LED and Button:

#define QuarkOne_LED_PORT				PORTB
#define QuarkOne_LED					1 << 1		// PIN B1		

#define QuarkOne_Button_PORT			PORTA
#define QuarkOne_Button					1 << 0		// PIN A0, if not RST


// ESP-01:

#define QuarkOne_ESP_USART				USARTE0	
#define QuarkOne_ESP_USART_PORT			PORTE	
#define QuarkOne_ESP_USART_TX			1 << 3		//PIN E3		
#define QuarkOne_ESP_USART_RX			1 << 2		//PIN E2
#define QuarkOne_ESP_USART_TX_DRE_vect	USARTE0_DRE_vect
#define QuarkOne_ESP_USART_RX_vect		USARTE0_RXC_vect

#define QuarkOne_ESP_GPIO0_2_PORT		PORTE				
#define QuarkOne_ESP_CH_PD_RST_PORT		PORTR				

#define QuarkOne_ESP_GPIO0				1 << 1		// PIN PE1 (pull up)
#define QuarkOne_ESP_GPIO2				1 << 0		// PIN PE1 
#define QuarkOne_ESP_CH_PD				1 << 1		// PIN PR1 (pull up)
#define QuarkOne_ESP_RST				1 << 0		// PIN PR0 

#define QuarkOne_ESP_PORTE_MASK			(QuarkOne_ESP_GPIO0 | !QuarkOne_ESP_GPIO2) //GPIO0 output, GPIO2 input
#define QuarkOne_ESP_PORTR_MASK			(QuarkOne_ESP_CH_PD | QuarkOne_ESP_RST)

// Helper Functions:
// Set the requisite pins to put the ESP-01 into normal mode (including the reset).
static inline void QuarkOneSetESP_NormalMode()
{
	// GPIO0 = 1 (or Hi-Z), GPIO2 = 1, CH_PD = 1 (or Hi-Z), RST = 1 (or Hi-Z)
	//QuarkOne_ESP_GPIO0_2_PORT.OUTSET = (QuarkOne_ESP_GPIO0 | QuarkOne_ESP_GPIO2);
	
	QuarkOne_ESP_GPIO0_2_PORT.DIRSET = (QuarkOne_ESP_GPIO0 | QuarkOne_ESP_GPIO2); //GPIO0,2 to output
	QuarkOne_ESP_GPIO0_2_PORT.OUTSET = (QuarkOne_ESP_GPIO0 | QuarkOne_ESP_GPIO2); //GPIO0,2 = 1
	Delay_MS(1);
	QuarkOne_ESP_CH_PD_RST_PORT.OUTCLR = QuarkOne_ESP_RST; //RST
	Delay_MS(1);
	QuarkOne_ESP_CH_PD_RST_PORT.OUTSET = QuarkOne_ESP_RST; //!RST
};

// Set the requisite pins to put the ESP-01 into flash mode (including the reset).
static inline void QuarkOneSetESP_FlashMode()
{
	// Flash Mode: GPIO0 = 0, GPIO2 = 1, CH_PD = 1 (or Hi-Z), RST = 1 (or Hi-Z)
	//QuarkOne_ESP_GPIO0_2_PORT.OUTSET = QuarkOne_ESP_GPIO2;
	QuarkOne_ESP_GPIO0_2_PORT.DIRSET = (QuarkOne_ESP_GPIO0 | QuarkOne_ESP_GPIO2); //GPIO0,2 to output
	QuarkOne_ESP_GPIO0_2_PORT.OUTCLR = QuarkOne_ESP_GPIO0; //GPIO0 = 0
	QuarkOne_ESP_GPIO0_2_PORT.OUTSET = QuarkOne_ESP_GPIO2; //GPIO2 = 1
	Delay_MS(1);
	QuarkOne_ESP_CH_PD_RST_PORT.OUTCLR = QuarkOne_ESP_RST; //RST
	Delay_MS(1);
	QuarkOne_ESP_CH_PD_RST_PORT.OUTSET = QuarkOne_ESP_RST; //!RST
};

static inline void QuarkOnePinSetup()
{
	QuarkOne_LED_PORT.DIRSET = QuarkOne_LED;	
	QuarkOne_LED_PORT.PIN1CTRL  = PORT_SRLEN_bm;	//Slew rate limiting enabled (limit current draw + EMI), tho I dont think it makes much difference
	QuarkOne_LED_PORT.OUTCLR = QuarkOne_LED; //Leave the LED off
	
	QuarkOne_Button_PORT.DIRCLR = QuarkOne_Button;
	QuarkOne_Button_PORT.PIN0CTRL = PORT_OPC_PULLUP_gc; //Enable a pullup on the button input
	
	QuarkOne_ESP_USART_PORT.DIRSET = QuarkOne_ESP_USART_TX;
	QuarkOne_ESP_USART_PORT.DIRCLR = QuarkOne_ESP_USART_RX;
	
	//QuarkOne_ESP_GPIO0_2_PORT.DIRSET = QuarkOne_ESP_PORTE_MASK;	
	QuarkOne_ESP_CH_PD_RST_PORT.DIRSET = QuarkOne_ESP_PORTR_MASK;
	
	QuarkOne_ESP_CH_PD_RST_PORT.OUTCLR = QuarkOne_ESP_RST;
	QuarkOne_ESP_CH_PD_RST_PORT.OUTSET = QuarkOne_ESP_CH_PD; //this should always stay high
};




// Returns 0 if button is not pressed, 1 otherwise.
static inline uint8_t QuarkOneGetButton()
{
	return (QuarkOne_Button_PORT.IN & QuarkOne_Button);
};

static inline void QuarkOneSetLEDOff()
{
	QuarkOne_LED_PORT.OUTCLR = QuarkOne_LED;
};

static inline void QuarkOneSetLEDOn()
{
	QuarkOne_LED_PORT.OUTSET = QuarkOne_LED;
};

static inline void QuarkOneSetLEDToggle()
{
	QuarkOne_LED_PORT.OUTTGL = QuarkOne_LED;
};

#endif /* QUARKONE_H_ */