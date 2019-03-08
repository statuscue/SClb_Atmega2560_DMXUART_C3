/*
 Name:		SClb_Atmega2560_DMXUART_C3.cpp
 Created:	08-Mar-19 16:23:03
 Author:	statuscue
 Editor:	http://hes.od.ua
*/

/*
	Copyright 2019-2019 by Yevhen Mykhailov
	Art-Net(TM) Designed by and Copyright Artistic Licence Holdings Ltd.
*/

#include "SClb_Atmega2560_DMXUART_C3.h"

#include "pins_arduino.h"
#include <inttypes.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>

SClb_Atmega2560_DMXUART_C3 SCDMXUSART3;

#define SCUCSRA_U3 UCSR3A						// USART register A
#define SCTXC_U3   TXC3							// tx buffer empty
#define SCUDRE_U3  UDRE3						// data ready
#define SCFE_U3    FE3							// frame error
#define SCU2X_U3   U2X3							// double speed

#define SCUCSRB_U3 UCSR3B						// USART register B
#define SCRXCIE_U3 RXCIE3						// rx interrupt enable bit
#define SCTXCIE_U3 TXCIE3						// tx interrupt enable bit
#define SCRXEN_U3  RXEN3						// rx enable bit
#define SCTXEN_U3  TXEN3						// tx enable bit

#define SCUCSRC_U3 UCSR3C						// USART register C
#define SCUSBS0_U3  USBS3						// stop bits
#define SCUCSZ0_U3 UCSZ30						// length
#define SCUPM0_U3  UPM30						// parity
#define SCUCSRRH_U3 UBRR3H						// USART baud rate register msb
#define SCUCSRRL_U3 UBRR3L						// USARTbaud rate register msb
#define SCUDR_U3   UDR3							// USART data register tx/rx

#define SCUSART_RX_vect_U3  USART3_RX_vect		// RX ISR
#define SCUSART_TX_vect_U3  USART3_TX_vect		// TX ISR

#define BIT_FRAME_ERROR_U3 (1<<SCFE_U3)
#define BIT_2X_SPEED_U3 (1<<SCU2X_U3)
#define FORMAT_8N2_U3 (3<<SCUCSZ0_U3) | (1<<SCUSBS0_U3)
#define FORMAT_8E1_U3 (3<<SCUCSZ0_U3) | (2<<SCUPM0_U3)
#define BIT_TX_ENABLE_U3  (1<<SCTXEN_U3)
#define BIT_TX_ISR_ENABLE_U3 (1<<SCTXCIE_U3)
#define BIT_RX_ENABLE_U3  (1<<SCRXEN_U3)
#define BIT_RX_ISR_ENABLE_U3 (1<<SCRXCIE_U3)

//***** baud rate defines
#define F_CLK_U3 				16000000UL
#define DMX_DATA_BAUD_U3		250000
#define DMX_BREAK_BAUD_U3 	 	99900
//99900

//***** states indicate current position in DMX stream
#define DMX_STATE_BREAK_U3 0
#define DMX_STATE_START_U3 1
#define DMX_STATE_DATA_U3 2
#define DMX_STATE_IDLE_U3 3

//***** status is if interrupts are enabled and IO is active
#define ISR_DISABLED_U3 		0
#define ISR_OUTPUT_ENABLED_U3 	1
#define ISR_INPUT_ENABLED_U3 	2

// **************************** global data (can be accessed in ISR)  ***************

uint8_t*  _shared_dmx_data_U3;
uint8_t   _shared_dmx_state_U3;
uint16_t  _shared_dmx_slot_U3;
uint16_t  _shared_max_slots_U3 = DMX_MIN_SLOTS_U3;
SCReceiveCallback _shared_receive_callback_U3 = NULL;

//************************************************************************************
// ************************  SClbHESDMXUART Output member functions  ********************

SClb_Atmega2560_DMXUART_C3::SClb_Atmega2560_DMXUART_C3(void) {
	_direction_pin = DIRECTION_PIN_NOT_USED_U3;	//optional
	_shared_max_slots_U3 = DMX_MAX_SLOTS_U3;
	_interrupt_status = ISR_DISABLED_U3;

	//zero buffer including _dmxData[0] which is start code
	for (int n = 0; n < DMX_MAX_SLOTS_U3 + 1; n++) {
		_dmxData[n] = 0;
	}
}


SClb_Atmega2560_DMXUART_C3::~SClb_Atmega2560_DMXUART_C3(void) {
	stop();
	_shared_dmx_data_U3 = NULL;
	_shared_receive_callback_U3 = NULL;
}

//  ***** start *****
//  sets up baud rate, bits and parity
//  sets globals accessed in ISR
//  enables transmission and tx interrupt

void SClb_Atmega2560_DMXUART_C3::startOutput(void) {
	if (_direction_pin != DIRECTION_PIN_NOT_USED_U3) {
		digitalWrite(_direction_pin, HIGH);
	}
	if (_interrupt_status == ISR_INPUT_ENABLED_U3) {
		stop();
	}
	if (_interrupt_status == ISR_DISABLED_U3) {	//prevent messing up sequence if already started...
		SCUCSRRH_U3 = (unsigned char)(((F_CLK_U3 + DMX_DATA_BAUD_U3 * 8L) / (DMX_DATA_BAUD_U3 * 16L) - 1) >> 8);
		SCUCSRRL_U3 = (unsigned char)((F_CLK_U3 + DMX_DATA_BAUD_U3 * 8L) / (DMX_DATA_BAUD_U3 * 16L) - 1);
		SCUCSRA_U3 &= ~BIT_2X_SPEED_U3;

		SCUDR_U3 = 0x0;     			//USART send register  
		_shared_dmx_data_U3 = dmxData();
		_shared_dmx_state_U3 = DMX_STATE_BREAK_U3;

		SCUCSRC_U3 = FORMAT_8N2_U3; 					//set length && stopbits (no parity)
		SCUCSRB_U3 |= BIT_TX_ENABLE_U3 | BIT_TX_ISR_ENABLE_U3;  //enable tx and tx interrupt
		_interrupt_status = ISR_OUTPUT_ENABLED_U3;
	}
}

//  ***** start *****
//  sets up baud rate, bits and parity
//  sets globals accessed in ISR
//  enables transmission and tx interrupt

void SClb_Atmega2560_DMXUART_C3::startInput(void) {
	if (_direction_pin != DIRECTION_PIN_NOT_USED_U3) {
		digitalWrite(_direction_pin, LOW);
	}
	if (_interrupt_status == ISR_OUTPUT_ENABLED_U3) {
		stop();
	}
	if (_interrupt_status == ISR_DISABLED_U3) {	//prevent messing up sequence if already started...
		SCUCSRRH_U3 = (unsigned char)(((F_CLK_U3 + DMX_DATA_BAUD_U3 * 8L) / (DMX_DATA_BAUD_U3 * 16L) - 1) >> 8);
		SCUCSRRL_U3 = (unsigned char)((F_CLK_U3 + DMX_DATA_BAUD_U3 * 8L) / (DMX_DATA_BAUD_U3 * 16L) - 1);
		SCUCSRA_U3 &= ~BIT_2X_SPEED_U3;

		_shared_dmx_data_U3 = dmxData();
		_shared_dmx_state_U3 = DMX_STATE_IDLE_U3;
		_shared_dmx_slot_U3 = 0;

		SCUCSRC_U3 = FORMAT_8N2_U3; 					//set length && stopbits (no parity)
		SCUCSRB_U3 |= BIT_RX_ENABLE_U3 | BIT_RX_ISR_ENABLE_U3;  //enable tx and tx interrupt
		_interrupt_status = ISR_INPUT_ENABLED_U3;
	}
}

//  ***** stop *****
//  disables interrupts

void SClb_Atmega2560_DMXUART_C3::stop(void) {
	if (_interrupt_status == ISR_OUTPUT_ENABLED_U3) {
		SCUCSRB_U3 &= ~BIT_TX_ISR_ENABLE_U3;  							//disable tx interrupt
		SCUCSRB_U3 &= ~BIT_TX_ENABLE_U3;     							//disable tx enable
	}
	else if (_interrupt_status == ISR_INPUT_ENABLED_U3) {
		SCUCSRB_U3 &= ~BIT_RX_ISR_ENABLE_U3;  							//disable rx interrupt
		SCUCSRB_U3 &= ~BIT_RX_ENABLE_U3;     							//disable rx enable	
	}
	_interrupt_status = ISR_DISABLED_U3;
}

void SClb_Atmega2560_DMXUART_C3::setDirectionPin(uint8_t pin) {
	_direction_pin = pin;
	pinMode(_direction_pin, OUTPUT);
}

//  ***** setMaxSlots *****
//  sets the number of slots sent per DMX frame
//  defaults to 512 or DMX_MAX_SLOTS_U3
//  should be no less DMX_MIN_SLOTS_U3 slots
//  the DMX standard specifies min break to break time no less than 1024 usecs
//  at 44 usecs per slot ~= 24

void SClb_Atmega2560_DMXUART_C3::setMaxSlots(int slots) {
	_shared_max_slots_U3 = max(slots, DMX_MIN_SLOTS_U3);
}

//  ***** getSlot *****
//  reads the value of a slot
//  see buffering note for ISR below 

uint8_t SClb_Atmega2560_DMXUART_C3::getSlot(int slot) {
	return _dmxData[slot];
}

//  ***** setSlot *****
//  sets the output value of a slot

void SClb_Atmega2560_DMXUART_C3::setSlot(int slot, uint8_t value) {
	_dmxData[slot] = value;
}

//  ***** dmxData *****
//  pointer to data buffer

uint8_t* SClb_Atmega2560_DMXUART_C3::dmxData(void) {
	return &_dmxData[0];
}

//  ***** setDataReceivedCallback *****
//  sets pointer to function that is called
//  on the break after a frame has been received
//  whatever happens in this function should be quick
//  ie set a flag so that processing of the received data happens
//  outside of the ISR.


void SClb_Atmega2560_DMXUART_C3::setDataReceivedCallback(SCReceiveCallback callback) {
	_shared_receive_callback_U3 = callback;
}


//************************************************************************************
// ************************ TX ISR (transmit interrupt service routine)  *************
//
// this routine is called when USART transmission is complete
// what this does is to send the next byte
// when that byte is done being sent, the ISR is called again
// and the cycle repeats...
// until _shared_max_slots_U3 worth of bytes have been sent on succesive triggers of the ISR
// and then on the next ISR...
// the break/mark after break is sent at a different speed
// and then on the next ISR...
// the start code is sent
// and then on the next ISR...
// the next data byte is sent
// and the cycle repeats...


ISR(SCUSART_TX_vect_U3) {
	switch (_shared_dmx_state_U3) {

	case DMX_STATE_BREAK_U3:
		// set the slower baud rate and send the break
		SCUCSRRH_U3 = (unsigned char)(((F_CLK_U3 + DMX_BREAK_BAUD_U3 * 8L) / (DMX_BREAK_BAUD_U3 * 16L) - 1) >> 8);
		SCUCSRRL_U3 = (unsigned char)((F_CLK_U3 + DMX_BREAK_BAUD_U3 * 8L) / (DMX_BREAK_BAUD_U3 * 16L) - 1);
		SCUCSRA_U3 &= ~BIT_2X_SPEED_U3;
		SCUCSRC_U3 = FORMAT_8E1_U3;
		_shared_dmx_state_U3 = DMX_STATE_START_U3;
		SCUDR_U3 = 0x0;
		break;		// <- DMX_STATE_BREAK_U3

	case DMX_STATE_START_U3:
		// set the baud to full speed and send the start code
		SCUCSRRH_U3 = (unsigned char)(((F_CLK_U3 + DMX_DATA_BAUD_U3 * 8L) / (DMX_DATA_BAUD_U3 * 16L) - 1) >> 8);
		SCUCSRRL_U3 = (unsigned char)((F_CLK_U3 + DMX_DATA_BAUD_U3 * 8L) / (DMX_DATA_BAUD_U3 * 16L) - 1);
		SCUCSRA_U3 &= ~BIT_2X_SPEED_U3;
		SCUCSRC_U3 = FORMAT_8N2_U3;
		_shared_dmx_slot_U3 = 0;
		SCUDR_U3 = _shared_dmx_data_U3[_shared_dmx_slot_U3++];	//send next slot (start code)
		_shared_dmx_state_U3 = DMX_STATE_DATA_U3;
		break;		// <- DMX_STATE_START_U3

	case DMX_STATE_DATA_U3:
		// send the next data byte until the end is reached
		SCUDR_U3 = _shared_dmx_data_U3[_shared_dmx_slot_U3++];	//send next slot
		if (_shared_dmx_slot_U3 > _shared_max_slots_U3) {
			_shared_dmx_state_U3 = DMX_STATE_BREAK_U3;
		}
		break;		// <- DMX_STATE_DATA_U3
	}
}

//***********************************************************************************
// ************************ RX ISR (receive interrupt service routine)  *************
//
// this routine is called when USART receives data
// wait for break:  if have previously read data call callback function
// then on next receive:  check start code
// then on next receive:  read data until done (in which case idle)
//
//  NOTE: data is not double buffered
//  so a complete single frame is not guaranteed
//  the ISR will continue to read the next frame into the buffer

ISR(SCUSART_RX_vect_U3) {
	uint8_t status_register = SCUCSRA_U3;
	uint8_t incoming_byte = SCUDR_U3;

	if (status_register & BIT_FRAME_ERROR_U3) {
		_shared_dmx_state_U3 = DMX_STATE_BREAK_U3;
		if (_shared_dmx_slot_U3 > 0) {
			if (_shared_receive_callback_U3 != NULL) {
				_shared_receive_callback_U3(_shared_dmx_slot_U3);
			}
		}
		_shared_dmx_slot_U3 = 0;
		return;
	}

	switch (_shared_dmx_state_U3) {

	case DMX_STATE_BREAK_U3:
		if (incoming_byte == 0) {						//start code == zero (DMX)
			_shared_dmx_state_U3 = DMX_STATE_DATA_U3;
			_shared_dmx_slot_U3 = 1;
		}
		else {
			_shared_dmx_state_U3 = DMX_STATE_IDLE_U3;
		}
		break;

	case DMX_STATE_DATA_U3:
		_shared_dmx_data_U3[_shared_dmx_slot_U3++] = incoming_byte;
		if (_shared_dmx_slot_U3 > DMX_MAX_SLOTS_U3) {
			_shared_dmx_state_U3 = DMX_STATE_IDLE_U3;			// go to idle, wait for next break
		}
		break;
	}
}

