/*
* This file provides functions for Infrared_communication like 8bit transmitting and 8 bit reading 
* Reading utilizes interrupt.
* This file is so complex so I think it'd be better not to change this file
* If you have to change, don't afraid of asking me!
* I can tell you but it's so hard
* Function involved with infrared communication are also defined in wiring.cpp and main.cpp
*
*/
//
//include
//
#include <avr/io.h>
#include <Arduino.h>
//#include "My_wiring.h"
#include "SoftwareIrDAINT.h"
//
//Const
//
///*
const uint16_t IrDAParam::BAUD_RATE = 38400;
const double IrDAParam::LEN_HIGH = 2.4 * 0.001 * 0.001;
const double IrDAParam::LEN_RX_PULSE = 2.2 * 0.001 * 0.001;
#ifdef F_CPU
const double IrDAParam::PULSEDELAY = ( (LEN_HIGH * F_CPU)+1 ) / 3.0;
const double IrDAParam::TXDELAY = ( (F_CPU/BAUD_RATE) - 3*PULSEDELAY - 8.5 ) / 3.0;
const double IrDAParam::STARTDELAY = ( 0.5*(F_CPU/BAUD_RATE+LEN_RX_PULSE) - 57 ) / 3.0;
const double IrDAParam::READDELAY = ( F_CPU/BAUD_RATE - 4 ) / 5.0;
const double IrDAParam::STOPDELAY = ( F_CPU/BAUD_RATE + 2.0 ) / 3.0;
#else
#error CPU frequency F_CPU undefined
#endif//*/
//
//Initialize private member
//
//
//Constructor
//
SoftwareIrDA::SoftwareIrDA(uint8_t channel){
	#if defined(IR_TXPIN1) && defined(IR_RXPIN1) && defined(IR_RXBIT1) && defined(IR_RXPORT1)
	if (channel == 1) {
		_receivePin = IR_RXPIN1;
		_transmitPin = IR_TXPIN1;
	}
	#endif /*IR_RXPIN1 && IR_RXBIT1 && IR_RXPORT1*/
	#if defined(IR_TXPIN2) && defined(IR_RXPIN2) && defined(IR_RXBIT2) && defined(IR_RXPORT2)
	else {
		_receivePin = IR_RXPIN2;
		_transmitPin = IR_TXPIN2;
	}
	#endif /*IR_RXPIN2 && IR_RXBIT2 && IR_RXPORT2*/

	*portModeRegister(digitalPinToPort(_transmitPin)) |= digitalPinToBitMask(_transmitPin);
	*portModeRegister(digitalPinToPort(_receivePin)) &= ~digitalPinToBitMask(_receivePin);
}
//
// Destructor
//
SoftwareIrDA::~SoftwareIrDA() {
	//end();
}
void SoftwareIrDA::transmitByte(uint8_t txByte) {

	uint8_t count1;
	uint8_t count2;
	uint16_t txByte2 = (uint16_t) txByte;
	volatile uint8_t *port = portOutputRegister(digitalPinToPort(_transmitPin));
	uint8_t high = *port | digitalPinToBitMask(_transmitPin);
	uint8_t low = *port & ~digitalPinToBitMask(_transmitPin);
	
	//cycle/bit = Txdelay + PulseDelay + other= 8.5 + (3*txDelay) + (3*pulseDelay)= cycles per second/bps = F_CPU/bps = F_CPU*T_PULSE (s)
	//Txdelay = (3*txDelay) -1
	//other =  10.5 cycles
	//PulseDelay(cycle/bit) = (3 * pulseDelay) - 1
	// Which have no mention about cycle are 1 cycle
		asm volatile(
			"cli								\n\t"
			"ldi	%B[txByte],		1			\n\t"	//stop bit
			"mov	__tmp_reg__,	%[high]		\n\t"	//set start bit
			"TxLoop:							\n\t"
			"mov	%[count1],		%[pulseDelay]	\n\t"
			"st		%a[port],		__tmp_reg__	\n\t"	//send bit
			"PulseDelay:						\n\t"
			"dec	%[count1]					\n\t"
			"brne	PulseDelay					\n\t"	//1(go) or 2(loop) cycle
			"st		%a[port],		%[low]		\n\t"	//OFF
			"mov	%[count2],		%[txDelay]	\n\t"
			"TxDelay:							\n\t"	//Txdelay(cycle/bit) = (3 * r20) - 1
			"dec	%[count2]					\n\t"
			"brne	TxDelay						\n\t"
			"mov	__tmp_reg__,	%[high]		\n\t"
			"sbrc	%A[txByte],		0			\n\t"	//if txByte(0) = 0, skip next order and 2 cycle
			"mov	__tmp_reg__,	%[low]		\n\t"
			"lsr	%B[txByte]					\n\t"
			"ror	%A[txByte]					\n\t"
			"brne	TxLoop						\n\t"	//1(go) or 2(loop) cycle
			"reti"
			:
			: [txDelay] "r" ((uint8_t)IrDAParam::TXDELAY),
			  [pulseDelay] "r" ((uint8_t)IrDAParam::PULSEDELAY),
			  [txByte] "r" (txByte2),
			  [count1] "r" (count1),
			  [count2] "r" (count2),
			  [high] "r" (high),
			  [low] "r" (low),
			  [port] "e" (port)
		);
}
uint8_t SoftwareIrDA::receiveByte(void) {
	uint8_t rxByte; 
	uint8_t count1;
	rxByte = (uint8_t)(IrDAParam::STARTDELAY + 0.5);
	//no mention about cycle is 1 cycle
	//Wait Start Bit + WaitFirstBit + 1PrepareRead = 5 + 3*fistDelay-1 + 2 = 0.5*F_CPU/BAUD_RATE
	//ReadLoop + toPrepareRead + PrepareRead= 5*readDelay-1 + 3 + 2 = F_CPU/BAUD_RATE
	//WaitStop = 3*READDELYA-1 - 3 + 2 = F_CPU/BAUD_RATE
	#if defined(IR_RXPIN1) && defined(IR_RXBIT1) && defined(IR_RXPORT1)
	if (_receivePin == IR_RXPIN1) {
			asm volatile(
				"mov	%[count1],	%[rxByte]	\n\t"
				"ldi	%A[rxByte],	0x80 		\n\t"	//bit shift counter, rxByte = 1000 0000
				"WaitFirstBit1:					\n\t"	//3cycle loop
				"dec	%[count1]				\n\t"
				"brne	WaitFirstBit1			\n\t"	//when return 2 cycle
				"PrepareRead1:					\n\t"
				"mov	%[count1],	%[readDelay]\n\t"
				"sec							\n\t"	//set C flag 1
				"ReadLoop1:						\n\t"
				"sbis	%[rxPort],	%[rxPin]	\n\t"	//2 cycle (skip)
				"clc							\n\t"	//If Input 0, clear C flag
				"dec	%[count1]				\n\t"
				"brne	ReadLoop1				\n\t"	//Return Loop or go next order 2cycle	
				"ror	%[rxByte]				\n\t"
				"brcc	PrepareRead1				\n\t"
				: [rxByte] "+r" (rxByte)
				: [readDelay] "r" ((uint8_t)(IrDAParam::READDELAY+0.5)),
				  [count1] "r" (count1),
				  [rxPort] "I" (IR_RXPORT1-0x20),//0x23-0x20
				  [rxPin] "I" (IR_RXBIT1)
				);
	}
#endif /*IR_RXPIN1 && IR_RXBIT1 && IR_RXPORT1*/
#if defined(IR_RXPIN2) && defined(IR_RXBIT2) && defined(IR_RXPORT2)
	else {
			asm volatile(
				"mov	%[count1],	%[rxByte]	\n\t"
				"ldi	%A[rxByte],	0x80 		\n\t"	//bit shift counter, rxByte = 1000 0000
				"WaitFirstBit2:					\n\t"	//3cycle loop
				"dec	%[count1]				\n\t"
				"brne	WaitFirstBit2			\n\t"	//when return 2 cycle
				"PrepareRead2:					\n\t"
				"mov	%[count1],	%[readDelay]\n\t"
				"sec							\n\t"	//set C flag 0
				"ReadLoop2:						\n\t"
				"sbis	%[rxPort],	%[rxPin]	\n\t"	//2 cycle (skip)
				"clc							\n\t"	//If Input 0, set C flag HIGH(1)
				"dec	%[count1]				\n\t"
				"brne	ReadLoop2				\n\t"	//Return Loop or go next order 2cycle
				"ror	%[rxByte]				\n\t"
				"brcc	PrepareRead2				\n\t"
				: [rxByte] "+r" (rxByte)
				: [readDelay] "r" ((uint8_t)(IrDAParam::READDELAY+0.5)),
				[count1] "r" (count1),
				[rxPort] "I" (IR_RXPORT2-0x20),
				[rxPin] "I" (IR_RXBIT2)
			);
		}
#endif /*IR_RXPIN2 && IR_RXBIT2 && IR_RXPORT2*/
	return rxByte;
}
