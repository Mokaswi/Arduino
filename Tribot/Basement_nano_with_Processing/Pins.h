/*
* This file defines Pins except pins for I2C(sensors) and Infrared transceivers
*If you want to know about these pins, please check Tiny_USI_TWI_master.h or SoftwareIrDAINT.h
*/
#ifndef Pins_h
#define Pins_h
// ATMEL ATTINY2313 4313
//							 (0) (17)		 (16)(15)
//								P		P	  V		P		P
//								D		A	  C		B		B
//								0		2	  C		7		6
//							+------------------+
//							|o								 |
//	 LED_G(1)PD1| 						     |PB5(14)
//	 LED_R(2)PA1|							     |PB4(13)*	SWI_SENSOR_FORE
//				(3)PA0|									 |PB3(12)*	SPRING_R
//				(4)PD2|									 |PB2(11)*	SPRING_B
//				(5)PD3|									 |PB1(10)		TORSIONAL_R
//							|									 |
//							+------------------+
//								P		P	  G		P		P
//								D		D	  N		D		B
//								4		5	  D		6		0
//							 (6) (7)*		 (8) (9)
//										S				T 	S
//										P				O 	W
//										R 			R 	I
//										I 			S 	_
//										N 			H 	S
//										G 			O 	E
//										_ 			N 	N
//										L 			A 	S
//														L 	E
//														_ 	R
//														L 	_
//																B
//																A
//																C
//																K
//																
// number in () is number as digital pin
// * indicates PWM port
//
//include
//
#include <stdint.h>
struct Pins{
	static const uint8_t LED_R = 2;		//PORTA1
	
	static const uint8_t SWI_SENSOR_BACK = 9;	//PORTB0 for backward sensor
	static const uint8_t TORSIONAL_R = 10;	//PORTB1
	static const uint8_t SPRING_B = 11;	//PORTB2
	static const uint8_t SPRING_R = 12;	//PORTB3
	static const uint8_t SWI_SENSOR_FORE = 13;	//PORTB4 for forward sensor
	
	static const uint8_t LED_G = 1;		//PORTD1
	static const uint8_t SPRING_L = 7;	//PORTD5
	static const uint8_t TORSIONAL_L = 8;	//PORTD6

	static const uint8_t SCANNING_READY = 254;
	static const uint8_t END = 255;
};
#endif /*Pins_h*/