#ifndef SoftwareIrDA_h
#define SoftwareIrDA_h
//
//include
//
#include <stdint.h>
//
//define
//
#if defined(__AVR_ATmega328__) | defined (__AVR_ATmega328P__)
#define IR_TXPIN1 4
#define IR_RXPIN1 3
#define IR_RXBIT1 PORTD3
#define IR_RXPORT1 &(PIND)

#elif defined(__AVR_ATtiny4313__) | defined(__AVR_ATtiny2313__)
#define IR_TXPIN1 3		//PORTA0
#define IR_RXPIN1 4		//PORTD2
#define IR_RXBIT1 PORTD2
#define IR_RXPORT1 &(PIND)
#define IR_TXPIN2 6		//PORTD4
#define IR_RXPIN2 5		//PORTD3
#define IR_RXBIT2 PORTD3
#define IR_RXPORT2 &(PIND)

#elif defined(__AVR_ATtiny24__) | defined(__AVR_ATtiny44__) | defined(__AVR_ATtiny84__)
#define IR_TXPIN1 8		//PORTD1
#define IR_RXPIN1 9
#define IR_RXBIT1 PORTA2
#define IR_RXPORT1 &(PINA)
#else
#error This avr is not supported
#endif
//
//const
//
/*
struct IrDAParam {
	static const uint16_t BAUD_RATE = 38400;
	static constexpr double LEN_HIGH = 2.4 * 0.001 * 0.001;;
	static constexpr double LEN_RX_PULSE = 2.2 * 0.001 * 0.001;
	#ifdef F_CPU
	static constexpr double PULSEDELAY = ( (LEN_HIGH * F_CPU)+1 ) / 3.0;
	static constexpr double TXDELAY = ( (F_CPU/BAUD_RATE) - 3*PULSEDELAY - 8.5 ) / 3.0;
	static constexpr double STARTDELAY = ( 0.5*(F_CPU/BAUD_RATE+LEN_RX_PULSE) - 57 ) / 3.0;
	static constexpr double READDELAY = ( F_CPU/BAUD_RATE - 4 ) / 5.0;
	static constexpr double STOPDELAY = ( F_CPU/BAUD_RATE + 2.0 ) / 3.0;
	#else
	#error CPU frequency F_CPU undefined
	#endif
	static const uint8_t TXPIN1;
	static const uint8_t TXPIN2;
	static const uint8_t RXPIN1;
	static const uint8_t RXPIN2;
	};//*/
///*
struct IrDAParam {
	static const uint16_t BAUD_RATE;
	static const double LEN_HIGH;
	static const double LEN_RX_PULSE;
	#ifdef F_CPU
	static const double PULSEDELAY;
	static const double TXDELAY;
	static const double STARTDELAY;
	static const double READDELAY;
	static const double STOPDELAY;
	#else
	#error CPU frequency F_CPU undefined
	#endif
	static const uint8_t TXPIN1;
	static const uint8_t TXPIN2;
	static const uint8_t RXPIN1;
	static const uint8_t RXPIN2;
};//*/
/*
struct IrDAParam {
	static const uint16_t BAUD_RATE = 38400;
	static const double LEN_HIGH = 2.4 * 0.001 * 0.001;
	static const double LEN_RX_PULSE = 2.2 * 0.001 * 0.001;
	#ifdef F_CPU
	static const double PULSEDELAY = ( (LEN_HIGH * F_CPU)+1 ) / 3.0;;
	static const double TXDELAY = ( (F_CPU/BAUD_RATE) - 3*PULSEDELAY - 8.5 ) / 3.0;
	static const double STARTDELAY = ( 0.5*(F_CPU/BAUD_RATE+LEN_RX_PULSE) - 57 ) / 3.0;
	static const double READDELAY = ( F_CPU/BAUD_RATE - 4 ) / 5.0;
	static const double STOPDELAY = ( F_CPU/BAUD_RATE + 2.0 ) / 3.0;
	#else
	#error CPU frequency F_CPU undefined
	#endif
	static const uint8_t TXPIN1;
	static const uint8_t TXPIN2;
	static const uint8_t RXPIN1;
	static const uint8_t RXPIN2;
	};//*/
//
//class
//
class SoftwareIrDA {
	
	private:
	//
	//private members
	//
	uint8_t _transmitPin;
	uint8_t _receivePin;
	volatile uint8_t rx_Buf[];
	volatile uint8_t rx_Idx;
	volatile uint8_t* _port;
	//
	//private methods
	//
	void setPINS(uint8_t recievePin, uint8_t tranamitPin);
	
	public:
	//
	// public methods
	//
	SoftwareIrDA(uint8_t channel);
	~SoftwareIrDA();
	void begin(uint8_t channel);	
	void transmitByte(uint8_t txByte);
	void transmitByte2channel(uint8_t txByte);
	uint8_t receiveByte(void);
	void transmit2Byte(uint16_t txByte);
	void transmitString(char *str);
	uint8_t available(void);
	uint8_t read(void);
};
#endif /* SoftwareIrDA_h */