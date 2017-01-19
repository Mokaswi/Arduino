/*
 * This file is for basic functions and interrupt functions
 * You don't have to change here
 * 
 */
//
//Includes
//
#include <avr/interrupt.h>
#include <Arduino.h>
#include "Messages.h"
#include "My_wiring.h"
//
//Constants
//
const uint8_t RX_BUF_LEN = 40;
//
//Global variables
//
volatile uint8_t Rx_LastRead = 0;        // number of bytes read so far
volatile uint8_t Rx_BytesAvail = 0;      // number of bytes requested but not read
//
//Global arrays
//
volatile uint8_t Rx_Buf[RX_BUF_LEN];
//
//SoftwareIrDa
//
SoftwareIrDA irPort(1);
//
//Interrupt functions(This function is called when Infrared transceiver reveive data)
//
ISR(INT1_vect) {//recieve
  Rx_Buf[Rx_BytesAvail] = irPort.receiveByte();//Receiving one charactor(8bit)
  Rx_BytesAvail++;
  if(EIFR & _BV(INTF1)) {//just in case, clear interrput flag otherwise, arduino call this function again accidentally
    EIFR |= _BV(INTF1);
  }
}
//
//Functions
//
uint8_t rxAvailable(void) {
  return Rx_BytesAvail-Rx_LastRead;
}
uint8_t rxRead(void) {
  return Rx_Buf[Rx_LastRead++];
}
void initializeRxIdxs(void) {
  Rx_BytesAvail = 0;
  Rx_LastRead = 0;
}
uint8_t isEmptyRxBuf(void) {
  if (Rx_BytesAvail == Rx_LastRead || Rx_BytesAvail > RX_BUF_LEN-2 ) {
    return true;
  }
  else {
    return false;
  }
}
void sendInfrared(uint8_t *infrared) {//send infrared message with infrared transceiver
  uint8_t *ptr = infrared;
  uint8_t saved_EIMSK = EIMSK;      ///Communication is off
  EIMSK &= ~_BV(INT1);
  do {
    delayMicroseconds(100);         //it's better to wait some microseconds
    irPort.transmitByte(*ptr++);    //Send one data
  }while( *(ptr-1) != Infrareds::EOM);
  if(EIFR & _BV(INTF1)) {//before communication turn on, you hace to clear interrupt flag if interrupt flag is on
    EIFR |= _BV(INTF1);
  }
  EIMSK = saved_EIMSK;      ///Communication is on
}
void showBuf(uint8_t buf[], uint8_t idx) {
  char i;
  for(i=0; i<idx; i++){
    Serial.print(buf[i]);
    Serial.print(' ');
  } 
  Serial.print('\n');
}
void showBuf(uint8_t *buf) {
  do {
    Serial.print(*buf++);
    Serial.print(' ');
  } while(*(buf-1) != Infrareds::EOM);
  Serial.print('\n');
}

