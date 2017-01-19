/*
* This file define the number of command which used in infrared message
* a character of infrared message is 8bit which is 0 ~ 255
* and also declare command which used in serial message
* serial message are defined in messages.cpp
* infrared message is used between Tribots, or Tribot and basement
* Serial message is used between laptop and arduino
*/
#ifndef Messages_h
#define Messages_h
//
//include
//
#include <avr/pgmspace.h>
//
//Constants
//
const uint8_t LEN_INFRARED_BOM = 1;
const uint8_t LEN_INFRARED_EOM = 1;
const uint8_t LEN_INFRARED_HEADER = 4;
const uint8_t IDX_TO_ID_ = LEN_INFRARED_BOM + 0;
const uint8_t IDX_RELAY_FLAG = LEN_INFRARED_BOM + 1;
const uint8_t IDX_FROM_ID = LEN_INFRARED_BOM + 2;
const uint8_t IDX_FROM_ROLE = LEN_INFRARED_BOM + 3;
const uint8_t IDX_COMMAND = LEN_INFRARED_BOM + LEN_INFRARED_HEADER + 0;
const uint8_t IDX_MESSAGE1 = IDX_COMMAND + 1;
const uint8_t IDX_MESSAGE2 = IDX_COMMAND + 2;
const uint8_t IDX_EOM = IDX_MESSAGE2 + 1;
//
//struct
//
struct Infrareds {
	
	static const uint8_t BOM = 0xC0;		//192
	static const char NO_MATCH = '0';	//48
	static const uint8_t EOM = 0xC1;		//193
	
	static const uint8_t AHEAD_FORWARD = 50;
	static const uint8_t AHEAD_BACKWARD = 51;
	static const uint8_t CHANGE_ROLE = 52;
	static const uint8_t DONE_TASK = 53;
	static const uint8_t ACK = 54;
	
	static const uint8_t LED_G_ON = 71;
	static const uint8_t LED_G_OFF = 72;
	static const uint8_t LED_R_ON = 73;
	static const uint8_t LED_R_OFF = 74;
	static const uint8_t SPRING_L_ON = 75;
	static const uint8_t SPRING_L_OFF = 76;
	static const uint8_t SPRING_R_ON = 77;
	static const uint8_t SPRING_R_OFF = 78;
	static const uint8_t SPRING_B_ON = 79;
	static const uint8_t SPRING_B_OFF = 80;
	static const uint8_t OMEGA_L_ON = 81;
	static const uint8_t OMEGA_L_OFF = 82;
	static const uint8_t OMEGA_R_ON = 83;
	static const uint8_t OMEGA_R_OFF = 84;
	static const uint8_t WRITE_ONE_PIN = 85;
	static const uint8_t WRITE_TWO_PINS = 86;
	
	
	static const uint8_t GET_DISTANCE_FORWARD = 200;
	static const uint8_t GET_DISTANCE_BACKWARD = 201;
	static const uint8_t GET_OFFSET_FORE_SENSOR = 202;
	static const uint8_t GET_OFFSET_BACK_SENSOR = 203;
	static const uint8_t SET_OFFSET_FORE_SENSOR = 204;
	static const uint8_t SET_OFFSET_BACK_SENSOR = 205;
	static const uint8_t SCAN_FORWARD = 206;
	static const uint8_t SCAN_BACKWARD = 207;
	static const uint8_t RECEIVE_DISTANCE = 208;
	static const uint8_t RECEIVE_OFFSET1 = 209;
	static const uint8_t RECEIVE_OFFSET2 = 210;
	
	static const uint8_t SET_NEUTRAL = 211;
	static const uint8_t CRAWL_FORWARD_AUTO = 220;
	static const uint8_t CRAWL_FORWARD_MANUAL = 221;
	static const uint8_t CRAWL_BACKWARD_AUTO = 222;
	static const uint8_t CRAWL_BACKWARD_MANUAL = 223; 
	static const uint8_t VERTICAL_JUMP_AUTO = 230;
	static const uint8_t VERTICAL_JUMP1_MANUAL = 231;
	static const uint8_t VERTICAL_JUMP2_MANUAL = 232;
	static const uint8_t JUMP_FORWARD_AUTO = 233;
	static const uint8_t JUMP_FORWARD_MANUAL = 234;
	static const uint8_t JUMP_BACKWARD_AUTO = 235;
	static const uint8_t JUMP_BACKWARD_MANUAL = 236;
	
	static const uint8_t ROLL_FORWARD1_AUTO = 240;
	static const uint8_t ROLL_FORWARD1_MANUAL = 241;
	static const uint8_t ROLL_BACKWARD_AUTO = 242;
	static const uint8_t ROLL_BACKWARD_MANUAL = 243;
	
	static const uint8_t FOLLOW = 250;
	static const uint8_t PUSH = 252;
	static const uint8_t STOP = 253;

};
struct Serials {
	static const char CHANGE_TARGET_ID[] PROGMEM;
	static const char CHANGE_DUTY[] PROGMEM;
	
	static const char AHEAD_FORWARD[] PROGMEM;
	static const char AHEAD_BACKWARD[] PROGMEM;
	static const char CHANGE_ROLE[] PROGMEM;
	static const char DONE_TASK[] PROGMEM;
	static const char ACK[] PROGMEM;
	
	static const char LED_G_ON[] PROGMEM;
	static const char LED_G_OFF[] PROGMEM;
	static const char LED_R_ON[] PROGMEM;
	static const char LED_R_OFF[] PROGMEM;
	static const char SPRING_L_ON[] PROGMEM;
	static const char SPRING_L_OFF[] PROGMEM;
	static const char SPRING_R_ON[] PROGMEM;
	static const char SPRING_R_OFF[] PROGMEM;
	static const char SPRING_B_ON[] PROGMEM;
	static const char SPRING_B_OFF[] PROGMEM;
	static const char OMEGA_L_ON[] PROGMEM;
	static const char OMEGA_L_OFF[] PROGMEM;
	static const char OMEGA_R_ON[] PROGMEM;
	static const char OMEGA_R_OFF[] PROGMEM;
	static const char WRITE_ONE_PIN[] PROGMEM;
	static const char WRITE_TWO_PINS[] PROGMEM;
	
	static const char GET_DISTANCE_FORWARD[] PROGMEM;
	static const char GET_DISTANCE_BACKWARD[] PROGMEM;
	static const char GET_OFFSET_FORE_SENSOR[] PROGMEM;
	static const char GET_OFFSET_BACK_SENSOR[] PROGMEM;
	static const char SET_OFFSET_FORE_SENSOR[] PROGMEM;
	static const char SET_OFFSET_BACK_SENSOR[] PROGMEM;
	static const char SCAN_FORWARD[] PROGMEM;
	static const char SCAN_BACKWARD[] PROGMEM;
	static const char RECEIVE_DISTANCE[] PROGMEM;
	static const char RECEIVE_OFFSET1[] PROGMEM;
	static const char RECEIVE_OFFSET2[] PROGMEM;
	static const char SET_NEUTRAL[] PROGMEM;
	
	static const char CRAWL_FORWARD_AUTO[] PROGMEM;
	static const char CRAWL_FORWARD_MANUAL[] PROGMEM;
	static const char CRAWL_BACKWARD_AUTO[] PROGMEM;
	static const char CRAWL_BACKWARD_MANUAL[] PROGMEM;
	static const char VERTICAL_JUMP_AUTO[] PROGMEM;
	static const char VERTICAL_JUMP1_MANUAL[] PROGMEM;
	static const char VERTICAL_JUMP2_MANUAL[] PROGMEM;
	static const char JUMP_FORWARD_AUTO[] PROGMEM;
	static const char JUMP_FORWARD_MANUAL[] PROGMEM;
	static const char JUMP_BACKWARD_AUTO[] PROGMEM;
	static const char JUMP_BACKWARD_MANUAL[] PROGMEM;
	static const char ROLL_FORWARD1_AUTO[] PROGMEM;
	static const char ROLL_FORWARD1_MANUAL[] PROGMEM;
	static const char ROLL_BACKWARD_AUTO[] PROGMEM;
	static const char ROLL_BACKWARD_MANUAL[] PROGMEM;
	static const char FOLLOW[] PROGMEM;
	static const char ESCAPE[] PROGMEM;
	static const char PUSH[] PROGMEM;
	static const char STOP[] PROGMEM;
	
	static const char CHECK_DISTANCE[] PROGMEM;
	static const char FINISH_DISTANCE[] PROGMEM;
};
#endif /*Messages_h*/