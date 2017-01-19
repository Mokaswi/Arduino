/*
* This file define serial message 
*/
//
//include
//
#include "Messages.h"
//
//Serial message defines
//
const char Serials::CHANGE_TARGET_ID[] PROGMEM = "changetargetid";
const char Serials::CHANGE_DUTY[] PROGMEM = "changeduty";

const char Serials::AHEAD_FORWARD[] PROGMEM = "aheadforward";
const char Serials::AHEAD_BACKWARD[] PROGMEM = "aheadbackward";
const char Serials::DONE_TASK[] PROGMEM = "donetask";
const char Serials::CHANGE_ROLE[] PROGMEM = "changerole";
const char Serials::ACK[] PROGMEM = "ack";

const char Serials::LED_G_ON[] PROGMEM = "ledgon";
const char Serials::LED_G_OFF[] PROGMEM = "ledgoff";
const char Serials::LED_R_ON[] PROGMEM = "ledron";
const char Serials::LED_R_OFF[] PROGMEM = "ledroff";
const char Serials::SPRING_L_ON[] PROGMEM = "springlon";
const char Serials::SPRING_L_OFF[] PROGMEM = "springloff";
const char Serials::SPRING_R_ON[] PROGMEM = "springron";
const char Serials::SPRING_R_OFF[] PROGMEM = "springroff";
const char Serials::SPRING_B_ON[] PROGMEM = "springbon";
const char Serials::SPRING_B_OFF[] PROGMEM = "springboff";
const char Serials::OMEGA_L_ON[] PROGMEM = "omegalon";
const char Serials::OMEGA_L_OFF[] PROGMEM ="omegaloff";
const char Serials::OMEGA_R_ON[] PROGMEM = "omegaron";
const char Serials::OMEGA_R_OFF[] PROGMEM = "omegaroff";
const char Serials::WRITE_ONE_PIN[] PROGMEM = "writeonepin";
const char Serials::WRITE_TWO_PINS[] PROGMEM = "writetwopin"; 

const char Serials::GET_DISTANCE_FORWARD[] PROGMEM = "getdistanceforward";
const char Serials::GET_DISTANCE_BACKWARD[] PROGMEM = "getdistancebackward";
const char Serials::GET_OFFSET_FORE_SENSOR[] PROGMEM = "getoffsetforesensor";
const char Serials::GET_OFFSET_BACK_SENSOR[] PROGMEM = "getoffsetbacksensor";
const char Serials::SET_OFFSET_FORE_SENSOR[] PROGMEM = "setoffsetforesensor";
const char Serials::SET_OFFSET_BACK_SENSOR[] PROGMEM = "setoffsetbacksensor";
const char Serials::SCAN_FORWARD[] PROGMEM = "scanforward";
const char Serials::SCAN_BACKWARD[] PROGMEM = "scanbackward";
const char Serials::RECEIVE_DISTANCE[] PROGMEM = "receivedistance";
const char Serials::RECEIVE_OFFSET1[] PROGMEM = "receiveoffset1";
const char Serials::RECEIVE_OFFSET2[] PROGMEM = "receiveoffset2";
const char Serials::SET_NEUTRAL[] PROGMEM = "setneutral";

const char Serials::CRAWL_FORWARD_AUTO[] PROGMEM = "crawlforwardauto";
const char Serials::CRAWL_FORWARD_MANUAL[] PROGMEM = "crawlforwardmanual";
const char Serials::CRAWL_BACKWARD_AUTO[] PROGMEM = "crawlbackwardauto";
const char Serials::CRAWL_BACKWARD_MANUAL[] PROGMEM = "crawlbackwardmanual";

const char Serials::VERTICAL_JUMP_AUTO[] PROGMEM = "verticaljumpauto";
const char Serials::VERTICAL_JUMP1_MANUAL[] PROGMEM = "verticaljump1manual";
const char Serials::VERTICAL_JUMP2_MANUAL[] PROGMEM = "verticaljump2manual";
const char Serials::JUMP_FORWARD_AUTO[] PROGMEM = "jumpforwardauto";
const char Serials::JUMP_FORWARD_MANUAL[] PROGMEM = "jumpforwardmanual";
const char Serials::JUMP_BACKWARD_AUTO[] PROGMEM = "jumpbackwardauto";
const char Serials::JUMP_BACKWARD_MANUAL[] PROGMEM = "jumpbackwardmanual";

const char Serials::ROLL_FORWARD1_AUTO[] PROGMEM = "rollforward1auto";
const char Serials::ROLL_FORWARD1_MANUAL[] PROGMEM = "rollforward1manual";
const char Serials::ROLL_BACKWARD_AUTO[] PROGMEM = "rollbackwardauto";
const char Serials::ROLL_BACKWARD_MANUAL[] PROGMEM = "rollbackwardmanual";

const char Serials::FOLLOW[] PROGMEM = "follow";
const char Serials::ESCAPE[] PROGMEM = "escape";
const char Serials::PUSH[] PROGMEM = "push";
const char Serials::STOP[] PROGMEM = "stop";

const char Serials::CHECK_DISTANCE[] PROGMEM = "checkdistance";
const char Serials::FINISH_DISTANCE[] PROGMEM = "finishdistance";
