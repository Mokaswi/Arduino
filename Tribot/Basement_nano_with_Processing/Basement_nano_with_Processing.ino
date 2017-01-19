/*
 * This file is for basement of Tribot version 3
 * you don't have to change anything in this file I hope....
 * If there is bug, you have to.....
 * you can upload this at normal experiments like pushing locomotion etc...
 */
//
//includes
//
#include <avr/interrupt.h>
#include "Pins.h"
#include "SoftwareIrDAINT.h"
#include "Messages.h"
#include "My_wiring.h"
//
//Constants
//
const uint8_t SERIAL_BUF_LEN = 255;
const uint8_t INFRARED_BUF_LEN = 40;
//
//Roles
const uint8_t BASEMENT = 100;
const uint8_t LEADER = 101;
const uint8_t MEASURER = 102;
const uint8_t PUSHER = 103;
const uint8_t RELAY = 104;
const uint8_t ALL = 105;
//
//This ID and Role
const uint8_t ID = 0;
const uint8_t ROLE = BASEMENT;

const uint16_t RECEIVE_TIMEOUT = 500;//1 seconds this is for Infrared commnication
//
//Global variables
//
uint8_t Serial_Idx = 0; //index
uint8_t Infrared_Idx = 0; //index

uint8_t Target_Id = 2;        //For remote control
uint8_t Duty_Spring_L = 70;   //No1 70 No2 70 No3 130
uint8_t Duty_Spring_R = 70;   //No1 60 No2 70 No3 90
uint8_t Duty_Spring_B = 80;   //No1 60 No2 80 No3 80
//
//Global arrays
//
char Serial_Buf[SERIAL_BUF_LEN];
uint8_t Infrared_Buf[INFRARED_BUF_LEN];

extern uint8_t Rx_Buf[];
//
//Functions
//
void setup() {
  Serial.begin(9600);
  EICRA |= _BV(ISC11); // INT1 fall down
  //In case, clear Interrpt flag
  if(EIFR & _BV(INTF1)) {
    EIFR |= _BV(INTF1);  
  }
  EIMSK |= _BV(INT1);  //allow interrupt at INT1 PIN
}
void loop() {
  if( rxAvailable() ) {//If Rx_buf has data Rx_buf declared in My_wiring.cpp
    rxReceiveEvent();
  } 
  if( Serial.available() ) {//If arduino receive message from laptop
    serialReceiveEvent();
  }
}
void rxReceiveEvent() {//function which is called when 
 /*
 * Infrared frame format
 * [BOM], [ID or Role which should receive message], [Relay_flag], [Command], [parameter1],[parameter2],,,[EOM]
 * BOM = Bigininng of message
 * EOM = End of message
 * When Relay_flag is 104, message is relayed by robots which has a role as relay
 */
  Infrared_Buf[0] = rxRead();//read one charactor from rx_Buf
  if (Infrared_Buf[0] == Infrareds::BOM) {//if found BOM
    Infrared_Idx = 1; //Initialize Infrared_Index
    unsigned long rxMillis = millis();// for timeout
    //
    //Beginning of Reading loop
    do {
      if (rxAvailable()){
        Infrared_Buf[Infrared_Idx++] = rxRead();//reading
      }
      if (millis() - rxMillis > RECEIVE_TIMEOUT || Infrared_Idx > INFRARED_BUF_LEN-2 ) {//If it fail to receive message propary
        Serial.println("Receive time out!");
        break;
      } 
    } while( Infrared_Buf[Infrared_Idx-1] != Infrareds::EOM );//keep loop until it find EOM
    //End of Reading loop
    //
    uint8_t saved_EIMSK = EIMSK;
    EIMSK &= ~_BV(INT1);  //Intruppt by infrared is off
    Serial.print("Time:");
    Serial.println(millis());
    //Serial.print(F("Infrared "));
    //showBuf(Infrared_Buf);
    if ( !(translateFromInfrared(Infrared_Buf)) ) {//if translation is faild
      Serial.println(F("Rx receive false"));
    }
    if( EIFR & _BV(INTF1) ) {//just in case clear interrput flag otherwise it might call interrput function  in worng way
      EIFR |= _BV(INTF1);
    }
    EIMSK = saved_EIMSK;    //load previous EIMSK which means interrput on
  }
  if( isEmptyRxBuf() ) {//If rx_buf is empty
    initializeRxIdxs();//reset Idxs
  }
}  
void serialReceiveEvent() {
 /*Plase set options of Serial moniter as Newline and 9600 baud otherwise it doesn't work well 
  * Serial frame format
  * [3 digits ID or Role which you want send message to] [Relay_flag] [Command] [parameter1] [parameter2],,,,,   
  * you should devide each messages with one space
  * For example 
  * 001 0 push 50(I use this messages when i start pushing experiment)
  * this means command no1 to push and threshold is 50mm without using relay_option
  * another example
  * 005 104 getdistanceforward
  * this means command no5 to measure distance with fore sensor with using relay_option
  * and  more
  * 105 104 crawlforwardmanual 6 50 33 20 0 15 0 30 4 50 50 0 25 20 0 15
  * 105 means this messages is sent to all Tribot. and you can send parameters after "command message"
  * If you want to make robot crawl with pre-set parameter, please use "crawlforwardauto"
  * If you wanna change duty, you should do following,
  * 000 0 changeduty 50 60 70
  * parameter1(here, 50) is for spring_L, parameter2(here, 60) is for spring_R parameter3(here, 70) is for spring_B
  * 
  * there is another option to control each spring sma quickly
  * you just have to type one charactor such as 1, 2, 3,
  */
  uint8_t translateFlag = 0;
  Serial_Buf[Serial_Idx] = Serial.read(); //Read serial
  if (Serial_Idx > SERIAL_BUF_LEN || Serial_Buf[Serial_Idx] == '\n') { //if receive EOM
    Serial_Buf[Serial_Idx] = '\0';//replace EOM with '\0'
    Serial.print(F("Serial:"));
    Serial.println(Serial_Buf); //show what itve received
    translateFlag = translateFromSerial(Serial_Buf);//translate serial messages to infrared messages
    //Serial.print("Sending ");
    //showBuf(Infrared_Buf);
    if ( translateFlag == true ) {//if it receive correct message
      //Serial.print("Time");
      //Serial.println(millis());
      sendInfrared(Infrared_Buf); //send messages via infrared transceiver
    }
    else if( translateFlag == 2) {
      Serial.println(F("this message is for this one"));
    }
    else {
      Serial.println(F("Receive false or wrong command"));//this mean failure to receive
    }
    Serial_Idx = 0; //initialize index
  }
  else { //if reading is not EOM
    Serial_Idx++; 
  }
}
void showTargetIdandDuty() {//To show TragetId adn Duty
  Serial.print("Target Id ");
  Serial.println(Target_Id);
  Serial.print("Duty L, R, B: ");
  Serial.print(Duty_Spring_L);
  Serial.print(" ");
  Serial.print(Duty_Spring_R);
  Serial.print(" ");
  Serial.println(Duty_Spring_B);
}
boolean translateFromSerial(char *buf) {
  /*this is one of the core functions which translate serial message to command
  *As I wrote above, Serial message is [IDorRole] [Relay flag] [command] [parameter....].
  *or just 1, 2, 3, ....
  *
  */
  int i = 0;
  uint8_t id_in_Infrared;       //id for Infrared message
  char *id_in_Serial;           //id for serial message
  uint8_t Relay_Flag_in_Infrared;
  char *Relay_Flag_in_Serial;
  uint8_t messages_in_Infrared[20] = {0};
  char *messages_in_Serial[20];
  if (strlen(buf) == 1){//if message is for quick control
    translateFromSerial2ForShortmessages(buf, messages_in_Infrared);
    if ( messages_in_Infrared[0] == Infrareds::NO_MATCH ) {
      return false;
    }
    id_in_Infrared = Target_Id; //send to target id
  }
  else {//Normal message
    id_in_Serial = strtok(buf, " ");//divide by ' ' and get first sequence
    if ( !isdigit(*id_in_Serial) || !isdigit(*(id_in_Serial+1)) || !isdigit(*(id_in_Serial+2)) ) {//id should be 3 digits
      Serial.println(F("Wrong number"));
      return false;
    }
    id_in_Infrared = (uint8_t)atoi(id_in_Serial);
    Relay_Flag_in_Serial = strtok(NULL, " ");//second time calling so get second sequence
    if ( Relay_Flag_in_Serial == NULL ) {//wrong message
      return false;
    } 
    Relay_Flag_in_Infrared = (uint8_t)atoi(Relay_Flag_in_Serial);//conver to int
    
    i = 0;
    //
    //loop for read command and parameters
    do {
      messages_in_Serial[i] = strtok(NULL, " ");//divide with ' '
    }while(messages_in_Serial[i++] != NULL);//until read every message
  
    for(i=1; messages_in_Serial[i] != NULL; i++) {
      messages_in_Infrared[i] = (uint8_t)atoi(messages_in_Serial[i]);
      if ( messages_in_Infrared[i] == Infrareds::BOM || messages_in_Infrared[i] == Infrareds::EOM ) {
        Serial.println("you can't use number:");
        Serial.print(Infrareds::BOM);
        Serial.print(" ");
        Serial.print(Infrareds::EOM);
        Serial.println();
        return false;
      }
    }
    messages_in_Infrared[i++] = Infrareds::EOM;
    messages_in_Infrared[i] = Infrareds::EOM;
    if ( messages_in_Serial[0] == NULL ) {
      return false;
    }
    translateFromSerial2ForLongmessages(messages_in_Serial[0], messages_in_Infrared);//here 
    if ( messages_in_Infrared[1] == Infrareds::NO_MATCH ) {
      return false;
    }
  }
  uint8_t *tmp = Infrared_Buf;
  *tmp++ = Infrareds::BOM;
  *tmp++ = id_in_Infrared; 
  *tmp++ = Relay_Flag_in_Infrared;
  *tmp++ = ID;
  *tmp++ = ROLE;
  for(i=0; messages_in_Infrared[i] != Infrareds::EOM; i++) {
   *tmp++ = messages_in_Infrared[i];
  }
  *tmp++ = Infrareds::EOM;
  /*
  Serial.print(F("Message means that "));
  for (i=0; messages_in_Serial[i] != NULL; i++){
    Serial.print(" ");
    Serial.print(messages_in_Infrared[i]);
    Serial.print(" ");
  }
  Serial.print(F(" to number "));
  Serial.println(id);
  //*/
  if ( id_in_Infrared == ID ) {
    return 2;
  }
  return true;
}
void translateFromSerial2ForShortmessages(char *serial, uint8_t *messages_in_Infrared) {//this function for quick control and set infrared messages
  /*
   * this function is for quick control
   * in this case arduino send Target_Id message and command write_one_pin or write_two_pin
   * and as parameters , it send duty
   */
  switch (*serial) {
    case '1':
      *messages_in_Infrared++ = Infrareds::WRITE_ONE_PIN;
      *messages_in_Infrared++ = Pins::SPRING_L;
      *messages_in_Infrared++ = Duty_Spring_L;
      break;
    case '2':
      *messages_in_Infrared++ = Infrareds::WRITE_ONE_PIN;
      *messages_in_Infrared++ = Pins::SPRING_L;
      *messages_in_Infrared++ = 0;
      break;
    case '3':
      *messages_in_Infrared++ = Infrareds::WRITE_ONE_PIN;
      *messages_in_Infrared++ = Pins::SPRING_R;
      *messages_in_Infrared++ = Duty_Spring_R;
      break;
    case '4':
      *messages_in_Infrared++ = Infrareds::WRITE_ONE_PIN;
      *messages_in_Infrared++ = Pins::SPRING_R;
      *messages_in_Infrared++ = 0;
      break;  
    case '5':
      *messages_in_Infrared++ = Infrareds::WRITE_ONE_PIN;
      *messages_in_Infrared++ = Pins::SPRING_B;
      *messages_in_Infrared++ = Duty_Spring_B;
      break;
    case '6':
      *messages_in_Infrared++ = Infrareds::WRITE_ONE_PIN;
      *messages_in_Infrared++ = Pins::SPRING_B;
      *messages_in_Infrared++ = 0;
      break;
    case '7':
      *messages_in_Infrared++ = Infrareds::WRITE_TWO_PINS;
      *messages_in_Infrared++ = Pins::SPRING_L;
      *messages_in_Infrared++ = Duty_Spring_L;
      *messages_in_Infrared++ = Pins::SPRING_R;
      *messages_in_Infrared++ = Duty_Spring_R;
      break;
    case '8':
      *messages_in_Infrared++ = Infrareds::WRITE_TWO_PINS;
      *messages_in_Infrared++ = Pins::SPRING_L;
      *messages_in_Infrared++ = 0;
      *messages_in_Infrared++ = Pins::SPRING_R;
      *messages_in_Infrared++ = 0;
      break;
    case '9':
      *messages_in_Infrared++ = Infrareds::WRITE_TWO_PINS;
      *messages_in_Infrared++ = Pins::SPRING_L;
      *messages_in_Infrared++ = Duty_Spring_L;
      *messages_in_Infrared++ = Pins::SPRING_B;
      *messages_in_Infrared++ = Duty_Spring_B;
      break;
    case '0':
      *messages_in_Infrared++ = Infrareds::WRITE_TWO_PINS;
      *messages_in_Infrared++ = Pins::SPRING_L;
      *messages_in_Infrared++ = 0;
      *messages_in_Infrared++ = Pins::SPRING_B;
      *messages_in_Infrared++ = 0;
      break;
    case '-':
      *messages_in_Infrared++ = Infrareds::WRITE_TWO_PINS;
      *messages_in_Infrared++ = Pins::SPRING_B;
      *messages_in_Infrared++ = Duty_Spring_B;
      *messages_in_Infrared++ = Pins::SPRING_R;
      *messages_in_Infrared++ = Duty_Spring_R;
      break;
    case '=':
      *messages_in_Infrared++ = Infrareds::WRITE_TWO_PINS;
      *messages_in_Infrared++ = Pins::SPRING_B;
      *messages_in_Infrared++ = 0;
      *messages_in_Infrared++ = Pins::SPRING_R;
      *messages_in_Infrared++ = 0;
      break;
    default:
      *messages_in_Infrared++ = Infrareds::NO_MATCH;
      break;
  }
  *messages_in_Infrared++ = Infrareds::EOM;
}
void translateFromSerial2ForLongmessages(char *serial, uint8_t *messages_in_Infrared) {
  /*
   * this is to translate nomal message to infrared command
   * I just compare command string until coincide 
   * normally you don't have to change here
   * but if you add message, you have to change here
   */
  uint8_t command = 0;
  if (!strcmp_P(serial, Serials::CHANGE_TARGET_ID)) {
    Target_Id = messages_in_Infrared[1];
    showTargetIdandDuty();
  }
  else if (!strcmp_P(serial, Serials::CHANGE_DUTY)) {
    Duty_Spring_L = messages_in_Infrared[1];
    Duty_Spring_R = messages_in_Infrared[2];
    Duty_Spring_B = messages_in_Infrared[3];
    showTargetIdandDuty();
  }
  else if (!strcmp_P(serial, Serials::AHEAD_FORWARD)) {
    command = Infrareds::AHEAD_FORWARD;
  }
  else if (!strcmp_P(serial, Serials::AHEAD_BACKWARD)) {
    command = Infrareds::AHEAD_BACKWARD;
  }
  else if (!strcmp_P(serial, Serials::CHANGE_ROLE)) {
    command = Infrareds::CHANGE_ROLE;
  }
  else if (!strcmp_P(serial, Serials::DONE_TASK)) {
    command = Infrareds::DONE_TASK;
  }
  else if (!strcmp_P(serial, Serials::ACK)) {
    command = Infrareds::ACK;
  }
  else if (!strcmp_P(serial, Serials::WRITE_ONE_PIN)) {
    command = Infrareds::WRITE_ONE_PIN;
  }
  else if (!strcmp_P(serial, Serials::WRITE_TWO_PINS)) {
    command = Infrareds::WRITE_TWO_PINS;
  }
  else if (!strcmp_P(serial, Serials::GET_DISTANCE_FORWARD)) {
    command = Infrareds::GET_DISTANCE_FORWARD;
  }
  else if (!strcmp_P(serial, Serials::GET_DISTANCE_BACKWARD)) {
    command = Infrareds::GET_DISTANCE_BACKWARD;
  }
  else if (!strcmp_P(serial, Serials::GET_OFFSET_FORE_SENSOR)) {
    command = Infrareds::GET_OFFSET_FORE_SENSOR;
  }
  else if (!strcmp_P(serial, Serials::GET_OFFSET_BACK_SENSOR)) {
    command = Infrareds::GET_OFFSET_BACK_SENSOR;
  }
  else if (!strcmp_P(serial, Serials::SET_OFFSET_FORE_SENSOR)) {
    command = Infrareds::SET_OFFSET_FORE_SENSOR;
  }
  else if (!strcmp_P(serial, Serials::SET_OFFSET_BACK_SENSOR)) {
    command = Infrareds::SET_OFFSET_BACK_SENSOR;
  }
  else if (!strcmp_P(serial, Serials::SCAN_FORWARD)) {
    command = Infrareds::SCAN_FORWARD;
  }
  else if (!strcmp_P(serial, Serials::SCAN_BACKWARD)) {
    command = Infrareds::SCAN_BACKWARD;
  }
  else if (!strcmp_P(serial, Serials::RECEIVE_DISTANCE)) {
    command = Infrareds::RECEIVE_DISTANCE;
  }
  else if (!strcmp_P(serial, Serials::RECEIVE_OFFSET1)) {
    command = Infrareds::RECEIVE_OFFSET1;
  }
  else if (!strcmp_P(serial, Serials::RECEIVE_OFFSET2)) {
    command = Infrareds::RECEIVE_OFFSET2;
  }
  else if (!strcmp_P(serial, Serials::SET_NEUTRAL)) {
    command = Infrareds::SET_NEUTRAL;
  }
  else if (!strcmp_P(serial, Serials::CRAWL_FORWARD_AUTO)) {
    command = Infrareds::CRAWL_FORWARD_AUTO;
  }
  else if (!strcmp_P(serial, Serials::CRAWL_BACKWARD_AUTO)) {
    command = Infrareds::CRAWL_BACKWARD_AUTO;
  }
  else if (!strcmp_P(serial, Serials::CRAWL_FORWARD_MANUAL)) {
    command = Infrareds::CRAWL_FORWARD_MANUAL;
  }
  else if (!strcmp_P(serial, Serials::CRAWL_BACKWARD_MANUAL)) {
    command = Infrareds::CRAWL_BACKWARD_MANUAL;
  }
  else if (!strcmp_P(serial, Serials::JUMP_FORWARD_AUTO)) {
    command = Infrareds::JUMP_FORWARD_AUTO;
  }
  else if (!strcmp_P(serial, Serials::JUMP_BACKWARD_AUTO)) {
    command = Infrareds::JUMP_BACKWARD_AUTO;
  }
  else if (!strcmp_P(serial, Serials::VERTICAL_JUMP_AUTO)) {
    command = Infrareds::VERTICAL_JUMP_AUTO;
  }
  else if (!strcmp_P(serial, Serials::VERTICAL_JUMP1_MANUAL)) {
    command = Infrareds::VERTICAL_JUMP1_MANUAL;
  }
  else if (!strcmp_P(serial, Serials::VERTICAL_JUMP2_MANUAL)) {
    command = Infrareds::VERTICAL_JUMP2_MANUAL;
  }
  else if (!strcmp_P(serial, Serials::ROLL_FORWARD1_AUTO)) {
    command = Infrareds::ROLL_FORWARD1_AUTO;
  }
  else if (!strcmp_P(serial, Serials::ROLL_FORWARD1_MANUAL)) {
    command = Infrareds::ROLL_FORWARD1_MANUAL;
  }
  else if (!strcmp_P(serial, Serials::ROLL_BACKWARD_AUTO)) {
    command = Infrareds::ROLL_BACKWARD_AUTO;
  }
  else if (!strcmp_P(serial, Serials::FOLLOW)) {
    command = Infrareds::FOLLOW;
  }
  else if (!strcmp_P(serial, Serials::PUSH)) {
    command = Infrareds::PUSH;
  }
  else if (!strcmp_P(serial, Serials::STOP)) {
    command = Infrareds::STOP;
  }
  else if (!strcmp_P(serial, Serials::CHECK_DISTANCE)) {}
  else if (!strcmp_P(serial, Serials::FINISH_DISTANCE)) {}
  else {
    command = Infrareds::NO_MATCH;
  }
  messages_in_Infrared[0] = command;
}
boolean translateFromInfrared(uint8_t *buf) {
  /*
   * This function is for translate infrared message
   * Infrared message is BOM, [ID or Role], [relay flag], [sender's ID], [sender's role], [command], [parameters],[EOM]
   */
  uint8_t receiver_Id;
  uint8_t relay_Flag;
  uint8_t sender_Id;
  uint8_t sender_Role;
  char command[40];

  buf += LEN_INFRARED_BOM;//set idx to id_idx
  receiver_Id = *buf++;
  relay_Flag = *buf++;
  sender_Id = *buf++;
  sender_Role = *buf++;
  translateFromInfrared2(buf, command);//translate command
  if (command == NULL) {
    return false;
  }
  else {
    ///*
    Serial.print(F("Command: "));
    Serial.print(command);
    Serial.print(F(" receiver's ID: "));
    Serial.print(receiver_Id);
    Serial.print(F(" relay flag: "));
    Serial.print(relay_Flag);
    Serial.print(F(" sender's id: "));
    Serial.print(sender_Id);
    Serial.print(F(" sender's role: "));
    Serial.println(sender_Role);
    //*/
    return true;
  }
}
char *translateFromInfrared2(uint8_t *infrared, char command[]){
  /*
   * This function translate infrared command to "real" command
   * For example, if ardouino receive "receive_distance" arduino show distance in serial monitor
   */
  uint16_t offset = 0;
  switch (*infrared) {
    case Infrareds::AHEAD_FORWARD:
      strcpy_P(command, Serials::AHEAD_FORWARD);
      break;
    case Infrareds::AHEAD_BACKWARD:
      strcpy_P(command, Serials::AHEAD_BACKWARD);
      break;
    case Infrareds::CHANGE_ROLE:
      strcpy_P(command, Serials::CHANGE_ROLE);
      break;
    case Infrareds::DONE_TASK:
      strcpy_P(command, Serials::DONE_TASK);
      break;
    case Infrareds::ACK:
      strcpy_P(command, Serials::ACK);
      Serial.println("ACK");
      break;
    case Infrareds::WRITE_ONE_PIN:
      strcpy_P(command, Serials::WRITE_ONE_PIN);
      break;
    case Infrareds::WRITE_TWO_PINS:
      strcpy_P(command, Serials::WRITE_TWO_PINS);
      break;  
    case Infrareds::LED_G_ON:
      strcpy_P(command, Serials::LED_G_ON);
      break;
    case Infrareds::LED_G_OFF:
      strcpy_P(command, Serials::LED_G_OFF);
      break;
    case Infrareds::LED_R_ON:
      strcpy_P(command, Serials::LED_R_ON);
      break;
    case Infrareds::LED_R_OFF:
      strcpy_P(command, Serials::LED_R_OFF);
      break;
    case Infrareds::SPRING_L_ON:
      strcpy_P(command, Serials::SPRING_L_ON);
      break;
    case Infrareds::SPRING_L_OFF:
      strcpy_P(command, Serials::SPRING_L_OFF);
      break;
    case Infrareds::SPRING_R_ON:
      strcpy_P(command, Serials::SPRING_R_ON);
      break;
    case Infrareds::SPRING_R_OFF:
      strcpy_P(command, Serials::SPRING_R_OFF);
      break;
    case Infrareds::SPRING_B_ON:
      strcpy_P(command, Serials::SPRING_B_ON);
      break;
    case Infrareds::SPRING_B_OFF:
      strcpy_P(command, Serials::SPRING_B_OFF);
      break;
    case Infrareds::OMEGA_L_ON:
      strcpy_P(command, Serials::OMEGA_L_ON);
      break;
    case Infrareds::OMEGA_L_OFF:
      strcpy_P(command, Serials::OMEGA_L_OFF);
      break;
    case Infrareds::OMEGA_R_ON:
      strcpy_P(command, Serials::OMEGA_R_ON);
      break;
    case Infrareds::OMEGA_R_OFF:
      strcpy_P(command, Serials::OMEGA_R_OFF);
      break;
    case Infrareds::GET_DISTANCE_FORWARD:
      strcpy_P(command, Serials::GET_DISTANCE_FORWARD);
      break;
    case Infrareds::GET_DISTANCE_BACKWARD:
      strcpy_P(command, Serials::GET_DISTANCE_BACKWARD);
      break;
    case Infrareds::GET_OFFSET_FORE_SENSOR:
      strcpy_P(command, Serials::GET_OFFSET_FORE_SENSOR);
      break;
    case Infrareds::GET_OFFSET_BACK_SENSOR:
      strcpy_P(command, Serials::GET_OFFSET_BACK_SENSOR);
      break;
    case Infrareds::SET_OFFSET_FORE_SENSOR:
      strcpy_P(command, Serials::SET_OFFSET_FORE_SENSOR);
      break;
    case Infrareds::SET_OFFSET_BACK_SENSOR:
      strcpy_P(command, Serials::SET_OFFSET_BACK_SENSOR);
      break;
    case Infrareds::SCAN_FORWARD:
      strcpy_P(command, Serials::SCAN_FORWARD);
      break;
    case Infrareds::SCAN_BACKWARD:
      strcpy_P(command, Serials::SCAN_BACKWARD);
      break;
    case Infrareds::RECEIVE_DISTANCE:
      strcpy_P(command, Serials::RECEIVE_DISTANCE);
      Serial.print(F("Distance:"));
      Serial.println(*(infrared+1));
      break;
    case Infrareds::RECEIVE_OFFSET1:
      strcpy_P(command, Serials::RECEIVE_OFFSET1);
      offset = (uint16_t)*(infrared+1) << 8;
      offset |= (uint16_t)*(infrared+2);
      Serial.print(F("Offset:"));
      Serial.println(offset);
      break;
    case Infrareds::RECEIVE_OFFSET2:
      strcpy_P(command, Serials::RECEIVE_OFFSET2);
      offset = (uint16_t)*(infrared+1) << 8;
      offset |= (uint16_t)*(infrared+2);
      Serial.print(F("Offset:"));
      Serial.println(offset);
      break;
    case Infrareds::SET_NEUTRAL:
      strcpy_P(command, Serials::SET_NEUTRAL);
      break; 
    case Infrareds::CRAWL_FORWARD_AUTO:
      strcpy_P(command, Serials::CRAWL_FORWARD_AUTO);
      break;
    case Infrareds::CRAWL_BACKWARD_AUTO:
      strcpy_P(command, Serials::CRAWL_BACKWARD_AUTO);
      break;
    case Infrareds::JUMP_FORWARD_AUTO:
      strcpy_P(command, Serials::JUMP_FORWARD_AUTO);
      break;
    case Infrareds::JUMP_BACKWARD_AUTO:
      strcpy_P(command, Serials::JUMP_BACKWARD_AUTO);
      break;
    case Infrareds::VERTICAL_JUMP_AUTO:
      strcpy_P(command, Serials::VERTICAL_JUMP_AUTO);
      break;
    case Infrareds::VERTICAL_JUMP1_MANUAL:
      strcpy_P(command, Serials::VERTICAL_JUMP1_MANUAL);
      break;
    case Infrareds::VERTICAL_JUMP2_MANUAL:
      strcpy_P(command, Serials::VERTICAL_JUMP2_MANUAL);
      break;
    case Infrareds::ROLL_FORWARD1_AUTO:
      strcpy_P(command, Serials::ROLL_FORWARD1_AUTO);
      break; 
    case Infrareds::ROLL_BACKWARD_AUTO:
      strcpy_P(command, Serials::ROLL_BACKWARD_AUTO);
      break; 
    case Infrareds::FOLLOW:
      strcpy_P(command, Serials::FOLLOW);
      break;
    case Infrareds::PUSH:
      strcpy_P(command, Serials::PUSH);
      break;
    case Infrareds::STOP:
      strcpy_P(command, Serials::STOP);
      break;
    default:
      command = NULL;
      break; 
  }
}
