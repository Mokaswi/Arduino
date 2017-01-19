//
//Includes
//
#include "SoftwareIrDAINT.h"
//
//SoftwareIrDA
//
extern SoftwareIrDA irPort1;
extern SoftwareIrDA irPort2;
//
//Function Prototypes
//
//
uint8_t rxAvailable(void);
uint8_t rxRead(void);
void initializeRxIdxs(void);
uint8_t isEmptyRxBuf(void);
void sendInfrared(uint8_t infrared[], char channel);
void sendInfrared(uint8_t infrared[]);

void showBuf(uint8_t buf[], uint8_t idx);
void showBuf(uint8_t buf[]);
