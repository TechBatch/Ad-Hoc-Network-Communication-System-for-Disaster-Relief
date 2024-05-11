#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>


/**************************************************/
#define DECODE_NEC
#include <IRremote.hpp>
#include <Arduino.h>
#include "RTOSConfig.h"

#define UINT8                   unsigned char
#define UINT16                  unsigned short int
#define UINT32                  unsigned int

/*************************************************/
/*******************HEADERS********************/

#define BU_NAME                 0x1

#define MU1_NAME                0x2
#define MU2_NAME                0x3
#define MU3_NAME                0x4                    

/*************************************************/
/*******************BASE MSGS********************/

#define BU_MSG_RECEIVED_MU1     0x1
#define BU_MSG_RECEIVED_MU2     0x2
#define BU_MSG_RECEIVED_MU3     0x3
#define BU_MSG_NOT_RECEIVED     0x0

#define BU_TALK_MU1             MU1_NAME
#define BU_TALK_MU2             MU2_NAME
#define BU_TALK_MU3             MU3_NAME
#define BU_AVAILABLE            0xF

#define FIRST                   0x1
#define SECOND                  0x2
#define THIRD                   0x3
#define INIT                    0x0

#define MU1_FOUND               0x1
#define MU2_FOUND               0x2
#define MU3_FOUND               0x3
#define NF                      0x0

/*************************************************/
/*******************MU MSGS**********************/

#define MU_MSG_RECEIVED         0x1
#define MU_MSG_NOT_RECEIVED     0x2

#define MU_TARGET_FOUND         0x1
#define MU_TARGET_NOT_FOUND     0x2

/*************************************************/
/***********TILE IDS*************/
#define UNKNOWN     0x77
#define TARGET      0x64

#define TILE_A1     0x01
#define TILE_A2     0x02
#define TILE_A3     0x03
#define TILE_A4     0x04
#define TILE_A5     0x05
#define TILE_A6     0x06
#define TILE_A7     0x07
#define TILE_A8     0x08
#define TILE_A9     0x09

#define TILE_B1     0x0B
#define TILE_B2     0x0C
#define TILE_B3     0x0D
#define TILE_B4     0x0E
#define TILE_B5     0x0F
#define TILE_B6     0x10
#define TILE_B7     0x11
#define TILE_B8     0x12
#define TILE_B9     0x13

#define TILE_C1     0x15
#define TILE_C2     0x16
#define TILE_C3     0x17
#define TILE_C4     0x18
#define TILE_C5     0x19
#define TILE_C6     0x1A
#define TILE_C7     0x1B
#define TILE_C8     0x1C
#define TILE_C9     0x1D

#define TILE_D1     0x1F
#define TILE_D2     0x20
#define TILE_D3     0x21
#define TILE_D4     0x22
#define TILE_D5     0x23
#define TILE_D6     0x24
#define TILE_D7     0x25
#define TILE_D8     0x26
#define TILE_D9     0x27

#define TILE_E1     0x29
#define TILE_E2     0x2A
#define TILE_E3     0x2B
#define TILE_E4     0x2C
#define TILE_E5     0x2D
#define TILE_E6     0x2E
#define TILE_E7     0x2F
#define TILE_E8     0x30
#define TILE_E9     0x31

#define TILE_F1     0x33
#define TILE_F2     0x34
#define TILE_F3     0x35
#define TILE_F4     0x36
#define TILE_F5     0x37
#define TILE_F6     0x38
#define TILE_F7     0x39
#define TILE_F8     0x3A
#define TILE_F9     0x3B

#define TILE_G1     0x3D
#define TILE_G2     0x3E
#define TILE_G3     0x3F
#define TILE_G4     0x40
#define TILE_G5     0x41
#define TILE_G6     0x42
#define TILE_G7     0x43
#define TILE_G8     0x44
#define TILE_G9     0x45

#define TILE_H1     0x47
#define TILE_H2     0x48
#define TILE_H3     0x49
#define TILE_H4     0x4A
#define TILE_H5     0x4B
#define TILE_H6     0x4C
#define TILE_H7     0x4D
#define TILE_H8     0x4E
#define TILE_H9     0x4F

#define TILE_I1     0x51
#define TILE_I2     0x52
#define TILE_I3     0x53
#define TILE_I4     0x54
#define TILE_I5     0x55
#define TILE_I6     0x56
#define TILE_I7     0x57
#define TILE_I8     0x58
#define TILE_I9     0x59


/*************************************************/
/***************COMMUNICATION PINS****************/

#define BU_TRANSMITTER         32
#define BU_RECEIVER            27

/*************************************************/
/*******************STRUCTS*********************/


typedef struct MU_STRUCT
{   
  unsigned int uiMUHeader; //initial given
  unsigned int uiMUReceivement = MU_MSG_NOT_RECEIVED; //No need
  unsigned int uiMUFinding = MU_TARGET_NOT_FOUND;   //Have to check
  unsigned int uiMUCurrentLocation = UNKNOWN;                 //Needed for GUI
  unsigned int uiMUTargetLocation = UNKNOWN;      //Have to check according to finding info
}MU;

typedef struct BU_STRUCT
{
  unsigned int uiBUHeader = BU_NAME;
  unsigned int uiBUReceivement = BU_MSG_NOT_RECEIVED;
  unsigned int uiBUTalk = BU_AVAILABLE;
  unsigned int uiBUWhoFound = NF; 
  unsigned int uiBUQueue = INIT;
  unsigned int uiBUTargetLocation = UNKNOWN;

}BU;

/*********************************/

unsigned int createMessageBU(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiTalk, unsigned int uiWhoFound, unsigned int uiQueue, unsigned int uiTargetLocation);
void parseMessageBU(unsigned int message); //BU parses the message coming from MU

unsigned int getHeader(unsigned int rawdata);
int checkFinding(MU* mu);
int checkDecode(unsigned int rawdata);
int checkHeader(unsigned int message);
void giveWhoFound(BU* bu, MU* mu);


void setTalk(BU* bu, MU* mu);
void resetTalk(BU* bu);
int checkTalk(BU* bu, MU* mu);


void resetReceivement(BU* bu);
void setReceivement(BU* bu, unsigned int rcv);
int checkReceivement(MU* mu);


void sayTargetLocation(BU* bu, MU* mu);

void taskSendMessageBU(void *pvParameters);
void taskReceiveMessageBU(void *pvParameters);

//void taskMotorControl(void *pvParameters);