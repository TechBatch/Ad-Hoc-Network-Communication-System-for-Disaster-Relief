#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>


/**************************************************/
//#define DECODE_NEC
#include <IRremote.hpp>
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>



#define UINT8                   unsigned char
#define UINT16                  unsigned short int
#define UINT32                  unsigned int
/*
 MU message holds data as 3 bytes -> 4-bit Header, 4-bit Finding Info, 1-byte Current Location, 1-byte Target Location
 BU message holds data as 3 bytes -> 4-bit Header, 4-bit Knowing Info, , 1-byte Target Location
*/
/*************************************************/
/*******************HEADERS********************/

#define BU1_NAME                0x1
#define BU2_NAME                0x2
#define BU3_NAME                0x3

#define MU1_NAME                0x4
#define MU2_NAME                0x5
#define MU3_NAME                0x6                 

/*************************************************/
/*******************BASE MSGS********************/

#define BU_MSG_RECEIVED         0x1
#define BU_MSG_NOT_RECEIVED     0x2

#define BU_BUSY                 0x2
#define BU_AVAILABLE            0x1

#define BU_TARGET_KNOWN         0x1
#define BU_TARGET_NOT_KNOWN     0x2

/*************************************************/
/*******************MU MSGS**********************/

#define MU_MSG_RECEIVED         0x1
#define MU_MSG_NOT_RECEIVED     0x2

#define MU_TARGET_FOUND         0x1
#define MU_TARGET_NOT_FOUND     0x2

/*************************************************/
/***********TILE IDS*************/
#define UNKNOWN     0x00

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

#define MU1_TRANSMITTER         5
#define MU1_RECEIVER            15

#define MU2_TRANSMITTER         5
#define MU2_RECEIVER            15

#define MU3_TRANSMITTER         5
#define MU3_RECEIVER            15

/*************************************************/
/*******************MOTOR PINS********************/

/*************************************************/
/*******************RFID PINS*********************/

#define SS_PIN                  17
#define RST_PIN                 27

/*************************************************/
/*******************STRUCTS*********************/


typedef struct MU_STRUCT
{   

    unsigned int uiMUHeader;
    unsigned int uiMUReceivement;
    unsigned int uiMUFinding = MU_TARGET_NOT_FOUND;
    unsigned int uiMUCurrentLocation;
    unsigned int uiMUTargetLocation = NOT_FOUND;  

}MU;

typedef struct BU_STRUCT
{
    unsigned int uiBUHeader;
    unsigned int uiBUReceivement;
    unsigned int uiBUAvailable = BU_AVAILABLE;
    unsigned int uiBUKnowledge = BU_TARGET_NOT_KNOWN;
    unsigned int uiBUTargetLocation = NOT_FOUND;

}BU;

/*********************************/

unsigned int createMessageMU(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiFinding, unsigned int uiCurrentLocation, unsigned int uiTargetLocation);
void parseMessageMU(unsigned int message); //MU parses the message coming from BU

void sendMessageMU(void* ptr); //Message sent by MU
void receiveMessageMU(void* ptr); //Message received by MU

int checkAvailability(BU* bu);
int checkDecode(unsigned int rawdata);
int checkReceivement(BU* bu);
int checkHeader(BU* bu, int i);

void resetReceivementMU(MU* mu);
void setReceivementMU(MU* mu);

int checkKnowing(BU* bu);

//void readRFID(void* ptr);

