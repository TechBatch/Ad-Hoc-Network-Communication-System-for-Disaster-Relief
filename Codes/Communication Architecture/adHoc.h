#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>




#define UINT8                   unsigned char
#define UINT16                  unsigned short int
#define UINT32                  unsigned int
/*
*Each message can transmit 1 byte of data and the name of the unit
*IF Target Found: 5 message is needed -> START, TARGET_FOUND, CURRENT_LOCATION, TARGET_LOCATION, STOP
*IF Target Not Found: 4 messsage is needed -> START, TARGET_NOT_FOUND, CURRENT_LOCATION, STOP
*/
/*************************************************/
/*******************HEADERS********************/

#define BU1_NAME                0x01
#define BU2_NAME                0x02
#define BU3_NAME                0x03

#define MU1_NAME                0x04
#define MU2_NAME                0x05
#define MU3_NAME                0x06                 

/*************************************************/
/*******************BASE MSGS********************/

#define BU_MSG_RECEIVED         0x01
#define BU_MSG_NOT_RECEIVED     0x02
#define BU_CONTINUE             0x03
#define BU_BUSY                 0x02
#define BU_AVAILABLE            0x01
#define BU_TARGET_FOUND         0x01
#define BU_TARGET_NOT_FOUND     0x02

/*************************************************/
/*******************MU MSGS**********************/

#define MU_MSG_RECEIVED         0x1
#define MU_MSG_NOT_RECEIVED     0x0
#define MU_CONTINUE             0x1
#define MU_WAITING              0x0
#define MU_TARGET_FOUND         0x1
#define MU_TARGET_NOT_FOUND     0x0

/*************************************************/
/***********CURRENT LOCATION MESSAGES*************/

#define CURRENT_LOCATION_A1     0x01
#define CURRENT_LOCATION_A2     0x02
#define CURRENT_LOCATION_A3     0x03
#define CURRENT_LOCATION_A4     0x04
#define CURRENT_LOCATION_A5     0x05
#define CURRENT_LOCATION_A6     0x06
#define CURRENT_LOCATION_A7     0x07
#define CURRENT_LOCATION_A8     0x08
#define CURRENT_LOCATION_A9     0x09

#define CURRENT_LOCATION_B1     0x0B
#define CURRENT_LOCATION_B2     0x0C
#define CURRENT_LOCATION_B3     0x0D
#define CURRENT_LOCATION_B4     0x0E
#define CURRENT_LOCATION_B5     0x0F
#define CURRENT_LOCATION_B6     0x10
#define CURRENT_LOCATION_B7     0x11
#define CURRENT_LOCATION_B8     0x12
#define CURRENT_LOCATION_B9     0x13

#define CURRENT_LOCATION_C1     0x15
#define CURRENT_LOCATION_C2     0x16
#define CURRENT_LOCATION_C3     0x17
#define CURRENT_LOCATION_C4     0x18
#define CURRENT_LOCATION_C5     0x19
#define CURRENT_LOCATION_C6     0x1A
#define CURRENT_LOCATION_C7     0x1B
#define CURRENT_LOCATION_C8     0x1C
#define CURRENT_LOCATION_C9     0x1D

#define CURRENT_LOCATION_D1     0x1F
#define CURRENT_LOCATION_D2     0x20
#define CURRENT_LOCATION_D3     0x21
#define CURRENT_LOCATION_D4     0x22
#define CURRENT_LOCATION_D5     0x23
#define CURRENT_LOCATION_D6     0x24
#define CURRENT_LOCATION_D7     0x25
#define CURRENT_LOCATION_D8     0x26
#define CURRENT_LOCATION_D9     0x27

#define CURRENT_LOCATION_E1     0x29
#define CURRENT_LOCATION_E2     0x2A
#define CURRENT_LOCATION_E3     0x2B
#define CURRENT_LOCATION_E4     0x2C
#define CURRENT_LOCATION_E5     0x2D
#define CURRENT_LOCATION_E6     0x2E
#define CURRENT_LOCATION_E7     0x2F
#define CURRENT_LOCATION_E8     0x30
#define CURRENT_LOCATION_E9     0x31

#define CURRENT_LOCATION_F1     0x33
#define CURRENT_LOCATION_F2     0x34
#define CURRENT_LOCATION_F3     0x35
#define CURRENT_LOCATION_F4     0x36
#define CURRENT_LOCATION_F5     0x37
#define CURRENT_LOCATION_F6     0x38
#define CURRENT_LOCATION_F7     0x39
#define CURRENT_LOCATION_F8     0x3A
#define CURRENT_LOCATION_F9     0x3B

#define CURRENT_LOCATION_G1     0x3D
#define CURRENT_LOCATION_G2     0x3E
#define CURRENT_LOCATION_G3     0x3F
#define CURRENT_LOCATION_G4     0x40
#define CURRENT_LOCATION_G5     0x41
#define CURRENT_LOCATION_G6     0x42
#define CURRENT_LOCATION_G7     0x43
#define CURRENT_LOCATION_G8     0x44
#define CURRENT_LOCATION_G9     0x45

#define CURRENT_LOCATION_H1     0x47
#define CURRENT_LOCATION_H2     0x48
#define CURRENT_LOCATION_H3     0x49
#define CURRENT_LOCATION_H4     0x4A
#define CURRENT_LOCATION_H5     0x4B
#define CURRENT_LOCATION_H6     0x4C
#define CURRENT_LOCATION_H7     0x4D
#define CURRENT_LOCATION_H8     0x4E
#define CURRENT_LOCATION_H9     0x4F

#define CURRENT_LOCATION_I1     0x51
#define CURRENT_LOCATION_I2     0x52
#define CURRENT_LOCATION_I3     0x53
#define CURRENT_LOCATION_I4     0x54
#define CURRENT_LOCATION_I5     0x55
#define CURRENT_LOCATION_I6     0x56
#define CURRENT_LOCATION_I7     0x57
#define CURRENT_LOCATION_I8     0x58
#define CURRENT_LOCATION_I9     0x59

/*************************************************/
/************TARGET LOCATION MESSAGES*************/

#define NO_DETECTION            0x00

#define TARGET_LOCATION_A1      0x01
#define TARGET_LOCATION_A2      0x02
#define TARGET_LOCATION_A3      0x03
#define TARGET_LOCATION_A4      0x04
#define TARGET_LOCATION_A5      0x05
#define TARGET_LOCATION_A6      0x06
#define TARGET_LOCATION_A7      0x07
#define TARGET_LOCATION_A8      0x08
#define TARGET_LOCATION_A9      0x09

#define TARGET_LOCATION_B1      0x0B
#define TARGET_LOCATION_B2      0x0C
#define TARGET_LOCATION_B3      0x0D
#define TARGET_LOCATION_B4      0x0E
#define TARGET_LOCATION_B5      0x0F
#define TARGET_LOCATION_B6      0x10
#define TARGET_LOCATION_B7      0x11
#define TARGET_LOCATION_B8      0x12
#define TARGET_LOCATION_B9      0x13

#define TARGET_LOCATION_C1      0x15
#define TARGET_LOCATION_C2      0x16
#define TARGET_LOCATION_C3      0x17
#define TARGET_LOCATION_C4      0x18
#define TARGET_LOCATION_C5      0x19
#define TARGET_LOCATION_C6      0x1A
#define TARGET_LOCATION_C7      0x1B
#define TARGET_LOCATION_C8      0x1C
#define TARGET_LOCATION_C9      0x1D

#define TARGET_LOCATION_D1      0x1F
#define TARGET_LOCATION_D2      0x20
#define TARGET_LOCATION_D3      0x21
#define TARGET_LOCATION_D4      0x22
#define TARGET_LOCATION_D5      0x23
#define TARGET_LOCATION_D6      0x24
#define TARGET_LOCATION_D7      0x25
#define TARGET_LOCATION_D8      0x26
#define TARGET_LOCATION_D9      0x27

#define TARGET_LOCATION_E1      0x29
#define TARGET_LOCATION_E2      0x2A
#define TARGET_LOCATION_E3      0x2B
#define TARGET_LOCATION_E4      0x2C
#define TARGET_LOCATION_E5      0x2D
#define TARGET_LOCATION_E6      0x2E
#define TARGET_LOCATION_E7      0x2F
#define TARGET_LOCATION_E8      0x30
#define TARGET_LOCATION_E9      0x31

#define TARGET_LOCATION_F1      0x33
#define TARGET_LOCATION_F2      0x34
#define TARGET_LOCATION_F3      0x35
#define TARGET_LOCATION_F4      0x36
#define TARGET_LOCATION_F5      0x37
#define TARGET_LOCATION_F6      0x38
#define TARGET_LOCATION_F7      0x39
#define TARGET_LOCATION_F8      0x3A
#define TARGET_LOCATION_F9      0x3B

#define TARGET_LOCATION_G1      0x3D
#define TARGET_LOCATION_G2      0x3E
#define TARGET_LOCATION_G3      0x3F
#define TARGET_LOCATION_G4      0x40
#define TARGET_LOCATION_G5      0x41
#define TARGET_LOCATION_G6      0x42
#define TARGET_LOCATION_G7      0x43
#define TARGET_LOCATION_G8      0x44
#define TARGET_LOCATION_G9      0x45

#define TARGET_LOCATION_H1      0x47
#define TARGET_LOCATION_H2      0x48
#define TARGET_LOCATION_H3      0x49
#define TARGET_LOCATION_H4      0x4A
#define TARGET_LOCATION_H5      0x4B
#define TARGET_LOCATION_H6      0x4C
#define TARGET_LOCATION_H7      0x4D
#define TARGET_LOCATION_H8      0x4E
#define TARGET_LOCATION_H9      0x4F

#define TARGET_LOCATION_I1      0x51
#define TARGET_LOCATION_I2      0x52
#define TARGET_LOCATION_I3      0x53
#define TARGET_LOCATION_I4      0x54
#define TARGET_LOCATION_I5      0x55
#define TARGET_LOCATION_I6      0x56
#define TARGET_LOCATION_I7      0x57
#define TARGET_LOCATION_I8      0x58
#define TARGET_LOCATION_I9      0x59

/*************************************************/
/***************COMMUNICATION PINS****************/

#define MU1_TRANSMITTER         5
#define MU1_RECEIVER            15

#define MU2_TRANSMITTER         5
#define MU2_RECEIVER            15

#define MU3_TRANSMITTER         5
#define MU3_RECEIVER            15

#define BU1_TRANSMITTER         5
#define BU1_RECEIVER            15

#define BU2_TRANSMITTER         5
#define BU2_RECEIVER            15

#define BU3_TRANSMITTER         5
#define BU3_RECEIVER            15

/*************************************************/
/*******************MOTOR PINS********************/

/*************************************************/
/*******************RFID PINS*********************/

UINT32 mu1_message = 0;
UINT32 mu2_message = 0;
UINT32 mu3_message = 0;

int createMUMessage(unsigned int uiHeader, unsigned int uiTarget, unsigned int uiReceive, unsigned int uiContinue, unsigned int uiCurrentLocation, unsigned int uiTargetLocation);

int createMUMessage(unsigned int uiHeader, unsigned int uiTarget, unsigned int uiReceive, unsigned int uiContinue,
                     unsigned int uiCurrentLocation, unsigned int uiTargetLocation)
{
    switch(uiHeader)
    {
        case MU1_NAME:
            mu1_message = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiTarget << 19) | (uiReceive << 18) | (uiContinue << 17) | (uiCurrentLocation << 8) | (uiTargetLocation << 0);
        break;

        case MU2_NAME:
            mu2_message = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiTarget << 19) | (uiReceive << 18) | (uiContinue << 17) | (uiCurrentLocation << 8) | (uiTargetLocation << 0);
        break;

        case MU3_NAME:
            mu3_message = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiTarget << 19) | (uiReceive << 18) | (uiContinue << 17) | (uiCurrentLocation << 8) | (uiTargetLocation << 0);
        break;
    }
}

int createBUMessage(unsigned int uiHeader, unsigned int uiTarget, unsigned int uiReceive, unsigned int uiContinue, unsigned int uiTargetLocation);

int createBUMessage(unsigned int uiHeader, unsigned int uiTarget, unsigned int uiReceive, unsigned int uiContinue, unsigned int uiTargetLocation)
{
    switch(uiHeader)
    {
        case BU1_NAME:
            mu1_message = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiTarget << 16) | (uiReceive << 12) | (uiContinue << 8) | (uiTargetLocation << 0);
        break;

        case BU2_NAME:
            mu2_message = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiTarget << 16) | (uiReceive << 12) | (uiContinue << 8) | (uiTargetLocation << 0);
        break;

        case BU3_NAME:
            mu3_message = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiTarget << 16) | (uiReceive << 12) | (uiContinue << 8) | (uiTargetLocation << 0);
        break;
    }
}