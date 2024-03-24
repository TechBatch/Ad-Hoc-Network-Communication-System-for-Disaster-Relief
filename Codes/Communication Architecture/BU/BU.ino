#include <comNetwork.h>

int createMessageBU(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiAvailable, unsigned int uiKnowing, unsigned int uiTargetLocation)
{
  unsigned int message;
  message = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiReceivement << 16) | (uiAvailable << 12) | (uiKnowing << 8) | (uiTargetLocation << 0);
  Serial.print("Base Message: ");
  Serial.println(message, HEX);
  return message;
}




void parseMessageBU(unsigned int message) //BU parses the message coming from MU
{
  unsigned int uiHeader = (message | 0x00F00000) >> 20;
  unsigned int uiReceivement = (message | 0x000C0000) >> 18;
  unsigned int uiFinding = (message | 0x00030000) >> 16;
  unsigned int uiCurrentLocation = (message | 0x0000FF00) >> 8;
  unsigned int uiTargetLocation = (message | 0x000000FF) >> 0;
  
  switch(uiHeader)
  {
    case MU1_NAME:

      BU1.uiMUHeader = uiHeader;
      BU1.uiMUReceivement = uiReceivement;
      BU1.uiMUFinding = uiFinding;
      BU1.uiMUCurrentLocation = uiCurrentLocation;
      BU1.uiMUTargetLocation = uiTargetLocation;

    break;

    case MU2_NAME:

      BU2.uiMUHeader = uiHeader;
      BU2.uiMUReceivement = uiReceivement;
      BU2.uiMUFinding = uiFinding;
      BU2.uiMUCurrentLocation = uiCurrentLocation;
      BU2.uiMUTargetLocation = uiTargetLocation;

    break;

    case MU3_NAME:

      BU3.uiMUHeader = uiHeader;
      BU3.uiMUReceivement = uiReceivement;
      BU3.uiMUFinding = uiFinding;
      BU3.uiMUCurrentLocation = uiCurrentLocation;
      BU3.uiMUTargetLocation = uiTargetLocation;

    break;
  }
}