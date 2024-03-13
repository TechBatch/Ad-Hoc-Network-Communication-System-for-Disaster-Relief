#include <comNetwork.h>


unsigned int uiMUAddress, uiMUCommand, uiMUHeader;

infoMU MU1;
infoMU MU2;
infoMU MU3;

infoBU BU1;
infoBU BU2;
infoBU BU3;

msgMU msgMU1;
msgMU msgMU2;
msgMU msgMU3;

msgBU msgBU1;
msgBU msgBU2;
msgBU msgBU3;

msgMU1.uiMUHeader = MU1_NAME;
msgMU2.uiMUHeader = MU2_NAME;
msgMU3.uiMUHeader = MU3_NAME;

msgBU1.uiMUHeader = BU1_NAME;
msgBU2.uiMUHeader = BU2_NAME;
msgBU3.uiMUHeader = BU3_NAME;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


unsigned int createMUMessage(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiFinding, unsigned int uiCurrentLocation, unsigned int uiTargetLocation)
{
  unsigned int message;
  message = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiReceivement << 18) | (uiFinding << 16) | (uiCurrentLocation << 8) | (uiTargetLocation << 0);
  Serial.print("Mobile Unit Message: ");
  Serial.println(message, HEX);
  return message;
}

int createBUMessage(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiAvailable, unsigned int uiKnowing, unsigned int uiTargetLocation)
{
  unsigned int message;
  message = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiReceivement << 16) | (uiAvailable << 12) | (uiKnowing << 8) | (uiTargetLocation << 0);
  Serial.print("Base Message: ");
  Serial.println(message, HEX);
  return message;
}

void parseMUMessage(unsigned int message) //BU parses the message coming from MU
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

void parseBUMessage(unsigned int message) //MU parses the message coming from BU
{
  unsigned int uiHeader = (message | 0x00F00000) >> 20;
  unsigned int uiReceivement = (message | 0x000F0000) >> 16;
  unsigned int uiAvailable = (message | 0x0000F000) >> 12;
  unsigned int uiKnowing = (message | 0x00000F00) >> 8;
  unsigned int uiTargetLocation = (message | 0x000000FF) >> 0;
  
  switch(uiHeader)
  {
    case BU1_NAME:

      MU1.uiBUHeader = uiHeader;
      MU1.uiBUReceivement = uiReceivement;
      MU1.uiBUAvailable = uiAvailable;
      MU1.uiBUKnowledge = uiKnowing;
      MU1.uiBUTargetLocation = uiTargetLocation;

    break;

    case BU2_NAME:

      MU2.uiBUHeader = uiHeader;
      MU2.uiBUReceivement = uiReceivement;
      MU2.uiBUAvailable = uiAvailable;
      MU2.uiBUKnowledge = uiKnowing;
      MU2.uiBUTargetLocation = uiTargetLocation;

    break;

    case BU3_NAME:

      MU3.uiBUHeader = uiHeader;
      MU3.uiBUReceivement = uiReceivement;
      MU3.uiBUAvailable = uiAvailable;
      MU3.uiBUKnowledge = uiKnowing;
      MU3.uiBUTargetLocation = uiTargetLocation;

    break;
  }
}
