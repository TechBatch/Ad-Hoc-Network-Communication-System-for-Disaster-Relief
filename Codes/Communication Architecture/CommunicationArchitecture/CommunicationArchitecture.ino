#include <comNetwork.h>


unsigned int uiMUAddress, uiMUCommand, uiMUHeader;

MU mu1;
MU mu2;
MU mu3;

BU bu1;
BU bu2;
BU bu3;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


int createMUMessage(unsigned int uiHeader, unsigned int uiTarget, unsigned int uiReceive, unsigned int uiContinue,
                     unsigned int uiCurrentLocation, unsigned int uiTargetLocation)
{
    switch(uiHeader)
    {
        case MU1_NAME:
            mu1.uiMessage = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiTarget << 19) | (uiReceive << 18) | (uiContinue << 17) | (uiCurrentLocation << 8) | (uiTargetLocation << 0);
        break;

        case MU2_NAME:
            mu2.uiMessage= ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiTarget << 19) | (uiReceive << 18) | (uiContinue << 17) | (uiCurrentLocation << 8) | (uiTargetLocation << 0);
        break;

        case MU3_NAME:
            mu3.uiMessage = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiTarget << 19) | (uiReceive << 18) | (uiContinue << 17) | (uiCurrentLocation << 8) | (uiTargetLocation << 0);
        break;
    }
}

int createBUMessage(unsigned int uiHeader, unsigned int uiTarget, unsigned int uiReceive, unsigned int uiContinue, unsigned int uiTargetLocation)
{
    switch(uiHeader)
    {
        case BU1_NAME:
            bu1.uiMessage = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiTarget << 16) | (uiReceive << 12) | (uiContinue << 8) | (uiTargetLocation << 0);
        break;

        case BU2_NAME:
            bu2.uiMessage = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiTarget << 16) | (uiReceive << 12) | (uiContinue << 8) | (uiTargetLocation << 0);
        break;

        case BU3_NAME:
            bu3.uiMessage = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiTarget << 16) | (uiReceive << 12) | (uiContinue << 8) | (uiTargetLocation << 0);
        break;
    }
}

void parseMUMessage(unsigned int message)
{
  uiMUAddress = IrReceiver.decodedIRData.address;
  uiMUCommand = IrReceiver.decodedIRData.command;
  uiMUHeader = ((uiMUAddress >> 20) & 0xFF);

  switch(uiMUHeader)
  {

    case MU1_NAME:
      mu1.

  }



}

