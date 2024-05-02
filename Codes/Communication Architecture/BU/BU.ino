#include "BUHeader.h"

/******COMMUNICATION INIT******/
MU mu1;
MU mu2;
MU mu3;
BU bu;

unsigned int currentLoc1;
unsigned int currentLoc2;
unsigned int currentLoc3;

void setup() 
{
  mu1.uiMUHeader = MU1_NAME;
  mu2.uiMUHeader = MU2_NAME;
  mu3.uiMUHeader = MU3_NAME;
  Serial.begin(115200);
  
  IrReceiver.begin(BU_RECEIVER);
  pinMode(BU_TRANSMITTER, OUTPUT);
  IrSender.begin(BU_TRANSMITTER);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);


  xTaskCreatePinnedToCore(
    taskMotorControl,     // Task function
    "MotorControlTask",   // Task name
    2048,                // Stack size (in words)
    NULL,                 // Task input parameter
    1,                    // Priority
    NULL,                 // Task handle
    0                     // Core to run the task (0 or 1)
  );
}


void loop() {
  
  receiveMessageBU();
  sendMessageBU();
}

void sendMessageBU(void) //BU continously send message that it has
{
  unsigned int msg;
  unsigned short int address;
  unsigned char command;

  msg = createMessageBU(bu.uiBUHeader, bu.uiBUReceivement, bu.uiBURange, bu.uiBUKnowledge, bu.uiBUTargetLocation);
  address = (unsigned int)((msg & 0x0000FFFF));
  command = (unsigned int)((msg & 0x00FF0000)>>16);

  IrSender.sendNEC(address,command,0);              //Send the message that is constructed from last infos in the MU struct
  delay(10);
}

void receiveMessageBU()
{
  if(!IrReceiver.decodeNEC())
  {
    IrReceiver.resume();
    return;
  }
  else
  {
    //delay(50);
    Serial.println(IrReceiver.decodedIRData.decodedRawData,HEX); 
    if(!checkDecode(IrReceiver.decodedIRData.decodedRawData))   //If the decode causes bit lost resume and return
    {
      Serial.println("Message is lost. Wait for the new receivement.");
      IrReceiver.resume();
      return;
    }
    else
    {
      Serial.print("Message is decoded: ");
      Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
      parseMessageBU(IrReceiver.decodedIRData.decodedRawData);  //Parse the message
      if(!checkHeader())                                     //Check the header if wrong resume and return
      {
        Serial.println("Message did not come from any MU");
        IrReceiver.resume();
        return;
      }
      else
      {
        switch(getHeader(IrReceiver.decodedIRData.decodedRawData))
        {
          case 2: //MU1
            Serial.println("Message is received from MU1");
            currentLoc1 = mu1.uiMUCurrentLocation;
            if(!checkFinding(&mu1))
            {
              Serial.println("MU1 does not know the location of target");
            }
            else
            {
              Serial.print("MU1 knows the target location: ");
              Serial.println(mu1.uiMUTargetLocation, HEX);
          
              setKnowledgeBU(&bu);
              sayTargetLocation(&bu, &mu1);
            }
          break;

          case 3: //MU2
            Serial.println("Message is received from MU2");
            currentLoc2 = mu2.uiMUCurrentLocation;
            if(!checkFinding(&mu2))
            {
              Serial.println("MU does not know the location of target");
            }
            else
            {
              Serial.print("MU2 knows the target location: ");
              Serial.println(mu2.uiMUTargetLocation, HEX);
              
              setKnowledgeBU(&bu);
              sayTargetLocation(&bu, &mu2);
            }
          break;

          case 4: //MU3
            Serial.println("Message is received from MU3");
            currentLoc3 = mu3.uiMUCurrentLocation;
            if(!checkFinding(&mu3))
            {
              Serial.println("MU does not know the location of target");
            }
            else
            {
              Serial.print("MU3 knows the target location: ");
              Serial.println(mu3.uiMUTargetLocation, HEX);
              
              setKnowledgeBU(&bu);
              sayTargetLocation(&bu, &mu3);
            }
          break;
        }
        IrReceiver.resume();
        return;
      }
    }
  }
}
/*****************************************************************************
* Function: createMessageBU(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiRange, unsigned int uiKnowing, unsigned int uiTargetLocation)
* Aim:  Creates messages for BU
******************************************************************************/
unsigned int createMessageBU(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiRange, unsigned int uiKnowing, unsigned int uiTargetLocation)
{
  unsigned int message;
  message = ((~((uiHeader << 4)|(uiReceivement)))<<24) |(uiHeader << 20) | (uiReceivement << 16) | (uiRange << 12) | (uiKnowing << 8) | (uiTargetLocation << 0);
  return message;
}



/*****************************************************************************
* Function: parseMessageBU(unsigned int message)
* Aim: Parsing the message that comes from MUs
******************************************************************************/

void parseMessageBU(unsigned int message) //BU parses the message coming from MU
{
  unsigned int uiHeader = (message & 0x00F00000) >> 20;
  unsigned int uiReceivement = (message & 0x000C0000) >> 18;
  unsigned int uiFinding = (message & 0x00030000) >> 16;
  unsigned int uiCurrentLocation = (message & 0x0000FF00) >> 8;
  unsigned int uiTargetLocation = (message & 0x000000FF) >> 0;

  Serial.print("Header: ");
  Serial.println(uiHeader, HEX);
  Serial.print("Finding: ");
  Serial.println(uiFinding, HEX);
  Serial.print("MU Location: ");
  Serial.println(uiCurrentLocation, HEX);
  Serial.print("Target Location: ");
  Serial.println(uiTargetLocation, HEX);

  
  switch(uiHeader)
  {
    case MU1_NAME:

      mu1.uiMUHeader = uiHeader;
      mu1.uiMUReceivement = uiReceivement;
      mu1.uiMUFinding = uiFinding;
      mu1.uiMUCurrentLocation = uiCurrentLocation;
      mu1.uiMUTargetLocation = uiTargetLocation;

    break;

    case MU2_NAME:

      mu2.uiMUHeader = uiHeader;
      mu2.uiMUReceivement = uiReceivement;
      mu2.uiMUFinding = uiFinding;
      mu2.uiMUCurrentLocation = uiCurrentLocation;
      mu2.uiMUTargetLocation = uiTargetLocation;

    break;

    case MU3_NAME:

      mu3.uiMUHeader = uiHeader;
      mu3.uiMUReceivement = uiReceivement;
      mu3.uiMUFinding = uiFinding;
      mu3.uiMUCurrentLocation = uiCurrentLocation;
      mu3.uiMUTargetLocation = uiTargetLocation;

    break;
  }
}

/*****************************************************************************
* Function: checkDecode(unsigned int rawdata)
* Aim: Checks the decode for MU
******************************************************************************/
int checkDecode(unsigned int rawdata)
{
  return ((rawdata & 0xFF000000) >> 24) == ((~(rawdata >> 16)) & 0x000000FF);
}

/*****************************************************************************
* Function: checkHeader(void) 
* Aim: Compares the message header with BU header for MU
******************************************************************************/
int checkHeader(void)  
{
  return ((mu1.uiMUHeader == MU1_NAME) && (mu2.uiMUHeader == MU2_NAME) && (mu3.uiMUHeader == MU3_NAME));
}

/*****************************************************************************
* Function: giveHeader(unsigned int rawdata)
* Aim: After some controls, returns the MU ID
******************************************************************************/
unsigned int getHeader(unsigned int rawdata)
{
  return ((rawdata & 0x00F00000) >> 20);
}
/*****************************************************************************
* Function: checkFinding(MU* mu)
* Aim: Checks the target is found or not
******************************************************************************/
int checkFinding(MU* mu)
{
  return (mu->uiMUFinding == MU_TARGET_FOUND);
}

/*****************************************************************************
* Function: setKnowledgeBU(BU* bu)
* Aim: When the location of target is known then set the knowledge
******************************************************************************/
void setKnowledgeBU(BU* bu)
{
  bu->uiBUKnowledge = BU_TARGET_KNOWN;
}

void resetKnowledgeBU(BU* bu)
{
  bu->uiBUKnowledge = BU_TARGET_NOT_KNOWN;
}

/*****************************************************************************
* Function: sayTargetLocation(BU* bu, MU* mu)
* Aim: Passes the target location to the BU
******************************************************************************/
void sayTargetLocation(BU* bu, MU* mu)
{
  bu->uiBUTargetLocation = mu->uiMUTargetLocation;
}

/*****************DC MOTOR **********************************************/

void taskMotorControl(void *pvParameters) 
{
  while (true) 
  {
    analogWrite(PIN_ENA, 100);
    digitalWrite(PIN_IN1, HIGH); // Motorun yönünü saat yönünde kontrol et   
    digitalWrite(PIN_IN2, LOW);  // Motorun yönünü saat yönünde kontrol et
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}