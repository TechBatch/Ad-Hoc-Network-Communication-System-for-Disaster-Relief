#include <MUHeader.h>

#define MU_ID   1
#define BU_ID   1


/******RFID DECLARATIONS******/
MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key; 
// Init array that will store new NUID 
byte nuidPICC[4];

/******COMMUNICATION INIT******/
MU mu1;
MU mu2;
MU mu3;

BU bu1;
BU bu2;
BU bu3;

MU1.uiMUHeader = MU1_NAME;
MU2.uiMUHeader = MU2_NAME;
MU3.uiMUHeader = MU3_NAME;

BU1.uiMUHeader = BU1_NAME;
BU2.uiMUHeader = BU2_NAME;
BU3.uiMUHeader = BU3_NAME;

unsigned int locTargetBU = UNKNOWN;
unsigned int locTargetMU = UNKNOWN;

SemaphoreHandle_t semReachTileMU;   //After reaching the communication tiles semaphore is taken
SemaphoreHandle_t semStartComMU;    //If base is available semaphore is taken
SemaphoreHandle_t semEndComMU;      //When the acknowledgement is received, semaphore is taken

 

void setup() {

  /******CREATE SEMAPHORES******/
  semReachTileMU = xSemaphoreCreateBinary(); //Initially empty
  semStartComMU = xSemaphoreCreateBinary(); //Initially empty
  semEndComMU = xSemaphoreCreateBinary();   //Initially empty

  /*******RFID INITS***************/
  Serial.begin(115200);
  SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522 

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  /****************COMMUNICATION***********************/
  switch(MU_ID)
  {
    case 1: 
      IrReceiver.begin(MU1_RECEIVER);
      pinMode(MU1_TRANSMITTER, OUTPUT);
    break;

    case 2: 
      IrReceiver.begin(MU2_RECEIVER);
      pinMode(MU2_TRANSMITTER, OUTPUT);
    break;

    case 3: 
      IrReceiver.begin(MU3_RECEIVER);
      pinMode(MU3_TRANSMITTER, OUTPUT);
    break;
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

/*********************COMMUNICATION FUNCTIONS*********************************/
unsigned int createMessageMU(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiFinding, unsigned int uiCurrentLocation, unsigned int uiTargetLocation)
{
  unsigned int message;
  message = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiReceivement << 18) | (uiFinding << 16) | (uiCurrentLocation << 8) | (uiTargetLocation << 0);
  Serial.print("Mobile Unit Message: ");
  Serial.println(message, HEX);
  return message;
}

void parseMessageMU(unsigned int message) //MU parses the message coming from BU
{
  unsigned int uiHeader = (message | 0x00F00000) >> 20;
  unsigned int uiReceivement = (message | 0x000F0000) >> 16;
  unsigned int uiAvailable = (message | 0x0000F000) >> 12;
  unsigned int uiKnowing = (message | 0x00000F00) >> 8;
  unsigned int uiTargetLocation = (message | 0x000000FF) >> 0;
  
  Serial.println("Message is being parsing:");
  Serial.print("Header: ");
  Serial.println(uiHeader, HEX);
  Serial.print("Receivement: ");
  Serial.println(uiReceivement, HEX);
  Serial.print("Available: ");
  Serial.println(uiAvailable, HEX);
  Serial.print("Knowing: ");
  Serial.println(uiKnowing, HEX);
  Serial.print("Target Location: ");
  Serial.println(uiTargetLocation, HEX);

  switch(uiHeader)
  {
    case BU1_NAME:

      bu1.uiBUHeader = uiHeader;
      bu1.uiBUReceivement = uiReceivement;
      bu1.uiBUAvailable = uiAvailable;
      bu1.uiBUKnowledge = uiKnowing;
      bu1.uiBUTargetLocation = uiTargetLocation;

    break;

    case BU2_NAME:

      bu2.uiBUHeader = uiHeader;
      bu2.uiBUReceivement = uiReceivement;
      bu2.uiBUAvailable = uiAvailable;
      bu2.uiBUKnowledge = uiKnowing;
      bu2.uiBUTargetLocation = uiTargetLocation;

    break;

    case BU3_NAME:

      bu3.uiBUHeader = uiHeader;
      bu3.uiBUReceivement = uiReceivement;
      bu3.uiBUAvailable = uiAvailable;
      bu3.uiBUKnowledge = uiKnowing;
      bu3.uiBUTargetLocation = uiTargetLocation;

    break;
  }
}
void sendMessageMU(void* ptr) //Send Message task - Valid for each mobile unit
{
  int i;
  unsigned int msg;
  unsigned short int address;
  unsigned char command;
  //xSemaphoreTake(semReachTileMU, portMAX_DELAY);    //Task waits here until the semaphore is given by RFID task (CAN BE CHANGED)
  xSemaphoreTake(semStartComMU, portMAX_DELAY);     //Task waits here until the base become available   

  switch(MU_ID)
  {
    case 1:
      msg = createMUMessage(msgMU1.uiHeader, msgMU1.uiReceivement, msgMU1.uiFinding, msgMU1.CurrentLocation, msgMU1.TargetLocation);
      IrSender.begin(MU1_TRANSMITTER,ENABLE_LED_FEEDBACK);
      Serial.print("Message from MU1 is ready : ");
      Serial.println(msg, HEX);
      address = (unsigned short int)((msg & 0x00FFFF00) >> 8);
      command = (unsigned char)(msg & 0x000000FF); 
    break;

    case 2:
      msg = createMUMessage(msgMU2.uiHeader, msgMU2.uiReceivement, msgMU2.uiFinding, msgMU2.CurrentLocation, msgMU2.TargetLocation);
      IrSender.begin(MU2_TRANSMITTER,ENABLE_LED_FEEDBACK);
      Serial.print("Message from MU2 is ready : ");
      Serial.println(msg, HEX);
      address = (unsigned short int)((msg & 0x00FFFF00) >> 8);
      command = (unsigned char)(msg & 0x000000FF);
    break;
    
    case 3:
      msg = createMUMessage(msgMU3.uiHeader, msgMU3.uiReceivement, msgMU3.uiFinding, msgMU3.CurrentLocation, msgMU3.TargetLocation);
      IrSender.begin(MU3_TRANSMITTER,ENABLE_LED_FEEDBACK);
      Serial.print("Message from MU3 is ready : ");
      Serial.println(msg, HEX);
      address = (unsigned short int)((msg & 0x00FFFF00) >> 8);
      command = (unsigned char)(msg & 0x000000FF);
    break;   
  }
  while(1)
  { 

    IrSender.sendNEC(address,command,0);
    Serial.print("Message from MU is sent: ");
    Serial.println(msg, HEX);

    if(xSemaphoreTake(semEndComMU, 500) == pdTRUE)
    {
      switch(MU_ID)
      {
        case 1:
          for(i=0;i<3;i++)
          {
            IrSender.sendNEC(address,command,0);
            Serial.print("ACK message from MU is sent: ");
            Serial.println(mu1.uiReceivement, HEX);
          }
          resetReceivementMU(&mu1);
          IrSender.begin(MU1_TRANSMITTER,DISABLE_LED_FEEDBACK);
          Serial.println("MU1 transmitter is closed");
        break;

        case 2:
          for(i=0;i<3;i++)
          {
            IrSender.sendNEC(address,command,0);
            Serial.print("ACK message from MU is sent: ");
            Serial.println(mu2.uiReceivement, HEX);
          }
          resetReceivementMU(&mu2);
          IrSender.begin(MU2_TRANSMITTER,DISABLE_LED_FEEDBACK);
          Serial.println("MU2 transmitter is closed");
        break;

        case 3:
          for(i=0;i<3;i++)
          {
            IrSender.sendNEC(address,command,0);
            Serial.print("ACK message from MU is sent: ");
            Serial.println(mu3.uiReceivement, HEX);
          }
          resetReceivementMU(&mu3);
          IrSender.begin(MU3_TRANSMITTER,DISABLE_LED_FEEDBACK);
          Serial.println("MU3 transmitter is closed");
        break; 
      }
      break;
    }
  }
}

void receiveMessageMU(void* ptr)
{
  IrReceiver.decodeNEC(); 
  if(IrReceiver.decodedIRData.protocol != NEC)
  {
    Serial.println("Decoding fails. Protocol is wrong");
    IrReceiver.resume();
    return;
  }
  else
  {
    if(!checkDecode(IrReceiver.decodedIRData.decodedRawData))
    {
      Serial.println("Message is lost. Wait for the new receivement.");
      IrReceiver.resume();
      return;
    }
    else
    {
      Serial.print("BU meesage is decoded: ");
      Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
      parseMessageMU(IrReceiver.decodedIRData.decodedRawData);
      switch(BU_ID)
      {
        case 1:
          if(!checkHeader(&bu1, BU_ID))
          {
            Serial.println("Message did not come from BU1")
            IrReceiver.resume();
            return;
          }
          else
          {
            if(!checkAvailability(&bu1))
            {
              Serial.println("BU is busy wait until becomes available.");
              IrReceiver.resume();
              return;
            }
            else
            {
              xSemaphoreGive(semStartComMU);
              if(checkReceivement(&bu1))
              {
                Serial.println("ACK Message and information are received from BU1");
                setReceivementMU(&mu1);
                xSemaphoreGive(semEndComMU);
              }
              IrReceiver.resume();
              return;
            }
          }
        break;

        case 2:
          if(!checkHeader(&bu2, BU_ID))
          {
            Serial.println("Message did not come from BU2")
            IrReceiver.resume();
            return;
          }
          else
          {
            if(!checkAvailability(&bu2))
            {
              Serial.println("BU is busy wait until becomes available.");
              IrReceiver.resume();
              return;
            }
            else
            {
              xSemaphoreGive(semStartComMU);
              if(checkReceivement(&bu2))
              {
                Serial.println("ACK Message and information are received from BU2");
                setReceivementMU(&mu2);
                xSemaphoreGive(semEndComMU);
              }
              IrReceiver.resume();
              return;
            }
          }
        break;

        case 3:
          if(!checkHeader(&bu3, BU_ID))
          {
            Serial.println("Message did not come from BU3")
            IrReceiver.resume();
            return;
          }
          else
          {
            if(!checkAvailability(&bu3))
            {
              Serial.println("BU is busy wait until becomes available.");
              IrReceiver.resume();
              return;
            }
            else
            {
              xSemaphoreGive(semStartComMU);
              if(checkReceivement(&bu3))
              {
                Serial.println("ACK Message and information are received from BU3");
                setReceivementMU(&mu3);
                xSemaphoreGive(semEndComMU);
              }
              IrReceiver.resume();
              return;
            }
          }
        break;
      }
    }
  }
}

int checkAvailability(BU* bu) //Returns 1 if BU is available
{
  return (bu->uiBUAvailable == BU_AVAILABLE);
}

int checkDecode(unsigned int rawdata) //Checks the decode operation
{
  return ((rawdata & 0xFF000000) >> 24) == (~(rawdata & 0x000000FF));
}

int checkHeader(BU* bu, int i)  //Returns 1 for each BU header if the program and MU matches
{
  switch(i)
  {
    case 1:return (bu->uiBUHeader == BU1_NAME);
    case 2:return (bu->uiBUHeader == BU2_NAME);
    case 3:return (bu->uiBUHeader == BU3_NAME);
  }
}

int checkReceivement(BU* bu)  //Returns 1 if BU received message from MU
{
  return (bu->uiBUReceivement == BU_MSG_RECEIVED)
}


int checkKnowing(BU* bu)  //returns 1 if target known by BU
{
    return bu->uiBUKnowledge == BU_TARGET_KNOWN;
}

void resetReceivementMU(MU* mu) //resets receivement
{
  mu->uiMUReceivement = MU_MSG_NOT_RECEIVED;
}

void setReceivementMU(MU* mu) //sets receivement
{
  mu->uiMUReceivement = MU_MSG_RECEIVED;
}
/************************RFID FUNCTIONS******************************/
/*
void readRFID(void* ptr)
{

}
*/