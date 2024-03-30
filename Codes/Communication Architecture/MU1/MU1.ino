#include "MUHeader.h"

/******RFID DECLARATIONS******/

MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key; 
// Init array that will store new NUID 
byte nuidPICC[4];

/******COMMUNICATION INIT******/
MU mu;
BU bu;

//mu.uiMUHeader = MU1_NAME; //This will change for other MUs

unsigned int locTargetBU = UNKNOWN; //Said by BU
unsigned int locTargetMU = UNKNOWN; //Found by MU

SemaphoreHandle_t semStartSendMU;    //Starts the sending message when it is taken
SemaphoreHandle_t semEndSendMU;      //Stops the sending message when it is taken
SemaphoreHandle_t semWaitReceiveMU;     //Stops the receivement when it is taken

BaseType_t taskID;

 

void setup() {

  Serial.begin(115200);

  /******CREATE SEMAPHORES******/
  //semReachTileMU = xSemaphoreCreateBinary(); //Initially empty
  semStartSendMU = xSemaphoreCreateBinary(); //Initially empty
  semEndSendMU = xSemaphoreCreateBinary();   //Initially empty
  semWaitReceiveMU = xSemaphoreCreateBinary();   //Initially empty

  /*******RFID INITS***************/
  
  SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522 

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  /****************COMMUNICATION***********************/
  IrReceiver.begin(MU_RECEIVER);
  pinMode(MU_TRANSMITTER, OUTPUT);
  IrSender.begin(MU_TRANSMITTER);

  /********************TASK CREATIONS******************/
  if((taskID = xTaskCreate(receiveMessageMU, "IR Receive", 2048, NULL, tskIDLE_PRIORITY, NULL)) != pdPASS)
  {
    Serial.println("Task MU: IR Receive creation is failed.");
  }

  if((taskID = xTaskCreate(sendMessageMU, "IR Send", 2048, NULL, tskIDLE_PRIORITY, NULL)) != pdPASS)
  {
    Serial.println("Task MU: IR Send creation is failed.");
  }

  if((taskID = xTaskCreate(readRFID, "ReadRFID", 2048, NULL, tskIDLE_PRIORITY, NULL)) != pdPASS)
  {
    Serial.println("Task MU: Read RFID creation is failed.");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

/*********************COMMUNICATION FUNCTIONS*********************************/

/*****************************************************************************
* Function: sendMessageMU(void* ptr)
* Aim:  First task of the MUs. Sends the messages continously if they are in range, otherwise transmitter is closed.
******************************************************************************/
void sendMessageMU(void* ptr) 
{
  unsigned int msg;
  unsigned short int address;
  unsigned char command;
  
  xSemaphoreGive(semWaitReceiveMU);                  //This semaphore is used to prevent that semEndSendMU becomes overgiven
  xSemaphoreTake(semStartSendMU, portMAX_DELAY);     //Task waits here until the base become available   
  
  setReceivementMU(&mu);
  msg = createMessageMU(mu.uiMUHeader, mu.uiMUReceivement, mu.uiMUFinding, mu.uiMUCurrentLocation, mu.uiMUTargetLocation);
  address = (unsigned short int)((msg & 0x00FFFF00) >> 8);
  command = (unsigned char)(msg & 0x000000FF); 
  
  Serial.print("Message from MU is ready : ");
  Serial.println(msg, HEX);

  digitalWriteFast(MU_TRANSMITTER, HIGH);               //Transmitter is open.
  IrSender.sendNEC(address,command,0);              //Send the message that is constructed from last infos in the MU struct
  
  resetReceivementMU(&mu);
  
  Serial.print("Message from MU is sent: ");
  Serial.println(msg, HEX);

  if(xSemaphoreTake(semEndSendMU, 50) == pdTRUE)    //Wait 50 msec, if sem is not given by the receiver, skip the if block
  {
    IrSender.IRLedOff();
    //digitalWriteFast(MU_TRANSMITTER, LOW);              //Transmitter is closed
    Serial.println("MU transmitter is closed");
    
    resetBURange(&bu);
    resetReceivementMU(&mu);                        //Reset the receivement information in the MU struct

    Serial.print("ACK message from MU is sent: ");
    Serial.println(mu.uiMUReceivement, HEX);        
  }
  IrSender.IRLedOff();
  //digitalWriteFast(MU_TRANSMITTER, LOW);                //Transmitter is closed
}


/*****************************************************************************
* Function: receiveMessageMU(void* ptr)
* Aim: Second task of the MU. Continuosly arrange the receive operation of the IR communication
******************************************************************************/
void receiveMessageMU(void* ptr)
{
  IrReceiver.decodeNEC(); 
  if(IrReceiver.decodedIRData.protocol != NEC)  //If the message protocol is not NEC resume and return
  {
    Serial.println("Decoding fails. Protocol is wrong");
    IrReceiver.resume();
    return;
  }
  else
  {
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
      parseMessageMU(IrReceiver.decodedIRData.decodedRawData);  //Parse the message
      if(!checkHeader(&bu))                                     //Check the header if wrong resume and return
      {
        Serial.println("Message did not come from BU");
        IrReceiver.resume();
        return;
      }
      else
      {
        if(!checkRange(&bu))      //BU always signals IN. Initially BU struct is OUT, if the first message comes from the BU skip this block otherwise resume and return
        {
          Serial.println("Not in proper range.");
          IrReceiver.resume();
          return;
        }
        else
        {
          if(checkReceivement(&bu, &mu))   //If the message that is sent from MU to BU is received by BU give semEndSendMU to end the Communication and close the transmitter of the MU
          {
            Serial.println("ACK Message and information are received from BU.");
            setReceivementMU(&mu);   //Set the receivement info of MU
            xSemaphoreGive(semEndSendMU);
            if(!checkKnowledge(&bu))
            {
              Serial.println("Target location is not known by the BU.");
            }
            else
            {
              locTargetBU = bu.uiBUTargetLocation;
            }
          }
          else
          {
            xSemaphoreTake(semWaitReceiveMU, portMAX_DELAY);  //Wait forever until the semaphore is given by sending task. It is used for preventing the semaphore below becomes overgiven
            xSemaphoreGive(semStartSendMU); //When this semaphore is given, sending task starts.
          }
          IrReceiver.resume();
          return;
        }
      }
    }
  }
}

/*****************************************************************************
* Function: createMessageMU(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiFinding, unsigned int uiCurrentLocation, unsigned int uiTargetLocation)
* Aim:  Creating messages in predetermined format for MUs
******************************************************************************/
unsigned int createMessageMU(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiFinding, unsigned int uiCurrentLocation, unsigned int uiTargetLocation)
{
  unsigned int message;
  message = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiReceivement << 18) | (uiFinding << 16) | (uiCurrentLocation << 8) | (uiTargetLocation << 0);
  Serial.print("Mobile Unit Message: ");
  Serial.println(message, HEX);
  return message;
}

/*****************************************************************************
* Function: parseMessageMU(unsigned int message)
* Aim:  Parsing the messages that come from BU and inserts the proper values to the BU struct
******************************************************************************/
void parseMessageMU(unsigned int message)
{
  unsigned int uiHeader = (message | 0x00F00000) >> 20;
  unsigned int uiReceivement = (message | 0x000F0000) >> 16;
  unsigned int uiRange = (message | 0x0000F000) >> 12;
  unsigned int uiKnowing = (message | 0x00000F00) >> 8;
  unsigned int uiTargetLocation = (message | 0x000000FF) >> 0;
  
  Serial.println("Message is being parsing:");
  Serial.print("Header: ");
  Serial.println(uiHeader, HEX);
  Serial.print("Receivement: ");
  Serial.println(uiReceivement, HEX);
  Serial.print("Range: ");
  Serial.println(uiRange, HEX);
  Serial.print("Knowing: ");
  Serial.println(uiKnowing, HEX);
  Serial.print("Target Location: ");
  Serial.println(uiTargetLocation, HEX);
  
  bu.uiBUHeader = uiHeader;
  bu.uiBUReceivement = uiReceivement;
  bu.uiBURange = uiRange;
  bu.uiBUKnowledge = uiKnowing;
  bu.uiBUTargetLocation = uiTargetLocation; 
}


/*****************************************************************************
* Function: checkRange(BU* bu)
* Aim: Checks the range info for MU
******************************************************************************/
int checkRange(BU* bu)
{
  return (bu->uiBURange == BU_IN);
}

/*****************************************************************************
* Function: checkDecode(unsigned int rawdata)
* Aim: Checks the decode for MU
******************************************************************************/
int checkDecode(unsigned int rawdata)
{
  return ((rawdata & 0xFF000000) >> 24) == (~(rawdata & 0x000000FF));
}

/*****************************************************************************
* Function: checkHeader(BU* bu) 
* Aim: Compares the message header with BU header for MU
******************************************************************************/
int checkHeader(BU* bu)  
{
  return (bu->uiBUHeader == BU_NAME);
}

/*****************************************************************************
* Function: checkReceivement(BU* bu, MU* mu)
* Aim: Checks the receivement info of BU for MU
******************************************************************************/
int checkReceivement(BU* bu, MU* mu)
{
  return (bu->uiBUReceivement == (mu->uiMUHeader-1));
}

/*****************************************************************************
* Function: resetBURange(BU* bu)
* Aim: When the first communicaton cycle is finished it pulls the range value out
******************************************************************************/
void resetBURange(BU* bu)
{
  bu->uiBURange = BU_OUT;
}

/*****************************************************************************
* Function: resetReceivementMU(MU* mu)
* Aim: Resets the receivement info for MU
******************************************************************************/
void resetReceivementMU(MU* mu) //resets receivement
{
  mu->uiMUReceivement = MU_MSG_NOT_RECEIVED;
}

/*****************************************************************************
* Function: setReceivementMU(MU* mu)
* Aim: Sets the receivement info for MU
******************************************************************************/
void setReceivementMU(MU* mu) //sets receivement
{
  mu->uiMUReceivement = MU_MSG_RECEIVED;
}

int checkKnowledge(BU* bu)
{
  return (bu->uiBUKnowledge == BU_TARGET_KNOWN);
}
/************************RFID FUNCTIONS******************************/

void readRFID(void* ptr)
{
  
  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  if (!rfid.PICC_IsNewCardPresent()) 
  {
    mu.uiMUCurrentLocation = encodeRFID(rfid.uid.uidByte);
  }
  // Verify if the NUID has been readed
  if ( !rfid.PICC_ReadCardSerial()) {
    return;
  }
  // Store NUID into nuidPICC array
  for (byte i = 0; i < 4; i++) {
    nuidPICC[i] = rfid.uid.uidByte[i];
  }
  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();
}

int encodeRFID(byte *buffer) {

  if(buffer[0] == 99 && buffer[1] == 79 && buffer[2] == 251 && buffer[3] == 28) {
    //Serial.println("A1");
    return 1;
  }

  else if(buffer[0] == 195 && buffer[1] == 69 && buffer[2] == 167 && buffer[3] == 16) {
    //Serial.println("A2");
    return 2;
  }

  else if(buffer[0] == 115 && buffer[1] == 135 && buffer[2] == 15 && buffer[3] == 29) {
    //Serial.println("A3");
    return 3;
  }

  else if(buffer[0] == 227 && buffer[1] == 99 && buffer[2] == 8 && buffer[3] == 29) {
    //Serial.println("A4");
    return 4;
  }

  else if(buffer[0] == 51 && buffer[1] == 140 && buffer[2] == 179 && buffer[3] == 12) {
    //Serial.println("A5");
    return 5;
  }

  else if(buffer[0] == 147 && buffer[1] == 3 && buffer[2] == 248 && buffer[3] == 36) {
    //Serial.println("A6");
    return 6;
  }

  else if(buffer[0] == 243 && buffer[1] == 94 && buffer[2] == 241 && buffer[3] == 36) {
    //Serial.println("A7");
    return 7;
  }

  else if(buffer[0] == 179 && buffer[1] == 59 && buffer[2] == 8 && buffer[3] == 37) {
    //Serial.println("A8");
    return 8;
  }

  else if(buffer[0] == 195 && buffer[1] == 203 && buffer[2] == 15 && buffer[3] == 37) {
    //Serial.println("A9");
    return 9;
  }

  else if(buffer[0] == 35 && buffer[1] == 201 && buffer[2] == 0 && buffer[3] == 29) {
    //Serial.println("B1");
    return 11;
  }

  else if(buffer[0] == 67 && buffer[1] == 7 && buffer[2] == 5 && buffer[3] == 29) {
    //Serial.println("B2");
    return 12;
  }

  else if(buffer[0] == 67 && buffer[1] == 30 && buffer[2] == 14 && buffer[3] == 29) {
    //Serial.println("B3");
    return 13;
  }

  else if(buffer[0] == 195 && buffer[1] == 145 && buffer[2] == 26 && buffer[3] == 29) {
    //Serial.println("B4");
    return 14;
  }

  else if(buffer[0] == 67 && buffer[1] == 158 && buffer[2] == 10 && buffer[3] == 29) {
    //Serial.println("B5");
    return 15;
  }

  else if(buffer[0] == 115 && buffer[1] == 230 && buffer[2] == 12 && buffer[3] == 29) {
    //Serial.println("B6");
    return 16;
  }

  else if(buffer[0] == 195 && buffer[1] == 53 && buffer[2] == 10 && buffer[3] == 37) {
    //Serial.println("B7");
    return 17;
  }

  else if(buffer[0] == 51 && buffer[1] == 145 && buffer[2] == 244 && buffer[3] == 36) {
    //Serial.println("B8");
    return 18;
  }

  else if(buffer[0] == 115 && buffer[1] == 246 && buffer[2] == 155 && buffer[3] == 236) {
    //Serial.println("B9");
    return 19;
  }

  else if(buffer[0] == 99 && buffer[1] == 32 && buffer[2] == 6 && buffer[3] == 37) {
    //Serial.println("C1");
    return 21;
  }

  else if(buffer[0] == 35 && buffer[1] == 173 && buffer[2] == 9 && buffer[3] == 37) {
    //Serial.println("C2");
    return 22;
  }

  else if(buffer[0] == 51 && buffer[1] == 4 && buffer[2] == 16 && buffer[3] == 37) {
    //Serial.println("C3");
    return 23;
  }

  else if(buffer[0] == 83 && buffer[1] == 78 && buffer[2] == 248 && buffer[3] == 36) {
    //Serial.println("C4");
    return 24;
  }

  else if(buffer[0] == 211 && buffer[1] == 200 && buffer[2] == 13 && buffer[3] == 37) {
    //Serial.println("C5");
    return 25;
  }

  else if(buffer[0] == 179 && buffer[1] == 118 && buffer[2] == 0 && buffer[3] == 37) {
    //Serial.println("C6");
    return 26;
  }

  else if(buffer[0] == 179 && buffer[1] == 13 && buffer[2] == 5 && buffer[3] == 37) {
    //Serial.println("C7");
    return 27;
  }
  
  else if(buffer[0] == 115 && buffer[1] == 150 && buffer[2] == 253 && buffer[3] == 28) {
    //Serial.println("C8");
    return 28;
  }

  else if(buffer[0] == 115 && buffer[1] == 64 && buffer[2] == 1 && buffer[3] == 37) {
    //Serial.println("C9");
    return 29;
  }

  else if(buffer[0] == 179 && buffer[1] == 142 && buffer[2] == 6 && buffer[3] == 37) {
    //Serial.println("D1");
    return 31;
  }

  else if(buffer[0] == 51 && buffer[1] == 0 && buffer[2] == 10 && buffer[3] == 37) {
    //Serial.println("D2");
    return 32;
  }

  else if(buffer[0] == 163 && buffer[1] == 201 && buffer[2] == 13 && buffer[3] == 37) {
    //Serial.println("D3");
    return 33;
  }

  else if(buffer[0] == 83 && buffer[1] == 163 && buffer[2] == 254 && buffer[3] == 36) {
    //Serial.println("D4");
    return 34;
  }

  else if(buffer[0] == 115 && buffer[1] == 59 && buffer[2] == 1 && buffer[3] == 37) {
    //Serial.println("D5");
    return 35;
  }

  else if(buffer[0] == 147 && buffer[1] == 49 && buffer[2] == 14 && buffer[3] == 37) {
    //Serial.println("D6");
    return 36;
  }

  else if(buffer[0] == 19 && buffer[1] == 130 && buffer[2] == 1 && buffer[3] == 37) {
    //Serial.println("D7");
    return 37;
  }

  else if(buffer[0] == 67 && buffer[1] == 41 && buffer[2] == 255 && buffer[3] == 36) {
   // Serial.println("D8");
    return 38;
  }

  else if(buffer[0] == 35 && buffer[1] == 244 && buffer[2] == 149 && buffer[3] == 17) {
    //Serial.println("D9");
    return 39;
  }

  else if(buffer[0] == 179 && buffer[1] == 213 && buffer[2] == 244 && buffer[3] == 36) {
    //Serial.println("E1");
    return 41;
  }

  else if(buffer[0] == 3 && buffer[1] == 242 && buffer[2] == 246 && buffer[3] == 36) {
    //Serial.println("E2");
    return 42;
  }

  else if(buffer[0] == 243 && buffer[1] == 169 && buffer[2] == 10 && buffer[3] == 37) {
    //Serial.println("E3");
    return 43;
  }

  else if(buffer[0] == 51 && buffer[1] == 19 && buffer[2] == 7 && buffer[3] == 37) {
    //Serial.println("E4");
    return 44;
  }

  else if(buffer[0] == 179 && buffer[1] == 119 && buffer[2] == 9 && buffer[3] == 29) {
    //Serial.println("E5");
    return 45;
  }

  else if(buffer[0] == 195 && buffer[1] == 7 && buffer[2] == 1 && buffer[3] == 29) {
    //Serial.println("E6");
    return 46;
  }

  else if(buffer[0] == 227 && buffer[1] == 92 && buffer[2] == 25 && buffer[3] == 29) {
    //Serial.println("E7");
    return 47;
  }

  else if(buffer[0] == 51 && buffer[1] == 57 && buffer[2] == 12 && buffer[3] == 37) {
    //Serial.println("E8");
    return 48;
  }

  else if(buffer[0] == 163 && buffer[1] == 188 && buffer[2] == 193 && buffer[3] == 17) {
    //Serial.println("E9");
    return 49;
  }

  else if(buffer[0] == 211 && buffer[1] == 27 && buffer[2] == 249 && buffer[3] == 36) {
    //Serial.println("F1");
    return 51;
  }

  else if(buffer[0] == 195 && buffer[1] == 122 && buffer[2] == 252 && buffer[3] == 36) {
    //Serial.println("F2");
    return 52;
  } 

  else if(buffer[0] == 243 && buffer[1] == 144 && buffer[2] == 21 && buffer[3] == 29) {
    //Serial.println("F3");
    return 53;
  }

  else if(buffer[0] == 35 && buffer[1] == 254 && buffer[2] == 6 && buffer[3] == 29) {
    //Serial.println("F4");
    return 54;
  }

  else if(buffer[0] == 131 && buffer[1] == 151 && buffer[2] == 4 && buffer[3] == 37) {
    //Serial.println("F5");
    return 55;
  }

  else if(buffer[0] == 243 && buffer[1] == 221 && buffer[2] == 246 && buffer[3] == 36) {
    //Serial.println("F6");
    return 56;
  }

  else if(buffer[0] == 67 && buffer[1] == 239 && buffer[2] == 247 && buffer[3] == 36) {
    //Serial.println("F7");
    return 57;
  }

  else if(buffer[0] == 195 && buffer[1] == 96 && buffer[2] == 16 && buffer[3] == 37) {
    //Serial.println("F8");
    return 58;
  }

  else if(buffer[0] == 227 && buffer[1] == 170 && buffer[2] == 139 && buffer[3] == 17) {
    //Serial.println("F9");
    return 59;
  }

  else if(buffer[0] == 243 && buffer[1] == 84 && buffer[2] == 245 && buffer[3] == 36) {
    //Serial.println("G1");
    return 61;
  } 

  else if(buffer[0] == 211 && buffer[1] == 252 && buffer[2] == 255 && buffer[3] == 36) {
    //Serial.println("G2");
    return 62;
  }

  else if(buffer[0] == 115 && buffer[1] == 246 && buffer[2] == 247 && buffer[3] == 36) {
    //Serial.println("G3");
    return 63;
  }

  else if(buffer[0] == 131 && buffer[1] == 222 && buffer[2] == 10 && buffer[3] == 37) {
    //Serial.println("G4");
    return 64;
  }

  else if(buffer[0] == 179 && buffer[1] == 29 && buffer[2] == 245 && buffer[3] == 36) {
    //Serial.println("G5");
    return 65;
  }

  else if(buffer[0] == 163 && buffer[1] == 101 && buffer[2] == 14 && buffer[3] == 37) {
    //Serial.println("G6");
    return 66;
  }

    else if(buffer[0] == 211 && buffer[1] == 136 && buffer[2] == 129 && buffer[3] == 17) {
    //Serial.println("G7");
    return 67;
  }

  else if(buffer[0] == 35 && buffer[1] == 247 && buffer[2] == 177 && buffer[3] == 17) {
    //Serial.println("G8");
    return 68;
  }

  else if(buffer[0] == 67 && buffer[1] == 138 && buffer[2] == 14 && buffer[3] == 37) {
    //Serial.println("G9");
    return 69;
  }
    
  else if(buffer[0] == 131 && buffer[1] == 76 && buffer[2] == 153 && buffer[3] == 236) {
    //Serial.println("H1");
    return 71;
  } 

  else if(buffer[0] == 19 && buffer[1] == 78 && buffer[2] == 255 && buffer[3] == 236) {
    //Serial.println("H2");
    return 72;
  }

  else if(buffer[0] == 131 && buffer[1] == 227 && buffer[2] == 128 && buffer[3] == 17) {
    //Serial.println("H3");
    return 73;
  }

  else if(buffer[0] == 51 && buffer[1] == 204 && buffer[2] == 153 && buffer[3] == 17) {
    //Serial.println("H4");
    return 74;
  }

  else if(buffer[0] == 83 && buffer[1] == 146 && buffer[2] == 181 && buffer[3] == 17) {
    //Serial.println("H5");
    return 75;
  }

  else if(buffer[0] == 195 && buffer[1] == 73 && buffer[2] == 255 && buffer[3] == 236) {
    //Serial.println("H6");
    return 76;
  }

    else if(buffer[0] == 19 && buffer[1] == 53 && buffer[2] == 204 && buffer[3] == 18) {
    //Serial.println("H7");
    return 77;
  }

  else if(buffer[0] == 163 && buffer[1] == 170 && buffer[2] == 224 && buffer[3] == 17) {
    //Serial.println("H8");
    return 78;
  }

  else if(buffer[0] == 51 && buffer[1] == 200 && buffer[2] == 6 && buffer[3] == 253) {
    //Serial.println("H9");
    return 79;
  }

  else if(buffer[0] == 3 && buffer[1] == 255 && buffer[2] == 247 && buffer[3] == 36) {
    //Serial.println("I1");
    return 81;
  } 

  else if(buffer[0] == 51 && buffer[1] == 254 && buffer[2] == 248 && buffer[3] == 36) {
    //Serial.println("I2");
    return 82;
  }

  else if(buffer[0] == 163 && buffer[1] == 239 && buffer[2] == 149 && buffer[3] == 17) {
    //Serial.println("I3");
    return 83;
  }

  else if(buffer[0] == 195 && buffer[1] == 28 && buffer[2] == 232 && buffer[3] == 17) {
    //Serial.println("I4");
    return 84;
  }

  else if(buffer[0] == 99 && buffer[1] == 51 && buffer[2] == 249 && buffer[3] == 36) {
    //Serial.println("I5");
    return 85;
  }

  else if(buffer[0] == 211 && buffer[1] == 74 && buffer[2] == 202 && buffer[3] == 17) {
    //Serial.println("I6");
    return 86;
  }

    else if(buffer[0] == 35 && buffer[1] == 59 && buffer[2] == 169 && buffer[3] == 18) {
    //Serial.println("I7");
    return 87;
  }

  else if(buffer[0] == 99 && buffer[1] == 209 && buffer[2] == 173 && buffer[3] == 17) {
    //Serial.println("I8");
    return 88;
  }

  else if(buffer[0] == 3 && buffer[1] == 204 && buffer[2] == 21 && buffer[3] == 17) {
    //Serial.println("I9");
    return 89;
  }

  else {
    //Serial.println("Unknown");
    return 0;
  }
}