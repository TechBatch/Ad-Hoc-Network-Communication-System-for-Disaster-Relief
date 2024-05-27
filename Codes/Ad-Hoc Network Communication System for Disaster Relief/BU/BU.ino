#include "BUHeader.h"


/*setTalk her zaman en son muı değişmeli ?*/
/******COMMUNICATION INIT******/
MU mu1;
MU mu2;
MU mu3;
BU bu;

unsigned int currentLoc1;
unsigned int currentLoc2;
unsigned int currentLoc3;
unsigned long int currentTime = 0;

unsigned long int timeout1;
unsigned long int timeout2;
unsigned long int timeout3;

unsigned long int interval = 10000;

unsigned int QueueNumber = 1;



void setup() 
{
  mu1.uiMUHeader = MU1_NAME;
  mu2.uiMUHeader = MU2_NAME;
  mu3.uiMUHeader = MU3_NAME;
  Serial.begin(115200);
  
  IrReceiver.begin(BU_RECEIVER);
  pinMode(BU_TRANSMITTER, OUTPUT);
  IrSender.begin(BU_TRANSMITTER);

  /*****************TASK CREATIONS*****************/
  xTaskCreatePinnedToCore(
    taskSendMessageBU,        // Task function
    "CommunicationTaskSendBU",   // Task name
    4096,                // Stack size (in words)
    NULL,                 // Task input parameter
    1,                    // Priority
    NULL,                 // Task handle
    PRO_CPU_NUM

  );
  xTaskCreatePinnedToCore(
    taskReceiveMessageBU,       // Task function
    "CommunicationTaskReceiveBU",   // Task name
    4096,                // Stack size (in words)
    NULL,                 // Task input parameter
    1,                    // Priority
    NULL,                 // Task handle
    APP_CPU_NUM
  );
}


void loop() { 
}
/*********************************************************/
/***************COMMUNICATION FUNCTIONS*******************/


/*****************************************************************************
* Function: createMessageBU(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiRange, unsigned int uiQueue, unsigned int uiTargetLocation)
* Aim:  Creates messages for BU
******************************************************************************/
unsigned int createMessageBU(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiTalk, unsigned int uiWhoFound, unsigned int uiQueue, unsigned int uiTargetLocation)
{
  unsigned int message;
  message = ((~((uiHeader << 4)|(uiReceivement)))<<24) |(uiHeader << 20) | (uiReceivement << 16) | (uiTalk << 12) | (uiWhoFound <<10) | (uiQueue << 8) | (uiTargetLocation << 0);
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

  if(uiHeader != BU_NAME)
  {
    Serial.print("Header: ");
    Serial.println(uiHeader, HEX);
    Serial.print("Receivement: ");
    Serial.println(uiReceivement, HEX);
    Serial.print("Finding: ");
    Serial.println(uiFinding, HEX);
    Serial.print("MU Location: ");
    Serial.println(uiCurrentLocation, DEC);
    Serial.print("Target Location: ");
    Serial.println(uiTargetLocation, DEC);
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
int checkHeader(unsigned int message)  
{
  return ((((message & 0x00F00000)>>20) == MU1_NAME) || (((message & 0x00F00000)>>20) == MU2_NAME) || (((message & 0x00F00000)>>20) == MU3_NAME));
}

int checkReceivement(MU* mu)
{
  return mu->uiMUReceivement == MU_MSG_RECEIVED;
}

void setReceivement(BU* bu, unsigned int rcv)
{
  bu->uiBUReceivement = rcv;
}
void resetReceivement(BU* bu)
{
  bu->uiBUReceivement = BU_MSG_NOT_RECEIVED;
}
void giveQueueNumber(BU* bu, MU* mu)
{
  bu->uiBUQueue = mu->uiMUID;
}
void giveWhoFound(BU* bu, MU* mu)
{
  bu->uiBUWhoFound = (mu->uiMUHeader-1);
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
* Function: sayTargetLocation(BU* bu, MU* mu)
* Aim: Passes the target location to the BU
******************************************************************************/
void sayTargetLocation(BU* bu, MU* mu)
{
  bu->uiBUTargetLocation = mu->uiMUTargetLocation;
}

void setTalk(BU* bu, MU* mu)
{
  bu->uiBUTalk = mu->uiMUHeader;
}
void resetTalk(BU* bu)
{
  bu->uiBUTalk = BU_AVAILABLE;
}
int checkTalk(BU* bu, MU* mu)
{
  return (bu->uiBUTalk == mu->uiMUHeader);
}
/*****************************************************************************/
/***************************TASKS*********************************************/

/***************** COMMUNICATION *********************************************/
void taskSendMessageBU(void *pvParameters){

  unsigned int msg;
  unsigned short int address;
  unsigned char command;

  while(1) {

    msg = createMessageBU(bu.uiBUHeader, bu.uiBUReceivement, bu.uiBUTalk, bu.uiBUWhoFound, bu.uiBUQueue, bu.uiBUTargetLocation);
    address = (unsigned int)((msg & 0x0000FFFF));
    command = (unsigned int)((msg & 0x00FF0000)>>16);

    if(bu.uiBUWhoFound !=NF )
    {
      Serial.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
      IrSender.sendNEC(address,command,0); 
    }  
    vTaskDelay(258 / portTICK_PERIOD_MS);
  }

}

void taskReceiveMessageBU(void *pvParameters){

  while(1) {
    if(IrReceiver.decodeNEC())
    {
      if(!checkDecode(IrReceiver.decodedIRData.decodedRawData))   //If the decode causes bit lost resume and return
      {
        //Serial.println("Message is lost. Wait for the new receivement.");
      }
      else
      {
        //Serial.print("Message is decoded: ");
        //Serial.println(IrReceiver.decodedIRData.decodedRawData,HEX);
        parseMessageBU(IrReceiver.decodedIRData.decodedRawData);
        if(!checkHeader(IrReceiver.decodedIRData.decodedRawData))   //Check the header if wrong resume and return
        {
          //Serial.println("Message did not come from any MU");
        }
        else //Mesaj MU dan geldiyse
        {
          if(bu.uiBUTargetLocation == UNKNOWN) //Target konumu bilinmiyorsa
          {
            switch(getHeader(IrReceiver.decodedIRData.decodedRawData)) //Mesaj kimden geldi bak
            {
              case 2: //MU1
                Serial.println("Message is received from MU1");
                currentLoc1 = mu1.uiMUCurrentLocation;
                if(!checkFinding(&mu1)) //MU targeti bilmiyorsa
                {
                  Serial.println("MU1 does not know the location of target");
                }
                else //MU targeti biliyorsa
                {
                  if(!checkReceivement(&mu1)) //MU daha önce base ile konuşmamışsa
                  {
                    Serial.print("MU1 knows the target location: ");
                    Serial.println(mu1.uiMUTargetLocation, HEX);
                    mu1.uiMUID = QueueNumber;
                    setReceivement(&bu, BU_MSG_RECEIVED_MU1);
                    setTalk(&bu, &mu1);
                    giveQueueNumber(&bu, &mu1);
                    sayTargetLocation(&bu, &mu1);
                    giveWhoFound(&bu,&mu1);
                    if(QueueNumber<THIRD)
                    {
                      QueueNumber++;
                    }                   
                    timeout1 = millis();
                  }
                }
                break;

              case 3: //MU2
                Serial.println("Message is received from MU2");
                if(!checkFinding(&mu2))
                {
                  Serial.println("MU2 does not know the location of target");
                }
                else
                {
                  if(!checkReceivement(&mu2))
                  {
                    Serial.print("MU2 knows the target location: ");
                    Serial.println(mu2.uiMUTargetLocation, HEX);
                    mu2.uiMUID = QueueNumber;
                    setReceivement(&bu, BU_MSG_RECEIVED_MU2);
                    setTalk(&bu, &mu2);
                    giveQueueNumber(&bu, &mu2);
                    sayTargetLocation(&bu, &mu2);
                    giveWhoFound(&bu,&mu2);
                    if(QueueNumber<THIRD)
                    {
                      QueueNumber++;
                    }
                    timeout2 = millis();
                  }
                }
                break;

              case 4: //MU3
                Serial.println("Message is received from MU3");
                if(!checkFinding(&mu3))
                {
                  Serial.println("MU3 does not know the location of target");
                }
                else
                {
                  if(!checkReceivement(&mu3))
                  {
                    Serial.print("MU3 knows the target location: ");
                    Serial.println(mu3.uiMUTargetLocation, HEX);
                    mu3.uiMUID = QueueNumber;
                    setReceivement(&bu, BU_MSG_RECEIVED_MU3);
                    setTalk(&bu, &mu3);
                    giveQueueNumber(&bu,  &mu3);
                    sayTargetLocation(&bu, &mu3);
                    giveWhoFound(&bu,&mu3);
                    if(QueueNumber<THIRD)
                    {
                      QueueNumber++;
                    }
                    timeout3 = millis();
                  }
                }
                break;

              default:
                IrReceiver.resume();
                break;
            }
          }
          else
          {
            if(bu.uiBUTalk == BU_AVAILABLE) //Base targeti biliyosa ve müsaitse
            {
              switch(getHeader(IrReceiver.decodedIRData.decodedRawData)) //kim olduğuna bak
              {
                case 2: //MU1
                  if(!checkReceivement(&mu1)) //Eğer daha önce baseden önemli mesaj almadıysa
                  {
                    Serial.println("BU started to communicate with MU1");
                    setTalk(&bu,&mu1);
                    setReceivement(&bu, BU_MSG_RECEIVED_MU1);
                    if(mu1.uiMUID == INIT)
                    {
                      mu1.uiMUID = QueueNumber;
                    }
                    giveQueueNumber(&bu,&mu1);
                    if(QueueNumber<THIRD)
                    {
                      QueueNumber++;
                    }
                    timeout1 = millis();
                  }
                  else //aldıysa
                  {
                    Serial.println("BU does not need to communicate with MU1");
                  }
                break;

                case 3: //MU2
                  if(!checkReceivement(&mu2)) //Eğer daha önce baseden önemli mesaj almadıysa
                  {
                    Serial.println("BU started to communicate with MU2");
                    setTalk(&bu,&mu2);
                    setReceivement(&bu, BU_MSG_RECEIVED_MU2);
                    if(mu2.uiMUID == INIT)
                    {
                      mu2.uiMUID = QueueNumber;
                    }
                    giveQueueNumber(&bu,&mu2);
                    if(QueueNumber<THIRD)
                    {
                      QueueNumber++;
                    }
                    timeout2 = millis();
                  }
                  else //aldıysa
                  {
                    Serial.println("BU does not need to communicate with MU2");
                  }
                break;

                case 4: //MU3
                  if(!checkReceivement(&mu3)) //Eğer daha önce baseden önemli mesaj almadıysa
                  {
                    Serial.println("BU started to communicate with MU3");
                    setTalk(&bu,&mu3);
                    setReceivement(&bu, BU_MSG_RECEIVED_MU3);
                    if(mu3.uiMUID == INIT)
                    {
                      mu3.uiMUID = QueueNumber;
                    }
                    giveQueueNumber(&bu,&mu3);
                    if(QueueNumber<THIRD)
                    {
                      QueueNumber++;
                    }
                    timeout3 = millis();                    
                  }
                  else //aldıysa
                  {
                    Serial.println("BU does not need to communicate with MU3");
                  }
                break;

                default:
                  IrReceiver.resume();
                  break;
              }
            }
            else //Konuşma devam ederken
            {
              switch(getHeader(IrReceiver.decodedIRData.decodedRawData)) //kim olduğuna bak
              {
                case 2: //MU1
                  if(!checkTalk(&bu, &mu1))
                  {
                    Serial.println("BU is not in communicate with MU1 right now.");
                  }
                  else
                  {
                    if(!checkReceivement(&mu1)) //Eğer daha önce baseden önemli mesaj almadıysa
                    {
                      if(millis() - timeout1 >= interval)
                      {
                        mu1.uiMUReceivement = MU_MSG_RECEIVED;
                        resetTalk(&bu);
                        resetReceivement(&bu);
                        timeout1 = millis();
                      }
                      Serial.println("MU1 could not get ack.");
                    }
                    else //aldıysa
                    {
                      resetTalk(&bu);
                      resetReceivement(&bu);
                    }                    
                  }
                break;

                case 3: //MU2
                  if(!checkTalk(&bu, &mu2))
                  {
                    Serial.println("BU is not in communicate with MU2 right now.");
                  }
                  else
                  {
                    if(!checkReceivement(&mu2)) //Eğer daha önce baseden önemli mesaj almadıysa
                    {
                      if(millis() - timeout2 >= interval)
                      {
                        mu2.uiMUReceivement = MU_MSG_RECEIVED;
                        resetTalk(&bu);
                        resetReceivement(&bu);
                        timeout2 = millis();
                      }
                      Serial.println("MU2 could not get ack.");
                    }
                    else //aldıysa
                    {
                      resetTalk(&bu);
                      resetReceivement(&bu);
                      
                    }                    
                  }
                break;

                case 4: //MU3
                  if(!checkTalk(&bu, &mu3))
                  {
                    Serial.println("BU is not in communicate with MU3 right now.");
                  }
                  else
                  {
                    if(!checkReceivement(&mu3)) //Eğer daha önce baseden önemli mesaj almadıysa
                    {
                      if(millis() - timeout3 >= interval)
                      {
                        mu3.uiMUReceivement = MU_MSG_RECEIVED;
                        resetTalk(&bu);
                        resetReceivement(&bu);
                        timeout3 = millis();
                      }
                      Serial.println("MU3 could not get ack.");
                    }
                    else //aldıysa
                    {
                      resetTalk(&bu);
                      resetReceivement(&bu);
                    }                    
                  }
                break;

                default:
                  IrReceiver.resume();
                break;
              }
            }
          }
        }
      }
    }
    IrReceiver.resume();
  }
}
/*****************DC MOTOR **********************************************/
/*void taskMotorControl(void *pvParameters) 
{
  while (true) 
  {
    analogWrite(PIN_ENA, 100);
    digitalWrite(PIN_IN1, HIGH); // Motorun yönünü saat yönünde kontrol et   
    digitalWrite(PIN_IN2, LOW);  // Motorun yönünü saat yönünde kontrol et
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}*/