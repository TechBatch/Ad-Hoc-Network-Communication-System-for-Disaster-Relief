#include "BUHeader.h"

/******COMMUNICATION INIT******/
MU mu1;
MU mu2;
MU mu3;
BU bu;

unsigned int currentLoc1;
unsigned int currentLoc2;
unsigned int currentLoc3;
unsigned long int currentTime = 0;

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
    Serial.println(uiCurrentLocation, HEX);
    Serial.print("Target Location: ");
    Serial.println(uiTargetLocation, HEX);
  }
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
void giveQueueNumber(BU* bu, unsigned int num)
{
  if(num == 1)
  {
    bu->uiBUQueue = FIRST;
  }
  else if(num == 2)
  {
    bu->uiBUQueue = SECOND;
  }
  else if(num == 3)
  {
    bu->uiBUQueue = THIRD;
  }
  else
  {
    bu->uiBUQueue = INIT;
  }
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

    IrSender.sendNEC(address,command,0);   

    vTaskDelay(400 / portTICK_PERIOD_MS);
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
        parseMessageBU(IrReceiver.decodedIRData.decodedRawData);  //Parse the message
        if(!checkHeader(IrReceiver.decodedIRData.decodedRawData))   //Check the header if wrong resume and return
        {
          //Serial.println("Message did not come from any MU");
        }
        else
        {
          if(bu.uiBUTargetLocation == UNKNOWN)
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
                  if(!checkReceivement(&mu1))
                  {
                    Serial.print("MU1 knows the target location: ");
                    Serial.println(mu1.uiMUTargetLocation, HEX);
                    
                    setReceivement(&bu, BU_MSG_RECEIVED_MU1);
                    setTalk(&bu, &mu1);
                    giveQueueNumber(&bu, QueueNumber);
                    sayTargetLocation(&bu, &mu1);
                    giveWhoFound(&bu,&mu1);
                    QueueNumber++;
                  }
                }
                break;

              case 3: //MU2
                Serial.println("Message is received from MU2");
                if(!checkFinding(&mu2))
                {
                  Serial.println("MU does not know the location of target");
                }
                else
                {
                  if(!checkReceivement(&mu2))
                  {
                    Serial.print("MU1 knows the target location: ");
                    Serial.println(mu1.uiMUTargetLocation, HEX);
                    
                    setReceivement(&bu, BU_MSG_RECEIVED_MU2);
                    setTalk(&bu, &mu2);
                    giveQueueNumber(&bu, QueueNumber);
                    sayTargetLocation(&bu, &mu2);
                    giveWhoFound(&bu,&mu1);
                    QueueNumber++;
                  }
                }
                break;

              case 4: //MU3
                Serial.println("Message is received from MU3");
                if(!checkFinding(&mu3))
                {
                  Serial.println("MU does not know the location of target");
                }
                else
                {
                  if(!checkReceivement(&mu3))
                  {
                    Serial.print("MU1 knows the target location: ");
                    Serial.println(mu1.uiMUTargetLocation, HEX);
                    
                    setReceivement(&bu, BU_MSG_RECEIVED_MU3);
                    setTalk(&bu, &mu3);
                    giveQueueNumber(&bu, QueueNumber);
                    sayTargetLocation(&bu, &mu3);
                    giveWhoFound(&bu,&mu1);
                    QueueNumber++;
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
                    giveQueueNumber(&bu,QueueNumber);
                    QueueNumber++;
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
                    giveQueueNumber(&bu,QueueNumber);
                    QueueNumber++;
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
                    giveQueueNumber(&bu,QueueNumber);
                    QueueNumber++;                    
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