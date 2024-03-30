#include "BUHeader.h"

/******COMMUNICATION INIT******/
MU mu1;
MU mu2;
MU mu3;
BU bu;

unsigned int currentLoc1;
unsigned int currentLoc2;
unsigned int currentLoc3;


BaseType_t taskID;

void setup() 
{
  mu1.uiMUHeader = MU1_NAME;
  mu2.uiMUHeader = MU2_NAME;
  mu3.uiMUHeader = MU3_NAME;
  int i;
  Serial.begin(115200);
  
  //Area 1
  IrReceiver1.begin(BU1_RECEIVER, ENABLE_LED_FEEDBACK);
  
  pinMode(BU1_TRANSMITTER, OUTPUT);
  IrSender1.begin(BU1_TRANSMITTER);
  digitalWriteFast(BU1_TRANSMITTER, HIGH);

  //Area 2
  IrReceiver2.begin(BU2_RECEIVER, ENABLE_LED_FEEDBACK);
  
  pinMode(BU2_TRANSMITTER, OUTPUT);
  IrSender2.begin(BU2_TRANSMITTER);
  digitalWriteFast(BU2_TRANSMITTER, HIGH);
  
  //Area 3
  IrReceiver3.begin(BU3_RECEIVER, ENABLE_LED_FEEDBACK);
  
  pinMode(BU3_TRANSMITTER, OUTPUT);
  IrSender3.begin(BU3_TRANSMITTER);
  digitalWriteFast(BU3_TRANSMITTER, HIGH);
  
  //Area 4
  IrReceiver4.begin(BU4_RECEIVER, ENABLE_LED_FEEDBACK);
  
  pinMode(BU4_TRANSMITTER, OUTPUT);
  IrSender4.begin(BU4_TRANSMITTER);
  digitalWriteFast(BU4_TRANSMITTER, HIGH);
  
  //Area 5
  IrReceiver5.begin(BU5_RECEIVER, ENABLE_LED_FEEDBACK);
  
  pinMode(BU5_TRANSMITTER, OUTPUT);
  IrSender5.begin(BU5_TRANSMITTER);
  digitalWriteFast(BU5_TRANSMITTER, HIGH);

  //Area 6
  IrReceiver6.begin(BU6_RECEIVER, ENABLE_LED_FEEDBACK);
  
  pinMode(BU6_TRANSMITTER, OUTPUT);
  IrSender6.begin(BU6_TRANSMITTER);
  digitalWriteFast(BU6_TRANSMITTER, HIGH);

  //Area 7
  IrReceiver7.begin(BU7_RECEIVER, ENABLE_LED_FEEDBACK);
  
  pinMode(BU7_TRANSMITTER, OUTPUT);
  IrSender7.begin(BU7_TRANSMITTER);
  digitalWriteFast(BU7_TRANSMITTER, HIGH);

  //Area 8
  IrReceiver8.begin(BU8_RECEIVER, ENABLE_LED_FEEDBACK);
  
  pinMode(BU8_TRANSMITTER, OUTPUT);
  IrSender8.begin(BU8_TRANSMITTER);
  digitalWriteFast(BU8_TRANSMITTER, HIGH);

  /********************TASK CREATIONS******************/
  for(i = 1; i < 9; i++)
  {
    if((taskID = xTaskCreate(receiveMessageBU, "IR Receive", 2048, (void*)i, tskIDLE_PRIORITY, NULL)) != pdPASS)
    {
      Serial.print("Task BU: IR Receive ");
      Serial.print(i,DEC);
      Serial.println(" creation is failed.");
    }
  }
  if((taskID = xTaskCreate(sendMessageBU, "IR Send", 2048, NULL, tskIDLE_PRIORITY, NULL)) != pdPASS)
  {
    Serial.println("Task BU: IR Send creation is failed.");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

void sendMessageBU(void* ptr) //BU continously send message that it has
{
  while(1)
  {
    unsigned int msg;
    unsigned short int address;
    unsigned char command;

    msg = createMessageBU(bu.uiBUHeader, bu.uiBUReceivement, bu.uiBURange, bu.uiBUKnowledge, bu.uiBUTargetLocation);
    address = (unsigned short int)((msg & 0x00FFFF00) >> 8);
    command = (unsigned char)(msg & 0x000000FF);

    Serial.print("Message from BU is ready : ");
    Serial.println(msg, HEX);

    IrSender1.sendNEC(address,command,0);              //Send the message that is constructed from last infos in the MU struct
    IrSender2.sendNEC(address,command,0); 
    IrSender3.sendNEC(address,command,0); 
    IrSender4.sendNEC(address,command,0); 
    IrSender5.sendNEC(address,command,0); 
    IrSender6.sendNEC(address,command,0); 
    IrSender7.sendNEC(address,command,0); 
    IrSender8.sendNEC(address,command,0);

    resetReceivementBU(&bu);                          //Reset the receivement after sending the message

    Serial.print("Message from BU is sent: ");
    Serial.println(msg, HEX);
  } 
}

void receiveMessageBU(void* ptr)
{
  int area = *((int*)ptr);
  while(1)
  {
    switch(area) //Activate according to task index
    {
      case 1:
        IrReceiver1.decodeNEC(); 
        if(IrReceiver1.decodedIRData.protocol != NEC)  //If the message protocol is not NEC resume and return
        {
          Serial.println("Decoding fails. Protocol is wrong");
          IrReceiver1.resume();
        }
        else
        {
          if(!checkDecode(IrReceiver1.decodedIRData.decodedRawData))   //If the decode causes bit lost resume and return
          {
            Serial.println("Message is lost. Wait for the new receivement.");
            IrReceiver1.resume();
          }
          else
          {
            Serial.print("Message is decoded: ");
            Serial.println(IrReceiver1.decodedIRData.decodedRawData, HEX);
            parseMessageBU(IrReceiver1.decodedIRData.decodedRawData);  //Parse the message
            if(!checkHeader())                                     //Check the header if wrong resume and return
            {
              Serial.println("Message did not come from any MU");
              IrReceiver1.resume();
            }
            else
            {
              switch(getHeader(IrReceiver1.decodedIRData.decodedRawData))
              {
                case 2: //MU1
                  Serial.println("Message is received from MU1");
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU1);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU2);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU3);
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
              IrReceiver1.resume();
            }
          }
        }
      break;

      case 2:
        IrReceiver2.decodeNEC(); 
        if(IrReceiver2.decodedIRData.protocol != NEC)  //If the message protocol is not NEC resume and return
        {
          Serial.println("Decoding fails. Protocol is wrong");
          IrReceiver2.resume();
        }
        else
        {
          if(!checkDecode(IrReceiver2.decodedIRData.decodedRawData))   //If the decode causes bit lost resume and return
          {
            Serial.println("Message is lost. Wait for the new receivement.");
            IrReceiver2.resume();
          }
          else
          {
            Serial.print("Message is decoded: ");
            Serial.println(IrReceiver2.decodedIRData.decodedRawData, HEX);
            parseMessageBU(IrReceiver2.decodedIRData.decodedRawData);  //Parse the message
            if(!checkHeader())                                     //Check the header if wrong resume and return
            {
              Serial.println("Message did not come from any MU");
              IrReceiver2.resume();
            }
            else
            {
              switch(getHeader(IrReceiver2.decodedIRData.decodedRawData))
              {
                case 2: //MU1
                  Serial.println("Message is received from MU1");
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU1);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU2);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU3);
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
              IrReceiver2.resume();
            }
          }
        }
      break;

      case 3:
        IrReceiver3.decodeNEC(); 
        if(IrReceiver3.decodedIRData.protocol != NEC)  //If the message protocol is not NEC resume and return
        {
          Serial.println("Decoding fails. Protocol is wrong");
          IrReceiver3.resume();
        }
        else
        {
          if(!checkDecode(IrReceiver3.decodedIRData.decodedRawData))   //If the decode causes bit lost resume and return
          {
            Serial.println("Message is lost. Wait for the new receivement.");
            IrReceiver3.resume();
          }
          else
          {
            Serial.print("Message is decoded: ");
            Serial.println(IrReceiver3.decodedIRData.decodedRawData, HEX);
            parseMessageBU(IrReceiver3.decodedIRData.decodedRawData);  //Parse the message
            if(!checkHeader())                                     //Check the header if wrong resume and return
            {
              Serial.println("Message did not come from any MU");
              IrReceiver3.resume();
            }
            else
            {
              switch(getHeader(IrReceiver3.decodedIRData.decodedRawData))
              {
                case 2: //MU1
                  Serial.println("Message is received from MU1");
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU1);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU2);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU3);
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
              IrReceiver3.resume();
            }
          }
        }
      break;

      case 4:
        IrReceiver4.decodeNEC(); 
        if(IrReceiver4.decodedIRData.protocol != NEC)  //If the message protocol is not NEC resume and return
        {
          Serial.println("Decoding fails. Protocol is wrong");
          IrReceiver4.resume();
        }
        else
        {
          if(!checkDecode(IrReceiver4.decodedIRData.decodedRawData))   //If the decode causes bit lost resume and return
          {
            Serial.println("Message is lost. Wait for the new receivement.");
            IrReceiver4.resume();
          }
          else
          {
            Serial.print("Message is decoded: ");
            Serial.println(IrReceiver4.decodedIRData.decodedRawData, HEX);
            parseMessageBU(IrReceiver4.decodedIRData.decodedRawData);  //Parse the message
            if(!checkHeader())                                     //Check the header if wrong resume and return
            {
              Serial.println("Message did not come from any MU");
              IrReceiver4.resume();
            }
            else
            {
              switch(getHeader(IrReceiver4.decodedIRData.decodedRawData))
              {
                case 2: //MU1
                  Serial.println("Message is received from MU1");
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU1);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU2);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU3);
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
              IrReceiver4.resume();
            }
          }
        }
      break;

      case 5:
        IrReceiver5.decodeNEC(); 
        if(IrReceiver5.decodedIRData.protocol != NEC)  //If the message protocol is not NEC resume and return
        {
          Serial.println("Decoding fails. Protocol is wrong");
          IrReceiver5.resume();
        }
        else
        {
          if(!checkDecode(IrReceiver5.decodedIRData.decodedRawData))   //If the decode causes bit lost resume and return
          {
            Serial.println("Message is lost. Wait for the new receivement.");
            IrReceiver5.resume();
          }
          else
          {
            Serial.print("Message is decoded: ");
            Serial.println(IrReceiver5.decodedIRData.decodedRawData, HEX);
            parseMessageBU(IrReceiver5.decodedIRData.decodedRawData);  //Parse the message
            if(!checkHeader())                                     //Check the header if wrong resume and return
            {
              Serial.println("Message did not come from any MU");
              IrReceiver5.resume();
            }
            else
            {
              switch(getHeader(IrReceiver5.decodedIRData.decodedRawData))
              {
                case 2: //MU1
                  Serial.println("Message is received from MU1");
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU1);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU2);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU3);
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
              IrReceiver5.resume();
            }
          }
        }
      break;

      case 6:
        IrReceiver6.decodeNEC(); 
        if(IrReceiver6.decodedIRData.protocol != NEC)  //If the message protocol is not NEC resume and return
        {
          Serial.println("Decoding fails. Protocol is wrong");
          IrReceiver6.resume();
        }
        else
        {
          if(!checkDecode(IrReceiver6.decodedIRData.decodedRawData))   //If the decode causes bit lost resume and return
          {
            Serial.println("Message is lost. Wait for the new receivement.");
            IrReceiver6.resume();
          }
          else
          {
            Serial.print("Message is decoded: ");
            Serial.println(IrReceiver6.decodedIRData.decodedRawData, HEX);
            parseMessageBU(IrReceiver6.decodedIRData.decodedRawData);  //Parse the message
            if(!checkHeader())                                     //Check the header if wrong resume and return
            {
              Serial.println("Message did not come from any MU");
              IrReceiver6.resume();
            }
            else
            {
              switch(getHeader(IrReceiver6.decodedIRData.decodedRawData))
              {
                case 2: //MU1
                  Serial.println("Message is received from MU1");
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU1);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU2);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU3);
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
              IrReceiver6.resume();
            }
          }
        }
      break;

      case 7:
        IrReceiver7.decodeNEC(); 
        if(IrReceiver7.decodedIRData.protocol != NEC)  //If the message protocol is not NEC resume and return
        {
          Serial.println("Decoding fails. Protocol is wrong");
          IrReceiver7.resume();
        }
        else
        {
          if(!checkDecode(IrReceiver7.decodedIRData.decodedRawData))   //If the decode causes bit lost resume and return
          {
            Serial.println("Message is lost. Wait for the new receivement.");
            IrReceiver7.resume();
          }
          else
          {
            Serial.print("Message is decoded: ");
            Serial.println(IrReceiver7.decodedIRData.decodedRawData, HEX);
            parseMessageBU(IrReceiver7.decodedIRData.decodedRawData);  //Parse the message
            if(!checkHeader())                                     //Check the header if wrong resume and return
            {
              Serial.println("Message did not come from any MU");
              IrReceiver7.resume();
            }
            else
            {
              switch(getHeader(IrReceiver7.decodedIRData.decodedRawData))
              {
                case 2: //MU1
                  Serial.println("Message is received from MU1");
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU1);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU2);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU3);
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
              IrReceiver7.resume();
            }
          }
        }
      break;

      case 8:
        IrReceiver8.decodeNEC(); 
        if(IrReceiver8.decodedIRData.protocol != NEC)  //If the message protocol is not NEC resume and return
        {
          Serial.println("Decoding fails. Protocol is wrong");
          IrReceiver8.resume();
        }
        else
        {
          if(!checkDecode(IrReceiver8.decodedIRData.decodedRawData))   //If the decode causes bit lost resume and return
          {
            Serial.println("Message is lost. Wait for the new receivement.");
            IrReceiver8.resume();
          }
          else
          {
            Serial.print("Message is decoded: ");
            Serial.println(IrReceiver8.decodedIRData.decodedRawData, HEX);
            parseMessageBU(IrReceiver8.decodedIRData.decodedRawData);  //Parse the message
            if(!checkHeader())                                     //Check the header if wrong resume and return
            {
              Serial.println("Message did not come from any MU");
              IrReceiver8.resume();
            }
            else
            {
              switch(getHeader(IrReceiver8.decodedIRData.decodedRawData))
              {
                case 2: //MU1
                  Serial.println("Message is received from MU1");
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU1);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU2);
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
                  setReceivementBU(&bu, BU_MSG_RECEIVED_MU3);
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
              IrReceiver8.resume();
            }
          }
        }
      break;
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
  message = ((~uiTargetLocation) << 24) |(uiHeader << 20) | (uiReceivement << 16) | (uiRange << 12) | (uiKnowing << 8) | (uiTargetLocation << 0);
  Serial.print("Base Message: ");
  Serial.println(message, HEX);
  return message;
}



/*****************************************************************************
* Function: parseMessageBU(unsigned int message)
* Aim: Parsing the message that comes from MUs
******************************************************************************/

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
  return ((rawdata & 0xFF000000) >> 24) == (~(rawdata & 0x000000FF));
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
  return ((rawdata | 0x00F00000) >> 20);
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
* Function: resetReceivementBU(BU* bu)
* Aim: Resets the acknowledgement
******************************************************************************/
void resetReceivementBU(BU* bu)
{
  bu->uiBUReceivement = BU_MSG_NOT_RECEIVED;
}

/*****************************************************************************
* Function: setReceivementBU(BU* bu, unsigned int rcv)
* Aim: Sets the acknowledgement
******************************************************************************/
void setReceivementBU(BU* bu, unsigned int rcv)
{
  bu->uiBUReceivement = rcv;
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