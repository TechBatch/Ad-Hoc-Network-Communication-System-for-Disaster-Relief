#include "MU1Header.h"

int FoundInfo = 0;
int QueueID = 0;
int march = 0;
//MU HAS 2 BIT INFO SPACE AS RECEIVEMENT IT IS NOT USE
/*****************************/
/******RFID DECLARATIONS******/

MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key; 
// Init array that will store new NUID 
byte nuidPICC[4];

/*******************************/
/******COMMUNICATION INIT******/

MU mu;
BU bu;

/******************************/
/******************************/

MPU6050 mpu;
MPU6050 accelgyro;

long duration;
float distanceCm;

bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int16_t ax, ay, az;
int16_t gx, gy, gz;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float angle;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
float GyroX;

float target = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float last_error = 0;
float error_lu_angle = 0;
float targetAngle = 0;
int target_location = 100; // Unknown
int red = 14;
int yellow = 32;
int green = 26;

bool dur = false;
int card_num = 0;
int old_card_num;
bool is_forward = true;
bool is_right = false;
bool is_left = false;
bool is_backward = false;
int last_loc_glob;

int current_loc_glob;
int old_diff_glob;
int diff_glob;
int direction_glob; //0: forward, 1:left, 2:right, 3:u
bool flag_stop_card = false;
bool cross_error = false;
int cross_loc; 
bool error_path_setup = false;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

const int offsetA = 1;
const int offsetB = 1;

vector<int>path;
vector<int>old_path;
bool flag_dur = false;

Motor motor_left = Motor(AIN1, AIN2, PWMA, offsetA, STBY,5000 ,8,1 );
Motor motor_right = Motor(BIN1, BIN2, PWMB, offsetB, STBY,5000 ,8,2 );


int first_position = UNKNOWN;
int second_position = UNKNOWN; 
bool setup_path = true;
bool flag_first_pos = true;
bool flag_second_pos = false;
int stop_list;
bool is_backward_after_rotate = false;
bool flag_is_backward_after_rotate = true;
int current_time_backward;
int card_num_under_target;
bool found_target = false;
bool return_base = false;
int time_zero = INT_MAX;
int cum_angle = 0;
float step_size;
int step_index = 0;
int init_phase = 0;
int error_case = 4;
int time_first;
bool error_setup_path = false;

// ==============================================
// ===               Sub Grid                 ===
// ==============================================

int MU_number = 1;
Grid g = createSubGrid(1, 0, 0, 0, 0);
Grid g_global = Grid(81);
float step_size_vec[13] = {5, -10, 15, -20, 25, -30, 35, -40, 45, -50, 55, -60, 65};

// ==============================================
// ===               PID TUNING               ===
// ==============================================

float Kp_backward = 4;
float Ki_backward = 0.04;
float Kd_backward = 3.5;
float Kp = 8;
float Ki = 0.05;
float Kd = 12;
float mtrSpd_r = 120;
float mtrSpd_l = 100;
float mtrSpd_backward_r = 120;
float mtrSpd_backward_l = 100;
int left_motor_fast = 100;
int right_motor_fast = 100;

void setup() {

  /************I2C & MPU***********/
  Serial.begin(115200);
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  delay(4000);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();
  if(MU_number == 1){
    mpu.setXAccelOffset(435.00000); 
    mpu.setYAccelOffset(-319.00000); 
    mpu.setZAccelOffset(5210.00000); 
    mpu.setXGyroOffset(128.00000);
    mpu.setYGyroOffset(-19.00000);
    mpu.setZGyroOffset(18.00000);
  }
  else if(MU_number == 2){
    mpu.setXAccelOffset(-1125.00000); 
    mpu.setYAccelOffset(-1413.00000); 
    mpu.setZAccelOffset(4301.00000); 
    mpu.setXGyroOffset(17.00000);
    mpu.setYGyroOffset(249.00000);
    mpu.setZGyroOffset(20.00000);
  }
  else {
    mpu.setXAccelOffset(-1223.00000); 
    mpu.setYAccelOffset(-1275.00000); 
    mpu.setZAccelOffset(4309.00000); 
    mpu.setXGyroOffset(19.00000);
    mpu.setYGyroOffset(323.00000);
    mpu.setZGyroOffset(38.00000);
  }
  mpu.setXAccelOffset(-1223.00000); 
  mpu.setYAccelOffset(-1275.00000); 
  mpu.setZAccelOffset(4309.00000); 
  mpu.setXGyroOffset(19.00000);
  mpu.setYGyroOffset(323.00000);
  mpu.setZGyroOffset(38.00000);

  // 1688 factory default for my test chip

  if (devStatus == 0) {
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      
      mpu.setDMPEnabled(true);

      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  }else {
      int a=0;
  }

  /************RFID***************/

  SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522 
  for (byte i = 0; i < 6; i++) 
  {
    key.keyByte[i] = 0xFF;
  }

  /*************DISTANCE SENSOR************/

  // pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  // pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  /******************DC MOTOR***********************/

  /*pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);*/


/******************LED***********************/
  //pinMode(yellow, OUTPUT); // yellow
  //pinMode(green, OUTPUT); // green
  //pinMode(red, OUTPUT); // red
  /******************COMMUNICATION*****************/
  IrReceiver.begin(MU_RECEIVER);
  pinMode(MU_TRANSMITTER, OUTPUT);
  IrSender.begin(MU_TRANSMITTER);

  /*****************TASK CREATIONS*****************/
  xTaskCreatePinnedToCore(
    taskSendMessage,        // Task function
    "CommunicationTask",   // Task name
    4096,                // Stack size (in words)
    NULL,                 // Task input parameter
    //tskIDLE_PRIORITY,                    // Priority
    1,
    NULL,                 // Task handle
    PRO_CPU_NUM
  );
  xTaskCreatePinnedToCore(
    taskReceiveMessage,        // Task function
    "MovementControllerTask",   // Task name
    4096,                // Stack size (in words)
    NULL,                 // Task input parameter
    1,                    // Priority
    NULL,                 // Task handle
    APP_CPU_NUM
  );
}

void loop() {
  // digitalWrite(trigPin, LOW);
  // delayMicroseconds(2);
  // // Sets the trigPin on HIGH state for 10 micro seconds
  // digitalWrite(trigPin, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(trigPin, LOW);
  
  // // Reads the echoPin, returns the sound wave travel time in microseconds
  // duration = pulseIn(echoPin, HIGH);
  
  // // Calculate the distance
  // distanceCm = duration * SOUND_SPEED/2;

  /*
    Serial.print("Error: ");
    Serial.println(error_case);
    
    // if(distanceCm < 10){
    //   brake(motor_left, motor_right); 
    //   delay(1000);
    //   return;
    // }
  */

  if (!dmpReady){
    return;
  } 
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


          angle = ypr[0] * 180/M_PI; //yaw (angle wrt z-axis) 
          angle = angle - cum_angle;
          //Serial.println(angle);
      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
          accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          

          GyroX = -gz; //kullanmıyoz sil

          
      #endif
  }

  if(setup_path){

    if(flag_first_pos){
      time_zero = millis();
    }

    if(((millis() - time_zero)> 800) && (error_case == 4)){
      brake(motor_left, motor_right); 
      delay(300);
      is_forward = false;
      is_backward = true;
      error_case = 5;
    }

    if (is_forward){
      //Serial.println("forward");
      PID_forward();
    }

    if(is_backward){
      PID_backward();
      time_zero = millis();
    }


    if((error_case == 0)){ // Step by step find direction
      last_error = 0;
      integral = 0;
      rotate_degree(step_size);
    }

    if ( ! rfid.PICC_IsNewCardPresent()) {
      return;
    }
    if ( ! rfid.PICC_ReadCardSerial()) {
      return;
    }
    
    mu.uiMUCurrentLocation = encodeRFID(rfid.uid.uidByte);
    
    if((mu.uiMUCurrentLocation == first_position) && (error_case == 5)){
      brake(motor_left, motor_right); 
      delay(300);
      is_forward = false;
      is_backward = false;
      time_zero = millis();
      error_case = 0;
      if(step_index > 12){
        step_index = 0;
      }
      step_size = step_size_vec[step_index];
      step_index += 1;
    }

    if((setup_path) && (mu.uiMUCurrentLocation != 100)){
      if(flag_first_pos){
        first_position = mu.uiMUCurrentLocation;
        flag_first_pos = false;
        flag_second_pos = true;
        time_zero = millis();
        error_case = 4;
        return;
      }
      if((flag_second_pos) && mu.uiMUCurrentLocation != first_position){ 
        second_position = mu.uiMUCurrentLocation;
        flag_second_pos = false;
        brake(motor_left, motor_right); 
      
        //Grid g = createSubGrid(3,0,84,0);
        if(error_setup_path && !return_base){
          for (int& elem : old_path) {
            g.changeTileStatus(elem, Visited::VISITED);
          }
          old_path.clear();
          path = func(g,second_position);
          stop_list = path.back();
          error_setup_path = false;
        }
        else if(return_base){
          path = g.calculatePath(second_position, 1);
          stop_list = path.back();
          error_setup_path = false;
        }
        else{
          //g.changeTileStatus(first_position, Visited::VISITED);
          path = func(g,second_position);
          stop_list = path.back();
        }
        for (int& elem : path) {
          Serial.println(elem);
        }
        setup_path = false; 
        old_diff_glob = second_position - first_position;
        time_zero = millis();
        error_case = 4;
        step_index = 0;
        old_path.push_back(first_position);
        is_forward = false;
        is_backward = false;
      }
    }
  }

  if(!dur && !setup_path) {
    
    if(((millis() - time_zero)> 700) && (error_case == 4)){ // || (error_case == 3)
      Serial.println("timeout");
      brake(motor_left, motor_right); 
      delay(300);
      is_forward = false;
      is_backward = true;
      error_case = 5;
    }
    /**/

    /**/
    if (is_forward){
      //Serial.println("forward");
      PID_forward();
    }
    else if (is_left == true || is_right == true){
      last_error = 0;
      integral = 0;
      rotate();
      time_zero = millis();
    }
    else if(is_backward){
      Serial.println("backward");
      PID_backward();
      time_zero = millis();
    }

    if((error_case == 0) && false){ // Cross Error 45 degree probably
      Serial.println("cross_error");
      int cross_step = encode_cross_direction(cross_loc, path[0], path[1]);
      rotate_degree(cross_step);
    }
    else if((error_case == 0) || (error_case == 2)){ // Step by step find direction
      last_error = 0;
      integral = 0;
      Serial.println("random error");
      rotate_degree(step_size);
    }

    /*
    else if(is_backward_after_rotate){
      if(flag_is_backward_after_rotate){
        current_time_backward = millis();
        flag_is_backward_after_rotate = false;
      }
      else if((current_time_backward + 150) < millis()){
        is_forward = true;
        is_backward_after_rotate = false;
        flag_is_backward_after_rotate = true;
      }
      //PID_backward();
    }
    */

    
    if ( ! rfid.PICC_IsNewCardPresent()) {
      return;
    }
    if ( ! rfid.PICC_ReadCardSerial()) {
      return;
    }

    mu.uiMUCurrentLocation = encodeRFID(rfid.uid.uidByte);
    Serial.print("Card num:");
    Serial.println(mu.uiMUCurrentLocation);
    time_zero = millis();

    if(found_target){
      if((mu.uiMUCurrentLocation != TARGET)){
        Serial.println("targetin altindaki karti okuduk");
        mu.uiMUTargetLocation = mu.uiMUCurrentLocation;
        
        //Grid g = createSubGrid(3,0,84,0);
        path.clear();
        path = g.calculatePath(mu.uiMUTargetLocation, 1);
        for (int& elem : path) {
          Serial.println(elem);
        }
        stop_list = path.back();
        //path.insert(path.begin(), last_loc);
        found_target = false;
        return_base = true;
        time_zero = millis();
        old_card_num = mu.uiMUCurrentLocation;
      }
      else{
        time_zero = millis();
        Serial.println("targeti tekrar okuduk");
        return;
      }
    }



    if(mu.uiMUCurrentLocation == path[0]){
      Serial.println("araba pathde");
      flag_is_backward_after_rotate = true;
      is_backward_after_rotate = false;
      last_loc_glob = path[0];
      current_loc_glob = path[1];
      time_zero = millis();
      
      if(error_case==5){
        is_backward = !is_backward;
        is_forward = !is_forward;
      }
      direction_glob = encode_direction(last_loc_glob, current_loc_glob, old_diff_glob); //0: forward, 1: left, 2: right, 3: u, 4: stop
      old_diff_glob = current_loc_glob - last_loc_glob;

      
      old_path.push_back(path.front());
      path.erase(path.begin());

      if(direction_glob != 0){          
        if(direction_glob == 1){ //Left turn
          Serial.println("left");
          brake(motor_left, motor_right); 
          delay(1000);              
          target += 90;
          if (target > 180){
            target -= 360;
          }
          is_left = true;
          is_forward = false;
          flag_dur = true;

          
        }
        else if(direction_glob == 2){ //Right turn
          Serial.println("right");
          brake(motor_left, motor_right); 
          delay(1000);
          target -= 90;
          if (target <= -180){
            target += 360;
          }
          is_right = true;
          is_forward = false;
          flag_dur = true;
        }
        else if(direction_glob == 3){
          Serial.println("backward direc");
          brake(motor_left, motor_right); 
          delay(1000);
          is_forward = false;
        }
        else if(direction_glob == 4){
          Serial.println("dur");
          brake(motor_left, motor_right); 
          dur = true;
        }
        time_zero = millis();
      }
      else{
        time_zero = millis();
        Serial.println("forward devam");
        //is_forward = true;
      }
      old_card_num = mu.uiMUCurrentLocation;
      time_zero = millis();
      error_case = 4;
      step_index = 0;
      
    }
    else{
      if((error_case == 5) && (old_card_num == mu.uiMUCurrentLocation)) {
        brake(motor_left, motor_right); 
        delay(300);
        time_zero = millis();
        error_case = 0;
        is_backward = false;
        if(step_index > 12){
          step_index = 0;
        }
        step_size = step_size_vec[step_index];
        step_index += 1;
        rfid.PCD_StopCrypto1();
        return;
      }
      if((old_card_num == mu.uiMUCurrentLocation)){
        Serial.println("ayni kart");
        time_zero = millis();
        rfid.PCD_StopCrypto1();
        return;
      }
      
      if((TARGET == mu.uiMUCurrentLocation) && (!return_base)){
        brake(motor_left, motor_right); 
        delay(300);
        //path.clear();
        Serial.println("target bulundu");
        found_target = true;
        time_zero = millis();
        return_base = true; 
        //old_card_num = mu.uiMUCurrentLocation;
        rfid.PCD_StopCrypto1();
        return;
      }
      if(mu.uiMUCurrentLocation == TARGET){
        time_zero = millis();
        rfid.PCD_StopCrypto1();
        return;
      }
      else{
        Serial.println("reset atiyoz");
        brake(motor_left, motor_right); 
        reset_global_var();
        error_path_setup = true;
        setup_path = true;
        path.clear();
        time_zero = millis();
      }
      
      /**/
    }
  }
  rfid.PCD_StopCrypto1();
}

/********************************************************/
/****************COMMUNICATION FUNCTIONS*****************/

unsigned int createMessageMU(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiFinding, unsigned int uiCurrentLocation, unsigned int uiTargetLocation) //Creating messages in predetermined format for MUs
{
  unsigned int message;
  message =((~((uiHeader<<4) | (uiReceivement << 2) | (uiFinding))) << 24) |(uiHeader << 20) | (uiReceivement << 18) | (uiFinding << 16) | (uiCurrentLocation << 8) | (uiTargetLocation << 0);
  return message;
}

void parseMessageMU(unsigned int message) //Parsing the messages that come from BU and inserts the proper values to the BU struct

{
  unsigned int uiHeader = (message & 0x00F00000) >> 20;
  unsigned int uiReceivement = (message & 0x000F0000) >> 16; // NO USE
  unsigned int uiTalk = (message & 0x0000F000) >> 12;        //NO USE
  unsigned int uiWhoFound = (message & 0x00000C00) >> 10;
  unsigned int uiQueue = (message & 0x00000300) >> 8;
  unsigned int uiTargetLocation = (message & 0x000000FF) >> 0;
  
  if(uiHeader == BU_NAME)
  {
    Serial.print("Header: ");
    Serial.println(uiHeader, HEX);
    Serial.print("Receivement: ");
    Serial.println(uiReceivement, HEX);
    Serial.print("Talk: ");
    Serial.println(uiTalk, HEX);
    Serial.print("Who Found: ");
    Serial.println(uiWhoFound, HEX);
    Serial.print("Queue: ");
    Serial.println(uiQueue, HEX);
    Serial.print("Target Location: ");
    Serial.println(uiTargetLocation, HEX);
  }
  bu.uiBUHeader = uiHeader;
  bu.uiBUReceivement = uiReceivement;
  bu.uiBUTalk = uiTalk;
  bu.uiBUWhoFound = uiWhoFound;
  bu.uiBUQueue = uiQueue;
  bu.uiBUTargetLocation = uiTargetLocation; 
}

void getWhoFound(BU* bu)
{
  if(bu->uiBUWhoFound != 0)
  {
    FoundInfo = bu->uiBUWhoFound;
  }
}

int checkDecode(unsigned int rawdata) //Checks the decode for MU
{
  return (((rawdata & 0xFF000000) >> 24) == ((~(rawdata >> 16)) & 0x000000FF));
}

int checkHeader(BU* bu)  //Compares the message header with BU header for MU
{
  return (bu->uiBUHeader == BU_NAME);
}


int checkTalk(BU* bu)
{
  return (bu->uiBUTalk == mu.uiMUHeader);
}
void learnTargetLocation(MU* mu, BU* bu)
{
  if(bu->uiBUTargetLocation != UNKNOWN)
  {
    mu->uiMUTargetLocation = bu->uiBUTargetLocation;
  }
}

void setFinding(MU* mu)
{
  mu->uiMUFinding = (mu->uiMUCurrentLocation == 0x64) ? MU_TARGET_FOUND : MU_TARGET_NOT_FOUND;
}

int checkReceivement(BU* bu, MU* mu)
{
  return (bu->uiBUReceivement == (mu->uiMUHeader-1)); 
}

void setReceivement(MU* mu)
{
  mu->uiMUReceivement = MU_MSG_RECEIVED;
}
/***********************************************/
/****************RFID FUNCTIONS*****************/

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
    return TARGET;
  }
}

/***************************************************/
/******************MOTOR FUNCTIONS******************/

void rotate_degree(float target_correction){
  float deltaangle = (target_correction - angle + target);

  if ((target == 180) && (angle < 0)){
    if(target_correction > 0){
      deltaangle = (target - target_correction + angle);
    }
    else{
      deltaangle = (360 - target - target_correction + angle);
    }
    
  }
  else if ((target == 180) && (angle > 0)){
    if(target_correction > 0){
      deltaangle = (360 + target - target_correction - angle);
    }
    else{
      deltaangle = (target - target_correction - angle);
    }
  }


  if (abs(deltaangle) <= 1){
    brake(motor_left, motor_right); 
    delay(300);
    cum_angle = cum_angle + target_correction;
    is_forward = true;
    error_case = 4;
    time_zero = millis();
    Serial.println("rotate angle good");
  } 
  else if((target_correction) > 0){
  
    motor_left.brake();
    motor_right.drive(right_motor_fast);
  
  }
  else if((target_correction) < 0){
  
    motor_right.brake();
    motor_left.drive(left_motor_fast);
  
  }
}

int error_in_path(int base_loc){

  if(card_num == path[0]){
    return 1; // MU is in the path
  }
  if((((base_loc - 10) == card_num) || ((base_loc + 10) == card_num) || ((base_loc - 1) == card_num) || ((base_loc + 1) == card_num))){
    return 2; // MU's direction is known
  }
  if((((base_loc - 11) == card_num) || ((base_loc + 11) == card_num) || ((base_loc - 9) == card_num) || ((base_loc + 9) == card_num))){
    return 3; // MU's direction is tilted
  }
  
  return 4; //what is happening bro


}

int encode_cross_direction(int c, int p0, int p1){
  if((((c-p1) == 1) && ((p1-p0) == 10)) || (((c-p1) == 10) && ((p1-p0) == -1)) || (((c-p1) == -1) && ((p1-p0) == -10)) || (((c-p1) == -10) && ((p1-p0) == 1))){
    return 45;
  }
  return -45;
}

int encode_direction(int last_loc,int current_loc, int old_diff){
  
  int diff = current_loc - last_loc;
  
  if ((diff == -1 &&  old_diff == 1) || (diff == -10 &&  old_diff == 10) || (diff == 1 &&  old_diff == -1) || (diff == 10 &&  old_diff == -10)){
    Serial.print("U_turn: ");
    Serial.println(last_loc);
    is_backward = !is_backward;
    return 3;
  }

  else if ((diff == 10 &&  old_diff == 1) || (diff == -1 &&  old_diff == 10) || (diff == -10 &&  old_diff == -1) || (diff == 1 &&  old_diff == -10)){
    if(is_backward){
      Serial.print("Right: ");
      Serial.println(last_loc);
      is_backward = !is_backward;
      return 2;

    }
    else{
      Serial.print("Left: ");
      Serial.println(last_loc);
      return 1;
    }
    
  }

  else if((diff == 10 &&  old_diff == -1) || (diff == 1 &&  old_diff == 10) || (diff == -10 &&  old_diff == 1) || (diff == -1 &&  old_diff == -10)) {
    if(is_backward){
      Serial.print("Left: ");
      Serial.println(last_loc);
      is_backward = !is_backward;
      return 1;
    }
    else{
      Serial.print("Right: ");
      Serial.println(last_loc);
      return 2;
    }
  }

  if (last_loc == stop_list && flag_dur == true){
    Serial.print("Stop");
    return 4;
  }

  return 0; //0: forward, 1: left, 2: right, 3: u, 4: stop
}

void PID_forward(){
  if(target == 180 && angle <0){
    error = -target - angle;
  }
  else{
    error = target - angle;

  } 
  
  integral = integral + error; 
  derivative = error - last_error; 

  error_lu_angle = (error * Kp) + (integral * Ki) + (derivative * Kd);  //derivative bastır derivative value negative değilse Kd negative olmalı -(abs(derivative*Kd)), burda işaretlerle ilgili bir karışıklık var bizim elimizdeki angleın işaretlerine bak

  if(error < 0){
    motor_left.drive(constrain(mtrSpd_l+abs(error_lu_angle), 0, 255));
    motor_right.drive(constrain(mtrSpd_r-abs(error_lu_angle), 0, 255));
  }
  
  else if(error > 0){ //setting the steering command if it is veering to the left
    motor_left.drive(constrain(mtrSpd_l-abs(error_lu_angle), 0, 255));  //bazı yüksek ya da düşük errorlü değerlerleri kaybediyoz heralde buna ifli bir şey ekeleyebiliriz.
    motor_right.drive(constrain(mtrSpd_r+abs(error_lu_angle), 0, 255));
  }
  last_error = error;

}

void PID_backward(){
  if(target == 180 && angle < 0){
    error = -target - angle;
  }
  else{
    error = target - angle;

  } 
  
  integral = integral + error; 
  derivative = error - last_error; 

  error_lu_angle = (error * Kp_backward) + (integral * Ki_backward) + (derivative * Kd_backward);  //derivative bastır derivative value negative değilse Kd negative olmalı -(abs(derivative*Kd)), burda işaretlerle ilgili bir karışıklık var bizim elimizdeki angleın işaretlerine bak

  if(error < 0){
    motor_left.drive(constrain(-mtrSpd_backward_l+abs(error_lu_angle), -255, 0));
    motor_right.drive(constrain(-mtrSpd_backward_r-abs(error_lu_angle), -255, 0));
  }
  
  else if(error > 0){ //setting the steering command if it is veering to the left
    motor_left.drive(constrain(-mtrSpd_backward_l-abs(error_lu_angle), -255, 0));  //bazı yüksek ya da düşük errorlü değerlerleri kaybediyoz heralde buna ifli bir şey ekeleyebiliriz.
    motor_right.drive(constrain(-mtrSpd_backward_r+abs(error_lu_angle), -255, 0));
  }
  last_error = error;

}


void rotate (){
  int deltaangle = (target - angle);

  if ((target == 180) && (angle < 0)){
    deltaangle = (target + angle);
  }


  if (abs(deltaangle) <= 1.0){
    brake(motor_left, motor_right); 
    delay(100);
    
    is_left = false;
    is_right = false;
    is_backward_after_rotate = true;
    time_zero = millis();
    is_forward = true;
  } 
  else {
    if (is_left) { //turn left
      motor_left.brake();
      motor_right.drive(right_motor_fast);
    } else if (is_right) {//turn right
      motor_left.drive(left_motor_fast);
      motor_right.brake();
    }
  }
}   

void reset_global_var(){
  is_forward = false;
  is_right = false;
  is_left = false;
  is_backward = false;
  targetAngle = 0;
  error = 0;
  integral = 0;
  derivative = 0;
  last_error = 0;
  error_lu_angle = 0;
  step_index = 0;
  error_case = 4;
  time_zero = millis();
  flag_first_pos = true;
  first_position = 0;
  second_position = 0; 
  flag_second_pos = false;
}
/***********************************************/
/****************GYRO FUNCTIONS*****************/

void dmpDataReady() {
    mpuInterrupt = true;
}

/***************************************************/
/*******************TASK FUNCTIONS*****************/

void taskSendMessage(void *pvParameters)
{
  unsigned int msg;
  unsigned short int address;
  unsigned char command;
  while(1)
  {
    msg = createMessageMU(mu.uiMUHeader, mu.uiMUReceivement, mu.uiMUFinding, mu.uiMUCurrentLocation, mu.uiMUTargetLocation);
    command = (unsigned int)((msg & 0x00FF0000) >> 16);
    address = (unsigned int)(msg & 0x0000FFFF);
    IrSender.sendNEC(address, command, 0);
    vTaskDelay(400 / portTICK_PERIOD_MS);
  }
}

void taskReceiveMessage(void *pvParameters)
{
  while(1)
  {
    if(IrReceiver.decodeNEC())
    {
      if(!checkDecode(IrReceiver.decodedIRData.decodedRawData))   //If the decode causes bit lost resume and return
      {
        Serial.println("Message is lost. Wait for the new receivement.");
      }
      else
      {
        //Serial.print("Message is decoded: ");
        //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
        parseMessageMU(IrReceiver.decodedIRData.decodedRawData);  //Parse the message
        if(!checkHeader(&bu))                                     //Check the header if wrong resume and return
        {
          //Serial.println("Message did not come from BU");
        }
        else  //Header doğruysa
        {
          if(!checkTalk(&bu)) //BU, MU ile konuşmuyorsa ya da genel mesaj yayınlıyorsa
          {
            Serial.print("BU does not communicate with MU");
            Serial.println((mu.uiMUHeader-1));
          }
          else //MU ile konuşuyosa
          {
            if(!checkReceivement(&bu, &mu))
            {
              Serial.println("BU did not get acknowledgement yet.");
            }
            else
            {
              setReceivement(&mu);
              QueueID = bu.uiBUQueue;
              learnTargetLocation(&mu,&bu);
              getWhoFound(&bu);
              march = 1; //Marş Marş !!!
            }
          }
        }
      }
    }
    IrReceiver.resume();
  }
}

void taskMovementMotor(void *pvParameters)
{
  while(1)
  {
    if (!dmpReady){
    continue;
    } 
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            angle = ypr[0] * 180/M_PI; //yaw (angle wrt z-axis) 
            // Serial.println(angle);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            GyroX = -gz; //kullanmıyoz sil 
        #endif
    }


    if(!dur) {

      // if(distanceCm < 10){
      //   brake(motor_left, motor_right); 
      //   vTaskDelay(1000 / portTICK_PERIOD_MS);
      //   return;
      // }
      if (is_forward){
        PID_forward();
      }
      else if (is_left == true || is_right == true){
          last_error = 0;
          integral = 0;
          rotate();
      }
      else if(is_backward){
        PID_backward();
      }
      else if(is_backward_after_rotate){
        if(flag_is_backward_after_rotate){
          current_time_backward = millis();
          flag_is_backward_after_rotate = false;
        }
        else if((current_time_backward + 150) < millis()){
          is_forward = true;
          is_backward_after_rotate = false;
          flag_is_backward_after_rotate = true;
        }
        PID_backward();
      }

      // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
      if (!rfid.PICC_IsNewCardPresent()) 
      {
        continue;
      }
      // Verify if the NUID has been readed
      if ( !rfid.PICC_ReadCardSerial()) {
        continue;
      }

      if(is_left == false && is_right == false){ // && is_backward_after_rotate==false
        mu.uiMUCurrentLocation = encodeRFID(rfid.uid.uidByte);
        Serial.print("Card num:");
        Serial.println(mu.uiMUCurrentLocation);

        if(mu.uiMUCurrentLocation == UNKNOWN){
          continue;
        }
        
        /*
        if((setup_path == true) && (mu.uiMUCurrentLocation !=  UNKNOWN)){
          if(flag_first_pos  == true){
            first_position = mu.uiMUCurrentLocation;
            flag_first_pos = false;
            flag_second_pos = true;
            continue;
          }
          if((flag_second_pos == true) && mu.uiMUCurrentLocation != first_position){ 
            second_position = mu.uiMUCurrentLocation;
            flag_second_pos = false;
            brake(motor_left, motor_right); 
          
            //Grid g = createSubGrid(3,0,84,0);

            // Serial.println("Before path");

            path = func(g,second_position+12);
            for (int& elem : path) {
                elem -= 12;
            }
            stop_list = path.back();
            setup_path = false; 

            old_diff_glob = second_position - first_position;
          }
        }
        if(TARGET == mu.uiMUCurrentLocation){
          setFinding(&mu);
          mu.uiMUTargetLocation = mu.uiMUCurrentLocation;
          brake(motor_left, motor_right); 
          path.clear();

          //Grid g = createSubGrid(3,0,84,0);
          path = g.calculatePath(mu.uiMUCurrentLocation+12, 44);
          
          for (int& elem : path) {
                elem -= 12;
          }
          stop_list = path.back();
          road_is_empty = true;
          flag_stop_card = true;
          //path.insert(path.begin(), last_loc);

        }
        if(flag_stop_card && (mu.uiMUCurrentLocation == stop_list)){
          brake(motor_left, motor_right); 
          dur = true;
        }


        if (!path.empty()){
          if(mu.uiMUCurrentLocation == path[0]){

            flag_is_backward_after_rotate = true;
            is_backward_after_rotate = false;
            last_loc_glob = path[0];
            current_loc_glob = path[1];
            

            direction_glob = encode_direction(last_loc_glob, current_loc_glob, old_diff_glob); //0: forward, 1: left, 2: right, 3: u, 4: stop
            old_diff_glob = current_loc_glob - last_loc_glob;

            if(path.size()>2){
              bool temp_is_back = is_backward;
              future_loc_glob = path[2];
              direction_future_glob = encode_direction(current_loc_glob, future_loc_glob, old_diff_glob);
              is_backward = temp_is_back;
            }
            else{
              road_is_empty = false; 
            }
            
            

            path.erase(path.begin());

            if(direction_glob != 0){
              if((direction_future_glob == direction_glob) && (road_is_empty)){ //U left turn
                if(direction_future_glob == 1){
                  old_diff_glob = future_loc_glob - current_loc_glob;

                  if(path.size()>2){
                    bool temp_is_back = is_backward;
                    current_loc_glob = path[1];
                    future_loc_glob = path[2];
                    direction_future_glob = encode_direction(current_loc_glob, future_loc_glob, old_diff_glob);
                    is_backward = temp_is_back;
                  }
                  else{
                    road_is_empty = false; 
                  }
                  if(direction_future_glob==0){
                    path.erase(path.begin());
                  }
                  
                  path.erase(path.begin());
                  // Serial.print("Left U");
                  

                  brake(motor_left, motor_right); 
                  vTaskDelay(1000 / portTICK_PERIOD_MS);              
                  target += 180;
                  if (target > 180){
                    target -= 360;
                  }
                  is_left = true;
                  is_forward = false;
                  flag_dur = true;
                  
                }
                else if(direction_future_glob == 2){ //U Right turn
                  old_diff_glob = future_loc_glob - current_loc_glob;

                  if(path.size()>2){
                    bool temp_is_back = is_backward;
                    current_loc_glob = path[1];
                    future_loc_glob = path[2];
                    direction_future_glob = encode_direction(current_loc_glob, future_loc_glob, old_diff_glob);
                    is_backward = temp_is_back;
                  }
                  else{
                    road_is_empty = false; 
                  }
                  if(direction_future_glob==0){
                    path.erase(path.begin());
                  }

                  path.erase(path.begin());
                  // Serial.print("Right U");
                  brake(motor_left, motor_right); 
                  vTaskDelay(1000 / portTICK_PERIOD_MS);              
                  target -= 180;
                  if (target <= -180){
                    target += 360;
                  }
                  is_right = true;
                  is_forward = false;
                  flag_dur = true;
                }
              }
              
              else{
                
                if(direction_glob == 1){ //Left turn
                  if((direction_future_glob == 0) || (direction_future_glob == 1) || (direction_future_glob == 4)){
                    path.erase(path.begin());
                  }
                  
                  brake(motor_left, motor_right); 
                  vTaskDelay(1000 / portTICK_PERIOD_MS);              
                  target += 90;
                  if (target > 180){
                    target -= 360;
                  }
                  is_left = true;
                  is_forward = false;
                  flag_dur = true;

                  
                }
                else if(direction_glob == 2){ //Right turn
                  if((direction_future_glob == 0) || (direction_future_glob == 2) || (direction_future_glob == 4)){
                    path.erase(path.begin());
                  }
                  brake(motor_left, motor_right); 
                  vTaskDelay(1000 / portTICK_PERIOD_MS);
                  target -= 90;
                  if (target <= -180){
                    target += 360;
                  }
                  is_right = true;
                  is_forward = false;
                  flag_dur = true;
                  
                  

                }
                else if(direction_glob == 3){
                  brake(motor_left, motor_right); 
                  vTaskDelay(1000 / portTICK_PERIOD_MS);
                  is_forward = false;
                }
                else if(direction_glob == 4){
                  brake(motor_left, motor_right); 
                  dur = true;
                }
              }
            }
          }
        }*/
      }
      
    }
    rfid.PCD_StopCrypto1();
  }
}