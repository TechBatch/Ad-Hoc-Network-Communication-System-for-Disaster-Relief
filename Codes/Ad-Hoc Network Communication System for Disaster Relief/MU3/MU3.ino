#include "MU3Header.h"

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

int IRstop = 0;

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
bool flag_dur = false;

Motor motor_left = Motor(AIN1, AIN2, PWMA, offsetA, STBY,5000 ,8,1 );
Motor motor_right = Motor(BIN1, BIN2, PWMB, offsetB, STBY,5000 ,8,2 );


int first_position = UNKNOWN; //0 olarak tanımlamak yerine unknown verdik sıkınıtı olacağını düşünmüyorum
int second_position = UNKNOWN; //0 olarak tanımlamak yerine unknown verdik sıkınıtı olacağını düşünmüyorum
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

// ==============================================
// ===               PID TUNING               ===
// ==============================================
/******************************************************************/
/******************************************************************/
//HEPSİNİ TEKRAR KONTROL ET
float Kp_backward = 4;
float Ki_backward = 0.04;
float Kd_backward = 3.5;
float Kp = 8;
float Ki = 0.05;
float Kd = 12;
float mtrSpd_r = 100; //MU2 ve MU3 için 65 olabilir
float mtrSpd_l = 100;
float mtrSpd_backward_r = 100; //MU2 ve MU3 için 65 olabilir
float mtrSpd_backward_l = 100;
int left_motor_fast = 80;
int right_motor_fast = 80;

// ==============================================
// ===               BASE VARIABLES           ===
// ==============================================

bool base_target_found = false;
bool check_obstacle = false;
int time_delay_const = 900;
int time_delay = time_delay_const;


int count_number_of_errors = 0;
int cross_step = 0;
bool if_cross_error = false;
int cross_card = 0;

// int MU_number = mu.uiMUHeader - 1; //BURAYA DİKKAT EDİN
int MU_number = mu.uiMUHeader - 1; //BURAYA DİKKAT EDİN

// ==============================================
// ===          Search Algoritm               ===
// ==============================================

// Target
int Target1, Target2, Target3; 
// Obstacles ELLE GIRILECEK OBSLER
int obs1 = 17;
int obs2 = 68; 
int obs3 = 73; 
// Grid initialitions
Grid g1       = createSubGrid(1, obs1, obs2, obs3);
Grid g2       = createSubGrid(2, obs1, obs2, obs3);
Grid g3       = createSubGrid(3, obs1, obs2, obs3);
Grid g_global = createSubGrid(0, obs1, obs2, obs3);
Grid g_self = Grid(1); 
// Grid update parameters
Path p1_setup, p2_setup, p3_setup;
vector<int> path1_setup, path2_setup, path3_setup; // Paths
int dead1, dead2, dead3;        // dead_tiles
// Checking variables
bool is_near_obstacle;

// Path variable
Path path_struct;

bool init_grid = true;

bool ultrasonic_error_happened = false;
float step_size_vec[13] = {-5, 10, -15, 20, -25, 30, -35, 40, -45, 50, -55, +60, -65};
float step_size_vec_right[13] = {-5, 8, -15, 18, -23, 28, -35, 40, -50, +60, 0, 0, 0};
//float step_size_vec_left[13] = {0, -3, 5, -8, 10, -15, 18, -23, 28, -35, 40, -50, +60};
float step_size_vec_left[13] = {5, -8, 15, -18, 23, -28, 35, -40, 50, -60, 0, 0, 0};
/*
float step_size_vec[13] = {-7.5, 15, -22.5, 30, -37.5, 45, -52.5, 60, -67.5, 75, -82.5, +90, -97.5};
*/
Tile* check_if_own_sub_grid; 
vector<int>path;
vector<int>old_path;
int base1, base2, base3, base_self; 
vector<int> g1_tiles; 
vector<int> g2_tiles; 
vector<int> g3_tiles; 


bool is_target_found_through_mu = false;
int going_sub_grid_location = 0;
bool is_going_sub_grid = false;
int ending_location = 0;
int movement = 0;
bool is_target_found_through_base = false;
bool talked_with_base = false; 
bool in_sub; 
bool is_searching = true; 
int go_to_target_location = 81;
bool is_going_target = false;
bool waiting_for_base = false;
bool flag_target_found_for_returning_base = true;
int FoundInfo = 0;
int QueueID = 0;
int march = 0;
bool flag_target_found_before = false;
bool first_thing_first = true;
const unsigned long redInterval = 100; // Blink interval for red LED
const unsigned long yellowInterval = 50; // Blink interval for yellow LED

// Variables to track timing
unsigned long previousRedMillis = 0;
unsigned long previousYellowMillis = 0;
bool finished = false;;
int redLedState = LOW;
int yellowLedState = LOW;
bool 
commmunicated_with_base = false;
int first_diff_global = 0;
#define trigPin     17
#define echoPin     34
bool edge_error_happened = false;
int which_edge;
int obs_edge;
int total_edge;

unsigned long startMillis;  // Store the start time
unsigned long currentMillis; 
bool wait_for_ultrasonic = false;

int count_ultrasonic = 0;
void setup() {

  Serial.begin(115200);
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  delay(3000);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();
  if(MU_number == 1){
    /*
    mpu.setXAccelOffset(435.00000); 
    mpu.setYAccelOffset(-319.00000); 
    mpu.setZAccelOffset(5210.00000); 
    mpu.setXGyroOffset(128.00000);
    mpu.setYGyroOffset(-19.00000);
    mpu.setZGyroOffset(18.00000);
    */

    mpu.setXAccelOffset(-5235.00000); 
    mpu.setYAccelOffset(-1190.00000); 
    mpu.setZAccelOffset(4685.00000); 
    mpu.setXGyroOffset(46.00000);
    mpu.setYGyroOffset(79.00000);
    mpu.setZGyroOffset(-47.00000);
    Kp_backward = 8; //4
    Ki_backward = 0.01;
    Kd_backward = 4;//6
  // PID_forward
    Kp = 8;
    Ki = 0.01;
    Kd = 4;
  // Forward hiz
    mtrSpd_r = 90;
    mtrSpd_l = 70;
  // Backward hiz
    mtrSpd_backward_r = 85;
    mtrSpd_backward_l = 70; //85
  // Turn hiz
    left_motor_fast = 70;
    right_motor_fast = 85;
    time_delay_const = 1500;
    time_delay = time_delay_const;

  }
  else if(MU_number == 2){
    /*
    mpu.setXAccelOffset(-1125.00000); 
    mpu.setYAccelOffset(-1413.00000); 
    mpu.setZAccelOffset(4301.00000); 
    mpu.setXGyroOffset(17.00000);
    mpu.setYGyroOffset(249.00000);
    mpu.setZGyroOffset(20.00000);
    */

    mpu.setXAccelOffset(-1205.00000); 
    mpu.setYAccelOffset(-1359.00000); 
    mpu.setZAccelOffset(4307.00000); 
    mpu.setXGyroOffset(18.00000);
    mpu.setYGyroOffset(249.00000);
    mpu.setZGyroOffset(20.00000);
    Kp_backward = 8; //4
    Ki_backward = 0.01;
    Kd_backward = 4; //12
  // PID_forward
    Kp = 8; //
    Ki = 0.01;
    Kd = 4; //8
  // Forward hiz
    mtrSpd_r = 90;//75
    mtrSpd_l = 80;
  // Backward hiz
    mtrSpd_backward_r = 90;
    mtrSpd_backward_l = 80;
  // Turn hiz
    left_motor_fast = 75;
    right_motor_fast = 80;
    time_delay_const = 1200; //2000
    time_delay = time_delay_const;

  }
  else if(MU_number == 3){
    /*
    mpu.setXAccelOffset(-5183.00000); 
    mpu.setYAccelOffset(-1205.00000); 
    mpu.setZAccelOffset(4695.00000); 
    mpu.setXGyroOffset(47.00000);
    mpu.setYGyroOffset(73.00000);
    mpu.setZGyroOffset(-32.00000);
    
    mpu.setXAccelOffset(609.00000); 
    mpu.setYAccelOffset(-251.00000); 
    mpu.setZAccelOffset(5199.00000); 
    mpu.setXGyroOffset(131.00000);
    mpu.setYGyroOffset(-17.00000);
    mpu.setZGyroOffset(2.00000);*/
    mpu.setXAccelOffset(435.00000); 
    mpu.setYAccelOffset(-319.00000); 
    mpu.setZAccelOffset(5210.00000); 
    mpu.setXGyroOffset(128.00000);
    mpu.setYGyroOffset(-19.00000);
    mpu.setZGyroOffset(18.00000);
    Kp_backward = 6;
    Ki_backward = 0.01;
    Kd_backward = 6;
    // PID_forward
    Kp = 6;
    Ki = 0.02;
    Kd = 4;
    // Forward hiz
    mtrSpd_r = 72;
    mtrSpd_l = 90;
    // Backward hiz
    mtrSpd_backward_r = 72; // 85
    mtrSpd_backward_l = 90; // 80
    // Turn hiz
    left_motor_fast = 90;
    right_motor_fast = 72;
    time_delay_const = 1600;
    time_delay = time_delay_const;
    int initial_values[] = {7, -12, 15, -20, 25, -30, 35, -40, 45, -50, 55, -60, 65};
    for (int i = 0; i < 13; i++) {
      step_size_vec[i] = initial_values[i];
    }
  }
  else{
    mpu.setXAccelOffset(-1223.00000); 
    mpu.setYAccelOffset(-1275.00000); 
    mpu.setZAccelOffset(4309.00000); 
    mpu.setXGyroOffset(19.00000);
    mpu.setYGyroOffset(323.00000);
    mpu.setZGyroOffset(38.00000);
  }

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

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  //sensor.start();

  //pinMode(yellow, OUTPUT); // yellow
  //pinMode(green, OUTPUT); // green
  //pinMode(red, OUTPUT); // red
  /******************COMMUNICATION*****************/
  IrReceiver.begin(MU_RECEIVER);
  pinMode(MU_TRANSMITTER, OUTPUT);
  IrSender.begin(MU_TRANSMITTER);

  /*g_global.addObstacle(obs1);
  g_global.addObstacle(obs2);
  g_global.addObstacle(obs2);

  g1.addObstacle(obs1);
  g1.addObstacle(obs2);
  g1.addObstacle(obs3);

  g2.addObstacle(obs1);
  g2.addObstacle(obs2);
  g2.addObstacle(obs3);

  g3.addObstacle(obs1);
  g3.addObstacle(obs2);
  g3.addObstacle(obs3);*/
  // IDs of tiles in the grid
  g1_tiles = g1.Tiles_IDs_vector();
  g2_tiles = g2.Tiles_IDs_vector();
  g3_tiles = g3.Tiles_IDs_vector();
  // Update grids 
  p1_setup = func(g1,g1_tiles[0]);
  p2_setup = func(g2,g2_tiles[0]);
  p3_setup = func(g3,g3_tiles[0]);
  dead1    = p1_setup.dead_tile;
  dead2    = p2_setup.dead_tile;
  dead3    = p3_setup.dead_tile;

  if (dead1 != -1){
    updateGrids(g1,g2,g3,dead1);}
  else if (dead2 != -1){
    updateGrids(g1,g2,g3,dead2);}
  else if (dead3 != -1){
    updateGrids(g1,g2,g3,dead3);}

  if(MU_number == 1){
    g_self = g1;
    base_self = g_self.base_tiles[0];
  }else if(MU_number == 2){
    g_self = g2;
    base_self = g_self.base_tiles[0];
  }else if(MU_number == 3){
    g_self = g3;
    base_self = g_self.base_tiles[0];
  }
  // Base assignments
  base1 = g1.base_tiles[0];
  base2 = g2.base_tiles[0];
  base3 = g3.base_tiles[0];

  pinMode(yellow, OUTPUT); // green
  pinMode(red, OUTPUT); // red
  digitalWrite(yellow, LOW);
  digitalWrite(red, LOW);
  /*****************TASK CREATIONS*****************/
  xTaskCreatePinnedToCore(
    taskSendMessage,        // Task function
    "CommunicationTask",   // Task name
    4096,                // Stack size (in words)
    NULL,                 // Task input parameter
    //tskIDLE_PRIORITY,                    // Priority
    1,
    NULL,                 // Task handle
    0
  );
  xTaskCreatePinnedToCore(
    taskReceiveMessage,        // Task function
    "MovementControllerTask",   // Task name
    4096,                // Stack size (in words)
    NULL,                 // Task input parameter
    NULL,                    // Priority
    NULL,                 // Task handle
    0
  );
}

void loop() {
  if(finished){
    if (millis() - previousRedMillis >= redInterval) {
      previousRedMillis = millis();  // Save the last time the LED state was changed
      redLedState = !redLedState;    // Toggle the LED state
      digitalWrite(red, redLedState); // Apply the new state to the LED
    }

    // Check if it's time to change the state of the yellow LED
    if (millis() - previousYellowMillis >= yellowInterval) {
      previousYellowMillis = millis();  // Save the last time the LED state was changed
      yellowLedState = !yellowLedState;    // Toggle the LED state
      digitalWrite(yellow, yellowLedState); // Apply the new state to the LED
    }
  }
  bool flag_base_com = false;
  
  

  
  
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

          //Serial.println(ypr[0]);
          
          angle = angle - cum_angle;
          if (angle > 180){
            angle -= 360;
          }
          if (angle < -180){
            angle += 360;
          }
      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
          accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          

          GyroX = -gz; //kullanmıyoz sil

          
      #endif
  }
  /*
  if(count_ultrasonic > 5){
    brake(motor_left, motor_right); 
    reset_global_var();
    error_path_setup = true;
    setup_path = true;
    path.clear();
    time_zero = millis();
    digitalWrite(yellow, LOW);
    digitalWrite(red, LOW);
    old_card_num = card_num;
    which_edge = edge_case(card_num);
    obs_edge = edge_case_of_obs(card_num);
    if((which_edge == 1) && (obs_edge == 2)){
      total_edge = 4;
    }
    else if((which_edge == 2) && (obs_edge == 1)){
      total_edge = 5;
    }
    else if((which_edge == 3) && (obs_edge == 1)){
      total_edge = 4;
    }
    else if((which_edge == 3) && (obs_edge == 2)){
      total_edge = 5;
    }
    else if((which_edge == 1) && (obs_edge == 3)){
      total_edge = 4;
    }
    else if((which_edge == 2) && (obs_edge == 3)){
      total_edge = 5;
    }
    else if((which_edge == 0) && (obs_edge != 0)){
      total_edge = obs_edge;
    }
    else if((which_edge != 0) && (obs_edge == 0)){
      total_edge = which_edge;
    }
    else if((which_edge == 5) && (obs_edge != 0)){
      total_edge = which_edge;
    }
    else if((which_edge == 4) && (obs_edge != 0)){
      total_edge = which_edge;
    }
    else if((which_edge != 0) && (obs_edge == 5)){
      total_edge = obs_edge;
    }
    else if((which_edge != 0) && (obs_edge == 4)){
      total_edge = obs_edge;
    }
    else{
      total_edge = 0;
    }


    if((total_edge == 3) || (total_edge == 4) || (total_edge == 5)){
      edge_error_happened = true;
      setup_path = false;
      is_backward = true;
      return;
    }
    wait_for_ultrasonic = false;
  }
  */

  if(count_ultrasonic > 1){
    if(old_diff_glob == 1){
      if(check_right_and_left(card_num - 10)){
        brake(motor_left, motor_right); 
        //delay(1000);
        target -= 90;
        if (target <= -180){
          target += 360;
        }
        is_right = true;
        is_forward = false;
        flag_dur = true;
      }
      else{
        brake(motor_left, motor_right); 
        //delay(1000);              
        target += 90;
        if (target > 180){
          target -= 360;
        }
        is_left = true;
        is_forward = false;
        flag_dur = true;
      }
    }
    else if(old_diff_glob == 10){
      if(check_right_and_left(card_num + 1)){
        brake(motor_left, motor_right); 
        //delay(1000);
        target -= 90;
        if (target <= -180){
          target += 360;
        }
        is_right = true;
        is_forward = false;
        flag_dur = true;
      }
      else{
        brake(motor_left, motor_right); 
        //delay(1000);              
        target += 90;
        if (target > 180){
          target -= 360;
        }
        is_left = true;
        is_forward = false;
        flag_dur = true;
      }
    }
    else if(old_diff_glob == -1){
      if(check_right_and_left(card_num + 10)){
        brake(motor_left, motor_right); 
        //delay(1000);
        target -= 90;
        if (target <= -180){
          target += 360;
        }
        is_right = true;
        is_forward = false;
        flag_dur = true;
      }
      else{
        brake(motor_left, motor_right); 
        //delay(1000);              
        target += 90;
        if (target > 180){
          target -= 360;
        }
        is_left = true;
        is_forward = false;
        flag_dur = true;
      }
    }
    else if(old_diff_glob == -10){
      if(check_right_and_left(card_num - 1)){
        brake(motor_left, motor_right); 
        //delay(1000);
        target -= 90;
        if (target <= -180){
          target += 360;
        }
        is_right = true;
        is_forward = false;
        flag_dur = true;
      }
      else{
        brake(motor_left, motor_right); 
        //delay(1000);              
        target += 90;
        if (target > 180){
          target -= 360;
        }
        is_left = true;
        is_forward = false;
        flag_dur = true;
      }
    }
    time_zero = millis();
    
    path.clear();
    time_zero = millis();
    digitalWrite(yellow, LOW);
    digitalWrite(red, LOW);
    old_card_num = card_num;
    ultrasonic_error_happened = true;
    old_card_num = card_num;
    count_ultrasonic = 0;
    wait_for_ultrasonic = false;
  }
  if(ultrasonic_error_happened){
    if (is_left == true || is_right == true){
      last_error = 0;
      integral = 0;
      rotate();
      time_zero = millis();
      //return;
    }
  }
  if(wait_for_ultrasonic){
    if (millis() - startMillis >= 500) {
      Serial.println("Waited for 1000 milliseconds");
      wait_for_ultrasonic = check_ultrasonic();
      bool wait_for_ultrasonic_temp1 = check_ultrasonic();
      bool wait_for_ultrasonic_temp2 = check_ultrasonic();
      if(wait_for_ultrasonic_temp2 || wait_for_ultrasonic_temp1 || wait_for_ultrasonic){
        wait_for_ultrasonic = true;
      }
      count_ultrasonic += 1;
      startMillis = millis();
      dur = true;
      if(!wait_for_ultrasonic){
        count_ultrasonic = 0;
        dur = false;
      } 
    }
    else{
      return;
    }
  }
  

  if(march == 1){ // march == 1
    if(waiting_for_base){
      flag_target_found_for_returning_base = false;
      waiting_for_base = false;

      GetTargetTiles(Target1, Target2, Target3, mu.uiMUTargetLocation, g_global,g1,g2,g3);
      if (MU_number == 1){
        go_to_target_location = Target1;
      }
      else if (MU_number == 2){
        go_to_target_location = Target2;
      }
      else if (MU_number == 3){
        go_to_target_location = Target3;
      }
      flag_target_found_before = true;
      time_zero = millis(); 
      dur = false;
      ending_location = go_to_target_location;

      path.clear();
      path_struct = TargetPath(QueueID, MU_number, FoundInfo, base_self, base1, base2, base3, Target1, Target2, Target3, g_global);
      path = path_struct.path;
      stop_list = path.back();
      time_zero = millis();
      old_card_num = base_self;
      is_going_target = true;
      movement = 1;
      digitalWrite(yellow, HIGH);
      digitalWrite(red, HIGH);

      direction_glob = encode_direction(path[0], path[1], old_diff_glob); //0: forward, 1: left, 2: right, 3: u, 4: stop
      old_diff_glob = path[1] - path[0];
      old_path.push_back(path.front());
      path.erase(path.begin());
      if(direction_glob != 0){          
        if(direction_glob == 1){ //Left turn
          Serial.println("left");
          brake(motor_left, motor_right); 
          //delay(1000);              
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
          //delay(1000);
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
          //delay(300);
          is_forward = !is_forward;
        }
        time_zero = millis();
      }
      else{
        if((!is_backward) && (!is_forward)){
          is_forward = true;
        }
        time_zero = millis();
        Serial.println("forward devam");
        //is_forward = true;
      }
      time_zero = millis();
      commmunicated_with_base = false;
    }
    /*
    if((flag_target_found_for_returning_base && !return_base) && (check_base_com_tile(card_num))){
      is_target_found_through_base = true;
      return_base = true;
      is_searching = false;
      flag_target_found_for_returning_base = false;   
      flag_target_found_before = true; 
      time_zero = millis();
      commmunicated_with_base = true;
    }
    */
  }
  
  if((bu.uiBUWhoFound != NF) && (flag_target_found_for_returning_base && !return_base) && (check_base_com_tile(card_num))){
    is_target_found_through_base = true;
    return_base = true;
    is_searching = false;
    flag_target_found_for_returning_base = false;   
    flag_target_found_before = true; 
    time_zero = millis();
    commmunicated_with_base = true;

  }
  /**/

  /**/
  if(commmunicated_with_base){
    if (millis() - previousRedMillis >= redInterval) {
      previousRedMillis = millis();  // Save the last time the LED state was changed
      redLedState = !redLedState;    // Toggle the LED state
      digitalWrite(red, redLedState); // Apply the new state to the LED
    }
  }


  
  if(edge_error_happened){
    finished = true;
    if(is_backward){
      PID_backward();
    }
    else if (is_left == true || is_right == true){
      last_error = 0;
      integral = 0;
      rotate();
      time_zero = millis();
      //return;
    }
    if ( ! rfid.PICC_IsNewCardPresent()) {
      return;
    }
    if ( ! rfid.PICC_ReadCardSerial()) {
      return;
    }
    card_num = encodeRFID(rfid.uid.uidByte);
    mu.uiMUCurrentLocation = card_num;
    if(card_num == old_card_num){
      return;
    }
    brake(motor_left, motor_right); 
    if((total_edge == 4) || (total_edge == 3)){
      is_backward = false;
      is_right = true;
      total_edge = 0; 
      target -= 90;
      if (target <= -180){
        target += 360;
      }
    }

    
    else if(total_edge == 5){
      is_left = true;
      is_backward = false;
      total_edge = 0; 
      target += 90;
      if (target > 180){
        target -= 360;
      }
    }
    old_card_num = card_num;
    

  }
  /**/
  if(setup_path && !edge_error_happened){

    if(flag_first_pos){
      time_zero = millis();
    }

    if(is_backward){      
      if(((millis() - time_zero)> (1.3*time_delay)) && ((error_case == 4) || (error_case == 5))){
        //brake(motor_left, motor_right); 
        //delay(300);
        is_forward = !is_forward;
        is_backward = !is_backward;
        error_case = 5;
        time_zero = millis();
        count_number_of_errors += 1;
        check_obstacle = true;
        time_delay += 15;
      }
      /**/
    }
    else if(is_forward){
      if(((millis() - time_zero)> (time_delay)) && ((error_case == 4) || (error_case == 5))){
        //brake(motor_left, motor_right); 
        //delay(300);
        is_forward = !is_forward;
        is_backward = !is_backward;
        error_case = 5;
        time_zero = millis();
        count_number_of_errors += 1;
        check_obstacle = true;
        time_delay += 15;
      }
    }
    

    if (is_forward){
      PID_forward();
    }

    if(is_backward){
      PID_backward();
      //time_zero = millis();
    }
    if(count_number_of_errors > 1){
      
      if(first_thing_first){
        not_finding_any_card(0);
      }
      else{

        which_edge = edge_case(old_card_num);
        obs_edge = edge_case_of_obs(old_card_num);
        if((which_edge == 1) && (obs_edge == 2)){
          total_edge = 4;
        }
        else if((which_edge == 2) && (obs_edge == 1)){
          total_edge = 5;
        }
        else if((which_edge == 3) && (obs_edge == 1)){
          total_edge = 4;
        }
        else if((which_edge == 3) && (obs_edge == 2)){
          total_edge = 5;
        }
        else if((which_edge == 1) && (obs_edge == 3)){
          total_edge = 4;
        }
        else if((which_edge == 2) && (obs_edge == 3)){
          total_edge = 5;
        }
        else if((which_edge == 0) && (obs_edge != 0)){
          total_edge = obs_edge;
        }
        else if((which_edge != 0) && (obs_edge == 0)){
          total_edge = which_edge;
        }
        else if((which_edge == 5) && (obs_edge != 0)){
          total_edge = which_edge;
        }
        else if((which_edge == 4) && (obs_edge != 0)){
          total_edge = which_edge;
        }
        else if((which_edge != 0) && (obs_edge == 5)){
          total_edge = obs_edge;
        }
        else if((which_edge != 0) && (obs_edge == 4)){
          total_edge = obs_edge;
        }
        else{
          total_edge = 0;
        }
        if((total_edge == 3) || (total_edge == 4) || (total_edge == 5)){
          reset_global_var();
          error_path_setup = true;
          setup_path = true;
          path.clear();
          time_zero = millis();
          digitalWrite(yellow, LOW);
          digitalWrite(red, LOW);
          old_card_num = card_num;
          edge_error_happened = true;
          //setup_path = false;
          is_backward = true;
          return;
        }
        not_finding_any_card(total_edge);
      }
      count_number_of_errors = 0;
      
    }

    if((error_case == 0) || (count_number_of_errors > 1)){ // Step by step find direction
      last_error = 0;
      integral = 0;
      rotate_degree(step_size);
      rfid.PCD_StopCrypto1();
      return;
    }

    if ( ! rfid.PICC_IsNewCardPresent()) {
      return;
    }
    if ( ! rfid.PICC_ReadCardSerial()) {
      return;
    }
    
    card_num = encodeRFID(rfid.uid.uidByte);
    mu.uiMUCurrentLocation = card_num;
    count_number_of_errors = 0;
    time_delay = time_delay_const;
    digitalWrite(yellow, LOW);
    digitalWrite(red, LOW);
    

    if((found_target) && (card_num != target_location)){
      card_num_under_target = card_num;
      mu.uiMUTargetLocation = card_num_under_target;
      setFinding(&mu);
      ending_location = base_self;
      
      found_target = false;
      return_base = true; 
      is_target_found_through_mu = true;
      is_searching = false;
      time_zero = millis();
      is_going_sub_grid = false;
      going_sub_grid_location = 0;
      movement = 1;
      
    }

    if((target_location == card_num) && (!return_base) && (!flag_target_found_before)){
      brake(motor_left, motor_right); 
      //delay(300);
      //path.clear();
      Serial.println("target bulundu");
      found_target = true;
      time_zero = millis();
      return_base = true; 
      //old_card_num = card_num;
      rfid.PCD_StopCrypto1();
      flag_target_found_before = true;
      
      return;
    }

    if((card_num == first_position) && (error_case == 5)){
      which_edge = edge_case(card_num);
      obs_edge = edge_case_of_obs(card_num);
      if((which_edge == 1) && (obs_edge == 2)){
        total_edge = 4;
      }
      else if((which_edge == 2) && (obs_edge == 1)){
        total_edge = 5;
      }
      else if((which_edge == 3) && (obs_edge == 1)){
        total_edge = 4;
      }
      else if((which_edge == 3) && (obs_edge == 2)){
        total_edge = 5;
      }
      else if((which_edge == 1) && (obs_edge == 3)){
        total_edge = 4;
      }
      else if((which_edge == 2) && (obs_edge == 3)){
        total_edge = 5;
      }
      else if((which_edge == 0) && (obs_edge != 0)){
        total_edge = obs_edge;
      }
      else if((which_edge != 0) && (obs_edge == 0)){
        total_edge = which_edge;
      }
      else if((which_edge == 5) && (obs_edge != 0)){
        total_edge = which_edge;
      }
      else if((which_edge == 4) && (obs_edge != 0)){
        total_edge = which_edge;
      }
      else if((which_edge != 0) && (obs_edge == 5)){
        total_edge = obs_edge;
      }
      else if((which_edge != 0) && (obs_edge == 4)){
        total_edge = obs_edge;
      }
      else{
        total_edge = 0;
      }
      if((total_edge == 3) || (total_edge == 4) || (total_edge == 5)){
        reset_global_var();
        error_path_setup = true;
        setup_path = true;
        path.clear();
        time_zero = millis();
        digitalWrite(yellow, LOW);
        digitalWrite(red, LOW);
        old_card_num = card_num;
        edge_error_happened = true;
        //setup_path = false;
        is_backward = true;
        return;
      }
      not_finding_any_card(total_edge);
    }

    if((setup_path) && (card_num != target_location)){
      if(flag_first_pos){
  
        first_position = card_num;
        flag_first_pos = false;
        flag_second_pos = true;
        time_zero = millis();
        error_case = 4;
        check_obstacle = true;
        return;
      }
      if((flag_second_pos) && card_num != first_position){ 
        if((check_corner(first_position)) && (!if_cross_error)){
          Serial.println("corner");
          cross_card = card_num;
          cross_step = 45;
          if_cross_error = true;
          time_zero = millis() - time_delay - 100;
          rfid.PCD_StopCrypto1();
          return;
          
        }else if(cross_card == card_num){
          Serial.println("ayni cross kart");
          time_zero = millis();
          rfid.PCD_StopCrypto1();
          return;
        }

        second_position = card_num;

        if(first_thing_first){
          first_diff_global = second_position - first_position;
        }
        first_thing_first = false;
        flag_second_pos = false;
        
        brake(motor_left, motor_right); 

        check_if_own_sub_grid = g_self.getTile(card_num);
        in_sub = (check_if_own_sub_grid != nullptr);

        if((!in_sub) && (is_searching)){
          ending_location = check_closest_tile(MU_number, second_position);
          going_sub_grid_location = ending_location; 
          movement = 1;
          is_going_sub_grid = true; 
        }
        else if(is_searching){
          digitalWrite(yellow, HIGH);
          digitalWrite(red, LOW);
          movement = 2;
          ending_location = 0;
        }
        else if(return_base){
          digitalWrite(yellow, LOW);
          digitalWrite(red, HIGH);
          ending_location = base_self;
          movement = 1;
        }
        else if(is_going_target){
          digitalWrite(yellow, HIGH);
          digitalWrite(red, HIGH);
          ending_location = go_to_target_location;
          movement = 3; 
        }

        if(movement == 1){
          path_struct = g_global.calculatePath(second_position, ending_location);
          path = path_struct.path;
          stop_list = path.back();
        }
        else if(movement == 2){
          for (int& elem : old_path) {
            g_self.changeTileStatus(elem, Visited::VISITED);
          }
          old_path.clear();
          path_struct = func(g_self,second_position);
          path = path_struct.path;
          stop_list = path.back();
        }
        else if(movement == 3){
          path.clear();
          path_struct = TargetPath(QueueID,MU_number, FoundInfo,second_position,base1,base2, base3, Target1, Target2, Target3,g_global);
          path = path_struct.path;
          stop_list = path.back();
        }
        
        
        old_diff_glob = second_position - first_position;
        int first_direction = encode_direction(path[0], path[1], old_diff_glob);
        if(first_direction == 0){
          is_forward = true;
          is_backward = false;
          old_card_num = card_num;
          old_diff_glob = path[1] - path[0];
          path.erase(path.begin());
          
        }
        else if((first_direction == 2) || (first_direction == 1)){
          is_forward = false;
          is_backward = false;
        }
        else if((first_direction == 3)){
          is_forward = false;
          is_backward = true;
          old_card_num = card_num;
          old_diff_glob = path[1] - path[0];
          path.erase(path.begin());
        }
        time_zero = millis();
        error_case = 4;
        step_index = 0;
        old_path.push_back(first_position);

        setup_path = false; 
        

      }
    }
  }

  if(!dur && !setup_path && !edge_error_happened) {
    
    if(is_backward){
      if(((millis() - time_zero)> (1.3*time_delay)) && ((error_case == 4) || (error_case == 5))){
        Serial.println("geri git");
        //brake(motor_left, motor_right); 
        //delay(300);
        is_forward = !is_forward;
        is_backward = !is_backward;
        error_case = 6;
        time_zero = millis();
        count_number_of_errors += 1;
        time_delay += 15;
      }
    }
    else if(is_forward){
      if(((millis() - time_zero)> time_delay) && ((error_case == 4) || (error_case == 5))){
        Serial.println("geri git");
        //brake(motor_left, motor_right); 
        //delay(300);
        is_forward = !is_forward;
        is_backward = !is_backward;
        error_case = 5;
        time_zero = millis();
        count_number_of_errors += 1;
        time_delay += 15;
      }
    }
    
    if (is_forward){
      //Serial.println("forward");
      PID_forward();
    }
    else if (is_left == true || is_right == true){
      //last_error = 0;
      //integral = 0;
      rotate();
      time_zero = millis();
      return;
    }
    else if(is_backward){
      PID_backward();
      //time_zero = millis();
    }
    if(count_number_of_errors > 1){
      which_edge = edge_case(old_card_num);
      obs_edge = edge_case_of_obs(old_card_num);
      if((which_edge == 1) && (obs_edge == 2)){
        total_edge = 4;
      }
      else if((which_edge == 2) && (obs_edge == 1)){
        total_edge = 5;
      }
      else if((which_edge == 3) && (obs_edge == 1)){
        total_edge = 4;
      }
      else if((which_edge == 3) && (obs_edge == 2)){
        total_edge = 5;
      }
      else if((which_edge == 1) && (obs_edge == 3)){
        total_edge = 4;
      }
      else if((which_edge == 2) && (obs_edge == 3)){
        total_edge = 5;
      }
      else if((which_edge == 0) && (obs_edge != 0)){
        total_edge = obs_edge;
      }
      else if((which_edge != 0) && (obs_edge == 0)){
        total_edge = which_edge;
      }
      else if((which_edge == 5) && (obs_edge != 0)){
        total_edge = which_edge;
      }
      else if((which_edge == 4) && (obs_edge != 0)){
        total_edge = which_edge;
      }
      else if((which_edge != 0) && (obs_edge == 5)){
        total_edge = obs_edge;
      }
      else if((which_edge != 0) && (obs_edge == 4)){
        total_edge = obs_edge;
      }
      else{
        total_edge = 0;
      }
      if((total_edge == 3) || (total_edge == 4) || (total_edge == 5)){
        reset_global_var();
        error_path_setup = true;
        setup_path = true;
        path.clear();
        time_zero = millis();
        digitalWrite(yellow, LOW);
        digitalWrite(red, LOW);
        old_card_num = card_num;
        edge_error_happened = true;
        //setup_path = false;
        //dur = true;
        is_backward = true;
        return;
      }

      not_finding_any_card(total_edge);
      count_number_of_errors = 0;
     
    }
    if((error_case == 0) || (count_number_of_errors > 1)){ // Step by step find direction
      last_error = 0;
      integral = 0;
      rotate_degree(step_size);
      rfid.PCD_StopCrypto1();
      return;
    }
    if ( ! rfid.PICC_IsNewCardPresent()) {
      return;
    }
    if ( ! rfid.PICC_ReadCardSerial()) {
      return;
    }

    card_num = encodeRFID(rfid.uid.uidByte);
    mu.uiMUCurrentLocation = card_num;
      // Get the current time
    // Check if the specified interval has passed
    

    time_zero = millis();
    count_number_of_errors = 0;
    

    if(found_target){
      time_delay = time_delay_const;
      if((card_num != target_location)){
        Serial.println("targetin altindaki karti okuduk");
        card_num_under_target = card_num;
        mu.uiMUTargetLocation = card_num_under_target;
        setFinding(&mu);
        ending_location = base_self;
        path.clear();
        path_struct = g_global.calculatePath(card_num_under_target, ending_location);
        path = path_struct.path;
        stop_list = path.back();
        for (int& elem : path) {
          Serial.println(elem);
        }
        //path.insert(path.begin(), last_loc);
        found_target = false;
        return_base = true; 
        is_target_found_through_mu = true;
        is_searching = false;
        time_zero = millis();
        //old_card_num = card_num;
        is_going_sub_grid = false;
        going_sub_grid_location = 0;
        movement = 1;
        digitalWrite(yellow, LOW);
        digitalWrite(red, HIGH);
      }
      else{
        time_zero = millis();
        Serial.println("targeti tekrar okuduk");
        return;
      }
    }
    if((going_sub_grid_location == card_num) && (is_going_sub_grid)){
      time_delay = time_delay_const;
      brake(motor_left, motor_right); 
      //delay(300);

      for (int& elem : old_path) {
        g_self.changeTileStatus(elem, Visited::VISITED);
      }
      old_path.clear();
      path.clear();
      path_struct = func(g_self,card_num);
      path = path_struct.path;
      stop_list = path.back();
      ending_location = 0;
      old_card_num = card_num;
      going_sub_grid_location = 0;
      is_going_sub_grid = false;
      movement = 2;
      digitalWrite(yellow, HIGH);
      digitalWrite(red, LOW);

    }
    if(is_target_found_through_base || (is_searching && (stop_list == card_num) && (path[0] == stop_list))){
      time_delay = time_delay_const;
      brake(motor_left, motor_right); 
      //delay(300);

      ending_location = base_self;
      path.clear();
      path_struct = g_global.calculatePath(card_num, ending_location);
      path = path_struct.path;
      stop_list = path.back();
      time_zero = millis();
      old_card_num = card_num;
      return_base = true;
      is_searching = false;
      is_target_found_through_base = false;
      movement = 1;
      if(!commmunicated_with_base){
        digitalWrite(yellow, LOW);
        digitalWrite(red, HIGH);
      }
      
    
      /*
      int first_direction = encode_direction(path[0], path[1], old_diff_glob);
      
      if(first_direction == 0){
        is_forward = true;
        is_backward = false;
      }
      else{
        is_forward = false;
        is_backward = false;
      }

      */
    }
    if(return_base && (base_self == card_num)){
      time_delay = time_delay_const;
      brake(motor_left, motor_right); 
      //delay(300);
      waiting_for_base = true;
      return_base = false;
      dur = true;
      old_diff_glob = card_num - old_card_num;
      old_card_num = card_num;
      flag_target_found_for_returning_base = false;
      return;
    }
        
    if(card_num == path[0]){
      time_delay = time_delay_const;
      for (int i = 0; i < 13; i++) {
        step_size_vec[i] *= -1.;
      }   
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
          //delay(1000);              
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
          //delay(1000);
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
          //delay(1000);
          is_forward = !is_forward; // false
        }
        else if(direction_glob == 4){
          Serial.println("dur");
          brake(motor_left, motor_right); 
          //delay(300);
          is_forward = false;
          is_backward = false;

          
          if(is_going_target){
            dur = true;
            is_going_target = false;
            finished = true;
            return;
          }
        }
        time_zero = millis();
      }
      else{
        if((!is_backward) && (!is_forward)){
          is_forward = true;
        }
        time_zero = millis();
        Serial.println("forward devam");
        //is_forward = true;
      }
      old_card_num = card_num;
      time_zero = millis();
      error_case = 4;
      step_index = 0;
      
    }
    // 1: saga dogru git
    // 2: sola dogru git
    // 3: arkaya git
    // 4: arkaya git saga don
    // 5: arkaya git sola don
    else{
      if(check_near_base_tile(card_num) && is_forward){
        wait_for_ultrasonic = check_ultrasonic();
        if(wait_for_ultrasonic){
          return;
        }
      }
      else{
        wait_for_ultrasonic = false;
      }
      startMillis = millis();
      if((error_case == 5) && (old_card_num == card_num)) {
        
        which_edge = edge_case(card_num);
        obs_edge = edge_case_of_obs(card_num);
        if((which_edge == 1) && (obs_edge == 2)){
          total_edge = 4;
        }
        else if((which_edge == 2) && (obs_edge == 1)){
          total_edge = 5;
        }
        else if((which_edge == 3) && (obs_edge == 1)){
          total_edge = 4;
        }
        else if((which_edge == 3) && (obs_edge == 2)){
          total_edge = 5;
        }
        else if((which_edge == 1) && (obs_edge == 3)){
          total_edge = 4;
        }
        else if((which_edge == 2) && (obs_edge == 3)){
          total_edge = 5;
        }
        else if((which_edge == 0) && (obs_edge != 0)){
          total_edge = obs_edge;
        }
        else if((which_edge != 0) && (obs_edge == 0)){
          total_edge = which_edge;
        }
        else if((which_edge == 5) && (obs_edge != 0)){
          total_edge = which_edge;
        }
        else if((which_edge == 4) && (obs_edge != 0)){
          total_edge = which_edge;
        }
        else if((which_edge != 0) && (obs_edge == 5)){
          total_edge = obs_edge;
        }
        else if((which_edge != 0) && (obs_edge == 4)){
          total_edge = obs_edge;
        }
        else{
          total_edge = 0;
        }

        if((total_edge == 3) || (total_edge == 4) || (total_edge == 5)){
          reset_global_var();
          error_path_setup = true;
          setup_path = true;
          path.clear();
          time_zero = millis();
          digitalWrite(yellow, LOW);
          digitalWrite(red, LOW);
          old_card_num = card_num;
          edge_error_happened = true;
          is_backward = true;
          return;
        }
        not_finding_any_card(total_edge);
        rfid.PCD_StopCrypto1();
        return;
      }
      
      if(error_case == 6){
        if(old_diff_glob == 1){
          if(check_right_and_left(card_num + 10)){
            brake(motor_left, motor_right); 
            //delay(1000);
            target -= 90;
            if (target <= -180){
              target += 360;
            }
            is_right = true;
            is_forward = false;
            flag_dur = true;
          }
          else{
            brake(motor_left, motor_right); 
            //delay(1000);              
            target += 90;
            if (target > 180){
              target -= 360;
            }
            is_left = true;
            is_forward = false;
            flag_dur = true;
          }
        }
        else if(old_diff_glob == 10){
          if(check_right_and_left(card_num - 1)){
            brake(motor_left, motor_right); 
            //delay(1000);
            target -= 90;
            if (target <= -180){
              target += 360;
            }
            is_right = true;
            is_forward = false;
            flag_dur = true;
          }
          else{
            brake(motor_left, motor_right); 
            //delay(1000);              
            target += 90;
            if (target > 180){
              target -= 360;
            }
            is_left = true;
            is_forward = false;
            flag_dur = true;
          }
        }
        else if(old_diff_glob == -1){
          if(check_right_and_left(card_num - 10)){
            brake(motor_left, motor_right); 
            //delay(1000);
            target -= 90;
            if (target <= -180){
              target += 360;
            }
            is_right = true;
            is_forward = false;
            flag_dur = true;
          }
          else{
            brake(motor_left, motor_right); 
            //delay(1000);              
            target += 90;
            if (target > 180){
              target -= 360;
            }
            is_left = true;
            is_forward = false;
            flag_dur = true;
          }
        }
        else if(old_diff_glob == -10){
          if(check_right_and_left(card_num + 1)){
            brake(motor_left, motor_right); 
            //delay(1000);
            target -= 90;
            if (target <= -180){
              target += 360;
            }
            is_right = true;
            is_forward = false;
            flag_dur = true;
          }
          else{
            brake(motor_left, motor_right); 
            //delay(1000);              
            target += 90;
            if (target > 180){
              target -= 360;
            }
            is_left = true;
            is_forward = false;
            flag_dur = true;
          }
        }
        time_zero = millis();
        error_case = 4;
        old_card_num = card_num;
      }
      
      if((old_card_num == card_num)){
        time_zero = millis();
        rfid.PCD_StopCrypto1();
        return;
      }
      
      if((target_location == card_num) && (!return_base) && (!flag_target_found_before)){
        time_delay = time_delay_const;
        brake(motor_left, motor_right); 
        //delay(300);
        //path.clear();
        Serial.println("target bulundu");
        found_target = true;
        time_zero = millis();
        return_base = true; 
        //old_card_num = card_num;
        rfid.PCD_StopCrypto1();
        flag_target_found_before = true;
        return;
      }
      if((check_corner(old_card_num)) && (!if_cross_error)){
        
        Serial.println("corner");
        cross_card = card_num;
        cross_step = encode_cross_direction(card_num, old_card_num, path[0]);
        if_cross_error = true;
        time_zero = millis() - time_delay - 1;
        rfid.PCD_StopCrypto1();
        return;

      }
      else if(cross_card == card_num){
        Serial.println("ayni cross kart");
        time_zero = millis();
        rfid.PCD_StopCrypto1();
        return;
      }
      
      if(card_num == target_location){
        time_zero = millis();
        rfid.PCD_StopCrypto1();
        return;
      }
      else{
        time_delay = time_delay_const;
        Serial.println("reset atiyoz");
        brake(motor_left, motor_right); 
        reset_global_var();
        error_path_setup = true;
        setup_path = true;
        path.clear();
        time_zero = millis();
        digitalWrite(yellow, LOW);
        digitalWrite(red, LOW);
        old_card_num = card_num;
        which_edge = edge_case(card_num);
        obs_edge = edge_case_of_obs(card_num);
        if((which_edge == 1) && (obs_edge == 2)){
          total_edge = 4;
        }
        else if((which_edge == 2) && (obs_edge == 1)){
          total_edge = 5;
        }
        else if((which_edge == 3) && (obs_edge == 1)){
          total_edge = 4;
        }
        else if((which_edge == 3) && (obs_edge == 2)){
          total_edge = 5;
        }
        else if((which_edge == 1) && (obs_edge == 3)){
          total_edge = 4;
        }
        else if((which_edge == 2) && (obs_edge == 3)){
          total_edge = 5;
        }
        else if((which_edge == 0) && (obs_edge != 0)){
          total_edge = obs_edge;
        }
        else if((which_edge != 0) && (obs_edge == 0)){
          total_edge = which_edge;
        }
        else if((which_edge == 5) && (obs_edge != 0)){
          total_edge = which_edge;
        }
        else if((which_edge == 4) && (obs_edge != 0)){
          total_edge = which_edge;
        }
        else if((which_edge != 0) && (obs_edge == 5)){
          total_edge = obs_edge;
        }
        else if((which_edge != 0) && (obs_edge == 4)){
          total_edge = obs_edge;
        }
        else{
          total_edge = 0;
        }


        if((total_edge == 3) || (total_edge == 4) || (total_edge == 5)){
          edge_error_happened = true;
          setup_path = false;
          is_backward = true;
          return;
        }
      }
    }
  }
  rfid.PCD_StopCrypto1();
}

void not_finding_any_card(int which_case){
  brake(motor_left, motor_right); 
  //delay(300);
  is_forward = false;
  is_backward = false;
  time_zero = millis();
  error_case = 0;
  if(step_index > 12){
    step_index = 0;
  }
  step_size = step_size_vec[step_index];
  if(which_case == 1){
    step_size = step_size_vec_right[step_index];
  }
  else if(which_case == 2){
    step_size = step_size_vec_left[step_index];
  }

  
  step_index += 1;

  if(if_cross_error){
    step_size = cross_step;
    if_cross_error = false;
    cross_card = 0;
  }
  /*
  else if(count_number_of_errors > 3){
    step_size *= 3;
  }
  */
}
bool check_base_com_tile(int loc){
  int good_tile[] = {21,31,41,52,63,64,65,56,47,37,27,16,5,4,3,12,13,22,32,42,53,54,55,46,36,26,15,14,23,33,43,44,45,35,25,24};
  for(int edge_loc:good_tile){
    if(loc == edge_loc){
      return true;
    }
  }
  return false;
}

bool check_near_base_tile(int loc){
  int good_tile[] = {33, 24, 35, 44, obs1 - 10, obs1 - 1, obs1 + 1, obs1 + 10, obs2 - 10, obs2 - 1, obs2 + 1, obs2 + 10, obs3 - 10, obs3 - 1, obs3 + 1, obs3+ 10};
  for(int edge_loc:good_tile){
    if(loc == edge_loc){
      return false;
    }
  }
  return true;
}

int edge_case_of_obs(int loc){


  int obs1_temp, obs2_temp, obs3_temp;

  if(obs1 != 0){
    obs1_temp = obs1;
  }
  else{
    obs1_temp = -100;
  }

  if(obs2 != 0){
    obs2_temp = obs2;
  }
  else{
    obs2_temp = -100;
  }
  if(obs3 != 0){
    obs3_temp = obs3;
  }
  else{
    obs3_temp = -100;
  }
  
  int north[]  = {obs1_temp - 10, obs1_temp-11, obs1_temp-9, obs2_temp - 10, obs2_temp - 11, obs2_temp-9, obs3_temp - 10, obs3_temp-11, obs3_temp-9, 23, 24, 25};
  int south[]  = {obs1_temp + 10, obs1_temp+11, obs1_temp+9, obs2_temp + 10, obs2_temp + 11, obs2_temp+9, obs3_temp + 10, obs3_temp+11, obs3_temp+9, 43, 44, 45};
  int east[]  = {obs1_temp + 9, obs1_temp-11, obs1_temp-1, obs2_temp + 9, obs2_temp-11, obs2_temp-1, obs3_temp + 9, obs3_temp-11, obs3_temp-1, 23, 33, 43};
  int west[]  = {obs1_temp + 11, obs1_temp+1, obs1_temp-9, obs2_temp + 11, obs2_temp+1, obs2_temp-9, obs3_temp + 11, obs3_temp+1, obs3_temp-9, 25, 35, 45};
  int x = 0;

  int angle_temp;
  angle_temp = angle + cum_angle;
  if (angle_temp > 180){
    angle_temp -= 360;
  }
  if (angle_temp < -180){
    angle_temp += 360;
  }

  for(int edge_loc:north){
    if(loc == edge_loc){
      if(((first_diff_global == 10) && (angle_temp < -45) && (angle_temp > -135) ) || ((first_diff_global == 1) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == -1) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == -10) && (angle_temp > 45) && (angle_temp < 135) )){
        x += 1; 
        // east
      }
      else if(((first_diff_global == 10) && (angle_temp < 135) && (angle_temp > 45) ) || ((first_diff_global == 1) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == -1) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == -10) && (angle_temp > -135) && (angle_temp < -45) )){
        x +=  2; 
        // west
      }
      else if(((first_diff_global == 10) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == 1) && (angle_temp > 45) && (angle_temp < 135) ) || ((first_diff_global == -1) && (angle_temp > -135) && (angle_temp < -45) ) || ((first_diff_global == -10) && (angle_temp > 135) && (angle_temp < 200) )){
        x +=  3; 
        // 
      }
    }
  }


  for(int edge_loc:south){
    if(loc == edge_loc){
      if(((first_diff_global == 10) && (angle_temp < -45) && (angle_temp > -135) ) || ((first_diff_global == 1) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == -1) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == -10) && (angle_temp > 45) && (angle_temp < 135) )){
        x +=  2; 
        //sağa gidiyo
      }
      else if(((first_diff_global == 10) && (angle_temp < 135) && (angle_temp > 45) ) || ((first_diff_global == 1) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == -1) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == -10) && (angle_temp > -135) && (angle_temp < -45) )){
        x +=  1; 
        //sola gidiyo
      }
      else if(((first_diff_global == 10) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == 1) && (angle_temp > -135) && (angle_temp < -45) ) || ((first_diff_global == -1) && (angle_temp > 45) && (angle_temp < 135) ) || ((first_diff_global == -10) && (angle_temp > -45) && (angle_temp < 45) )){
        x +=  3; 
        //aşağı
      }
    }
  }


  for(int edge_loc:west){
    if(loc == edge_loc){
      if(((first_diff_global == 10) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == 1) && (angle_temp > 45) && (angle_temp < 135) ) || ((first_diff_global == -1) && (angle_temp > -135) && (angle_temp < -45) ) || ((first_diff_global == -10) && (angle_temp > 135) && (angle_temp < 200) )){
        x +=  1; 
        //yukarı
      }
      else if(((first_diff_global == 10) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == 1) && (angle_temp > -135) && (angle_temp < -45) ) || ((first_diff_global == -1) && (angle_temp > 45) && (angle_temp < 135) ) || ((first_diff_global == -10) && (angle_temp > -45) && (angle_temp < 45) )){
        x +=  2; 
        //aşağı
      }
      else if(((first_diff_global == 10) && (angle_temp < 135) && (angle_temp > 45) ) || ((first_diff_global == 1) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == -1) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == -10) && (angle_temp > -135) && (angle_temp < -45) )){
        x +=  3; 
        //sola gidiyo
      }
    }
  }

  for(int edge_loc:east){
    if(loc == edge_loc){
      if(((first_diff_global == 10) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == 1) && (angle_temp > 45) && (angle_temp < 135) ) || ((first_diff_global == -1) && (angle_temp > -135) && (angle_temp < -45) ) || ((first_diff_global == -10) && (angle_temp > 135) && (angle_temp < 200) )){
        x +=  2; 
        //yukarı
      }
      else if(((first_diff_global == 10) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == 1) && (angle_temp > -135) && (angle_temp < -45) ) || ((first_diff_global == -1) && (angle_temp > 45) && (angle_temp < 135) ) || ((first_diff_global == -10) && (angle_temp > -45) && (angle_temp < 45) )){
        x +=  1; 
        //aşağı
      }
      else if(((first_diff_global == 10) && (angle_temp < -45) && (angle_temp > -135) ) || ((first_diff_global == 1) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == -1) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == -10) && (angle_temp > 45) && (angle_temp < 135) )){
        x +=  3; 
        //sağa gidiyo
      }
    }
  }

  return x;
}

int edge_case(int loc){
  int north[] = {81, 82, 83, 84, 85, 86, 87, 88, 89};
  int east[] = {9, 19,29,39,49,59,69,79, 89};
  int west[] = {1, 11, 21, 31, 41, 51, 61, 71, 81};
  int south[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  

  int x = 0;

  // 1: saga dogru git
  // 2: sola dogru git
  // 3: arkaya git
  // 4: arkaya git saga don
  // 5: arkaya git sola don
  int angle_temp;
  angle_temp = angle + cum_angle;
  if (angle_temp > 180){
    angle_temp -= 360;
  }
  if (angle_temp < -180){
    angle_temp += 360;
  }

  for(int edge_loc:north){
    if(loc == edge_loc){
      if(((first_diff_global == 10) && (angle_temp < -45) && (angle_temp > -135) ) || ((first_diff_global == 1) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == -1) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == -10) && (angle_temp > 45) && (angle_temp < 135) )){
        x += 1; 
        // east
      }
      else if(((first_diff_global == 10) && (angle_temp < 135) && (angle_temp > 45) ) || ((first_diff_global == 1) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == -1) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == -10) && (angle_temp > -135) && (angle_temp < -45) )){
        x +=  2; 
        // west
      }
      else if(((first_diff_global == 10) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == 1) && (angle_temp > 45) && (angle_temp < 135) ) || ((first_diff_global == -1) && (angle_temp > -135) && (angle_temp < -45) ) || ((first_diff_global == -10) && (angle_temp > 135) && (angle_temp < 200) )){
        x +=  3; 
        // 
      }
    }
  }


  for(int edge_loc:south){
    if(loc == edge_loc){
      if(((first_diff_global == 10) && (angle_temp < -45) && (angle_temp > -135) ) || ((first_diff_global == 1) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == -1) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == -10) && (angle_temp > 45) && (angle_temp < 135) )){
        x +=  2; 
        //sağa gidiyo
      }
      else if(((first_diff_global == 10) && (angle_temp < 135) && (angle_temp > 45) ) || ((first_diff_global == 1) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == -1) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == -10) && (angle_temp > -135) && (angle_temp < -45) )){
        x +=  1; 
        //sola gidiyo
      }
      else if(((first_diff_global == 10) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == 1) && (angle_temp > -135) && (angle_temp < -45) ) || ((first_diff_global == -1) && (angle_temp > 45) && (angle_temp < 135) ) || ((first_diff_global == -10) && (angle_temp > -45) && (angle_temp < 45) )){
        x +=  3; 
        //aşağı
      }
    }
  }


  for(int edge_loc:west){
    if(loc == edge_loc){
      if(((first_diff_global == 10) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == 1) && (angle_temp > 45) && (angle_temp < 135) ) || ((first_diff_global == -1) && (angle_temp > -135) && (angle_temp < -45) ) || ((first_diff_global == -10) && (angle_temp > 135) && (angle_temp < 200) )){
        x +=  1; 
        //yukarı
      }
      else if(((first_diff_global == 10) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == 1) && (angle_temp > -135) && (angle_temp < -45) ) || ((first_diff_global == -1) && (angle_temp > 45) && (angle_temp < 135) ) || ((first_diff_global == -10) && (angle_temp > -45) && (angle_temp < 45) )){
        x +=  2; 
        //aşağı
      }
      else if(((first_diff_global == 10) && (angle_temp < 135) && (angle_temp > 45) ) || ((first_diff_global == 1) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == -1) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == -10) && (angle_temp > -135) && (angle_temp < -45) )){
        x +=  3; 
        //sola gidiyo
      }
    }
  }

  for(int edge_loc:east){
    if(loc == edge_loc){
      if(((first_diff_global == 10) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == 1) && (angle_temp > 45) && (angle_temp < 135) ) || ((first_diff_global == -1) && (angle_temp > -135) && (angle_temp < -45) ) || ((first_diff_global == -10) && (angle_temp > 135) && (angle_temp < 200) )){
        x +=  2; 
        //yukarı
      }
      else if(((first_diff_global == 10) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == 1) && (angle_temp > -135) && (angle_temp < -45) ) || ((first_diff_global == -1) && (angle_temp > 45) && (angle_temp < 135) ) || ((first_diff_global == -10) && (angle_temp > -45) && (angle_temp < 45) )){
        x +=  1; 
        //aşağı
      }
      else if(((first_diff_global == 10) && (angle_temp < -45) && (angle_temp > -135) ) || ((first_diff_global == 1) && (angle_temp > -45) && (angle_temp < 45) ) || ((first_diff_global == -1) && (angle_temp > 135) && (angle_temp < 200) ) || ((first_diff_global == -10) && (angle_temp > 45) && (angle_temp < 135) )){
        x +=  3; 
        //sağa gidiyo
      }
    }
  }

  return x;
}
/********************************************************/
/****************COMMUNICATION FUNCTIONS*****************/

unsigned int createMessageMU(unsigned int uiHeader, unsigned int uiReceivement, unsigned int uiFinding, unsigned int uiCurrentLocation, unsigned int uiTargetLocation) //Creating messages in predetermined format for MUs
{
  unsigned int message;
  message =((~((uiHeader<<4) | (uiReceivement << 2) | (uiFinding))) << 24) |(uiHeader << 20) | (uiReceivement << 18) | (uiFinding << 16) | (uiCurrentLocation << 8) | (uiTargetLocation << 0);
  return message;
}

bool check_right_and_left(int loc){
  int possible[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 24, 25, 26, 27, 28, 29, 31, 32, 33, 35, 36, 37, 38, 39, 41, 42, 43, 44, 45, 46, 47, 48, 49, 51, 52, 53, 54, 55, 56, 57, 58, 59, 61, 62, 63, 64, 65, 66, 67, 68, 69, 71, 72, 73, 74, 75, 76, 77, 78, 79, 81, 82, 83, 84, 85, 86, 87, 88, 89};
  bool x = false;
  for(int edge_loc:possible){
    if(loc == edge_loc){
      x = true;
    }
  }
  if((loc == obs1) || (loc == obs2) || (loc == obs3)){
    x = false;
  }
  return x;

}

int check_ultrasonic(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, 3000);
  
  // // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  time_zero = millis();
  if((distanceCm < 15) && (distanceCm != 0.00)){
    Serial.println(distanceCm);
    brake(motor_left, motor_right); 
    rfid.PCD_StopCrypto1();
    dur = true;
    return true;
  }
  return false;
}

void rotate_edge(){
  int deltaangle = (target - angle);

  if ((target == 180) && (angle < 0)){
    deltaangle = (-target - angle);
  }


  if (abs(deltaangle) <= 1.0){
    brake(motor_left, motor_right); 
    //delay(100);
    
    is_left = false;
    is_right = false;
    is_backward_after_rotate = true;
    time_zero = millis();
    is_forward = true;
  } 
  else {
    if (is_left) { //turn left
      motor_left.drive(-left_motor_fast);
      motor_right.brake();
    } else if (is_right) {//turn right
      motor_left.brake();
      motor_right.drive(-right_motor_fast);
    }
  }




}

/*
bool looking_outside_or_not(int orientation, int which_edge){
  if((angle > -45) && (angle < 45){
    if(orientation == which_edge){
      return 0;
    }
    
  }
}
*/


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
 // mu->uiMUFinding = (mu->uiMUCurrentLocation == 0x64) ? MU_TARGET_FOUND : MU_TARGET_NOT_FOUND;
   mu->uiMUFinding = MU_TARGET_FOUND;
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

  else if(buffer[0] == 115 && buffer[1] == 203 && buffer[2] == 7 && buffer[3] == 37) {
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

  else if(buffer[0] == 211 && buffer[1] == 21 && buffer[2] == 5 && buffer[3] == 37) {
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

  else if(buffer[0] == 146 && buffer[1] == 38 && buffer[2] == 173 && buffer[3] == 81) {
    //Serial.println("F3");
    return 53;
  }

  else if(buffer[0] == 19 && buffer[1] == 238 && buffer[2] == 195 && buffer[3] == 36) {
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


void rotate (){
  int deltaangle = (target - angle);

  if ((target == 180) && (angle < 0)){
    deltaangle = (-target - angle);
  }

  if(is_left){
    if ((deltaangle <= 2) && (deltaangle >= 0)){
      brake(motor_left, motor_right); 
      is_left = false;
      is_right = false;
      is_backward_after_rotate = true;
      time_zero = millis();
      is_forward = true;
      if(edge_error_happened){
        edge_error_happened = false;
        setup_path = true;
        finished = false;
        reset_global_var();
      }
      if(ultrasonic_error_happened){
        ultrasonic_error_happened = false;
        setup_path = true;
        finished = false;
        reset_global_var();
        dur = false;
      }
    }
    else{
      motor_left.brake();
      motor_right.drive(right_motor_fast);
    }
  }
  else if(is_right){
    if ((deltaangle >= -2) && (deltaangle <= 0)){
      brake(motor_left, motor_right); 
      is_left = false;
      is_right = false;
      is_backward_after_rotate = true;
      time_zero = millis();
      is_forward = true;
      if(edge_error_happened){
        edge_error_happened = false;
        setup_path = true;
        finished = false;
        reset_global_var();
      }
      if(ultrasonic_error_happened){
        ultrasonic_error_happened = false;
        setup_path = true;
        finished = false;
        reset_global_var();
        dur = false;
      }
    }
    else{
      motor_left.drive(left_motor_fast);
      motor_right.brake();
    }
  }
} 


/***************************************************/
/******************MOTOR FUNCTIONS******************/

int check_closest_tile(int sub_tile,int loc){
  vector<int> tiles;
  int closest_element;
  if (sub_tile == 1){
    tiles = g1.Tiles_IDs_vector();
    closest_element = findClosestElement(tiles,loc);
  }
  else if (sub_tile == 2){
    tiles = g2.Tiles_IDs_vector();
    closest_element = findClosestElement(tiles,loc);
  }
  else if (sub_tile == 3){
    tiles = g3.Tiles_IDs_vector();
    closest_element = findClosestElement(tiles,loc);
  }
  return closest_element;
} 
// int check_closest_tile(int sub_tile,int loc){
//   int firstDigit, secondDigit, loc_tile;
//   if (loc >= 10 && loc <= 99) {
//     firstDigit = loc / 10;
//     secondDigit = loc % 10;
//   }
//   else{
//     firstDigit = 0;
//     secondDigit = loc;
//   }
//   if(secondDigit <= 3){
//     loc_tile = 1;
//   }
//   else if(firstDigit < 4){
//     loc_tile = 2;
//   }
//   else{
//     loc_tile = 3;
//   }

//   if(sub_tile == 1){
//     return firstDigit*10 + 3;
//   }
//   if(sub_tile == 2){
//     if(loc_tile == 1){
//       return firstDigit*10 + 4;
//     }   
//     else if(secondDigit != 4){
//       return 30 + secondDigit;
//     }
//     else{
//       return 35;
//     }
//   }
//   if(sub_tile == 3){
//     if(loc_tile == 1){
//       return firstDigit*10 + 4;
//     }   
//     else{
//       return 40 + secondDigit;
//     }
//   }
// } 


/*
void rotate_degree(float target_correction){
  float deltaangle = (target_correction - angle + target);

  if ((target == 180) && (angle < 0)){
    if(target_correction > 0){
      deltaangle = (-target + target_correction - angle);
    }
    else{
      deltaangle = (-target + target_correction - angle);
    }
    
  }
  else if ((target == 180) && (angle > 0)){
    if(target_correction > 0){
      deltaangle = (-angle + target_correction + target);
    }
    else{
      deltaangle = (-angle + target_correction + target);
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
  else if((deltaangle) > 0){
  
    motor_left.brake();
    motor_right.drive(right_motor_fast);
  
  }
  else if((deltaangle) < 0){
  
    motor_right.brake();
    motor_left.drive(left_motor_fast);
  
  }


}
*/
void rotate_degree(float target_correction){
  float deltaangle = (target_correction - angle + target);

  if ((target == 180) && (angle < 0)){
    if(target_correction > 0){
      deltaangle = (-target + target_correction - angle);
    }
    else{
      deltaangle = (-target + target_correction - angle);
    }
    
  }
  else if ((target == 180) && (angle > 0)){
    if(target_correction > 0){
      deltaangle = (-angle + target_correction + target);
    }
    else{
      deltaangle = (-angle + target_correction + target);
    }
  }

  if (abs(deltaangle) <= 1){
    brake(motor_left, motor_right); 
    //delay(300);
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

int check_corner(int base_loc){

  if((((base_loc - 11) == card_num) || ((base_loc + 11) == card_num) || ((base_loc - 9) == card_num) || ((base_loc + 9) == card_num))){
    return true; // MU's direction is tilted
  }
  return false; //what is happening bro
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


  

void reset_global_var(){
  is_right = false;
  is_left = false;
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
    if(!IRstop)
    { 
      if(mu.uiMUFinding == MU_TARGET_FOUND)
      {
        IrSender.sendNEC(address, command, 0);
      }
      else
      {
        if(bu.uiBUTalk == BU_AVAILABLE || bu.uiBUTalk == mu.uiMUHeader)
        {
          IrSender.sendNEC(address, command, 0);
        }
      }
    }
    vTaskDelay(258 / portTICK_PERIOD_MS);
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
            if(mu.uiMUReceivement == MU_MSG_RECEIVED)
            {
              IRstop = 1;
              march = 1;
            }
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
            }
          }
        }
      }
    }
    IrReceiver.resume();
  }
}
/*
//HC_SR04 *HC_SR04::_instance=NULL;
HC_SR04 *HC_SR04::_instance(NULL);

HC_SR04::HC_SR04(int trigger, int echo, int interrupt, int max_dist)
    : _trigger(trigger), _echo(echo), _int(interrupt), _max(max_dist), _finished(false)
{
  if(_instance==0) _instance=this;    
}

void HC_SR04::begin(){
  pinMode(_trigger, OUTPUT);
  digitalWrite(_trigger, LOW);
  pinMode(_echo, INPUT);  
  attachInterrupt(_int, _echo_isr, CHANGE);
}

void HC_SR04::start(){
  _finished=false;
  digitalWrite(_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger, LOW);  
}

unsigned int HC_SR04::getRange(bool units){
  return (_end-_start)/((units)?58:148);
}

void HC_SR04::_echo_isr(){
  HC_SR04* _this=HC_SR04::instance();
  
  switch(digitalRead(_this->_echo)){
    case HIGH:
      _this->_start=micros();
      break;
    case LOW:
      _this->_end=micros();
      _this->_finished=true;
      break;
  }   
}

*/