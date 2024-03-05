#include <AFMotor.h>
#include <SoftwareSerial.h>
//SoftwareSerial mySerial(9, 10); // RX, TX
char data;
int ss = 255;

AF_DCMotor motor_1(1);
AF_DCMotor motor_2(2);
AF_DCMotor motor_3(3);
AF_DCMotor motor_4(4);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");
  

  // turn on motor
  motor_1.setSpeed(ss);
  motor_2.setSpeed(ss);
  motor_3.setSpeed(ss);
  motor_4.setSpeed(ss);
 
  motor_1.run(RELEASE);
  motor_2.run(RELEASE);
  motor_3.run(RELEASE);
  motor_4.run(RELEASE);
}

void ileri(){
  
 motor_1.setSpeed(ss);
 motor_2.setSpeed(ss);
 motor_3.setSpeed(ss);
 motor_4.setSpeed(ss);
 
 motor_1.run(FORWARD);
 motor_2.run(RELEASE);
 motor_3.run(BACKWARD);
 motor_4.run(RELEASE);

  
}

void geri(){


  
 motor_1.setSpeed(ss);
 motor_2.setSpeed(ss);
 motor_3.setSpeed(ss);
 motor_4.setSpeed(ss);
 
 motor_1.run(FORWARD);
 motor_2.run(BACKWARD);
 motor_3.run(FORWARD);
 motor_4.run(BACKWARD);

  
}

void sol(){

 motor_1.setSpeed(ss);
 motor_2.setSpeed(ss);
 motor_3.setSpeed(ss);
 motor_4.setSpeed(ss);
 
 motor_1.run(FORWARD);
 motor_2.run(FORWARD);
 motor_3.run(BACKWARD);
 motor_4.run(BACKWARD);

 
  
}

void sag(){

 motor_1.setSpeed(ss);
 motor_2.setSpeed(ss);
 motor_3.setSpeed(ss);
 motor_4.setSpeed(ss);
 
 motor_1.run(RELEASE);
 motor_2.run(FORWARD);
 motor_3.run(RELEASE);
 motor_4.run(BACKWARD);
  
  
}

void dur(){

 
  motor_1.run(RELEASE);
  motor_2.run(RELEASE);
  motor_3.run(RELEASE);
  motor_4.run(RELEASE);
  
}

void loop() {

 delay(5000);  
 ileri();

 delay(5000);

 sag();
 delay(5000);
/*     
 geri();
 delay(5000);
     
 sol();
 delay(5000);
  
 sag();
 delay(5000);
 dur();
 delay(5000);*/

/*
 motor_1.setSpeed(ss);
 motor_1.run(FORWARD);
 
 delay(5000);
 motor_1.run(RELEASE);

 motor_2.setSpeed(ss);
 motor_2.run(FORWARD);
 delay(5000);

 motor_2.run(RELEASE);

 motor_3.setSpeed(ss);
 motor_3.run(FORWARD);
 delay(5000);

 motor_3.run(RELEASE);

 motor_4.setSpeed(ss);
 motor_4.run(FORWARD);
 delay(5000);

 motor_4.run(RELEASE);
 */
}
