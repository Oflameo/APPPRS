/* 
 * rosserial::std_msgs::Time Test
 * Publishes current time
 */
#include <Arduino.h>
#include <ros.h>
#include <stdint.h>
//#include <ros/time.h>
//#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>


//#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <std_msgs/MultiArrayDimension.h>
#include<Wire.h> //wire library is just for I2C stuff
//#include <std_msgs/Int32MultiArray.h>

#include <Servo.h>


const int MPU=0x68;  // I2C address of the MPU-60D0
const int tachPinA = 2;
const int tachPinB = 3;

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ,dist,tacState, totalDist = 0;
unsigned long oldTime;
float Speed=0;
float TicCoef=1.3522*.5; // mm/tic

ros::NodeHandle  nh;


std_msgs:: Int16MultiArray test;
ros::Publisher p("testDat", &test);

const int steeringServoPin = 11;
const int speedServoPin = 12;


//**Steering Servo setup***********
Servo steeringServo;
int maxSteerAngle = 23;
int servoTrim = 0;  //positive trim goes left
int servoRange = 90;  //rotational range of servo, from full CW to full CCW
int steeringDemand;
//************************




//***************
// **NOTE: need to fix limits here!!!*****
//***************

//**Velocity Servo setup***********
Servo speedServo;
float maxSpeed = 4;  // m/s
int speedTrim = 5;  //positive trim goes left
int speedRange = 90;  //rotational range of servo, from full CW to full CCW
int speedDemand;
//************************

  float speedP = 1.0;
  float speedFF = 10.0;
  float speedSP = 0.0;
  
  int maxPower = 30;
  int motorPower = 0;
  





//**Loop control variables****
int sampleDurration = 100; // time to sample sensors in ms
unsigned long sampleStartTime;
unsigned int numSamples;


const int calCycles = 400;

int16_t GyXcal = 0;
int16_t GyYcal = 0;
int16_t GyZcal = 0;
int16_t AcXcal = 0;
int16_t AcYcal = 0;
int16_t AcZcal = 0;

int32_t AcXAccm, AcYAccm, AcZAccm, GyXAccm, GyYAccm, GyZAccm;



  void steerCommand_cb( const std_msgs::Float32& steer_cmd_msg)
  {
  setSteerAngle(steer_cmd_msg.data);
//    setSteerAngle(cmd_msg.data[1]);

  }
  
  void speedCommand_cb( const std_msgs::Float32& speed_cmd_msg)
  {
  //setSteerAngle(cmd_msg.data);
  //setPower(speed_cmd_msg.data);
    speedSP = speed_cmd_msg.data;
  }


//ros::Subscriber<std_msgs::Float32MultiArray> sub("carCommand", carCommand_cb);
ros::Subscriber<std_msgs::Float32> steerSub("steerCommand", steerCommand_cb);

ros::Subscriber<std_msgs::Float32> speedSub("speedCommand", speedCommand_cb);



void setup()
{
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(57600);
  pinMode(7,OUTPUT); //set them to digital inputs and outputs
  pinMode(3,INPUT);
  pinMode(2,INPUT);
  digitalWrite(7,HIGH);
  pinMode(13, OUTPUT);
  nh.initNode();
  //test.layout.dim_length=1;
  test.data_length=5;
  //test.layout.dim[0].label = "AX" ;
  //test.layout.dim[0].size = 8;
  //test.layout.dim[0].stride = 1*8;
  test.layout.data_offset = 0;
  nh.subscribe(steerSub);
  nh.subscribe(speedSub);
  
  nh.advertise(p);
  totalDist=0;
  attachInterrupt(1, TachRead, CHANGE);

  //Set pin 52 to high (5v) to act as power for IMU breakout board
  pinMode(52,OUTPUT);
  digitalWrite(52,HIGH);

  //Steering setup
  steeringServo.attach(steeringServoPin);
  steeringDemand = 0;
  speedServo.attach(speedServoPin);
  speedDemand = 0;
  
  
  
  //setSteerAngle(0);
  
  
  delay(3000);
  
  // ** Sample gyro for set number of cycles **
  for (int i = 0; i < calCycles; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
   
    // All of these numbers are 2 bytes long, signed floats
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    AcXcal += AcX;
    AcYcal += AcY;
    AcZcal += AcZ;
    GyXcal += GyX;
    GyYcal += GyY;
    GyZcal += GyZ;
  }
  
  AcXcal /= calCycles;
  AcYcal /= calCycles;
  AcZcal /= calCycles;
  GyXcal /= calCycles;
  GyYcal /= calCycles;
  GyZcal /= calCycles;
  
  /*
  delay(500);
  //Steering test and reset
  setSteerAngle(-5);
  //Serial.print("Servo-15");
  delay(500);
  setSteerAngle(5);
  //Serial.print("Servo 15");
  delay(500);
  setSteerAngle(0); 
  //Serial.print("Servo 0");
  */

  oldTime=millis();
 }


void loop()
{  
  
  

  

  AcXAccm = 0;
  AcYAccm = 0;
  AcZAccm = 0;
  GyXAccm = 0;
  GyYAccm = 0;
  GyZAccm = 0;

  numSamples = 0;

  sampleStartTime = millis();
  
  /*do
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);  // request a total of 14 registers

    // All of these numbers are 2 bytes long, signed floats
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    AcX = AcX - AcXcal;
    AcY = AcY - AcYcal;
    AcZ = AcZ - AcZcal;
    GyX = GyX - GyXcal;
    GyY = GyY - GyYcal;
    GyZ = GyZ - GyZcal;
    
    AcXAccm += AcX;
    AcYAccm += AcY;
    AcZAccm += AcZ;
    GyXAccm += GyX;
    GyYAccm += GyY;
    GyZAccm += GyZ;
    
    numSamples++;
    
   // delay(1);
    
  } while (millis() <= sampleStartTime + sampleDurration);
   
   
   
  AcX = AcXAccm / numSamples;
  AcY = AcYAccm / numSamples;
  AcZ = AcZAccm / numSamples;
  GyX = GyXAccm / numSamples;
  GyY = GyYAccm / numSamples;
  GyZ = GyZAccm / numSamples;
  */
  
  // **** Read IMU ****
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    AcX = AcX - AcXcal;
    AcY = AcY - AcYcal;
    AcZ = AcZ - AcZcal;
    GyX = GyX - GyXcal;
    GyY = GyY - GyYcal;
    GyZ = GyZ - GyZcal;

  
  test.data[1]=GyX;
  test.data[2]=GyY;
  test.data[3]=GyZ;
  
  test.data[4]=dist;
//  Serial.print
  Speed=(dist*TicCoef/1000)/(float(millis()-oldTime)/1000);
  oldTime=millis();
  totalDist=totalDist+dist;
  dist=0;  
  
  motorPower = speedSP*speedFF;// + (speedSP - Speed);
  setPower(motorPower);
  
  //Serial.print("speed: ");Serial.println(Speed);
  


  //Serial.print("GX: "); Serial.print(GyX);
  //Serial.print("\tdist: "); Serial.print(totalDist);
  //Serial.print("\tGZ: "); Serial.println(GyZ);

  p.publish( &test );
  nh.spinOnce();
  delay(100);
}


void TachRead ()
{
    if (digitalRead(tachPinA)==digitalRead(tachPinB))
      dist--;
    else
      dist++;
}

void setSteerAngle(float _angleIn)
{
  _angleIn = 1*constrain(_angleIn, -1*maxSteerAngle, maxSteerAngle) + servoTrim + 90;
  steeringServo.write(_angleIn);
}

void setPower(float _powerIn)
{
  _powerIn = constrain(_powerIn, -1*maxPower, maxPower);
  _powerIn = map(_powerIn, -100, 100, 0, 180);
 //Serial.print("power set: "); Serial.print(_powerIn);
  speedServo.write(_powerIn);
}
