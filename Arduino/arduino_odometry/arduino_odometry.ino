/* 
 * rosserial::std_msgs::Time Test
 * Publishes current time
 */
 #include <Arduino.h>
#include <ros.h>
#include <stdint.h>
//#include <ros/time.
//#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>


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

ros::NodeHandle  nh;


std_msgs:: Int16MultiArray test;
ros::Publisher p("testDat", &test);

const int steeringServoPin = 11;

//**Servo setup***********
Servo steeringServo;
int maxSteerAngle = 35;
int servoTrim = 5;  //positive trim goes left
int servoRange = 90;  //rotational range of servo, from full CW to full CCW
int steeringDemand;
//************************

const int calCycles = 400;
long GyXcal = 0;
long GyYcal = 0;
long GyZcal = 0;
long AcXcal = 0;
long AcYcal = 0;
long AcZcal = 0;

void servo_cb( const std_msgs::Float32& cmd_msg)
  {
  setServo(cmd_msg.data);
  }


ros::Subscriber<std_msgs::Float32> sub("servo", servo_cb);


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
  nh.subscribe(sub);
  
  nh.advertise(p);
  totalDist=0;
  attachInterrupt(1, TachRead, CHANGE);

  //Set pin 52 to high (5v) to act as power for IMU breakout board
  pinMode(52,OUTPUT);
  digitalWrite(52,HIGH);

  //Steering setup
  steeringServo.attach(steeringServoPin);
  steeringDemand = 0;
  
  delay(1000);
  
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
  
  delay(500);
  //Steering test and reset
  setServo(-15);
  delay(250);
  setServo(15);
  delay(250);
  setServo(0); 
 }


void loop()
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
  
  test.data[1]=GyX;
  test.data[2]=GyY;
  test.data[3]=GyZ;
  
  test.data[4]=dist;
  totalDist=totalDist+dist;
  dist=0;  

  Serial.print("X: "); Serial.print(GyX);
  Serial.print("\tY: "); Serial.print(GyY);
  Serial.print("\tZ: "); Serial.println(GyZ);

  p.publish( &test );
  nh.spinOnce();
  delay(100);
}


void TachRead ()
{
    if (digitalRead(tachPinA)==digitalRead(tachPinB))
      dist++;
    else
      dist--;
}

void setServo(float _angleIn)
{
  _angleIn = -1*constrain(_angleIn, -1*maxSteerAngle, maxSteerAngle) + servoTrim + 90;
  steeringServo.write(_angleIn);
}

