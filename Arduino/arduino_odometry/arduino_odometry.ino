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
//#include <std_msgs/Int32.h>


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
//float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

ros::NodeHandle  nh;

//std_msgs::Float32MultiArray test;

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


void servo_cb( const std_msgs::Int16& cmd_msg){
setServo(cmd_msg.data);
}

ros::Subscriber<std_msgs::Int16> sub("servo", servo_cb);


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


  //Move steering servo to confirm operational
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
 
 //AcX=0;
 //AcY=1;
 //AcZ=2;
 //Tmp=3;
 //GyX=4;
 //GyY=5;
 //GyZ=6;
  
  // All of these numbers are 2 bytes long, signed floats
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //dist=0;
  //int tacA, tacB;
  /*
  for(int x=0; x<200; x++)
  {
    tacA=digitalRead(3);
    tacB=digitalRead(2);
    if (tacState != tacB)
    //if ((~(~(tacState))) = ~((int16_t)digitalRead(2)));
    {
      if (tacA==tacB)
      {
        dist--; //going forwards
      }
      if(tacA!=tacB)
      {
        dist++;//going backwards
      } 
    }
    //if ((tacState < 3)&& ((int)digitalRead(2)>3));
     // {
      //  dist++;      
      //}
    tacState=tacB; 
    
  }*/
//totalDist=totalDist+dist;
  
test.data[1]=GyX;
test.data[2]=GyY;
test.data[3]=GyZ;
  
test.data[4]=dist;
totalDist=totalDist+dist;
dist=0;  
  //Serial.print("\n Total Dist = "); Serial.print(totalDist);
  //Serial.print(" | AcY = "); Serial.print(test.data[1]);
  //Serial.print(" | AcZ = "); Serial.print(test.data[2]);
  //Serial.print(" | Tmp = "); Serial.print(test.data[3]);  //equation for temperature in degrees C from datasheet
  //Serial.print(" | GyX = "); Serial.print( test.data[4]);
  //Serial.print(" | GyY = "); Serial.print(test.data[5]);
  //Serial.print(" | GyZ = "); Serial.println(test.data[6]);

 steeringDemand = 0;  // Get steering angle demand from ROS
  
  setServo(steeringDemand);
  
  
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

void setServo(int _angleIn)
{
  _angleIn = -1*constrain(_angleIn, -1*maxSteerAngle, maxSteerAngle) + servoTrim + 90;
  steeringServo.write(_angleIn);
}

