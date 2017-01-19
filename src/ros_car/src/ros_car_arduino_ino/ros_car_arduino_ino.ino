////////////////////////////////////////////////////////////////////////////////
// PREPROCESSOR MACRO DEFINITIONS

#define USE_SW_SERVO

#define USE_STEER_SERVO_PID // Uncomment to use custom steering PID control

//#define USE_FTDI

//#define USE_SMALL_CAR

#define ROS_CONNECT

//#define DEBUG_MSGS

//#define PUBLISH_FULL_ODOM_MSG
//#define PUBLISH_IMU_MSGw

/////////////////////////////////////////////////////////////////////////////////

#define PI 3.14159265359
#define RAD_2_DEG(X) (X*180/PI)
#define DEG_2_RAD(X) (X*PI/180)
#define MILLIS_2_SEC(X) (float(X)/1000)
#define MICROS_2_SEC(X) (double(X)/1000000)

#define SEC_2_MILLIS(X) int(X*1000)
#define SEC_2_MICROS(X) int(X*1000000)

/////////////////////////////////////////////////////////////////////////////////
// INCLUDES

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <Wire.h>

#ifdef USE_SW_SERVO 
  #include <SoftwareServo.h>
#else
  #include <Servo.h>
#endif

/////////////////////////////////////////////////////////////////////////////////
// MCU PIN CONFIGURATION

const int tach_pin_A = 18;
const int tach_pin_B = 19;

const int steer_servo_out_pin = 11;
const int speed_servo_out_pin = 10;

const int steer_servo_in_pin = 2; // Steering angle poteniometer input ADC

const int MPU = 0x68; // I2C address of the MPU-60D0

/////////////////////////////////////////////////////////////////////////////////
// DEFINE PARAMETERS AND TUNING

const int ros_baud_rate = 57600;
const int debug_baud_rate = 57600;

const unsigned long loop_period = 40; // [ms]
const unsigned long imu_update_period = 100; // [ms]
const unsigned long publish_period = 100; // [ms]

unsigned long setup_finish_time;

const float wheel_base = 0.74295; // [m] Axle to axle distance
const float min_turn_radius = 0.1; // [m]

const float min_steer_angle = -23, max_steer_angle = 23;
const float min_steer_speed = -45, max_steer_speed = 45;

const float min_base_speed = 0.0, max_base_speed = 1.0;
const float min_base_accel = -3.0, max_base_accel = 3.0;

const int min_drive_power = -40, max_drive_power = 40; // [-100 .. 100]

const int steer_servo_trim = 0;  //positive trim goes left
const int speed_servo_trim = 0; 

const float drive_deadband = 0.05; // [m/s] Command velocity below which inputs are zeroed

const int steer_servo_count_min = 0;
const int steer_servo_count_max = 255;
const float steer_servo_angle_at_count_min = -60;
const float steer_servo_angle_at_count_max = 60;

float steer_servo_angle_per_count;
float steer_servo_angle_offset;

#ifndef USE_SMALL_CAR
  const float tach_dist_per_count = 0.000557231;
#else
  const float tach_dist_per_count = 0.00995115;
#endif

/////////////////////////////////////////////////////////////////////////////////
// DEFINE CONTROLLER GAINS

#ifndef USE_SMALL_CAR

  //************** Power wheels car speed tuning **********************
  const float drive_kff = 4.5; // feedforward
  const float drive_kp = 1.0;
  const float drive_ki = 0.1;
  const float drive_forward_bias = 2.5; //Power required to start motor going forward
  const float drive_backward_bias = 0; //Power required to start motor going in reverse
  const int drive_control_offset = 0;
  
  const float steering_calibration = -1.5;
  
  const float steer_kff = 0.0; // Feedforward 4.5
  const float steer_kp = 5/0; // Proportional gain
  const float steer_kd = 0.5; // Derivative gain
  const float steer_ki = 0.1; // Integral gain

#else

  //****************** RC car speed tuning **************************
  const float drive_kff = 4.0; // feedforward
  const float drive_kp = 10.0;
  const float drive_ki = 1.0;
  const float drive_forward_bias = 9; //Power required to start motor going forward
  const float drive_backward_bias = 9; //Power required to start motor going in reverse
  const int drive_control_offset = 0;
  
#endif

/////////////////////////////////////////////////////////////////////////////////
// INITIALIZE GLOBAL VARIABLES

unsigned long last_loop_time = 0; // [ms]
unsigned long last_imu_update_time = 0; // [ms]
unsigned long last_publish_time = 0; // [ms]

unsigned long last_tach_read_time = 0; // [us]
unsigned long last_steer_servo_read_time = 0; // [us]
unsigned long last_cmd_vel_time = 0; // [us]
unsigned long last_odom_time = 0; // [us]
unsigned long last_drive_control_time = 0; // [us]
unsigned long last_steer_control_time = 0; // [us]

int tach_count = 0;
int steer_servo_count = 0;

float base_speed_cmd = 0.0;
float base_omega_cmd = 0.0;
float steer_angle_cmd = 0.0;
float turn_radius_cmd = 0.0;

float last_base_speed_cmd = 0.0;
float last_steer_angle_cmd = 0.0;

// State estimates from sensor measurements

float steer_angle_est = 0.0;
float steer_angle_speed_est = 0.0;

float base_speed_est = 0.0;
float base_omega_est = 0.0;
float base_yaw_est = 0.0;

float odom_x = 0.0;
float odom_y = 0.0;

//float ros_odom_dist = 0.0;
//float ros_odom_yaw = 0.0;

float drive_int_err = 0.0; // Drive controller integrator error
float steer_angle_int_err = 0.0; // Steer controller integrator error

float last_drive_err = 0.0;
float last_steer_angle_err = 0.0;

// IMU PARAMETERS

const int calCycles = 1000;

int32_t AcX_raw,AcY_raw,AcZ_raw,Tmp_raw,GyX_raw,GyY_raw,GyZ_raw;
int32_t GyXcal = 0;
int32_t GyYcal = 0;
int32_t GyZcal = 0;
int32_t AcXcal = 0;
int32_t AcYcal = 0;
int32_t AcZcal = 0;

const float Gy_scale = 1.0/131;
const float Ac_scale = 1.0/16384;

const float GyX_scale = Gy_scale;
const float GyY_scale = Gy_scale;
const float GyZ_scale = Gy_scale;
const float AcX_scale = Ac_scale;
const float AcY_scale = Ac_scale;
const float AcZ_scale = Ac_scale;

float AcX,AcY,AcZ;
float GyX,GyY,GyZ;

int32_t AcXAccm, AcYAccm, AcZAccm, GyXAccm, GyYAccm, GyZAccm;

unsigned long sampleStartTime;
unsigned int numSamples;

#ifdef USE_SW_SERVO

  SoftwareServo steer_servo;
  SoftwareServo speed_servo;
  
#else

  Servo steer_servo;
  Servo speed_servo;
  
#endif

/////////////////////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPE DECLARATIONS

// Note: Variable names use lowercase_underscore convention, Functions use camelCase

void  readSteerAngle( void );
void  setSteerAngle( float );
void  setSteerPower ( float );
void  tachRead( void );
void  setDrivePower( float );
float rateLimit( float, float, float, float, float );
void  updateOdometry( void );
//void  cmdVelCb( const geometry_msgs::Twist& );
void  cmdCarCb( const ackermann_msgs::AckermannDrive& );
void  driveControlUpdate( void );
void  steerControlUpdate( void );
void  publishRosMessages( void );
void  updateIMU( void );
void  calibrateIMU( void );
float fsign( float );

/////////////////////////////////////////////////////////////////////////////////

#ifdef ROS_CONNECT

  ros::NodeHandle  nh;
  
  geometry_msgs::TransformStamped odom_tf;

  tf::TransformBroadcaster odom_broadcaster;
  
  //geometry_msgs::Twist cmd_vel_msg;
  //ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCb);
  
  ackermann_msgs::AckermannDrive ackermann_msg;
  ros::Subscriber<ackermann_msgs::AckermannDrive> cmdCarSub("cmd_car", &cmdCarCb);

  std_msgs::Float32 base_speed_est_msg;
  ros::Publisher baseSpeedEstPub("base_speed_est", &base_speed_est_msg);
  std_msgs::Float32 base_omega_est_msg;
  ros::Publisher baseOmegaEstPub("base_omega_est", &base_omega_est_msg);
  
  std_msgs::Float32 odom_x_msg;
  ros::Publisher odomXPub("odom_x", &odom_x_msg);
  //std_msgs::Float32 odom_y_msg;
  //ros::Publisher odomYPub("odom_y", &odom_y_msg);
  std_msgs::Float32 odom_yaw_msg;
  ros::Publisher odomYawPub("odom_yaw", &odom_yaw_msg);
    
  #if PUBLISH_IMU_MSG
  
    std_msgs:: Int16MultiArray imu;
    ros::Publisher imuPub("imuDat", &imu); 
    
  #endif
      
#endif

/////////////////////////////////////////////////////////////////////////////////
// Function Definitions

void readSteerAngle( void )
{
  
    steer_servo_count = analogRead(steer_servo_in_pin);
    
    float _steer_angle_est = steer_servo_angle_per_count*float(steer_servo_count) + steer_servo_angle_offset;
    
    steer_angle_speed_est = (_steer_angle_est - steer_angle_est)/MICROS_2_SEC(micros() - last_steer_servo_read_time);
    
    steer_angle_est = _steer_angle_est;
    
    last_steer_servo_read_time = micros(); 
                  
}


void tachRead( void )
{
  
    last_tach_read_time = micros(); // Update tach_time on each tach pulse event
    
    if (digitalRead(tach_pin_A) == digitalRead(tach_pin_B))
      tach_count--;
    else
      tach_count++; 
              
}

/////////////////////////////////////////////////////////////////////////////////


void cmdCarCb(const ackermann_msgs::AckermannDrive& _ackermann_msg) {

  base_speed_cmd = _ackermann_msg.speed;
  steer_angle_cmd = _ackermann_msg.steering_angle; // Published in degrees
  
  //double cmd_vel_dt = MICROS_2_SEC(micros() - last_cmd_vel_time);
  
  // Enforce rate limits
  //base_speed_cmd = rateLimit( last_base_speed_cmd, base_speed_cmd, cmd_vel_dt, min_base_accel, max_base_accel);
  //steer_angle_cmd = rateLimit( last_steer_angle_cmd, steer_angle_cmd, cmd_vel_dt, min_steer_speed, max_steer_speed);
  
  // Enforce absolute limits
  //base_speed_cmd = constrain(base_speed_cmd,min_base_speed,max_base_speed); 
  //steer_angle_cmd = constrain(steer_angle_cmd,min_steer_angle,max_steer_angle);
    
  // last_base_speed_cmd = base_speed_cmd;
  // last_steer_angle_cmd = steer_angle_cmd;
  // last_cmd_vel_time = micros();
  
  //nh.loginfo("cmd_vel msg received");
 
}

/////////////////////////////////////////////////////////////////////////////////


void updateOdometry( void ) {
  
    unsigned long _last_tach_read_time;
    unsigned long  _last_odom_time;
    int  _tach_count;
  
    noInterrupts(); // Stop interrupts to prevent race condition during tach update
    
      _last_tach_read_time = last_tach_read_time;
      _last_odom_time = last_odom_time;
      _tach_count = tach_count;
      tach_count = 0;
    
    interrupts(); // restart interrupts
  
    double _odom_dt = MICROS_2_SEC(_last_tach_read_time - _last_odom_time);
    
    double _odom_dist = _tach_count * tach_dist_per_count; // distance traveled since last odometry update

    base_yaw_est += base_omega_est * MICROS_2_SEC(micros() - last_odom_time);
            
    last_odom_time = micros();
    
    ////////////////////////////////////////////////////////////////////////////
      
    base_speed_est = _odom_dist/_odom_dt; // Estimated base linear velocity
        
    base_omega_est = GyZ; // TODO Kalman filter to fuse vehicle model / odometry / IMU
    
    // These are reset each time odom message is published
//    ros_odom_dist += _odom_dist;
//    ros_odom_yaw += base_omega_est*_odom_dt;

    /*
    float _odom_vx = base_speed_est*cos(base_yaw_est);
    float _odom_vy = base_speed_est*sin(base_yaw_est);
        
    odom_x += _odom_vx*_odom_dt;
    odom_y += _odom_vy*_odom_dt;
    */ 
 
    ////////////////////////////////////////////////////////////////////////////
    
    #ifdef ROS_CONNECT
    
      ros::Time _current_ros_time = nh.now();
      
      geometry_msgs::Quaternion _odom_orientation = tf::createQuaternionFromYaw(base_yaw_est);
      //geometry_msgs::Quaternion _odom_orientation = tf::createQuaternionFromRPY(0,0,base_yaw_est);
         
      odom_tf.header.stamp = _current_ros_time;
  
      odom_tf.transform.translation.x = odom_x;
      odom_tf.transform.translation.y = odom_y;
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.rotation = _odom_orientation;
            
      //////////////////////////////////////////////////////////////////////////////
      
      #ifdef PUBLISH_FULL_ODOM_MSG
  
        odom_msg.header.stamp = _current_ros_time;
     
        // Set the position
        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = _odom_orientation;
    
        // Set the velocity
        odom_msg.twist.twist.linear.x = _odom_vx;
        odom_msg.twist.twist.linear.y = _odom_vy;
        odom_msg.twist.twist.angular.z = base_omega_est;
      
     #else
   
        base_speed_est_msg.data = base_speed_est; //base_speed_est;
        base_omega_est_msg.data = base_omega_est;
        odom_x_msg.data = _odom_dist;
        //odom_y_msg.data = odom_y;
        odom_yaw_msg.data = base_yaw_est;
             
     #endif
   
     
    publishRosMessages();
      
    #endif
    
}

/////////////////////////////////////////////////////////////////////////////////

void driveControlUpdate( void )
{  
  
    //base_speed_cmd = 1.0;
    
    // Compute drive error
    float _drive_err = base_speed_cmd - base_speed_est;
//    float _drive_err = 1 - base_speed_est;
   
    // Compute feedback control effort
    float _drive_power = drive_kff*base_speed_cmd + drive_kp*_drive_err;// + drive_ki*drive_int_err; // -100 .. 100

    if (base_speed_cmd > drive_deadband) {
      _drive_power += drive_forward_bias;
    } else if ( base_speed_cmd < -drive_deadband) {
      _drive_power -= drive_backward_bias;
    } else {
      _drive_power = 0;
      drive_int_err = 0; 
    }
    
    setDrivePower(_drive_power);
    
    drive_int_err += _drive_err*MICROS_2_SEC(micros() - last_drive_control_time);
    
    last_drive_control_time = micros();
   
}

/////////////////////////////////////////////////////////////////////////////////

void steerControlUpdate( void )
{     
  
   #ifdef USE_STEER_SERVO_PID
    
      float _steer_angle_err = steer_angle_cmd - steer_angle_est;
      
      float _steer_angle_der_err = (_steer_angle_err - last_steer_angle_err)/(micros() - last_steer_control_time);
  
      steer_angle_int_err += _steer_angle_err*MICROS_2_SEC(micros() - last_steer_control_time);
         
      float steer_servo_power = steer_kp*_steer_angle_err + steer_kd*_steer_angle_der_err + steer_ki*steer_angle_int_err + steer_kff*steer_angle_cmd; // PID + Feedfoward control law
      
      setSteerPower(steer_servo_power);
      
      last_steer_angle_err = _steer_angle_err;
      
      last_steer_control_time = micros();
          
   #else

     _steer_angle_cmd = constrain(steering_calibration*_steer_angle_cmd, min_steer_angle, max_steer_angle) + steer_servo_trim + 90;
     
     setSteerAngle(_steer_angle_cmd); 
     
   #endif
   
   // nh.loginfo("set angle");
   
}

/////////////////////////////////////////////////////////////////////////////////

void setSteerAngle(float _steer_angle_in) 
{
  
    steer_servo.write(_steer_angle_in);
      
}

/////////////////////////////////////////////////////////////////////////////////

void setSteerPower(float _power_in)
{
  
  _power_in = constrain(_power_in, min_drive_power, max_drive_power);
  
  _power_in = map(_power_in, -100, 100, 0, 180);
   
  steer_servo.write(_power_in);
    
}


void setDrivePower(float _power_in)
{
  
  _power_in = constrain(_power_in, min_drive_power, max_drive_power);
  
  _power_in = map(_power_in, -100, 100, 0, 180);
   
  speed_servo.write(_power_in + drive_control_offset);

}

/////////////////////////////////////////////////////////////////////////////////

void  publishRosMessages( void ) {
  
  #ifdef ROS_CONNECT
  
    //odom_broadcaster.sendTransform(odom_tf);
    
    #ifdef PUBLISH_FULL_ODOM_MSG
    
      odomPub.publish(&odom_msg);
      
    #else
    
      baseSpeedEstPub.publish(&base_speed_est_msg);
      baseOmegaEstPub.publish(&base_omega_est_msg);
      odomXPub.publish(&odom_x_msg);
      //odomYPub.publish(&odom_y_msg);
      odomYawPub.publish(&odom_yaw_msg);
      
      // Reset ros odom incremental values after each message is published
      //ros_odom_dist = 0.0;
      //ros_odom_yaw = 0.0;
    
    #endif
    
    #ifdef PUBLISH_IMU_MSG
    
      imuPub.publish(&odom_yaw_msg);
    
    #endif
  
    //steer_angle_cmd_msg.data = steer_angle_cmd;
    //steerAngleCmdPub.publish( &steer_angle_cmd_msg );
    
    last_publish_time = millis();
    
    //nh.loginfo("publishRosMessages");
    
  #endif
  
}

/////////////////////////////////////////////////////////////////////////////////

void calibrateIMU( void ) {
  
  #ifdef ROS_CONNECT
    nh.loginfo("Calibrating Gyros");
  #else
    Serial.println("Calibrating Gyros");
  #endif
  
  delay(3000);  // delay to allow system to settle before calibration
  
  for (int i = 0; i < calCycles; i++) 
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
   
    // All of these numbers are 2 bytes long, signed floats
    AcX_raw = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY_raw = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ_raw = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp_raw = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX_raw = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY_raw = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ_raw = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    AcXcal += AcX_raw;
    AcYcal += AcY_raw;
    AcZcal += AcZ_raw;
    GyXcal += GyX_raw;
    GyYcal += GyY_raw;
    GyZcal += GyZ_raw;
  }
  
  AcXcal /= calCycles;
  AcYcal /= calCycles;
  AcZcal /= calCycles;
  GyXcal /= calCycles;
  GyYcal /= calCycles;
  GyZcal /= calCycles;
  
  #ifdef ROS_CONNECT
    nh.loginfo("Gyros Calibrated");
  #else
    Serial.print("Gyros Calibrated");
  #endif
  
  //delay(500);
  
  setup_finish_time = millis();
  
}

/////////////////////////////////////////////////////////////////////////////////

void updateIMU( void ) {
 
  AcXAccm = 0;
  AcYAccm = 0;
  AcZAccm = 0;
  GyXAccm = 0;
  GyYAccm = 0;
  GyZAccm = 0;

  // **** Read IMU ****
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  
  AcX_raw = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY_raw = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ_raw = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp_raw = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX_raw = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY_raw = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ_raw = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
  AcX = (AcX_raw - AcXcal) * AcX_scale;
  AcY = (AcY_raw - AcYcal) * AcZ_scale;
  AcZ = (AcZ_raw - AcZcal) * AcZ_scale;
  GyX = DEG_2_RAD((GyX_raw - GyXcal) * GyX_scale);
  GyY = DEG_2_RAD((GyY_raw - GyYcal) * GyY_scale);
  GyZ = DEG_2_RAD((GyZ_raw - GyZcal) * GyZ_scale);
  
  #if (defined ROS_CONNECT && defined PUBLISH_IMU_MSG)
  
    imu.data[0] = GyX;
    imu.data[1] = GyY;
    imu.data[2] = GyZ;
    imu.data[3] = AcX;
    imu.data[4] = AcY;
    imu.data[5] = AcZ; 
  
  #endif
  
  last_imu_update_time = millis();
  
  //nh.loginfo("updateIMU");
  
}

/////////////////////////////////////////////////////////////////////////////////

void setup() {

//  Serial.begin(debug_baud_rate);
//  Serial3.begin(57600);
  
//  Serial3.println("begin setup");

 ////////////////////////////////////////////////////////////////////////////////////
 
 steer_servo_angle_per_count = (steer_servo_angle_at_count_max-steer_servo_angle_at_count_min)/(steer_servo_count_max - steer_servo_count_min);
 steer_servo_angle_offset = steer_servo_angle_at_count_min - steer_servo_angle_per_count*steer_servo_count_min;
   
  ///////////////////////////////////////////////////////////////////////////////////
  // TODO set these pins in header file
  
  pinMode(7,OUTPUT); //set them to digital inputs and outputs
  pinMode(3,INPUT);
  pinMode(2,INPUT);
  digitalWrite(7,HIGH);
  pinMode(13, OUTPUT);
  
  attachInterrupt(5, tachRead, CHANGE);
  
  steer_servo.attach(steer_servo_out_pin);
  speed_servo.attach(speed_servo_out_pin);
  
  // Set pin 52 to high (5v) to act as power for IMU breakout board
//  pinMode(52,OUTPUT);
//  digitalWrite(52,HIGH);
  
  // IMU initialization
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  /////////////////////////////////////////////////////////////////////////////////

  #ifdef ROS_CONNECT
  
    #ifdef PUBLISH_IMU_MSG
      imu.data_length = 5;
    #endif
  
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
 
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    
    // nh.subscribe(cmdVelSub);
    nh.subscribe(cmdCarSub);
    
    #ifdef PUBLISH_FULL_ODOM_MSG
    
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_link";    

      nh.advertise(odomPub);
      
    #else
    
      nh.advertise(baseSpeedEstPub);
      nh.advertise(baseOmegaEstPub);
      nh.advertise(odomXPub);
      //nh.advertise(odomYPub);
      nh.advertise(odomYawPub);
      
    #endif
    
    #ifdef PUBLISH_IMU_MSG
      nh.advertise(imuPub);
    #endif
    
    odom_broadcaster.init(nh);
    
    // nh.advertise(steerAngleCmdPub); 
    
    //nh.loginfo("ros connected");
    
  #endif
  
  #ifdef DEBUG_MSGS
    Serial.println("Starting Gyro Calibration");
  #endif
  /////////////////////////////////////////////////////////////////////////////////
 
 //  Serial3.println("Callibrating IMU");
 
  calibrateIMU();
   
  setSteerAngle(0.0);
  setDrivePower(0.0);

  driveIntegratorReset();
/*

  setDrivePower(0);
    while (millis() < 5000) {
      delay(40);
      SoftwareServo::refresh();
    }
  setDrivePower(10);
  while (millis() < 7000) {
      delay(40);
      SoftwareServo::refresh();
    }
  setDrivePower(0.0);
  
  */
  /////////////////////////////////////////////////////////////////////////////////
  // Hold servo signals at zero during setup period so that motor drivers self calibrate
  /*
  while( millis() < 3000) {
    
    #ifdef USE_SW_SERVO
      SoftwareServo::refresh();
    #endif
    
  }
  */
  
//  delay(2000);

  #ifdef DEBUG_MSGS
    Serial.println("Setup Finished");
  #endif

  nh.loginfo("setup finished");

//    Serial3.println("setup finished");

  last_loop_time = millis();
  
  // delay(startup_delay);
  
 }

/////////////////////////////////////////////////////////////////////////////////

void loop() { 

  if ( (millis() - last_imu_update_time) >= imu_update_period ) {
   updateIMU();      
  }
  
  readSteerAngle();

  updateOdometry();
  
  steerControlUpdate();
  
  driveControlUpdate();
  
  /////////////////////////////////////////////////////////////////////////////////
 
 #ifdef ROS_CONNECT
   
      nh.spinOnce();

//      if ( (millis() - last_publish_time) >= publish_period ) {
   //     publishRosMessages();
  //    }
   
  #else 
  
    // Serial.println("");
    
  #endif
  
  #ifdef USE_SW_SERVO
    SoftwareServo::refresh();
  #endif

  if (micros() > last_tach_read_time + 1000000)
    base_speed_est = 0;
  
  /////////////////////////////////////////////////////////////////////////////////
  // Wait until loop period is reached
  
  while( (millis() - last_loop_time) < loop_period)
      delay(1);

  //Serial3.println(GyZ_raw);

  last_loop_time = millis();
  
}

/////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS

float fsign(float _x) {

  if (_x >= 0)
    return 1;
  else
    return -1;
    
}

/////////////////////////////////////////////////////////////////////////////////

float rateLimit(float _x_0, float _x_1, float _dt, float _min_rate_lim, float _max_rate_lim) {
  
  float _rate = constrain((_x_1 - _x_0)/_dt,_min_rate_lim,_max_rate_lim);
  
  return ( _x_0 + _rate*_dt );
  
}

/////////////////////////////////////////////////////////////////////////////////

void driveIntegratorReset( void ) {
   drive_int_err = 0;
}

