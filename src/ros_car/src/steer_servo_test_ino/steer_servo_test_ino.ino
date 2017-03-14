// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 


const int steer_servo_out_pin = 11;
const int steer_servo_in_pin = 2; // Steering angle poteniometer input ADC
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int steer_servo_count = 0;
 
void setup() 
{ 
  myservo.attach(steer_servo_out_pin);  // attaches the servo on pin 9 to the servo object 
  Serial.begin(57600);
} 
 
 
void loop() 
{ 
  
    myservo.write(100);    

    steer_servo_count = analogRead(steer_servo_in_pin);    
    Serial.println(steer_servo_count);
    
    delay(15);
    
} 
