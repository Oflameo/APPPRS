// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 


const int steer_servo_out_pin = 11;
const int steer_servo_in_pin = 2; // Steering angle poteniometer input ADC
 
Servo servo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int steer_servo_count = 0;
 
void setup() 
{ 
  servo.attach(steer_servo_out_pin);  // attaches the servo on pin 9 to the servo object 
  Serial.begin(57600);
  servo.write(95);
} 
 
 
void loop() 
{ 
  
  if (Serial.available()) { 
  
    servo.write(110);    

    
    
    delay(2000);
    
    servo.write(95);
    
    delay(500);
    
    steer_servo_count = analogRead(steer_servo_in_pin);    
    Serial.println(steer_servo_count);
    
    while (Serial.available()){
      Serial.read();
    }
    
  }
  
    
  
    
    
} 
