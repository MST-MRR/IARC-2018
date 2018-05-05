#include <Servo.h>
#include <stream.h>
Servo gimbal; 

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600);
  gimbal.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  if(Serial.available() > 0){
    
    pos = Serial.parseInt();
    Serial.println(pos);
    gimbal.write(pos);  
    if(pos > 90) 
      pinMode(13, OUTPUT);
    else
      pinMode(13, INPUT);
    delay(50);   
  }
}
