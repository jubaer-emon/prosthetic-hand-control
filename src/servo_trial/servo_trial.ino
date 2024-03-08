#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver hand = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(115200);
  Serial.println("8 channel Servo test!");

  hand.begin();
  hand.setPWMFreq(60);
}

void loop() {
  for (int servo=0; servo<6; servo++) { 
    hand.setPWM(servo,0,150);
  }
  delay(2000);
  for (int servo=0; servo<6; servo++) { 
    hand.setPWM(servo,0,650);
  }
  delay(2000);
}
