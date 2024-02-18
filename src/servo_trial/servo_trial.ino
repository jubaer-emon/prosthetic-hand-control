#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver hand = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  hand.begin();
  hand.setPWMFreq(60);
}

int servo = 0;
void loop() {
  //for (int servo=0; servo<5; servo++) { 
  hand.setPWM(servo,0,150);
  //}
  delay(1000);
  //for (int servo=0; servo<5; servo++) { 
  hand.setPWM(servo,0,650);
  //}
  delay(1000);

}
