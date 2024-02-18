#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  650 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2500 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();
// Make sure to connect SDA to A4 and SCL to A5

SoftwareSerial hc06(0, 1);

#define HAND_OPEN 0
#define HAND_CLOSE 1
#define THUMB_POSITIVE 2
#define THUMB_NEUTRAL 3
#define THUMB_NEGATIVE 4
#define WRIST_UP 5
#define WRIST_RELAX 6
#define WRIST_DOWN 7
#define COMMANDS_SIZE 8
String commands[] = {"open", "close", "positive", "neutral", "negative", "up", "relax", "down"};

int baudRate = 9600;
String command = "";
int currentPos = 0;
int fingerPins = 5;
int fingerStartPin = 0;
int fingerEndPin = 4;
int thumbPin = 5;
int wristPin = 6;

boolean isEmgEnabled = false;
boolean isHc06Enabled = true;

void setup() {
  Serial.begin(baudRate);
  if (isHc06Enabled) hc06.begin(baudRate);

  Serial.print("Checking servo driver I2C interface availibility: ");
  byte servoDriverAddress = 40; // default address 0x40
  Wire.beginTransmission(servoDriverAddress);
  byte error = Wire.endTransmission();
  if (error == 0) Serial.println("Available");
  else Serial.println("Unavailable. Make sure to connect SDA to A4 and SCL to A5");
  
  servoDriver.begin();
  servoDriver.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  /* Warning:
     In theory the internal oscillator (clock) is 25MHz but it really isn't
     that precise. You can 'calibrate' this by tweaking this number until
     you get the PWM update frequency you're expecting!
     The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     is used for calculating things like writeMicroseconds()
     Analog servos run at ~50 Hz updates, It is importaint to use an
     oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     1) Attach the oscilloscope to one of the PWM signal pins and ground on
        the I2C PCA9685 chip you are setting the value for.
     2) Adjust setOscillatorFrequency() until the PWM update frequency is the
        expected value (50Hz for most ESCs)
     Setting the value here is specific to each individual I2C PCA9685 chip and
     affects the calculations for the PWM update frequency.
     Failure to correctly set the int.osc value will cause unexpected PWM results
  servoDriver.setOscillatorFrequency(27000000);
  */

//  while (!Serial); // only for Leonardo: wait for Serial Monitor
  Serial.println("Program initialized. Enter commands:");
}

void loop() {  
  if (isEmgEnabled && Serial.availableForWrite()) {
    int emg_0 = analogRead(A0);
    int emg_1 = analogRead(A1);
    Serial.println(String(emg_0) + ' ' + String(emg_1));
  }

  if (Serial.available() > 0) {
    String input = Serial.readString();
    setState(input);
  }

  while (hc06.available() > 0) {
    char ch = hc06.read();
    if (ch != ' ') {
      command += ch;
    }
    else {
      Serial.println(command);
      setState(command);
      command = "";
    }
  }

  //setFingersPos(0);
  delay(10);
}

void setState(String cmd){
  cmd.toLowerCase();

  Serial.println(cmd);
  
  int cmd_no = 0;
  while (cmd_no < COMMANDS_SIZE) {
    if (cmd == commands[cmd_no]) {
      break;
    }
    cmd_no++;
  }

  Serial.println(cmd_no);

  switch (cmd_no) {
    case HAND_OPEN:
      setFingersPosGrad(0);
      break;
    case HAND_CLOSE:
      setFingersPosGrad(90);
      break;
    case THUMB_POSITIVE:
      setThumbPosGrad(50);
      break;
    case THUMB_NEUTRAL:
      setThumbPosGrad(75);
      break;
    case THUMB_NEGATIVE:
      setThumbPosGrad(100);
      break;
    case WRIST_UP:
      setWristPosGrad(35);
      break;
    case WRIST_RELAX:
      setWristPosGrad(50);
      break;
    case WRIST_DOWN:
      setWristPosGrad(65);
      break;
      
  }
}

void setFingersPosGrad(int targetPos) {
  Serial.println(targetPos);
  if (targetPos > currentPos) {
    for (int pos = currentPos; pos < targetPos; pos++) {
      setFingersPos(pos);
    }
    currentPos = targetPos;
  }
  else if (targetPos < currentPos) {
    for (int pos = currentPos; pos > targetPos; pos--) {
      setFingersPos(pos);
    }
    currentPos = targetPos;
  }
  setFingersPos(currentPos);
}

void setFingersPos(int percent) {
  for (int fingerPin = 0; fingerPin < fingerPins; fingerPin++) {
    setServoPos(fingerPin, percent);
  }
}

void setThumbPosGrad(int targetPos) {
  Serial.println(targetPos);
  if (targetPos > currentPos) {
    for (int pos = currentPos; pos < targetPos; pos++) {
      setServoPos(thumbPin, currentPos);
    }
    currentPos = targetPos;
  }
  else if (targetPos < currentPos) {
    for (int pos = currentPos; pos > targetPos; pos--) {
      setServoPos(thumbPin, currentPos);
    }
    currentPos = targetPos;
  }
  setServoPos(thumbPin, currentPos);
}

void setWristPosGrad(int targetPos) {
  Serial.println(targetPos);
  if (targetPos > currentPos) {
    for (int pos = currentPos; pos < targetPos; pos++) {
      setServoPos(wristPin, pos);
    }
    currentPos = targetPos;
  }
  else if (targetPos < currentPos) {
    for (int pos = currentPos; pos > targetPos; pos--) {
      setServoPos(wristPin, pos);
    }
    currentPos = targetPos;
  }
  setServoPos(wristPin, currentPos);
}

void setServoPos(int servo, int percent) {
  int pulselen = map(percent, 0, 100, SERVOMIN, SERVOMAX);
  servoDriver.setPWM(servo, 0, pulselen);

  //int microsec = map(percent, 0, 100, USMIN, USMAX);
  //servoDriver.writeMicroseconds(servo, microsec);
}
