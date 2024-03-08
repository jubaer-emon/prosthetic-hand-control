#include <BluetoothSerial.h>
#include <Adafruit_PWMServoDriver.h>
#include <arduino-timer.h>
#include <SSVAnySensor.h>

// for debug
auto timer = timer_create_default();

bool printRuntime(void *) {
  Serial.print(millis() / 1000);
  Serial.println("s running");
  return true;
}

bool toggleLED(void *) {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  return true;
}

// bluetooth
BluetoothSerial SerialBT;
String btName = "Hand-ESP32-BT-Slave";

void sendSerialToBluetooth() {
  while (Serial.available()) {
    SerialBT.write(Serial.read());
  }
}

void receiveBluetoothMsg() {
  while (SerialBT.available()) {
    char ch = SerialBT.read();
    static String cmd = "";
    if (ch != '\n') {
      cmd += ch;
    }
    else {
      //      Serial.print("received: '");
      //      Serial.print(cmd);
      //      Serial.println("'");
      setHandState(cmd);
      cmd = "";
    }
  }
}

// servo
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  650 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

#define HAND_OPEN 0
#define HAND_CLOSE 1
#define THUMB_POSITIVE 2
#define THUMB_NEUTRAL 3
#define THUMB_NEGATIVE 4
#define WRIST_EXTEN 5
#define WRIST_RELAX 6
#define WRIST_FLEX 7
#define THUMB 8
#define INDEX 9
#define MIDDLE 10
#define RING 11
#define PINKY 12
#define CALIBRATE 13
#define COMMANDS_SIZE 14
String commands[] = {"open", "close", "positive", "neutral", "negative", "extension", "relax", "flexion", "thumb", "index", "middle", "ring", "pinky","calibrate"};
int currentPos[COMMANDS_SIZE] = {0};

int thumbFinger = 7;
int indexFinger = 1;
int middleFinger = 2;
int ringFinger = 3;
int pinkyFinger = 4;
int thumbLocker = 5;
int wrist = 6;

void setHandState(String cmd) {
  cmd.toLowerCase();
  
  Serial.print("Recieved command: ");
  Serial.println(cmd);
  
  for (int cmd_no=0; cmd_no < COMMANDS_SIZE; cmd_no++) {
    if (cmd != commands[cmd_no]) continue;
    doCommand(cmd_no);
    break;
  }
}

void doCommand(int cmd_no) {
//  Serial.print("Executing command: ");
//  Serial.println(cmd_no);
  switch (cmd_no) {
    case HAND_OPEN:
      moveFingers(0);
      break;
    case HAND_CLOSE:
      moveFingers(180);
      break;
    case THUMB_POSITIVE:
      setServoPos(thumbLocker, 175);
      break;
    case THUMB_NEUTRAL:
      setServoPos(thumbLocker, 150);
      break;
    case THUMB_NEGATIVE:
      setServoPos(thumbLocker, 110);
      break;
    case WRIST_EXTEN:
      setServoPos(wrist, 45);
      break;
    case WRIST_RELAX:
      setServoPos(wrist, 90);
      break;
    case WRIST_FLEX:
      setServoPos(wrist, 135);
      break;
    case THUMB:
      setServoPos(thumbFinger, 180 - currentPos[thumbFinger]);
      break;
    case INDEX:
      setServoPos(indexFinger, 180 - currentPos[indexFinger]);
      break;
    case MIDDLE:
      setServoPos(middleFinger, 180 - currentPos[middleFinger]);
      break;
    case RING:
      setServoPos(ringFinger, 180 - currentPos[ringFinger]);
      break;
    case PINKY:
      setServoPos(pinkyFinger, 180 - currentPos[pinkyFinger]);
      break;
  }
}

void setServoPos(int servo, int ang) {
  int pulselen = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  servoDriver.setPWM(servo, 0, pulselen);

  //  Serial.print("Servo ");
  //  Serial.print(servo);
  //  Serial.print(" from ");
  //  Serial.print(currentPos[servo]);
  //  Serial.print(" to ");
  //  Serial.println(ang);
  currentPos[servo] = ang;
}

void setServosPos(int first, int last, int ang) {
  for (int i = first; i <= last; i++) {
    setServoPos(i, ang);
  }
}

void moveFingers(int ang) {
  setServoPos(thumbFinger, ang);
  setServoPos(indexFinger, ang);
  setServoPos(middleFinger, ang);
  setServoPos(ringFinger, ang);
  setServoPos(pinkyFinger, ang);
}

// emg sensor

float innerEmg, outerEmg;

float measureInnerEmg(SSVAnySensor &Self) {
  innerEmg = analogRead(13); //left
  return innerEmg;
}
float measureOuterEmg(SSVAnySensor &Self) {
  outerEmg = analogRead(2); //right
  return outerEmg;
}

float innerEmgAvg, innerEmgStdev;
float outerEmgAvg, outerEmgStdev;

bool reportInnerEmg(SSVAnySensor &Self) {
  innerEmgAvg = Self.DataAverage();
  innerEmgStdev = Self.DataUnbiasedStDev();
  Self.DataClear();
  return true;
}
bool reportOuterEmg(SSVAnySensor &Self) {
  outerEmgAvg = Self.DataAverage();
  outerEmgStdev = Self.DataUnbiasedStDev();
  Self.DataClear();
  return true;
}

SSVAnySensor InnerEmgSensor(1, measureInnerEmg, reportInnerEmg);
SSVAnySensor OuterEmgSensor(1, measureOuterEmg, reportOuterEmg);

bool printEmg(void *) {
  if (Serial.availableForWrite()) {
    Serial.print(innerEmg);
    Serial.print('\t');
    Serial.print(outerEmg);
    Serial.print('\t');
    Serial.print(innerEmgAvg);
    Serial.print('\t');
    Serial.print(outerEmgAvg);
    Serial.print('\t');
    Serial.print(innerEmgStdev);
    Serial.print('\t');
    Serial.print(outerEmgStdev);
    Serial.println();
  }
  return true;
}

float innerEmgThreshold=100, outerEmgThreshold=100;

bool emgControl(void *) {
  bool innerEmgHigh = innerEmgStdev>innerEmgThreshold;
  bool outerEmgHigh = outerEmgStdev>outerEmgThreshold;
  bool innerEmgHigher = innerEmgStdev>outerEmgStdev;

  if (innerEmgHigh && innerEmgHigher) {
      doCommand(HAND_CLOSE);
  }
  if (outerEmgHigh && !innerEmgHigher) {
      doCommand(HAND_OPEN);
  }
  return true;
}

bool readSerial(void *) {
  if (Serial.available()) {
    char ch = Serial.read();
    static String input = "";
    if (ch != '\n') {
      input += ch;
    }
    else {
      innerEmgThreshold = input.toFloat();
      outerEmgThreshold = input.toFloat();
      input = "";
    }
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // setup bluetooth
  SerialBT.begin(btName);
  //  Serial.println(btName + " started");

  // setup servo driver
  servoDriver.begin();
  servoDriver.setPWMFreq(SERVO_FREQ);

  // timed functions
//  timer.every(10000, printRuntime);
  timer.every(1000, toggleLED);

  InnerEmgSensor.setAutoReportTrigger(ART_ByInterval);
  InnerEmgSensor.setReportInterval(500);
  InnerEmgSensor.setAutoMeasTrigger(AMT_ByInterval);
  InnerEmgSensor.setMeasurementInterval(1);
  
  OuterEmgSensor.setAutoReportTrigger(ART_ByInterval);
  OuterEmgSensor.setReportInterval(500);
  OuterEmgSensor.setAutoMeasTrigger(AMT_ByInterval);
  OuterEmgSensor.setMeasurementInterval(1);
  
  timer.every(50, printEmg);
  timer.every(100, emgControl);
  timer.every(500, readSerial);
}

void loop() {
//  sendSerialToBluetooth();
  receiveBluetoothMsg();
  timer.tick();
  
  InnerEmgSensor.process();
  OuterEmgSensor.process();
}
