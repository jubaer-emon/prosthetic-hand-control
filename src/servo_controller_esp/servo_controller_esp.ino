#include <BluetoothSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <arduino-timer.h>
#include <Statistic.h>

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
// Make sure to connect SDA to A4 and SCL to A5

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

int firstFinger = 0;
int thumbFinger = 0;
int indexFinger = 1;
int middleFinger = 2;
int ringFinger = 3;
int pinkyFinger = 4;
int lastFinger = 4;
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
      setServosPos(firstFinger, lastFinger, 0);
      break;
    case HAND_CLOSE:
      setServosPos(firstFinger, lastFinger, 180);
      break;
    case THUMB_POSITIVE:
      setServoPos(thumbLocker, 180);
      break;
    case THUMB_NEUTRAL:
      setServoPos(thumbLocker, 135);
      break;
    case THUMB_NEGATIVE:
      setServoPos(thumbLocker, 100);
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
    case CALIBRATE:
      calibrateThreshold();
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

// emg
float emg1=0,emg2=0;
int sample_interval = 10; // 10ms -> 100Hz

bool getEmgData(void *) {
  emg1 = analogRead(13); //left
  emg2 = analogRead(2); //right
  return true;
}

bool printEmgData(void *) {
  if (Serial.availableForWrite()) {
    Serial.print(emg1);
//    Serial.print((double)emg1/4095, 5);
    Serial.print('\t');
    Serial.println(emg2);
//    Serial.println((double)emg2/4095, 5);
  }
  return true;
}

statistic::Statistic<float, uint32_t, true> calEmg1,calEmg2;
float emg1Average=2000, emg1Variance=10, emg2Average=2000, emg2Variance=10;

void calibrateThreshold() {
//  for (auto ch : "Assume positon 1") SerialBT.write(ch);
  Serial.println("Assume positon 1");
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
  Serial.println("Recording");
  digitalWrite(LED_BUILTIN, HIGH);
  for (int i=0; i<500; i++) {
    getEmgData(NULL);
    calEmg1.add(emg1);
    calEmg2.add(emg2);
    delay(sample_interval);
  }
  float emg1Average1 = calEmg1.average();
  float emg1Variance1 = calEmg1.unbiased_stdev();
  calEmg1.clear();
  float emg2Average1 = calEmg2.average();
  float emg2Variance1 = calEmg2.unbiased_stdev();
  calEmg2.clear();

  Serial.println("Assume positon 2");
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
  Serial.println("Recording");
  digitalWrite(LED_BUILTIN, HIGH);
  for (int i=0; i<500; i++) {
    getEmgData(NULL);
    calEmg1.add(emg1);
    calEmg2.add(emg2);
    delay(sample_interval);
  }
  float emg1Average2 = calEmg1.average();
  float emg1Variance2 = calEmg1.unbiased_stdev();
  calEmg1.clear();
  float emg2Average2 = calEmg2.average();
  float emg2Variance2 = calEmg2.unbiased_stdev();
  calEmg2.clear();

  emg1Average = (emg1Average1+emg1Average2)/2;
  emg1Variance = (emg1Variance1+emg1Variance2)/2;

  emg2Average = (emg2Average1+emg2Average2)/2;
  emg2Variance = (emg2Variance1+emg2Variance2)/2;
  
  Serial.print(emg1);
  Serial.print('\t');
  Serial.print(emg2);
  Serial.print('\t');
  Serial.print(emg1Average);
  Serial.print('\t');
  Serial.print(emg2Average);
  Serial.print('\t');
  Serial.print(emg1Variance);
  Serial.print('\t');
  Serial.print(emg2Variance);
  Serial.print('\t');
  Serial.println();
}

void controlWithEmg() {
  bool emg1State = emg1<emg1Average, emg2State = emg2<emg2Average;
  static bool prevEmg1State = !emg1State, prevEmg2State = !emg2State;

  if (emg1State ^ prevEmg1State) {
    if (emg1State) {
      doCommand(HAND_OPEN);
    }
    else {
      doCommand(HAND_CLOSE);
    }
    prevEmg1State = emg1State;
  }
  if (emg2State ^ prevEmg2State) {
    if (emg2State) {
      doCommand(INDEX);
    }
    else {
      doCommand(INDEX);
    }
    prevEmg2State = emg2State;
  }
}

statistic::Statistic<float, uint32_t, true> emg1Data,emg2Data;
float emgThreshold=100;

bool emgControl(void *) {
  if (emg1Data.count() < 100) {
    emg1Data.add(emg1);
    emg2Data.add(emg2);
  }
  else {    
    bool emg1State = emg1Data.average()<emg1Average, emg2State = emg2Data.average()<emg2Average;
    static bool prevEmg1State = !emg1State, prevEmg2State = !emg2State;
  
    if (emg1State ^ prevEmg1State) {
      if (emg1State) {
        doCommand(HAND_OPEN);
      }
      else {
        doCommand(HAND_CLOSE);
      }
      prevEmg1State = emg1State;
    }
    if (emg2State ^ prevEmg2State) {
      if (emg2State) {
        doCommand(INDEX);
      }
      else {
        doCommand(INDEX);
      }
      prevEmg2State = emg2State;
    }

    Serial.print(emg1);
    Serial.print('\t');
    Serial.print(emg2);
    Serial.print('\t');
    Serial.print(emg1Data.average());
    Serial.print('\t');
    Serial.print(emg2Data.average());
    Serial.print('\t');
    Serial.print(emg1Data.unbiased_stdev());
    Serial.print('\t');
    Serial.print(emg2Data.unbiased_stdev());
    Serial.print('\t');
    Serial.println();
    
    emg1Data.clear();
    emg2Data.clear();
  }
  return true;
}

bool emgControl2(void *) {
  if (emg1Data.count() < 100) {
    emg1Data.add(emg1);
    emg2Data.add(emg2);
  }
  else {    
    float emg1Stdev = emg1Data.unbiased_stdev(), emg2Stdev = emg2Data.unbiased_stdev();
    bool emg1High = emg1Stdev>emgThreshold, emg2High = emg2Stdev>emgThreshold;
    bool emg1Higher = emg1Stdev>emg2Stdev;
  
//    if (emg1High && emg2High) {
//      if (emg1Higher) {
//        doCommand(HAND_CLOSE);
//      }
//      else {
//        doCommand(HAND_OPEN);
//      }
//    }

    if (emg1High && emg1Higher) {
        doCommand(HAND_CLOSE);
    }
    if (emg2High && !emg1Higher) {
        doCommand(HAND_OPEN);
    }

    Serial.print(emg1);
    Serial.print('\t');
    Serial.print(emg2);
    Serial.print('\t');
    Serial.print(emg1Data.average());
    Serial.print('\t');
    Serial.print(emg2Data.average());
    Serial.print('\t');
    Serial.print(emg1Data.unbiased_stdev());
    Serial.print('\t');
    Serial.print(emg2Data.unbiased_stdev());
    Serial.print('\t');
    Serial.println();
    
    emg1Data.clear();
    emg2Data.clear();
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
  timer.every(sample_interval, getEmgData);
//  timer.every(sample_interval, printEmgData);
//  timer.every(sample_interval, emgControl);
  timer.every(sample_interval, emgControl2);
  timer.every(1000, toggleLED);
  //timer.every(10000, printRuntime);
}

void loop() {
//  sendSerialToBluetooth();
  receiveBluetoothMsg();
//  controlWithEmg();
  timer.tick();
}
