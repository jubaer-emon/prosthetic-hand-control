#include <BluetoothSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <arduino-timer.h>

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
#define COMMANDS_SIZE 13
String commands[] = {"open", "close", "positive", "neutral", "negative", "extension", "relax", "flexion", "thumb", "index", "middle", "ring", "little"};
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

  int cmd_no = 0;
  while (cmd_no < COMMANDS_SIZE) {
    if (cmd == commands[cmd_no]) {
      break;
    }
    cmd_no++;
  }

  //  if (cmd_no < COMMANDS_SIZE) {
  //    Serial.print("Executing command: ");
  //    Serial.println(cmd_no);
  //  }

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
int emg1,emg2;

bool getEmgData(void *) {
  emg1 = analogRead(35);
  emg2 = analogRead(4);
  return true;
}

bool printEmgData(void *) {
  if (Serial.availableForWrite()) {
    Serial.print((double)analogRead(35)/4095, 5);
    Serial.print((double)analogRead(35)/4095, 5);
    Serial.print('\t');
    Serial.println((double)analogRead(4)/4095, 5);
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
  timer.every(10, printEmgData); // 10ms -> 100Hz
  timer.every(1000, toggleLED);
  //timer.every(10000, printRuntime);
}

void loop() {
  sendSerialToBluetooth();
  receiveBluetoothMsg();
  timer.tick();
}
