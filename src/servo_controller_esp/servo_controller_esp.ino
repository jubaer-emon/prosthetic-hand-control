#include <BluetoothSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

void periodicDebugMsg() {  
  static unsigned long previousMillis = 0;
  if (millis() - previousMillis > 10000)
  {
    previousMillis = millis();
    Serial.print(previousMillis/1000);
    Serial.println("s running");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
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
  char ch;
  static String cmd = "";
  while (SerialBT.available()) {
    ch = SerialBT.read();
    if (ch!='\n') {
      cmd += ch;
    }
    else {
      Serial.print("received: ");
      Serial.println(cmd);
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

String commands[] = {"open", "close", "positive", "neutral", "negative", "extension", "relax", "flexion"};
#define HAND_OPEN 0
#define HAND_CLOSE 1
#define THUMB_POSITIVE 2
#define THUMB_NEUTRAL 3
#define THUMB_NEGATIVE 4
#define WRIST_EXTEN 5
#define WRIST_RELAX 6
#define WRIST_FLEX 7
#define COMMANDS_SIZE 8

int firstFinger = 0;
int lastFinger = 4;
int thumb = 5;
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

  if (cmd_no < COMMANDS_SIZE) {
    Serial.print("Executing command: ");
    Serial.println(cmd_no);
  }

  switch (cmd_no) {
    case HAND_OPEN:
      setServosPos(firstFinger, lastFinger, 0);
      break;
    case HAND_CLOSE:
      setServosPos(firstFinger, lastFinger, 170);
      break;
    case THUMB_POSITIVE:
      setServoPos(thumb, 180);
      break;
    case THUMB_NEUTRAL:
      setServoPos(thumb, 135);
      break;
    case THUMB_NEGATIVE:
      setServoPos(thumb, 100);
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
  }
}

void setServoPos(int servo, int ang) {
  int pulselen = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  servoDriver.setPWM(servo, 0, pulselen);
}

void setServosPos(int first, int last, int ang) {
  for (int i = first; i < last; i++) {
    setServoPos(i, ang);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // setup bluetooth
  SerialBT.begin(btName);
  Serial.println(btName + " started");

  // setup servo driver
  servoDriver.begin();
  servoDriver.setPWMFreq(SERVO_FREQ);
}

void loop() {
  sendSerialToBluetooth();
  receiveBluetoothMsg();
  periodicDebugMsg();
}
