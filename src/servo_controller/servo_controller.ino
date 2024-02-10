#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();
SoftwareSerial hc06(0,1);

#define HAND_OPEN 0 
#define HAND_CLOSE 1 
#define HAND_GRAB 2
#define COMMANDS_SIZE 3
String commands[] = {"open","close","grab"};

int baudRate = 9600;
String command = "";
int handState = HAND_OPEN;
int currentPos = 0;
int servoCount = 5;

boolean isEmgEnabled = false;
boolean isHc06Enabled = true;
boolean isServoDriverEnabled = false;

void setup() {
  if (isHc06Enabled) hc06.begin(baudRate);
  if (isServoDriverEnabled) servoDriver.begin();
  Serial.begin(baudRate);

  servoDriver.begin();
  /* Warning:
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  servoDriver.setOscillatorFrequency(27000000);
  servoDriver.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void loop() {
  if (isEmgEnabled && Serial.availableForWrite()) {
    int emg_0 = analogRead(A0);
    int emg_1 = analogRead(A1);
    
    Serial.println(String(emg_0)+' '+String(emg_1));
  }

  if (hc06.available()) {
    char ch = hc06.read();
    if (ch != ' ') {
      command += ch;
    }
    else {
      Serial.println(command);
      setHandState(command);
      command = "";
    }
  }

  switch (handState) {
    case HAND_OPEN:
      setServosPos(0);
      break;
    case HAND_CLOSE:
      setServosPos(100);
      break; 
    case HAND_GRAB:
      setServosPos(100);
      break;
  }
  
  delay(10);
}

void setHandState(String cmd) {
  cmd.toLowerCase();
  for (int i=0; i<COMMANDS_SIZE; i++) {
    if (cmd == commands[i]) {
      handState = i;
    }
  }
}

void setServosPos(int targetPos) {
  if (targetPos > currentPos) {
    for (int pos=currentPos; pos<targetPos; pos++) {
      servosPos(pos);
    }
  } 
  else {
    for (int pos=currentPos; pos>targetPos; pos--) {
      servosPos(pos);
    }    
  }
}

void servosPos(int percent) {
  for (int i=0; i<servoCount; i++) {

//    pulselen = map(percent, 0, 100, SERVOMIN, SERVOMAX);
//    servoDriver.setPWM(i, 0, pulselen);
    
    int microsec = map(percent, 0, 100, USMIN, USMAX);
    servoDriver.writeMicroseconds(i, microsec);
  }
}
