I am an engineer and this a project I made, one issue about this is all the header file I wrote is in function structure but for a industrial product I assume a class based approach is better in all aspects.

keeping that in mind you are an expert in Esp32 codding in C++ and C, I hired you to refactor the current code into class based structure // with oop concept while retaining everything in the code I gave, meaning just change the structure that's it.

since I don't have time I hired you.

Project Structure
/Anemometer
├── lib
│    ├── AxisControl
│    │      └── AxisControl.h
│    ├── bluetooth_app
│    │     ├── BluetoothHandler.h
│    │     └── BluetoothHandler.cpp
│    ├── BT_String
│    │     ├── BluetoothCommand.h
│    │     └── BluetoothHandler.cpp //might not be needed
│    ├── button
│    │     ├── debounce.h
│    │     └── debounce.cpp
│    ├── buzzer
│    │     ├── buzzer.h
│    │     └── buzzer.cpp
│    ├── EEPROM
│    │     ├── rom.h
│    │     └── rom.cpp
│    ├── led_ring
│    │     ├── ring.h
│    │     └── ring.cpp
│    ├── linefollow
│    │     ├── linefollow.h
│    │     └── linefollow.cpp
│    ├── motor
│    │     ├── motor.h
│    │     └── motor.cpp
│    ├── pindef
│    │     ├── pindef.h
│    │     └── pindef.cpp
│    ├── var
│    │     ├── var.h
│    │     └── var.cpp
├── src
│    ├── main.cpp

all the .h files

var.h
```
#ifndef VAR_H
#define VAR_H


extern double P;
extern double I;
extern double D;
extern int calibrationMin[4];
extern int calibrationMax[4];
extern int threshold[9];
extern int maxpwm;
extern int previousError;
extern int ref_line;
extern int stopCount;
extern int axisCount;
#endif // VAR_H
```
pindef.h
```
#ifndef PINDEF_H
#define PINDEF_H

#define VBAT 15
#define LED 17  // exchange with led
#define B_START 12
#define B_CAL 21 // exchange with led // 12
#define BUZZER 2 
#define R_RPWM 23
#define R_LPWM 22
#define L_RPWM 19
#define L_LPWM 18
#define LS_1 13
#define LS_2 34
#define LS_3 14
#define LS_4 27
#define IR_1 26
#define IR_2 25
#define IR_3 33
#define IR_4 32
#define IR_5 35

extern void pinSetup();

#endif // PINDEF_H
```
motor.h
```
#ifndef MOTOR_H
#define MOTOR_H


extern void enableMotor();
extern void disableMotor();
extern void left(int pwm);
extern void right(int pwm);
extern void brake();
#endif // MOTOR_H
```
linefollow.h
```
#ifndef LINEFOLLOW_H
#define LINEFOLLOW_H
#include <QTRSensors.h>

#define SensorCount 4


extern void sensorInit();
extern void sensorCalibrate();
extern uint16_t sensorValues[SensorCount];
extern void PID_Linefollow();
#endif // LINEFOLLOW_H
```
ring.h
```
#ifndef RING_H
#define RING_H

#define NUM_LEDS 16

extern void ledSetup();
extern void staticRed();
extern void staticGreen();
extern void staticBlue();
extern void breathingLedsBlue(unsigned long currentMillis, unsigned long &lastBreathingTime, uint8_t &brightness, int &direction);
extern void patternBlink(unsigned long currentMillis, unsigned long &lastBlinkTime);
extern void patternTheaterChase(unsigned long currentMillis, unsigned long &lastTheaterTime);
extern void patternColorWave(unsigned long currentMillis, unsigned long &lastColorWaveTime, uint8_t &startHue);
extern void rotatingComet(unsigned long currentMillis, unsigned long &lastCometTime);
extern void patternBlinkRed(unsigned long currentMillis, unsigned long &lastBlinkTime);

#endif // RING_H
```
rom.h
```
// rom.h
#ifndef ROM_H
#define ROM_H

#include <Preferences.h>
#include "var.h"

extern Preferences preferences;

void initialize_rom();
void update_rom();

#endif

```
buzzer.h
```
#ifndef BUZZER_H
#define BUZZER_H

#define SHORT_BEEP 100     // "bep"
#define MEDIUM_BEEP 300    // "beeeep"
#define LONG_BEEP 500      // "beeeeeeeeeeeeep"
#define GAP 100

extern void beepTone(int duration);
extern void buzzerStart();
extern void buzzerCalibrationStart();
extern void buzzerCalibrationClose();
extern void buzzerLineFound();
extern void buzzerTimerEnd();
extern void buzzerTimerStart();
#endif // BUZZER_H

```
debounce.h
```
#ifndef DEBOUNCE_H
#define DEBOUNCE_H

extern int isButtonPressed(int pin);
#endif // DEBOUNCE_H
```
BluetoothCommand.h
```
#ifndef BLUETOOTHCOMMAND_H
#define BLUETOOTHCOMMAND_H

//Bluetooth Commands
#define CMD_GO "GO"
#define CMD_STOP "STOP"
#define CMD_RESET "RESET"
#define CMD_RESTART "RESTART"
#define CMD_CALIBRATE "CALIBRATE"

#endif //BLUETOOTHCOMMAND_H
```
BluetoothHandler.h
```
#ifndef BLUETOOTH_HANDLER_H
#define BLUETOOTH_HANDLER_H

#include <BluetoothSerial.h>

extern BluetoothSerial SerialBT; // Declare the SerialBT object

// Function declarations
extern void initialize_bluetooth(const char* deviceName);
extern bool is_bluetooth_connected();
String receive_bluetooth_command();

#endif // BLUETOOTH_HANDLER_H

```
AxisControl.h
```
#ifndef AXISCONTROL_H
#define AXISCONTROL_H

#include <Arduino.h>
#include "motor.h"

extern int stopCount;      
extern int axisCount;      

inline bool axisCompletedCondition() {
  return (stopCount <= 0);
}

// turn90Degrees() performs a 90° turn using your motor control functions.
inline void turn90Degrees(bool clockwise) {
  enableMotor();
  if (clockwise) {
    left(100);    // Use needed PWM value
    right(-100);  // Use Negative PWM value 
  } else {
    left(-100);
    right(100);
  }
  delay(500);
  brake(); // Stop the motors after the turn
}

#endif // AXISCONTROL_H

```


all the .cpp files

var.cpp
```
#include "var.h"

double P = 0.035;
double I = 0.0;
double D = 1;
int calibrationMin[4] = {0};
int calibrationMax[4] = {0};
int threshold[9] = {0};
int maxpwm = 50;
int previousError = 0;
int ref_line = 1500;
int stopCount = 0;
int axisCount = 2;
```
pindef.cpp
```
#include "pindef.h"
#include <arduino.h>

void pinSetup(){
    pinMode(B_START,INPUT_PULLDOWN);
    pinMode(B_CAL,INPUT_PULLDOWN);
    pinMode(VBAT,INPUT);
    pinMode(BUZZER,OUTPUT);
    pinMode(L_LPWM,OUTPUT);
    pinMode(L_RPWM,OUTPUT);
    pinMode(R_LPWM,OUTPUT);
    pinMode(R_RPWM,OUTPUT);
    pinMode(5,OUTPUT);
}
```
motor.cpp
```
#include "motor.h"
#include "pindef.h"
#include <Arduino.h>
#include "BTS7960.h"

BTS7960 leftMotor(5, L_LPWM, L_RPWM);
BTS7960 rightMotor(5, R_LPWM, R_RPWM);

void enableMotor() {
    leftMotor.Enable();
    rightMotor.Enable();
}

void disableMotor() {
    leftMotor.Disable();
    rightMotor.Disable();
}

void left(int pwm) {
    if (pwm >= 0) {
        leftMotor.TurnLeft(pwm);
    } else {
        leftMotor.TurnRight(-pwm); // Convert negative PWM to positive
    }
}

void right(int pwm) {
    if (pwm >= 0) {
        rightMotor.TurnLeft(pwm);
    } else {
        rightMotor.TurnRight(-pwm); // Convert negative PWM to positive
    }
}

void brake(){
    analogWrite(L_LPWM,100);
    analogWrite(L_RPWM,100);
    analogWrite(R_LPWM,100);
    analogWrite(R_RPWM,100);
    delay(500);   
}
```
linefollow.cpp
```
#include <Arduino.h>
#include <QTRSensors.h>
#include "linefollow.h"
#include "var.h"
#include "motor.h"
#include "pindef.h"
#include "rom.h"

uint16_t sensorValues[SensorCount];
QTRSensors qtr;

void sensorInit() {
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){LS_1,LS_2,LS_3,LS_4}, SensorCount);
    qtr.calibrate(QTRReadMode::On);
    delay(100);
     for (int sensor = 0; sensor < SensorCount; sensor++) {
        qtr.calibrationOn.minimum[sensor] = calibrationMin[sensor];
        qtr.calibrationOn.maximum[sensor] = calibrationMax[sensor];
    }
}


void sensorCalibrate() {
  int avg;
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate(QTRReadMode::On);
    if(i >= 50 && i <= 200){
      right(125);
      left(125);
    } 
    else{
      right(0);
      left(0);
    }
  }
  for (int sensor = 0; sensor < SensorCount; sensor++) {
    calibrationMin[sensor] = qtr.calibrationOn.minimum[sensor];
    calibrationMax[sensor] = qtr.calibrationOn.maximum[sensor];
     threshold[sensor] = (qtr.calibrationOn.minimum[sensor] + qtr.calibrationOn.maximum[sensor])/2;
     avg=avg+threshold[sensor];
}
avg=avg/4;
for(int i=4;i<9;i++){
    threshold[i]=avg;
}
update_rom();
}







int rsp = 0;
int lsp = 0;
int position = 0;
int the = 800;
int error = 0;

void robot_control(int error){
  P = error;
  D = error - previousError;

  float PIDvalue = ((0.035 * P) +  (1 * D));
  previousError = error;

  lsp = maxpwm + PIDvalue;  // to be applied to the right motor
  rsp = maxpwm - PIDvalue;  // to be applied to the left motor

  // Constrain speeds to the range -255 to 255.
  lsp = constrain(lsp, -150, 150);
  rsp = constrain(rsp, -150, 150);

  // Drive the motors using the custom motor functions.
  right(rsp);  // right motor
  left(lsp);   // left motor

}




void PID_Linefollow(){
  position = qtr.readLineBlack(sensorValues);
  error = position - ref_line;  // compute error relative to target
  
  // If all sensor readings are above the threshold 'the', assume the line is lost.
  if(sensorValues[0] >= 800 && sensorValues[1] >= 800 &&
     sensorValues[2] >= 800 && sensorValues[3] >= 800) {
       
      // Use the previous error to decide which way to turn.
      if(previousError < 0){
        // If the previous error was negative, the line was on the right.
        // Turn right: reverse the right motor and drive the left motor forward.
        right(50);
        left(-150);
        //SerialBT.println("Line lost: Turning Right");
      }
      else{
        // Else, turn left.
        right(-150);
        left(50);
        //SerialBT.println("Line lost: Turning Left");
      }
      return;  // Skip PID control when recovering from a lost line.
  }
  else {
    // Otherwise, perform normal PID line following.
    robot_control(error);
  }
}

```
ring.cpp
```
#include <Arduino.h>
#include "FastLED.h"
#include "ring.h"
#include "pindef.h"

CRGB leds[NUM_LEDS];

// ------------------ PATTERN: BLINK 🔵 ------------------
void patternBlink(unsigned long currentMillis, unsigned long &lastBlinkTime) {
  static bool state = false;
  if (currentMillis - lastBlinkTime >= 500) { // 500 ms interval
    lastBlinkTime = currentMillis;
    state = !state;
    fill_solid(leds, NUM_LEDS, state ? CRGB::Blue : CRGB::Black);
    FastLED.show();
  }
}

void patternBlinkRed(unsigned long currentMillis, unsigned long &lastBlinkTime) {
  static bool state = false;
  if (currentMillis - lastBlinkTime >= 500) { // 500 ms interval
    lastBlinkTime = currentMillis;
    state = !state;
    fill_solid(leds, NUM_LEDS, state ? CRGB::Red : CRGB::Black);
    FastLED.show();
  }
}

// ------------------ PATTERN: THEATER CHASE 🔴 ------------------
void patternTheaterChase(unsigned long currentMillis, unsigned long &lastTheaterTime) {
  static int offset = 0;
  if (currentMillis - lastTheaterTime >= 100) { // 100 ms interval
    lastTheaterTime = currentMillis;
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ((i + offset) % 3 == 0) ? CRGB::Red : CRGB::Black;
    }
    FastLED.show();
    offset = (offset + 1) % 3;
  }
}

// ------------------ PATTERN: COLOR WAVE 🌈 ------------------
void patternColorWave(unsigned long currentMillis, unsigned long &lastColorWaveTime, uint8_t &startHue) {
  if (currentMillis - lastColorWaveTime >= 30) { // 30 ms interval
    lastColorWaveTime = currentMillis;
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CHSV(startHue + (i * 10), 255, 255);
    }
    startHue += 5; // Gradually shift the hue
    FastLED.show();
  }
}

// ------------------ PATTERN: ROTATING COMET ☄️ ------------------
void rotatingComet(unsigned long currentMillis, unsigned long &lastCometTime) {
  static int cometPos = 0;
  if (currentMillis - lastCometTime >= 50) { // 50 ms interval
    lastCometTime = currentMillis;
    fadeToBlackBy(leds, NUM_LEDS, 40);

    int tailLength = 4;
    for (int i = 0; i < tailLength; i++) {
      int pos = (cometPos - i + NUM_LEDS) % NUM_LEDS;
      leds[pos] = CHSV(100, 255, 255 - (i * 60));
    }

    cometPos = (cometPos + 1) % NUM_LEDS;
    FastLED.show();
  }
}

// ------------------ PATTERN: BREATHING LEDS 💡 ------------------
void breathingLedsBlue(unsigned long currentMillis, unsigned long &lastBreathingTime, uint8_t &brightness, int &direction) {
  if (currentMillis - lastBreathingTime >= 30) { // 30 ms interval
    lastBreathingTime = currentMillis;
    brightness += direction;
    if (brightness > 250 || brightness < 5) direction = -direction;

    fill_solid(leds, NUM_LEDS, CHSV(180, 255, brightness));
    FastLED.show();
  }
}

void patternColorWavedelay() {
  static uint8_t startHue = 0;
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(startHue + (i * 10), 255, 255);
  }
  startHue += 5; // Gradually shift the hue
  FastLED.show();
  delay(30);
}

void ledSetup() {
  FastLED.addLeds<WS2811, LED, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();
  FastLED.setBrightness(255);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  for(int i=0;i<100;i++) patternColorWavedelay();
}


void staticRed() {
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
}

// 🟢 Green
void staticGreen() {
  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
}

// 🔵 Blue
void staticBlue() {
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
}
```
rom.cpp
```
#include "rom.h"
#include "var.h"
Preferences preferences;
void initialize_rom() {
    preferences.begin("rom", false);
    calibrationMin[0] = preferences.getInt("l3n", 10);
    calibrationMin[1] = preferences.getInt("l2n", 10);
    calibrationMin[2] = preferences.getInt("l1n", 10);
    calibrationMin[3] = preferences.getInt("cn", 10);
    calibrationMax[0] = preferences.getInt("l3m", 10);
    calibrationMax[1] = preferences.getInt("l2m", 10);
    calibrationMax[2] = preferences.getInt("l1m", 10);
    calibrationMax[3] = preferences.getInt("cm", 10);
    preferences.end();
     for (uint8_t i = 0; i < 7; i++)
  {
    threshold[i] = (calibrationMin[i] + calibrationMax[i])/2;
  } 
}

void update_rom() {
      preferences.putInt("l3n", calibrationMin[0]);
         preferences.putInt("l2n", calibrationMin[1]);
         preferences.putInt("l1n", calibrationMin[2]);
         preferences.putInt("cn", calibrationMin[3]);
         preferences.putInt("l3m", calibrationMax[0]);
         preferences.putInt("l2m", calibrationMax[1]);
         preferences.putInt("l1m", calibrationMax[2]);
         preferences.putInt("cm", calibrationMax[3]);
    preferences.end();
}

```
buzzer.cpp
```
#include <Arduino.h>
#include "pindef.h"
#include "buzzer.h"

void beepTone(int duration) {
  digitalWrite(BUZZER, HIGH);
  delay(duration);
  digitalWrite(BUZZER, LOW);
}


void buzzerStart() {
  beepTone(LONG_BEEP);  // Long beep ("beeeeeeeeeeeeep")
  delay(GAP);           // Gap (you can adjust if needed)
  beepTone(SHORT_BEEP); // Short beep ("bep")
}

// buzzer 3: "calibration start bep bep beeeep"
// This plays two short beeps followed by a medium beep.
void buzzerCalibrationStart() {
  beepTone(SHORT_BEEP); // First short beep ("bep")
  delay(GAP);
  beepTone(SHORT_BEEP); // Second short beep ("bep")
  delay(GAP);
  beepTone(MEDIUM_BEEP); // Medium beep ("beeeep")
}

// buzzer 4: "calibration close beeeep bep bep"
// This plays a medium beep followed by two short beeps.
void buzzerCalibrationClose() {
  beepTone(MEDIUM_BEEP); // Medium beep ("beeeep")
  delay(GAP);
  beepTone(SHORT_BEEP);  // First short beep ("bep")
  delay(GAP);
  beepTone(SHORT_BEEP);  // Second short beep ("bep")
}

// buzzer 5: "Line found bep bep bep"
// This plays three short beeps.
void buzzerLineFound() {
  beepTone(SHORT_BEEP); // First short beep ("bep")
  delay(GAP);
  beepTone(SHORT_BEEP); // Second short beep ("bep")
  delay(GAP);
  beepTone(SHORT_BEEP); // Third short beep ("bep")
}

// buzzer 6: "timer end bep beeeep bep"
// This plays a short beep, a medium beep, then a short beep.
void buzzerTimerEnd() {
  beepTone(SHORT_BEEP);  // First short beep ("bep")
  delay(GAP);
  beepTone(MEDIUM_BEEP); // Medium beep ("beeeep")
  delay(GAP);
  beepTone(SHORT_BEEP);  // Second short beep ("bep")
}

void buzzerTimerStart() {
  beepTone(MEDIUM_BEEP); // Medium beep ("beeeep")
  delay(GAP);
  beepTone(SHORT_BEEP);  // First short beep ("bep")
  delay(GAP);
  beepTone(SHORT_BEEP);  // Second short beep ("bep")
}
```
debounce.cpp
```
#include "debounce.h"
#include <arduino.h>
#include "buzzer.h"

int isButtonPressed(int pin) {
  if (digitalRead(pin) == HIGH) {
    while (digitalRead(pin) == HIGH) {
      delay(25);
    }
    beepTone(100);
    return 0;
  } else return 1;
  
}
```
BluetoothCommand.cpp
```
// not added yet add if necessary
```
BluetoothHandler.cpp
```
#include "BluetoothHandler.h"
#include <Arduino.h>

// Create a BluetoothSerial instance

bool wasConnected = false;  // Track connection status

void initialize_bluetooth(const char* deviceName) {
    SerialBT.begin(deviceName);
    Serial.println("Bluetooth Started. Waiting for connection...");
}

bool is_bluetooth_connected() {
    return SerialBT.hasClient();
}

String receive_bluetooth_command() {
    if (SerialBT.available()) {
        String receivedCommand = SerialBT.readStringUntil('\n');
        receivedCommand.trim();
        Serial.print("Received Command: ");
        Serial.println(receivedCommand);
        return receivedCommand;
    }
    return "";
}

```
AxisControl.cpp
```
// not added yet add if necessary
```

This is the main.cpp file
```
#include <Arduino.h>
#include "pindef.h"    
#include "debounce.h"  
#include "motor.h"
#include "linefollow.h"
#include "var.h"
#include "rom.h"     
#include "buzzer.h"
#include "FastLED.h"
#include "ring.h"
#include "BluetoothSerial.h"
#include "BluetoothHandler.h"
#include "BluetoothCommand.h"
#include "AxisControl.h"

BluetoothSerial SerialBT;

// For handling LED patterns and timings
unsigned long patternChangeTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastTheaterTime = 0;
unsigned long lastColorWaveTime = 0;
unsigned long lastCometTime = 0;
unsigned long lastBreathingTime = 0;
uint8_t startHue = 0;
uint8_t brightness = 0;
int direction = 5;
int currentPattern = 0;

// Define states for the state machine; note the new WAIT_FOR_START state.
enum SystemState {
  CALIBRATION,
  WAIT_FOR_START,   // New: Wait for the user to press Start (with calibration button released)
  LINE_FOLLOWING,
  AT_STOP_WAIT,
  USER_CONFIRMATION,
  CHANGE_AXIS,
  X_AXIS,
  NEGATIVE_X_AXIS,
  Y_AXIS,
  NEGATIVE_Y_AXIS,
};

SystemState currentState = CALIBRATION;
SystemState previousAxis = X_AXIS; // For changing axis


// For handling the wait timer in AT_STOP_WAIT
unsigned long waitStartTime = 0;
const unsigned long waitDuration = 120000; // 2 minutes in milliseconds
unsigned long currentMillis = 0;
unsigned long skipTime = 1750;

void setup() {
 SerialBT.begin("Anemometer");
  initialize_rom();
  sensorInit();
  pinSetup();
  buzzerStart();
  ledSetup();
  initialize_bluetooth("Anemometer"); //TODO: preffered name for the device
}

void loop() {
  String appCommand = receive_bluetooth_command(); // Receive command from the app
  Serial.println(appCommand);
  currentMillis = millis(); // Update current time at the beginning of loop

  switch (currentState) {

    case CALIBRATION:
      SerialBT.println("CALIBRATION");
      patternBlink(currentMillis, lastBlinkTime);
      
      // When the calibration button is pressed (active-low: !isButtonPressed returns true)
      if (!isButtonPressed(B_CAL) || appCommand == CMD_CALIBRATE) {  
        buzzerCalibrationStart();
        //SerialBT.println("Calibration started");
        enableMotor();
        staticRed();
        sensorCalibrate();
        staticGreen();
        buzzerCalibrationClose();
        //SerialBT.println("Calibration finished");
        // After calibration, do not start motion immediately.
        // Transition to WAIT_FOR_START, where the user must press Start.
        currentState = WAIT_FOR_START;
        SerialBT.println("Changing state to START");
        delay(1000);
      } 
      // Alternatively, if the start button is pressed and calibration values exist,
      // skip calibration and go to WAIT_FOR_START.
      else if (!isButtonPressed(B_START) || appCommand == CMD_GO) {  
        //SerialBT.println("Skipping calibration; using saved calibration values");
        currentState = WAIT_FOR_START;
        SerialBT.println("Waiting to start");
        delay(1000);
      }
      break;

    case WAIT_FOR_START:
    //SerialBT.println("WAIT_FOR_START: Waiting for user to press Start (while calibration button is released)");
      // Show a waiting LED pattern
      patternTheaterChase(currentMillis, lastTheaterTime);
      // Only if the Start button is pressed AND the calibration button is NOT pressed,
      // then enable the motors and move to LINE_FOLLOWING.
      if (!isButtonPressed(B_START)|| appCommand == CMD_GO) {
        //SerialBT.println("Start button pressed. Beginning movement.");
        enableMotor();
        SerialBT.println("Changing state to LINEFOLLOW");
        currentState = LINE_FOLLOWING;
        waitStartTime = currentMillis ;
        staticGreen();
        delay(1000);
      }
      break;

    case LINE_FOLLOWING:
    //SerialBT.println("LINE_FOLLOWING");
      patternBlinkRed(currentMillis, lastBlinkTime);
      PID_Linefollow();
      
      // Allow user to request a recalibration at any time by pressing the calibration button.
      // (Again, checking with active-low logic.)
      if (!isButtonPressed(B_CAL) || appCommand == CMD_CALIBRATE) {
        SerialBT.println("calibration");
         currentState = CALIBRATION;
         left(0);
         right(0);
         //SerialBT.println("User requested recalibration. Switching to CALIBRATION.");
         delay(1000);
      }
      
      // Check if any line sensor detects the line (using the threshold value)
      if ((analogRead(IR_1) < threshold[5] || analogRead(IR_2) < threshold[5] ||
          analogRead(IR_3) < threshold[5] || analogRead(IR_4) < threshold[5] || analogRead(IR_5) < threshold[5]) && (currentMillis - waitStartTime >= skipTime)) {
        brake();
        disableMotor();
        stopCount++;
        SerialBT.println(stopCount);
        currentState = AT_STOP_WAIT;
        buzzerLineFound();
        buzzerTimerStart();
        //SerialBT.println("Line detected. Switching to AT_STOP_WAIT.");
        staticBlue();
        disableMotor();
        // Start the wait timer
        waitStartTime = currentMillis;
        delay(2000);
      }
      break;

    case AT_STOP_WAIT:
      // During the wait period, keep the motors disabled and run an LED pattern.
      disableMotor();
      //SerialBT.println("AT_STOP_WAIT");
      if (appCommand == CMD_STOP){
        disableMotor();
        currentState = WAIT_FOR_START;
      }
      

      rotatingComet(currentMillis, lastCometTime);
      if (!isButtonPressed(B_CAL) || appCommand == CMD_CALIBRATE) {
        currentState = USER_CONFIRMATION;
        if(stopCount == axisCount){
          SerialBT.println("EOL going to origin");
          currentState = CHANGE_AXIS;
          waitStartTime = currentMillis;
          enableMotor();
          stopCount = 0;
          break;
        }
      }

      // Non-blocking wait: once the wait duration has elapsed, transition to USER_CONFIRMATION.
      if (currentMillis - waitStartTime >= waitDuration) {
        currentState = USER_CONFIRMATION;
        waitStartTime = 0;  // Reset the timer
        buzzerTimerEnd();
        staticGreen();
        //SerialBT.println("2-minute wait complete. Switching to USER_CONFIRMATION.");
        delay(1000);
      }
      if(stopCount == axisCount){
        SerialBT.println("EOL going to origin");
        currentState = CHANGE_AXIS;
        waitStartTime = currentMillis;
        enableMotor();
        stopCount = 0;
        break;
      }
      break;

    case USER_CONFIRMATION:
    //SerialBT.println("USER_CONFIRMATION: Waiting for user confirmation to resume operation");
      // Provide a LED pattern to show the system is awaiting confirmation.
      patternTheaterChase(currentMillis, lastTheaterTime);
      // Only if the Start button is pressed AND the calibration button is not pressed, then resume.
      if (!isButtonPressed(B_START) && isButtonPressed(B_CAL) || appCommand == CMD_GO) {
        //SerialBT.println("User confirmed. Resuming operation.");
        enableMotor();
        currentState = LINE_FOLLOWING;
        waitStartTime = currentMillis;
        staticGreen();
        delay(1000);
      }
      break;

      case CHANGE_AXIS:
      patternBlinkRed(currentMillis, lastBlinkTime); // change it to binking purple
      maxpwm = 100;
      PID_Linefollow();

      if ((analogRead(IR_1) < threshold[5] || analogRead(IR_2) < threshold[5] ||
          analogRead(IR_3) < threshold[5] || analogRead(IR_4) < threshold[5] || analogRead(IR_5) < threshold[5]) && (currentMillis - waitStartTime >= 2500)) {
        brake();
        disableMotor();
        maxpwm = 50;
        if (previousAxis == X_AXIS) {
          currentState = NEGATIVE_X_AXIS;
          previousAxis = NEGATIVE_X_AXIS;
        } else if (previousAxis == NEGATIVE_X_AXIS) {
          currentState = Y_AXIS;
          previousAxis = Y_AXIS;
        } else if (previousAxis == Y_AXIS) {
          currentState = NEGATIVE_Y_AXIS;
          previousAxis = NEGATIVE_Y_AXIS;
        } else { 
          currentState = WAIT_FOR_START; // Fallback
        }
        SerialBT.println("Changing axis based on previous axis");
        buzzerLineFound();
        staticBlue();
        delay(2000);
      }
      break;

      case X_AXIS:
      SerialBT.println("X_AXIS active");
      PID_Linefollow(); 
      // Check if the axis is complete using stop count
      if (axisCompletedCondition() && (currentMillis - waitStartTime >= 2500)) {
        brake();
        disableMotor();
        previousAxis = X_AXIS;
        currentState = CHANGE_AXIS;  // Move to axis-change state or next axis state
        waitStartTime = currentMillis;
        delay(2000);
      }
      break;
  
    case NEGATIVE_X_AXIS:
      SerialBT.println("NEGATIVE_X_AXIS active");
      PID_Linefollow();
      if (axisCompletedCondition() && (currentMillis - waitStartTime >= 2500)) {
        brake();
        disableMotor();
        previousAxis = NEGATIVE_X_AXIS;
        // Perform a 90° turn
        turn90Degrees(true);
        currentState = CHANGE_AXIS;
        waitStartTime = currentMillis;
        delay(2000);
      }
      break;
  
    case Y_AXIS:
      SerialBT.println("Y_AXIS active");
      PID_Linefollow();
      if (axisCompletedCondition() && (currentMillis - waitStartTime >= 2500)) {
        brake();
        disableMotor();
        previousAxis = Y_AXIS;
        currentState = CHANGE_AXIS;
        waitStartTime = currentMillis;
        delay(2000);
      }
      break;
  
    case NEGATIVE_Y_AXIS:
      SerialBT.println("NEGATIVE_Y_AXIS active");
      PID_Linefollow();
      if (axisCompletedCondition() && (currentMillis - waitStartTime >= 2500)) {
        brake();
        disableMotor();
        previousAxis = NEGATIVE_Y_AXIS;
        // Perform a 90° turn
        turn90Degrees(true);
        currentState = CHANGE_AXIS; //TODO: Or if cycle complete, return to WAIT_FOR_START/IDLE
        waitStartTime = currentMillis;
        delay(2000);
      }
      break;
      
      break;

    default:
      // Fallback: if for some reason an undefined state is reached, go to a safe state.
      currentState = AT_STOP_WAIT;
      break;
  }
}

```

also from the main.cpp if we could make the cases also as class based header file to improve modularity and readability it would help me a lot.

please do this in one go.

no changes for the logic and everything I implemented.






new structure
/Anemometer
├── lib
│    ├── AxisControl
│    │      ├── AxisControl.h
│    │      └── AxisControl.cpp
│    ├── BluetoothApp
│    │      ├── BluetoothHandler.h
│    │      └── BluetoothHandler.cpp
│    ├── BT_String
│    │      └── BluetoothCommand.h
│    ├── Button
│    │      ├── Debounce.h
│    │      └── Debounce.cpp
│    ├── Buzzer
│    │      ├── Buzzer.h
│    │      └── Buzzer.cpp
│    ├── EEPROM
│    │      ├── ROM.h
│    │      └── ROM.cpp
│    ├── LEDRing
│    │      ├── LEDRing.h
│    │      └── LEDRing.cpp
│    ├── LineFollow
│    │      ├── LineFollow.h
│    │      └── LineFollow.cpp
│    ├── Motor
│    │      ├── Motor.h
│    │      └── Motor.cpp
│    ├── PinDef
│    │      ├── PinDef.h
│    │      └── PinDef.cpp
│    ├── SystemVars
│    │      ├── SystemVars.h
│    │      └── SystemVars.cpp
│    └── StateMachine
│           ├── RobotController.h
│           └── RobotController.cpp
└── src
     └── main.cpp
#   C l a s s - b a s e d - A n e m o m e t e r - A r c h i t e c t u r e  
 