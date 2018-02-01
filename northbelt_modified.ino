/*
 * Based on NorthBelt project by David Singleton 
 * https://github.com/dps/northbelt
 * The code is modified for JumpNorth suit made by KOBAKANT
 * for hardware documentation please see the below post
 * http://www.kobakant.at/KOBA/jump-north-for-rachel/
 */

#include <Wire.h>
#include <LSM303.h>
#include <EEPROM.h>

/* pinouts
S ---|3 14|--- SW
SE---|2 15|--- W
E ---|1 16|--- NW
NE---|0 17|--- N
        18|--- SDA
Btn-|11 19|--- SCL
---|VCC VCC|---
 */

#define DEBUG 1

LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = { -32768, -32768, -32768};

int CALIBRATE_LOOPS = 500;
int VIBE_BASE = 0;
int VIBE_COUNT = 8;
int SMOOTHING_DEGREES = 5;
int LED_PIN = 13;

int motorPin[] = { 0, 1, 2, 3, 14, 15, 16,17};

int EEPROM_CALIBRATED_ADDR = 0;
int EEPROM_CALIBRATION_VECTOR_ADDR = 1;

float CENTER_WEDGE = (360 / VIBE_COUNT) / 2;

int currentVibePin = VIBE_BASE;

// to initiate the calibration
int calibrationButton = 11;
bool cal = 1;
bool lastcal = 1;

int vibrateCycles = 0;
float heading = 0.0;
float lastVibratedHeading = 180.0;


void dumpCalibrationData() {
  Serial.print("calibration data: ");
  Serial.print(running_min.x);
  Serial.print(" ");
  Serial.print(running_min.y);
  Serial.print(" ");
  Serial.print(running_min.z);
  Serial.print(", ");
  Serial.print(running_max.x);
  Serial.print(" ");
  Serial.print(running_max.y);
  Serial.print(" ");
  Serial.print(running_max.z);
  Serial.println();
}

void calibrate() {

  Serial.println("calibrate");
  int led = HIGH;

  for (int i = 0; i < CALIBRATE_LOOPS; i++) {
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_min.z = min(running_min.z, compass.m.z);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    running_max.z = max(running_max.z, compass.m.z);

    digitalWrite(LED_PIN, led);
    if (led == HIGH) {
      led = LOW;
    } else {
      led = HIGH;
    }

    delay(50);
  }

  if (DEBUG) {
    dumpCalibrationData();
  }

}

bool isCalibrated() {
  // Unwritten addresses have the value 255.
  // We write 128 when marking uncalibrated so we can
  // distinguish the two states.
  // We write 0 when calibration data is stored.
  int val = EEPROM.read(EEPROM_CALIBRATED_ADDR);
  if (DEBUG) {
    Serial.print("calibrated? ");
    Serial.println(val);
  }
  return val < 128;

}

int16_t readInt16FromEEPROM(int addr) {
  int16_t ret;
  byte* p = (byte*)(void*)&ret;
  for (int i = 0; i < sizeof(int16_t); i++) {
    *p++ = EEPROM.read(addr++);
  }
  return ret;
}

void writeInt16ToEEPROM(int addr, int16_t val) {
  byte* p = (byte*)(void*)&val;
  for (int i = 0; i < sizeof(int16_t); i++) {
    EEPROM.write(addr++, *p++);
  }
}

void readCalibrationData() {
  int addr = EEPROM_CALIBRATION_VECTOR_ADDR;
  running_min.x = readInt16FromEEPROM(addr);
  addr += sizeof(int16_t);
  running_min.y = readInt16FromEEPROM(addr);
  addr += sizeof(int16_t);
  running_min.z = readInt16FromEEPROM(addr);
  addr += sizeof(int16_t);

  running_max.x = readInt16FromEEPROM(addr);
  addr += sizeof(int16_t);
  running_max.y = readInt16FromEEPROM(addr);
  addr += sizeof(int16_t);
  running_max.z = readInt16FromEEPROM(addr);
  addr += sizeof(int16_t);

  if (DEBUG) {
    Serial.print("READ from EEPROM ");
    dumpCalibrationData();
  }
}

void writeCalibrationData() {
  int addr = EEPROM_CALIBRATION_VECTOR_ADDR;
  writeInt16ToEEPROM(addr, running_min.x);
  addr += sizeof(int16_t);
  writeInt16ToEEPROM(addr, running_min.y);
  addr += sizeof(int16_t);
  writeInt16ToEEPROM(addr, running_min.z);
  addr += sizeof(int16_t);
  writeInt16ToEEPROM(addr, running_max.x);
  addr += sizeof(int16_t);
  writeInt16ToEEPROM(addr, running_max.y);
  addr += sizeof(int16_t);
  writeInt16ToEEPROM(addr, running_max.z);
  addr += sizeof(int16_t);

  EEPROM.write(EEPROM_CALIBRATED_ADDR, 0);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();

  // set calibration button as input and enable pull up resister
  pinMode(calibrationButton, INPUT_PULLUP);

  // set all the motor pin to be output
  for (int i = 0; i < VIBE_COUNT; i++) {
    pinMode(motorPin[i], OUTPUT);
  }

  // indicator LED to be output
  pinMode(LED_PIN, OUTPUT);

  // if the device is not calibrated, run the calibration sequence
  if (!isCalibrated()) {
    calibrate();
    writeCalibrationData();
  } else {
    readCalibrationData();
  }

  compass.m_min = running_min;
  compass.m_max = running_max;
}

void maybeVibrateHeading() {
    if (DEBUG) {
    Serial.println();
    Serial.print("heading ");
    Serial.print(heading);
  }
  
  int motor = 0;
  int wedge = 0;
  if (abs(lastVibratedHeading - heading) > SMOOTHING_DEGREES) {
    lastVibratedHeading = heading;

    wedge = (((int)(heading )) % 360);
    wedge = wedge / (360 / VIBE_COUNT); // deciding motor number
    

    if (DEBUG) {
      Serial.print("  :  ");
      Serial.print(wedge);
    }
    
  // starts new motor only when the direction has changed
    if (currentVibePin!= wedge){
    digitalWrite(motorPin[currentVibePin], LOW);
    digitalWrite(motorPin[wedge], HIGH);
    if (DEBUG) {
    Serial.print("  :onononon");
    }
    currentVibePin = wedge;
    vibrateCycles = 16;
    }
    
  }
}

void vibrateClock() {
  if (vibrateCycles > 0) {
    vibrateCycles--;
    if (vibrateCycles == 0) {
      digitalWrite(motorPin[currentVibePin], LOW);
    }
    if (DEBUG) {
    Serial.print("  : ");
    Serial.print(vibrateCycles);
    }
  }
}

void loop() {

  // check the calibration initiation button
  //----------------------------------------
  lastcal = cal;
  cal = digitalRead(calibrationButton);
  
  if (cal != lastcal && cal == LOW) {
    Serial.println("START calibration");
    calibrate();
    writeCalibrationData();
    compass.m_min = running_min;
    compass.m_max = running_max;
    Serial.println("DONE!");
    delay(1000);
  }
  //----------------------------------------

  compass.read();

  /*
    compass.heading((LSM303::vector<int>){0, 0, 1});
     => use the +Z axis as a reference (points directly through the PCB from
     silkscreened side to populated side).
  */
  heading = compass.heading((LSM303::vector<int>){0, 0, 1});
  maybeVibrateHeading();
  vibrateClock();

  delay(50);
}
