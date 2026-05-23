#include "Config.h"
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Adafruit_BNO08x.h>

// --- HARDWARE OBJECTS ---
SparkFun_VL53L5CX sensorRight;
SparkFun_VL53L5CX sensorFront;
SparkFun_VL53L5CX sensorDiag;
SparkFun_VL53L5CX sensorLeft;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

// Pins
const int LP_PIN_RIGHT = 15; 
const int LP_PIN_DIAG = 16;
const int LP_PIN_FRONT = 17;
const int LP_UNUSED_1 = 20;
const int LP_PIN_LEFT = 21; 

// Bluetooth Parsing Vars
int appP = 30, appD = 150;
int multiP = 1, multiD = 1;

// ===================================================================================
// --- INIT HARDWARE ---
// ===================================================================================
void initHardware() {
  Serial.begin(115200); Serial1.begin(9600);
  Wire.begin(); Wire.setClock(400000);

  // 1. Silence All Sensors
  int allPins[] = {15, 16, 17, 20, 21};
  for(int i=0; i<5; i++) {
      pinMode(allPins[i], OUTPUT); digitalWrite(allPins[i], LOW);
  }
  delay(50);

  // 2. Init Right (0x54)
  digitalWrite(LP_PIN_RIGHT, HIGH); delay(20);
  if (sensorRight.begin()) {
     sensorRight.setAddress(0x54); 
     sensorRight.setResolution(4*4); 
     sensorRight.setRangingFrequency(60); // Fast for wall follow
     sensorRight.startRanging();
  }

  // 3. Init Diag (0x56)
  digitalWrite(LP_PIN_DIAG, HIGH); delay(20);
  if (sensorDiag.begin()) {
     sensorDiag.setAddress(0x56); 
     sensorDiag.setResolution(4*4); 
     sensorDiag.setRangingFrequency(30); 
     sensorDiag.startRanging();
  }

  // 4. Init Front (0x58)
  digitalWrite(LP_PIN_FRONT, HIGH); delay(20);
  if (sensorFront.begin()) {
     sensorFront.setAddress(0x58); 
     sensorFront.setResolution(4*4); 
     sensorFront.setRangingFrequency(30); 
     sensorFront.startRanging();
  }

  // 6. Init Left (Pin 21 -> 0x5A)
  
  digitalWrite(LP_PIN_LEFT, HIGH); delay(20);
  if (sensorLeft.begin()) {
     sensorLeft.setAddress(0x5A); 
     sensorLeft.setResolution(4*4); 
     sensorLeft.setRangingFrequency(30); 
     sensorLeft.startRanging();
  }

  // 5. Init IMU
  if (!bno08x.begin_I2C()) while(1);
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 2000); 
}

// ===================================================================================
// --- SENSOR READING (SCHEDULER) ---
// ===================================================================================
void readSensors() {
  // 1. IMU (Always)
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float r = sensorValue.un.gameRotationVector.real;
      float i = sensorValue.un.gameRotationVector.i;
      float j = sensorValue.un.gameRotationVector.j;
      float k = sensorValue.un.gameRotationVector.k;
      float yaw = atan2(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k));
      sensors.yaw = yaw * (180.0f / PI);
    }
  }

  // 2. Right Sensor (High Priority - Every Loop)
  if (sensorRight.isDataReady()) {
      VL53L5CX_ResultsData data;
      if (sensorRight.getRangingData(&data)) {
          // Store raw data into global struct for the Logic layer to use
          // Note: Logic layer calculates the "Error", HAL just gives distance
          // We store the Center zone for generic use, but Logic might need array access
          // For simplicity in this architecture, we pass the generic "distance" 
          // Logic Layer will call specific function if it needs zones.
          sensors.rightDist = data.distance_mm[5]; 
      }
  }

  // 3. Front/Diag Scheduler (Low Priority - Every 3rd Loop)
  static int sched = 0; sched++;
  if (sched >= 3) {
      sched = 0;
      
      if (sensorFront.isDataReady()) {
          VL53L5CX_ResultsData data;
          if (sensorFront.getRangingData(&data)) sensors.frontDist = data.distance_mm[5];
      }
      
      if (sensorDiag.isDataReady()) {
          VL53L5CX_ResultsData data;
          if (sensorDiag.getRangingData(&data)) sensors.diagDist = data.distance_mm[5];
      }

      if (sensorLeft.isDataReady()) {
          VL53L5CX_ResultsData data;
          if (sensorLeft.getRangingData(&data)) sensors.leftDist = data.distance_mm[5];
      }
  }
}

// ===================================================================================
// --- BLUETOOTH TUNING ---
// ===================================================================================
void readBluetooth() {
  while (Serial1.available()) {
    byte val = Serial1.read();
    static byte buf[2]; static int idx = 0;
    buf[idx++] = val;
    
    if (idx == 2) { 
        byte cmd = buf[0]; byte value = buf[1];
        idx = 0;
        
        // DEBUG: Print what we received
        Serial.print("[BT] Cmd:"); Serial.print(cmd);
        Serial.print(" Val:"); Serial.println(value);

        switch (cmd) {
            case 1: appP = value; break;
            case 2: multiP = value; break;
            case 5: appD = value; break;
            case 6: multiD = value; break;
            case 3: tunings.baseSpeed = map(value, 0, 100, 0, 255); break;
            case 7: 
                if (value == 1) { 
                    // START
                    Serial.println(">>> BT START REQUEST <<<"); // Debug Print
                    if (robot.mode == RobotState::IDLE) {
                        robot.mode = RobotState::DRIVE_STRAIGHT;
                        robot.targetHeading = sensors.yaw; 
                        robot.resetPID = true;
                    }
                } else { 
                    // STOP
                    Serial.println(">>> BT STOP REQUEST <<<"); // Debug Print
                    robot.mode = RobotState::IDLE; 
                    tunings.baseSpeed = 0; // Force Speed 0
                }
                break;
        }

        float newP = (float)appP / pow(10, multiP);
        float newD = (float)appD / pow(10, multiD);

        // Update Global Tunings
        if (robot.logicMode == 1) { // Turn Mode
            tunings.turnKp = newP; tunings.turnKd = newD;
        } else { // Wall Mode
            tunings.wallKp = newP; tunings.wallKd = newD;
        }
    }
  }
}

// --- ENCODER PINS ---
const int L_ENC_A = 3; const int L_ENC_B = 2;
const int R_ENC_A = 4; const int R_ENC_B = 5;

// --- ISRs ---
void readLeftEncoder() {
  if (digitalRead(L_ENC_B) > 0) sensors.leftEnc++; else sensors.leftEnc--;
}
void readRightEncoder() {
  if (digitalRead(R_ENC_B) > 0) sensors.rightEnc++; else sensors.rightEnc--;
}

// --- INIT HELPER ---
// Call this inside initHardware() !!!
void initEncoders() {
  pinMode(L_ENC_A, INPUT); pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT); pinMode(R_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), readRightEncoder, RISING);
}
void resetEncoders() {
  noInterrupts();
  sensors.leftEnc = 0;
  sensors.rightEnc = 0;
  interrupts();
}