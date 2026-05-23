#include "Config.h"

// --- GLOBAL INSTANCES ---
SensorData sensors;
Tunings tunings;
RobotState robot;

const int BTN_PIN = 12; // Ensure this matches your wiring!

void setup() {
  initHardware(); 
  initMotion();
  pinMode(12, INPUT_PULLUP);
  // Default Tunings
  tunings.wallKp = 0.05;  tunings.wallKd = 0.03;
  tunings.turnKp = 3.0;   tunings.turnKd = 5.0;
  tunings.baseSpeed = 0;  
  
  robot.mode = RobotState::IDLE;
  robot.targetHeading = 0;
  robot.resetPID = true;

  // Visual Ready
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); delay(500); digitalWrite(13, LOW);
}

void loop() {
  readSensors();      
  readBluetooth();    
  handleButton();     

  if (robot.mode != RobotState::IDLE) {
      runMazeSolver(); 
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) { // Slower print (5Hz) to keep Serial clean
      printDashboard();
      lastPrint = millis();
  }
}

// --- UPDATED DASHBOARD (SEE EVERYTHING) ---
void printDashboard() {
  // 1. MOTION STATE (What the wheels are doing)
  Serial.print("ACT:"); 
  if(robot.mode == RobotState::IDLE) Serial.print("STOP");
  else if(robot.mode == RobotState::DRIVE_STRAIGHT) Serial.print("DRIVE");
  else if(robot.mode == RobotState::TURN_ACTIVE) Serial.print("TURN");
  
  // 2. LOGIC MODE (What the brain is thinking)
  Serial.print(" | MOD:"); 
  if(robot.logicMode == 0) Serial.print("WALL");
  else if(robot.logicMode == 1) Serial.print("TURN");
  else if(robot.logicMode == 2) Serial.print("MAZE");

  // 3. SENSORS
  Serial.print(" | F:"); Serial.print(sensors.frontDist);
  Serial.print(" R:"); Serial.print(sensors.rightDist);
  
  // 4. PID (Active Values)
  Serial.print(" | Kp:"); 
  if(robot.logicMode == 1) Serial.print(tunings.turnKp);
  else Serial.print(tunings.wallKp);
  
  Serial.println();
}
// ===================================================================================
// --- BUTTON HANDLER (FIXED: SHORT VS LONG PRESS) ---
// ===================================================================================
void handleButton() {
  static int lastButtonState = HIGH; 
  static unsigned long lastDebounceTime = 0;
  static unsigned long pressStartTime = 0; // To measure how long you hold it
  
  int currentReading = digitalRead(12); // Pin 12

  // 1. DETECT PRESS DOWN (Start Timer)
  if (lastButtonState == HIGH && currentReading == LOW) {
      if (millis() - lastDebounceTime > 50) { // Fast debounce
          pressStartTime = millis(); 
          lastDebounceTime = millis();
      }
  }

  // 2. DETECT RELEASE (Calculate Duration & Act)
  if (lastButtonState == LOW && currentReading == HIGH) {
      if (millis() - lastDebounceTime > 50) {
          lastDebounceTime = millis();
          
          unsigned long duration = millis() - pressStartTime;

          // --- LOGIC: LONG PRESS (> 1 Second) ---
          if (duration > 1000) {
              // 1. Cycle Mode
              robot.logicMode++;
              if (robot.logicMode > 2) robot.logicMode = 0;
              
              Serial.print(">>> MODE CHANGED TO: "); Serial.println(robot.logicMode);
              
              // 2. Blink LED to confirm Mode (1x=Mode0, 2x=Mode1, 3x=Mode2)
              for(int i=0; i <= robot.logicMode; i++) {
                 digitalWrite(13, HIGH); delay(200); 
                 digitalWrite(13, LOW); delay(200);
              }

          } 
          // --- LOGIC: SHORT PRESS (< 1 Second) ---
          else {
              if (robot.mode == RobotState::IDLE) {
                 Serial.println(">>> STARTING <<<");
                 robot.mode = RobotState::DRIVE_STRAIGHT;
                 robot.targetHeading = sensors.yaw; 
                 robot.resetPID = true;
              } else {
                 Serial.println(">>> STOPPING <<<");
                 robot.mode = RobotState::IDLE;
              }
          }
      }
  }
  
  lastButtonState = currentReading;
}