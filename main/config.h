#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- 1. SENSOR DATA (Written by HAL, Read by Brain) ---
struct SensorData {
  float yaw;            // Current Heading (0-360)
  int frontDist;        // Front Sensor (mm)
  int diagDist;         // Diagonal Sensor (mm)
  int rightDist;        // Right Sensor (mm)
  int leftDist;
  long leftEnc;         // Encoder Ticks
  long rightEnc;        // Encoder Ticks
};

// --- 2. TUNING PARAMETERS (Written by App, Read by Motion) ---
struct Tunings {
  // Mode 0: Wall Follow Stability
  float wallKp;
  float wallKd;
  
  // Mode 1: Turning Precision
  float turnKp;
  float turnKd;

  // Speeds
  int baseSpeed;
};

// --- 3. ROBOT STATE / COMMANDS (Written by Brain, Executed by Motion) ---
struct RobotState {
  // Control Modes
  enum Mode { IDLE, DRIVE_STRAIGHT, TURN_ACTIVE };
  Mode mode;
  int logicMode;
  // Targets
  float targetHeading;    // The compass heading we want to face
  float wallOffset;       // The "Nudge" from the wall follower
  
  // Flags
  bool resetPID;          // If true, Motion layer wipes error history
};

// --- GLOBAL EXTERNS ---
extern SensorData sensors;
extern Tunings tunings;
extern RobotState robot;
void resetEncoders();
#endif