#include "Config.h"
#include <IntervalTimer.h>

IntervalTimer pidTimer;

// Motor Pins
const int L_PWMA = 6; const int L_IN1 = 7; const int L_IN2 = 8;
const int R_IN1 = 10; const int R_IN2 = 9; const int R_PWMB = 11;

// Internal PID Memory
float lastHeadingError = 0;
float lastTurnError = 0;

// ===================================================================================
// --- ISR INIT ---
// ===================================================================================
void initMotion() {
    pinMode(L_PWMA, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_PWMB, OUTPUT);
    
    // Start Heartbeat (200Hz = 5000us)
    pidTimer.begin(motionISR, 5000);
}

// ===================================================================================
// --- THE HEARTBEAT (ISR) ---
// ===================================================================================
void motionISR() {
    
    // 1. HANDLE RESETS (Transition Fix)
    if (robot.resetPID) {
        lastHeadingError = 0;
        lastTurnError = 0;
        robot.resetPID = false; // Ack
    }

    // 2. SAFETY STOP
    if (robot.mode == RobotState::IDLE) {
        stopMotors();
        return;
    }

    // 3. DRIVE STRAIGHT (Heading Lock + Wall Offset)
    if (robot.mode == RobotState::DRIVE_STRAIGHT) {
        
        // Target = Compass Heading + Wall Follower Nudge
        float effectiveTarget = robot.targetHeading + robot.wallOffset;
        
        float error = getAngleError(effectiveTarget, sensors.yaw);
        float dTerm = error - lastHeadingError;
        lastHeadingError = error;

        // Use Wall Tunings
        float output = (error * tunings.wallKp) + (dTerm * tunings.wallKd);
        
        move(tunings.baseSpeed - output, tunings.baseSpeed + output);
    }

    // 4. TURN (Precision Spot Turn)
    // Handles 90 or 180 automatically based on targetHeading
    else if (robot.mode == RobotState::TURN_ACTIVE) {
        
        float error = getAngleError(robot.targetHeading, sensors.yaw);
        
        // Stop Condition (Tolerance < 1.0 degree)
        if (abs(error) < 1.0) {
            stopMotors();
            robot.mode = RobotState::IDLE; // Self-Terminate
            return;
        }

        float dTerm = error - lastTurnError;
        lastTurnError = error;

        // Use Turn Tunings
        float output = (error * tunings.turnKp) + (dTerm * tunings.turnKd);
        
        // Stiction / Min Power
        output = constrain(output, -150, 150);
        if (output > 0 && output < 45) output = 45;
        if (output < 0 && output > -45) output = -45;

        move(-output, output);
    }
}

// ===================================================================================
// --- HELPERS ---
// ===================================================================================
float getAngleError(float target, float current) {
  float error = target - current;
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  return error;
}

void move(int left, int right) {
  left = constrain(left, -255, 255); right = constrain(right, -255, 255);
  if (left > 0) { digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW); analogWrite(L_PWMA, left); }
  else { digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH); analogWrite(L_PWMA, abs(left)); }
  if (right > 0) { digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW); analogWrite(R_PWMB, right); }
  else { digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH); analogWrite(R_PWMB, abs(right)); }
}

void stopMotors() { move(0,0); }