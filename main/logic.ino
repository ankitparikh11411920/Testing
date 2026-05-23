#include "Config.h"

// --- MAZE STATES ---
enum MazeState { CHECKING, RT_ALIGN_WALL, RT_BLIND_DRIVE, EXECUTING_TURN, EXIT_CLEARANCE };
MazeState state = CHECKING;

unsigned long stateTimer = 0;

// ===================================================================================
// --- MAIN LOGIC LOOP (Runs in void loop) ---
// ===================================================================================
// ===================================================================================
// --- MAIN BRAIN ---
// ===================================================================================
void runMazeSolver() {

  // -----------------------------------------------------------------
  // MODE 0: WALL STABILITY TUNING
  // -----------------------------------------------------------------
  if (robot.logicMode == 0) {
      robot.mode = RobotState::DRIVE_STRAIGHT; // Force Drive
      
      // Strict Wall Follow Logic (No Gap/Wall checks)
      if (sensors.rightDist < 300 && sensors.rightDist > 0) {
          int error = 90 - sensors.rightDist; 
          robot.wallOffset = error * 0.1; 
          robot.wallOffset = constrain(robot.wallOffset, -20, 20);
      } else {
          robot.wallOffset = 0; 
      }
      return; 
  }

  // -----------------------------------------------------------------
  // MODE 1: TURN ACCURACY TUNING
  // -----------------------------------------------------------------
  else if (robot.logicMode == 1) {
      static int stage = 0;
      static unsigned long timer = 0;

      // Simple Sequencer
      if (stage == 0) { // Wait 1s then Turn Left
          if (millis() - timer > 1000) {
              robot.targetHeading += 90;
              robot.mode = RobotState::TURN_ACTIVE;
              stage = 1;
          }
      } 
      else if (stage == 1) { // Wait for finish
          if (robot.mode == RobotState::IDLE) {
              timer = millis();
              stage = 2;
          }
      }
      else if (stage == 2) { // Wait 1s then Turn Right
           if (millis() - timer > 1000) {
              robot.targetHeading -= 90; // Go back
              robot.mode = RobotState::TURN_ACTIVE;
              stage = 3;
           }
      }
      else if (stage == 3) { // Wait for finish then reset
           if (robot.mode == RobotState::IDLE) {
              stage = 0; // Loop forever
              timer = millis();
           }
      }
      return;
  }

  // -----------------------------------------------------------------
  // MODE 2: FULL MAZE SOLVER (Your FSM)
  // -----------------------------------------------------------------
  else {
      runFullMazeLogic(); // Refactored below for cleanliness
  }
}
void runFullMazeLogic() {
  
  // ---------------------------------------------------------
  // STATE: CHECKING (Decide what to do)
  // ---------------------------------------------------------
  if (state == CHECKING) {
      
      // PRIORITY 1: RIGHT GAP (Opportunity)
      // Filter: Must see gap consistently to avoid noise
      if (sensors.diagDist > 250 && sensors.diagDist < 2000) {
           Serial.println("[LOGIC] Right Gap -> Init Right Turn");
           
           // Stop Wall Following immediately
           robot.wallOffset = 0;
           robot.resetPID = true;

           // Sub-Case Decision
           if (sensors.frontDist < 200 && sensors.frontDist > 0) {
               state = RT_ALIGN_WALL; // Use Wall
           } else {
               state = RT_BLIND_DRIVE; // Use Encoders
               resetEncoders(); // Helper needs to be accessible or reset manually
               // Note: You might need to add resetEncoders to HAL or Main header
               sensors.leftEnc = 0; sensors.rightEnc = 0; // Manual reset attempt via struct if mapped? 
               // Better: Add command.resetEncoders flag in Config.h if strictly layered.
           }
           return;
      }

      // PRIORITY 2: FRONT BLOCKED (Obstacle)
      if (sensors.frontDist < 100 && sensors.frontDist > 0) {
          robot.mode = RobotState::IDLE; // Stop momentarily
          
          // Sub-Check: Dead End or Left Turn?
          if (sensors.leftDist < 150 && sensors.leftDist > 0) {
              Serial.println("[LOGIC] Dead End -> Turn 180");
              robot.targetHeading += 180; // 180 Turn
          } else {
              Serial.println("[LOGIC] Front Block -> Turn Left");
              robot.targetHeading += 90;  // Left Turn
          }
          
          robot.mode = RobotState::TURN_ACTIVE;
          state = EXECUTING_TURN;
          return;
      }

      // PRIORITY 3: FOLLOW RIGHT WALL (Maintenance)
      // Logic: Just set the mode. The Motion Layer handles the PID.
      robot.mode = RobotState::DRIVE_STRAIGHT;
      
      // Calculate Wall Offset Logic (Simple P-Controller for Offset)
      // Logic Layer calculates the *Angle Request*, Motion executes it.
      // This keeps "Math" in Brain, "Execution" in Body.
      
      if (sensors.rightDist < 300 && sensors.rightDist > 0) {
          int error = 90 - sensors.rightDist; // Target 9cm
          // Simple conversion: 10mm error = 1 degree offset
          robot.wallOffset = error * 0.1; 
          robot.wallOffset = constrain(robot.wallOffset, -20, 20);
      } else {
          robot.wallOffset = 0; // No wall, just IMU lock
      }
  }

  // ---------------------------------------------------------
  // STATE: RIGHT TURN SEQUENCES
  // ---------------------------------------------------------
  
  // CASE A: Align to Front Wall
  else if (state == RT_ALIGN_WALL) {
      robot.mode = RobotState::DRIVE_STRAIGHT;
      robot.wallOffset = 0; // Pure IMU Lock
      
      // Drive until Front Sensor is 6cm (Center of cell)
      if (sensors.frontDist < 60 && sensors.frontDist > 0) {
          startRightTurn();
      }
  }

  // CASE B: Blind Drive (Encoders)
  else if (state == RT_BLIND_DRIVE) {
      robot.mode = RobotState::DRIVE_STRAIGHT;
      robot.wallOffset = 0;
      
      // Drive 10cm (approx 5500 ticks)
      long avgEnc = (abs(sensors.leftEnc) + abs(sensors.rightEnc)) / 2;
      if (avgEnc > 5500) {
          startRightTurn();
      }
  }

  // ---------------------------------------------------------
  // STATE: WAITING FOR TURN
  // ---------------------------------------------------------
  else if (state == EXECUTING_TURN) {
      // The Motion Layer sets mode to IDLE when turn finishes
      if (robot.mode == RobotState::IDLE) {
          // Turn Done. Reset for Exit Drive.
          // Reset Encoders logic here (via flag)
          // command.resetEncoders = true; 
          sensors.leftEnc = 0; sensors.rightEnc = 0; // Manual
          
          state = EXIT_CLEARANCE;
          robot.mode = RobotState::DRIVE_STRAIGHT;
          robot.targetHeading = sensors.yaw; // Lock new heading
          robot.resetPID = true; // Clear Derivative History
      }
  }

  // ---------------------------------------------------------
  // STATE: EXIT CLEARANCE (Move away from corner)
  // ---------------------------------------------------------
  else if (state == EXIT_CLEARANCE) {
      // Drive 5cm to get sensors clear of the wall edge
      long avgEnc = (abs(sensors.leftEnc) + abs(sensors.rightEnc)) / 2;
      if (avgEnc > 2750) {
          state = CHECKING; // Resume Wall Following
          robot.resetPID = true; // Fresh start for Wall Follower
      }
  }
}

// Helper to trigger the turn
void startRightTurn() {
    Serial.println("[LOGIC] Executing Right Turn");
    robot.targetHeading -= 90;
    robot.mode = RobotState::TURN_ACTIVE;
    state = EXECUTING_TURN;
}