// Complex Movement Test for Mecanum Wheels: Orbit + Spin
// This sketch combines Forward/Backward movement components (for the orbit) 
// and Rotation components (for the body spin).

#include <SPI.h>
#include <DxlMaster2.h>
#include "arduDrive.hpp" // Header for Dynamixel registers

// --- DYNAMIXEL MOTOR DEFINITIONS ---
#define DRV1_DXL_ID 71
#define DRV2_DXL_ID 72
#define DXL_BAUD 57600 // Baud rate is defined but NOT passed to the device init function

DynamixelDevice drive1(DRV1_DXL_ID); // Typically controls M1 (Front-Left) and M2 (Rear-Left)
DynamixelDevice drive2(DRV2_DXL_ID); // Typically controls M1 (Front-Right) and M2 (Rear-Right)

// --- MOVEMENT CONSTANTS ---
const int16_t BASE_SPEED = 250;      // Base speed for pure movements
const uint32_t DURATION_MS = 3000;   // 3 seconds duration for each movement

// Components for ORBIT + SPIN motion
const int16_t ORBIT_FORWARD_SPEED = 200; // Speed component for forward motion (orbit radius)
const int16_t ORBIT_TURN_BIAS = 50;      // Differential speed for turning right (creates the circle)
const int16_t SPIN_RATE = 100;           // Speed component for spinning the body (rotation)

// --- CONTROL VARIABLES ---
unsigned long lastMovementTime = 0;
// 0: Orbit + Spin, 1: Strafe Right, 2: Pure Spin CCW
uint8_t movementState = 0; 


/**
 * Sets individual goal speeds for all four motors (M1 and M2 on Drive1/Drive2).
 * Speed should be an integer between -1000 and 1000.
 * POSITIVE value moves the wheel FORWARD, NEGATIVE moves BACKWARD.
 * * Assuming mapping:
 * Drive1 M1: Front Left (FL)
 * Drive1 M2: Rear Left (RL)
 * Drive2 M1: Front Right (FR)
 * Drive2 M2: Rear Right (RR)
 */
void setIndividualMotorSpeeds(
    int16_t d1m1_spd, int16_t d1m2_spd, 
    int16_t d2m1_spd, int16_t d2m2_spd) {
    
  // Drive1 (Left side controller)
  drive1.write(M1_GOAL_SPEED_L, d1m1_spd);
  drive1.write(M2_GOAL_SPEED_L, d1m2_spd);
  
  // Drive2 (Right side controller)
  drive2.write(M1_GOAL_SPEED_L, d2m1_spd);
  drive2.write(M2_GOAL_SPEED_L, d2m2_spd);
}

/**
 * Stops all four motors.
 */
void stopMotors() {
  setIndividualMotorSpeeds(0, 0, 0, 0);
}


void setup() {
  Serial.begin(57600); // Standard baud rate for output
  
  // Initialize DXL devices
  // --- FIX APPLIED HERE: Removed DXL_BAUD argument from init() call ---
  drive1.init(); 
  drive2.init();
  
  // Set PID for speed control (using values from 06_lineFollowing.ino)
  drive1.write(PID_P, (uint8_t)300);
  drive1.write(PID_I, (uint8_t)50);
  drive1.write(PID_D, (uint8_t)10);

  drive2.write(PID_P, (uint8_t)300);
  drive2.write(PID_I, (uint8_t)50);
  drive2.write(PID_D, (uint8_t)10);
  
  stopMotors(); // Ensure motors are stopped initially
  lastMovementTime = millis();
}


void loop() {
  if (millis() - lastMovementTime >= DURATION_MS) {
    lastMovementTime = millis();
    
    // 1. Stop for a brief moment
    stopMotors();
    delay(500); 

    movementState++;
    if (movementState > 2) {
      movementState = 0; // Cycle back to the first state
    }
    
    switch (movementState) {
      case 0: {
        // === STATE 0: ORBIT (Forward/Turn Right) + SPIN (CCW) ===
        // This is the "Planet" motion: Moving forward in a circle while spinning.
        
        // ORBIT Component (Creates a Right Turn):
        // Left Motors (FL, RL) run faster than Right Motors (FR, RR).
        int16_t orbit_left_speed = ORBIT_FORWARD_SPEED + ORBIT_TURN_BIAS;  // e.g., 250
        int16_t orbit_right_speed = ORBIT_FORWARD_SPEED - ORBIT_TURN_BIAS; // e.g., 150
        
        // SPIN Component (CCW Rotation):
        // FL(+), FR(-), RL(+), RR(-)
        int16_t spin_fl = SPIN_RATE;
        int16_t spin_fr = -SPIN_RATE;
        int16_t spin_rl = SPIN_RATE;
        int16_t spin_rr = -SPIN_RATE;
        
        // COMBINED SPEED (Vector Sum)
        int16_t fl_total = orbit_left_speed  + spin_fl; 
        int16_t rl_total = orbit_left_speed  + spin_rl; 
        int16_t fr_total = orbit_right_speed + spin_fr; 
        int16_t rr_total = orbit_right_speed + spin_rr; 
        
        Serial.println("STATE 0: Orbit (Forward/Turn Right) + Spin CCW...");
        setIndividualMotorSpeeds(
          fl_total,  // FL: Total Speed
          rl_total,  // RL: Total Speed
          fr_total,  // FR: Total Speed
          rr_total   // RR: Total Speed
        );
        break;
      }
        
      case 1: {
        // === STATE 1: PURE STRAFE RIGHT ===
        // Pattern: FL(+), FR(-), RL(-), RR(+) (Strafe Right)
        Serial.println("STATE 1: Pure Strafe Right...");
        setIndividualMotorSpeeds(
           BASE_SPEED,   // FL: Forward
          -BASE_SPEED,  // RL: Backward
          -BASE_SPEED,  // FR: Backward
           BASE_SPEED   // RR: Forward
        );
        break;
      }
        
      case 2: {
        // === STATE 2: PURE SPIN CCW (Counter-Clockwise) ===
        // Pattern: FL(+), FR(-), RL(+), RR(-) (Spin in Place)
        Serial.println("STATE 2: Pure Spin CCW...");
        setIndividualMotorSpeeds(
           BASE_SPEED,  // FL: Forward
           BASE_SPEED,  // RL: Forward
          -BASE_SPEED,  // FR: Backward
          -BASE_SPEED   // RR: Backward
        );
        break;
      }
    }
  }
}

// Function to handle DXL master communication errors (optional, but good practice)
void DxlMaster2_Error(const uint8_t dxl_err_code) {
  // Simple error reporting
  Serial.print("DXL Error: ");
  Serial.println(dxl_err_code);
}