// Sideways Movement Test for Mecanum Wheels (Based on 06_lineFollowing.ino)

#include <SPI.h>
#include <DxlMaster2.h>
#include "arduDrive.hpp" // Header for Dynamixel registers

// --- DYNAMIXEL MOTOR DEFINITIONS ---
#define DRV1_DXL_ID 71
#define DRV2_DXL_ID 72
#define DXL_BAUD 57600

DynamixelDevice drive1(DRV1_DXL_ID); // Left Side Motors
DynamixelDevice drive2(DRV2_DXL_ID); // Right Side Motors

// --- MOVEMENT CONSTANTS ---
const int16_t BASE_SPEED = 250; // Use a moderate speed for testing (adjust as needed)
const uint32_t DURATION_MS = 2000; // 2 seconds duration for each movement

// --- CONTROL VARIABLES ---
unsigned long lastMovementTime = 0;
bool movingRight = true;


/**
 * Sets individual goal speeds for all four motors (M1 and M2 on Drive1/Drive2).
 * Speed should be an integer between -1000 and 1000.
 * A POSITIVE value in the DXL register moves the wheel FORWARD.
 * A NEGATIVE value in the DXL register moves the wheel BACKWARD.
 */
void setIndividualMotorSpeeds(
    int16_t d1m1_spd, int16_t d1m2_spd, 
    int16_t d2m1_spd, int16_t d2m2_spd) {
    
  // Drive 1 (Left Side)
  drive1.write(M1_GOAL_SPEED_L, d1m1_spd);
  drive1.write(M2_GOAL_SPEED_L, d1m2_spd);
  
  // Drive 2 (Right Side)
  drive2.write(M1_GOAL_SPEED_L, d2m1_spd);
  drive2.write(M2_GOAL_SPEED_L, d2m2_spd);
}

/**
 * Commands all motors to stop.
 */
void stopMotors() {
  setIndividualMotorSpeeds(0, 0, 0, 0);
}


void setup() {
  Serial.begin(115200);
  Serial.println("Mecanum Sideways Test Initialized.");
  
  // --- Dynamixel Motor Setup (Copied from 06_lineFollowing.ino) ---
  DxlMaster.begin(DXL_BAUD);
  drive1.protocolVersion(1);
  drive2.protocolVersion(1);
  
  // Set Motor Mode to Speed (2), Enable AB Encoder Mode, Set Max Power
  drive1.write(MOTOR_MODE, (uint8_t)2);
  drive1.write(ENCODER_MODE, (uint8_t)ENCODER_MODE_AB);
  drive1.write(MAX_POWER_L, (uint16_t)1000);
  
  drive2.write(MOTOR_MODE, (uint8_t)2);
  drive2.write(ENCODER_MODE, (uint8_t)ENCODER_MODE_AB);
  drive2.write(MAX_POWER_L, (uint16_t)1000);

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

    if (movingRight) {
      // 2. MOVE LEFT (Pattern: D1M1 BACK, D1M2 FWD, D2M1 FWD, D2M2 BACK)
      // This is the pattern you requested for moving Left.
      Serial.println("Moving Left for 2 seconds...");
      setIndividualMotorSpeeds(
        -BASE_SPEED,  // Drive1 M1: Backward
         BASE_SPEED,  // Drive1 M2: Forward
         BASE_SPEED,  // Drive2 M1: Forward
        -BASE_SPEED   // Drive2 M2: Backward
      );
    } else {
      // 3. MOVE RIGHT (Reverse of the Left pattern)
      Serial.println("Moving Right for 2 seconds...");
      setIndividualMotorSpeeds(
         BASE_SPEED,  // Drive1 M1: Forward
        -BASE_SPEED,  // Drive1 M2: Backward
        -BASE_SPEED,  // Drive2 M1: Backward
         BASE_SPEED   // Drive2 M2: Forward
      );
    }
    
    // Toggle the direction for the next cycle
    movingRight = !movingRight;
  }
  
  // Small delay to prevent blocking the loop completely
  delay(10); 
}
