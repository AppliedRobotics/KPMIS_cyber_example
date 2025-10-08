// Line Following Car Code For a line with 7cm width

#include <SPI.h>
#include <DxlMaster2.h>
#include "arduDrive.hpp" // Header for Dynamixel registers

// --- DYNAMIXEL MOTOR DEFINITIONS ---
#define DRV1_DXL_ID 71
#define DRV2_DXL_ID 72
#define DXL_BAUD 57600

DynamixelDevice drive1(DRV1_DXL_ID); // Left Side Motors
DynamixelDevice drive2(DRV2_DXL_ID); // Right Side Motors

// --- SPEED CONSTANTS ---
// Speeds are arbitrary values up to 1000
const int16_t MAX_SPEED = 300;
const int16_t LESS_SPEED = 200;
const int16_t TURN_SPEED_FAST = 300;
const int16_t TURN_SPEED_SLOW = 50;
const int16_t STOP_SPEED = 0;

int16_t leftSpeed = STOP_SPEED;
int16_t rightSpeed = STOP_SPEED;


// --- SENSOR DEFINITIONS---
// Pins from left to right: S0, S1, S2, S3
const uint8_t lineSensorPins[4] = {3, 6, 4, 7};
bool sensorStates[4] = {false, false, false, false}; // Stores current sensor states

// 1 = White (Off Line), 0 = Black (On Line)


/*
  Function to set motor speeds and send commands
*/
void setMotorSpeeds(int16_t left_spd, int16_t right_spd) {
  // Drive1 controls the left side, Drive2 controls the right side.
  // The right motor speed must be NEGATED for both motors to drive forward.
  int16_t actual_right_spd = -right_spd; 
  
  // Set goal speed for both motors on Drive 1 (Left side)
  drive1.write(M1_GOAL_SPEED_L, left_spd);
  drive1.write(M2_GOAL_SPEED_L, left_spd);
  
  // Set goal speed for both motors on Drive 2 (Right side)
  drive2.write(M1_GOAL_SPEED_L, actual_right_spd);
  drive2.write(M2_GOAL_SPEED_L, actual_right_spd);
}


void setup() {
  Serial.begin(115200);
  Serial.println("Line Follower Initialized.");
  
  // --- Line Sensor Setup ---
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(lineSensorPins[i], INPUT);
  }
  
  // --- Dynamixel Motor Setup (From 06_lineFollowing.ino) ---
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

  // Set PID for speed control (values copied from 06_lineFollowing.ino)
  drive1.write(PID_P, (uint8_t)300);
  drive1.write(PID_I, (uint8_t)50);
  drive1.write(PID_D, (uint8_t)10);

  drive2.write(PID_P, (uint8_t)300);
  drive2.write(PID_I, (uint8_t)50);
  drive2.write(PID_D, (uint8_t)10);
  
  delay(1000); // Wait for setup to complete
}


void loop() {
  uint8_t patternValue = 0;
  
  // 1. Read Sensors and Create Pattern Value
  // The original code uses: sensorStates[i] = !digitalRead(lineSensorPins[i]);
  // which means '1' (true) is LOW signal, and '0' (false) is HIGH signal.
  // Assuming the user's rule: 1 = White, 0 = Black, we need to match the digitalRead
  // to the user's definition of 1/0. 
  // If the sensor is over WHITE (1), it returns a value that, when inverted, matches 1.
  // Since the user states 1=WHITE and 0=BLACK, we assume the digitalRead() already 
  // returns the inverted value (LOW for white/high reflection, HIGH for black/low reflection)
  // which is common for line sensors.

  for (uint8_t i = 0; i < 4; i++) {
    // !digitalRead() is used as in the original 06 code, where 1 means line detected (White or Black, depending on sensor calibration)
    // We assume !digitalRead() = 1 (True) means WHITE (as per user's table: 1=White, 0=Black)
    sensorStates[i] = !digitalRead(lineSensorPins[i]);
    
    // Combine states into a single 4-bit integer: (S0<<3) | (S1<<2) | (S2<<1) | (S3<<0)
    // S0 is the most significant bit (leftmost sensor)
    if (sensorStates[i]) {
      patternValue |= (1 << (3 - i));
    }
  }

  // 2. Apply Line Following Logic
  switch (patternValue) {
    
    // --- Go Forward States ---
    case 9:  // 1, 0, 0, 1 (White, Black, Black, White - Line Centered)
      leftSpeed = MAX_SPEED;
      rightSpeed = MAX_SPEED;
      break;

    // --- Forward Less Speed States (Minor Correction) ---
    case 13: // 1, 1, 0, 1 (Line shifted slightly right)
    case 11: // 1, 0, 1, 1 (Line shifted slightly left)
      leftSpeed = LESS_SPEED;
      rightSpeed = LESS_SPEED;
      break;
      
    // --- Turn Left States (Line is too far right) ---
    case 12: // 1, 1, 0, 0 
    case 14: // 1, 1, 1, 0 
      leftSpeed = TURN_SPEED_SLOW; // Low speed on left (to move left)
      rightSpeed = TURN_SPEED_FAST; // High speed on right (to pivot)
      break;
      
    // --- Turn Right States (Line is too far left) ---
    case 3:  // 0, 0, 1, 1
    case 7:  // 0, 1, 1, 1
      leftSpeed = TURN_SPEED_FAST; // High speed on left (to pivot)
      rightSpeed = TURN_SPEED_SLOW; // Low speed on right (to move right)
      break;

    // --- Stop States ---
    case 15: // 1, 1, 1, 1 (All White / Lost)
    case 0:  // 0, 0, 0, 0 (All Black / Lost)
    default: // Any other unhandled case
      leftSpeed = STOP_SPEED;
      rightSpeed = STOP_SPEED;
      break;
  }
  
  // 3. Send Motor Commands
  setMotorSpeeds(leftSpeed, rightSpeed);

  // Optional: Print status for debugging
  Serial.print("Pattern: ");
  Serial.print(patternValue, BIN);
  Serial.print("\tLeft: ");
  Serial.print(leftSpeed);
  Serial.print("\tRight: ");
  Serial.println(rightSpeed);

  delay(50); // Loop delay
}