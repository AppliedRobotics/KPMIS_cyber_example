// Combined Car Control Code with Line Following, Bump Sensors, Ultrasonic, and Sideways Drive

// --- LIBRARIES ---
#include <SPI.h>
#include <DxlMaster2.h>
#include "arduDrive.hpp"
#include "arduUnoKeyboard.hpp"

// --- DYNAMIXEL MOTOR DEFINITIONS ---
#define DRV1_DXL_ID 71
#define DRV2_DXL_ID 72
#define DXL_BAUD 57600 // Baud rate for DXL communication

// FIX: Explicitly define DXL_SUCCESS to resolve the compilation error.
#define DXL_SUCCESS 0

DynamixelDevice drive1(DRV1_DXL_ID); // Left Side Motors
DynamixelDevice drive2(DRV2_DXL_ID); // Right Side Motors

// --- SPEED CONSTANTS --- 
const int16_t MAX_SPEED = 150;
const int16_t LESS_SPEED = 50;
const int16_t TURN_SPEED_FAST = 150;
const int16_t TURN_SPEED_SLOW = 50;
const int16_t STOP_SPEED = 0;
const int16_t REVERSE_SPEED = -50;
const int16_t SIDEWAYS_SPEED = 100; // Speed for sideways movement

int16_t leftSpeed = STOP_SPEED;
int16_t rightSpeed = STOP_SPEED;

// --- SENSOR DEFINITIONS ---
// Line sensors: Pins from left to right: S0, S1, S2, S3
const uint8_t lineSensorPins[4] = {3, 6, 4, 7};
bool sensorStates[4] = {false, false, false, false};

// Bump sensors
const uint8_t contactLeftPin = 8;     // Left bump sensor (Assuming rear-mounted for safety while reversing)
const uint8_t contactRightPin = 5;    // Right bump sensor (Assuming rear-mounted for safety while reversing)
bool leftBumpState = false;
bool rightBumpState = false;
bool previousLeftBump = false;
bool previousRightBump = false;

// Ultrasonic sensor (CORRECTED PINS: A0 and A1)
const uint8_t trigPin = A0;           // Ultrasonic trigger pin (Analog Pin 0)
const uint8_t echoPin = A1;           // Ultrasonic echo pin (Analog Pin 1)

// --- STATE VARIABLES ---
enum CarState {
  STATE_IDLE,
  STATE_LINE_FOLLOWING,
  STATE_SIDEWAYS_TEST, 
  STATE_OBSTACLE_REVERSING,   // Car is reversing away from an obstacle
  STATE_BUMP_STOPPED,         // Car stopped due to rear bump during reverse
  STATE_BUMP_RECOVERING       // Car is briefly driving forward to clear the bump
};

CarState currentState = STATE_IDLE;
CarState previousState = STATE_IDLE;

// Bump recovery variables
unsigned long bumpStopTime = 0;
// We no longer need BUMP_STOP_DURATION/BUMP_RECOVERING logic as the ultrasonic handles resumption.

// --- ULTRASONIC CONSTANTS (NEW/UPDATED) ---
const int OBSTACLE_REVERSE_TRIGGER_DISTANCE = 15; // Critical distance: If <= 15cm, START reversing
const int OBSTACLE_RESUME_DISTANCE = 20;          // Clear distance: If > 20cm, STOP reversing and RESUME line following

// --- TIMING VARIABLES ---
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 50; // Read sensors/run logic every 50ms
unsigned long lastButtonCheck = 0;
const unsigned long BUTTON_CHECK_INTERVAL = 50; 
unsigned long lastUltrasonicRead = 0;
const unsigned long ULTRASONIC_READ_INTERVAL = 100;
unsigned long lastSidewaysChange = 0; // Timer for changing sideways direction
const unsigned long SIDEWAYS_DURATION = 3000; // Change sideways direction every 3 seconds

// Ultrasonic variables
long duration;
int distance = 0; // Initialize distance

// Button state tracking
uint8_t previousButtonMask = 0;

// Sideways movement variables
bool isMovingRight = true;


/*
  Function to set motor speeds and send commands (for forward/reverse/turning)
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

/*
 * Sets individual goal speeds for all four motors (M1 and M2 on Drive1/Drive2).
 * This function is REQUIRED for Mecanum sideways movement.
 * Drive1 is Left Side, Drive2 is Right Side.
 * NOTE: The motors on Drive 2 (Right Side) inputs are NEGATED to correct for
 * physical mounting when using a standard Mecanum speed model.
 */
void setIndividualMotorSpeeds(
    int16_t d1m1_spd, int16_t d1m2_spd, 
    int16_t d2m1_spd, int16_t d2m2_spd) {
    
  // Drive 1 (Left Side) uses input speeds directly
  drive1.write(M1_GOAL_SPEED_L, d1m1_spd);
  drive1.write(M2_GOAL_SPEED_L, d1m2_spd);
  
  // Drive 2 (Right Side) negates the input speeds
  drive2.write(M1_GOAL_SPEED_L, -d2m1_spd); 
  drive2.write(M2_GOAL_SPEED_L, -d2m2_spd); 
}

/*
  Function to stop all motors
*/
void stopMotors() {
  setMotorSpeeds(STOP_SPEED, STOP_SPEED);
}

/*
  Read ultrasonic sensor distance
*/
int readUltrasonic() {
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH for 10 microsecond pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  // Set a timeout to prevent locking up if no echo is received
  duration = pulseIn(echoPin, HIGH, 50000); 
  
  // Calculating the distance (speed of sound is 0.034 cm/µs)
  if (duration > 0) {
      distance = duration * 0.034 / 2;
  } else {
      // If pulseIn timed out, assume max distance or an error (set a safe high value)
      distance = 400; 
  }
  
  return distance;
}

/*
  Read bump sensors
*/
void readBumpSensors() {
  previousLeftBump = leftBumpState;
  previousRightBump = rightBumpState;
  
  // Inverted due to INPUT_PULLUP (LOW = pressed, HIGH = unpressed)
  leftBumpState = !digitalRead(contactLeftPin);    
  rightBumpState = !digitalRead(contactRightPin); 
}

/*
  Handle button input - BUTTON1 and BUTTON2
  Button 1: Toggles Line Following.
  Button 2: Toggles Sideways Test.
*/
void handleButtonInput() {
  uint8_t currentButtonMask = keyboard.getKeyboardMask();
  
  // Detects which buttons just went from not pressed (0) to pressed (1)
  uint8_t pressedButtons = currentButtonMask & (~previousButtonMask);
  
  if (pressedButtons) {
    if (pressedButtons & ArduUnoKeyboard::BUTTON1) {
      // Toggle state between IDLE and LINE_FOLLOWING
      if (currentState == STATE_LINE_FOLLOWING) { 
        currentState = STATE_IDLE; // Stop
        stopMotors();
      } else {
        // Start Line Following (or return to it from a recovery state)
        currentState = STATE_LINE_FOLLOWING;
      }
    } else if (pressedButtons & ArduUnoKeyboard::BUTTON2) {
      // Toggle state between IDLE and SIDEWAYS_TEST
      if (currentState == STATE_SIDEWAYS_TEST) {
        currentState = STATE_IDLE; // Stop
        stopMotors();
      } else {
        // Start Sideways Movement
        currentState = STATE_SIDEWAYS_TEST;
        // Reset timer to start movement immediately
        lastSidewaysChange = millis();
        isMovingRight = true; // Start by moving right
      }
    }
  }
  
  // Ensure that if we transition from an obstacle state using a button, 
  // we go to IDLE first for safety.
  if (currentState == STATE_BUMP_STOPPED || 
      currentState == STATE_OBSTACLE_REVERSING) {
      
      if (pressedButtons & (ArduUnoKeyboard::BUTTON1 | ArduUnoKeyboard::BUTTON2)) {
          currentState = STATE_IDLE;
          stopMotors();
      }
  }

  previousButtonMask = currentButtonMask;
}

/*
  Line following logic
*/
void lineFollowing() {
  uint8_t patternValue = 0;
  
  // Read line sensors and create pattern 
  for (uint8_t i = 0; i < 4; i++) {
    // Sensor is 1 (on line/black) when digitalRead is LOW
    sensorStates[i] = !digitalRead(lineSensorPins[i]);
    if (sensorStates[i]) {
      // NOTE: We reverse the order (3-i) to map the sensor array {3,6,4,7} (L to R) to the bit pattern (Bit 3 to Bit 0)
      patternValue |= (1 << (3 - i)); 
    }
  }

  // Apply line following logic (same as previous version)
  switch (patternValue) {
    // --- Go Straight ---
    case 9:  // 1001 - Centered
      leftSpeed = MAX_SPEED;
      rightSpeed = MAX_SPEED;
      break;
    
    // --- Slight Correction ---
    case 13: // 1101 - Slightly right
    case 11: // 1011 - Slightly left
      leftSpeed = LESS_SPEED;
      rightSpeed = LESS_SPEED;
      break;
      
    // --- Sharp Right Turn ---
    case 12: // 1100 
    case 14: // 1110 
      leftSpeed = TURN_SPEED_FAST; 
      rightSpeed = TURN_SPEED_SLOW;
      break;
      
    // --- Sharp Left Turn ---
    case 3:  // 0011
    case 7:  // 0111
      leftSpeed = TURN_SPEED_SLOW; 
      rightSpeed = TURN_SPEED_FAST;
      break;
      
    // --- Lost/Stop ---
    case 15: // All black 
    case 0:  // All white
    default:
      leftSpeed = STOP_SPEED;
      rightSpeed = STOP_SPEED;
      break;
  }
  
  setMotorSpeeds(leftSpeed, rightSpeed);
}

/*
  Sideways movement logic: cycles between moving right and left.
  Uses the SIDEWAYS_DURATION timer.
*/
void sidewaysMovement() {
  // Check if it's time to change direction
  if (millis() - lastSidewaysChange >= SIDEWAYS_DURATION) {
    lastSidewaysChange = millis();
    isMovingRight = !isMovingRight; // Toggle direction
  }

  // Mecanum Sideways Movement Patterns:
  // D1M1 (Front Left), D1M2 (Rear Left), D2M1 (Front Right), D2M2 (Rear Right)
  
  if (isMovingRight) {
    // MOVE RIGHT: D1M1 FWD, D1M2 BACK, D2M1 BACK, D2M2 FWD
    setIndividualMotorSpeeds(
      SIDEWAYS_SPEED,     // D1 M1: FWD (+)
      -SIDEWAYS_SPEED,    // D1 M2: BACK (-)
      SIDEWAYS_SPEED,     // D2 M1: Input + (Result DXL - -> Physical BACK)
      -SIDEWAYS_SPEED     // D2 M2: Input - (Result DXL + -> Physical FWD)
    );
  } else {
    // MOVE LEFT: D1M1 BACK, D1M2 FWD, D2M1 FWD, D2M2 BACK
    setIndividualMotorSpeeds(
      -SIDEWAYS_SPEED,    // D1 M1: BACK (-)
      SIDEWAYS_SPEED,     // D1 M2: FWD (+)
      -SIDEWAYS_SPEED,    // D2 M1: Input - (Result DXL + -> Physical FWD)
      SIDEWAYS_SPEED      // D2 M2: Input + (Result DXL - -> Physical BACK)
    );
  }
}

/*
  Handle bump sensor events. Only triggers when reversing (STATE_OBSTACLE_REVERSING)
  to act as a rear safety stop.
*/
void handleBumpSensors() {
  readBumpSensors();
  
  // 1. SAFETY STOP: Only check bump sensors when the car is in a reversing state.
  if ((leftBumpState || rightBumpState) && currentState == STATE_OBSTACLE_REVERSING) {
    // Safety check - we hit something in the back while moving backward.
    currentState = STATE_BUMP_STOPPED;
    stopMotors();
    // No need for a timed recovery, the user must clear the path manually or press a button.
  }
  
  // 2. Clear STATE_BUMP_STOPPED: If the car was stopped by a bump and the bump is clear, 
  //    we transition back to reversing.
  if (currentState == STATE_BUMP_STOPPED) {
      if (!leftBumpState && !rightBumpState) {
          // Bump sensors are clear, resume reversing to clear the front obstacle
          currentState = STATE_OBSTACLE_REVERSING;
          setMotorSpeeds(REVERSE_SPEED, REVERSE_SPEED);
      } else {
          // Still bumped, remain stopped
          stopMotors();
      }
  }
  
  // Note: BUMP_RECOVERING state and logic is removed as it's not needed if 
  // the ultrasonic dictates when to stop reversing.
}

/*
  Handle ultrasonic obstacle detection.
  Logic: If distance <= 15cm, reverse. Reverse until distance > 20cm, then resume line following.
*/
void handleUltrasonic() {
  // 1. Check if it's time to read the sensor
  if (millis() - lastUltrasonicRead >= ULTRASONIC_READ_INTERVAL) {
    lastUltrasonicRead = millis();
    distance = readUltrasonic();
  }
  
  // 2. Main Logic - Only apply obstacle logic if we are supposed to be line following 
  //    or are currently reversing from an obstacle.

  // A. Trigger Reversing: If car is line following AND detects an obstacle too close.
  if (currentState == STATE_LINE_FOLLOWING && distance > 0 && distance <= OBSTACLE_REVERSE_TRIGGER_DISTANCE) {
    currentState = STATE_OBSTACLE_REVERSING;
    setMotorSpeeds(REVERSE_SPEED, REVERSE_SPEED);
  } 
  
  // B. Continue/Maintain Reversing: If car is already reversing, keep going 
  //    until the path is sufficiently clear.
  else if (currentState == STATE_OBSTACLE_REVERSING) {
    if (distance > OBSTACLE_RESUME_DISTANCE) {
      // Path is clear (> 20cm) - resume line following
      currentState = STATE_LINE_FOLLOWING;
    } else {
      // Path is NOT yet clear (<= 20cm) - maintain reverse speed.
      // NOTE: Motor speed is set once upon entering STATE_OBSTACLE_REVERSING,
      // and here we ensure we don't accidentally stop.
      setMotorSpeeds(REVERSE_SPEED, REVERSE_SPEED);
    }
  }
  
  // C. Safety Stop / Bump Check
  // The handleBumpSensors() function is called in loop() and will check for 
  // rear bumps only when in STATE_OBSTACLE_REVERSING and override the state to STATE_BUMP_STOPPED.
}


void setup() {
  // CRITICAL: Ensure your Serial Monitor is set to 115200 baud!
  Serial.begin(115200);
  Serial.println("--- Combined Car Control Initialized ---");
  
  // --- Sensor Pin Setup ---
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(lineSensorPins[i], INPUT);
  }
  pinMode(contactLeftPin, INPUT_PULLUP);
  pinMode(contactRightPin, INPUT_PULLUP);
  // Ultrasonic (A0/A1)
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // --- Dynamixel Motor Setup ---
  DxlMaster.begin(DXL_BAUD);
  drive1.protocolVersion(1);
  drive2.protocolVersion(1);
  
  // Configure Motors
  drive1.write(MOTOR_MODE, (uint8_t)2); // Speed Mode
  drive1.write(ENCODER_MODE, (uint8_t)ENCODER_MODE_AB);
  drive1.write(MAX_POWER_L, (uint16_t)1000);
  drive2.write(MOTOR_MODE, (uint8_t)2);
  drive2.write(ENCODER_MODE, (uint8_t)ENCODER_MODE_AB);
  drive2.write(MAX_POWER_L, (uint16_t)1000);

  // Set PID for speed control (Example values)
  drive1.write(PID_P, (uint8_t)300);
  drive1.write(PID_I, (uint8_t)50);
  drive1.write(PID_D, (uint8_t)10);
  drive2.write(PID_P, (uint8_t)300);
  drive2.write(PID_I, (uint8_t)50);
  drive2.write(PID_D, (uint8_t)10);
  
  // Initialize SPI for keyboard
  SPI.begin();
  
  delay(1000);
  Serial.println("Ready. Car control system is active. Press Button 1 (Line) or 2 (Sideways).");
}

void loop() {
  
  // 1. Handle button input (Runs every 50ms)
  if (millis() - lastButtonCheck >= BUTTON_CHECK_INTERVAL) {
    lastButtonCheck = millis();
    handleButtonInput();
  }
  
  // 2. Update sensor readings and main logic (Runs every 50ms)
  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = millis();
    
    // Read ultrasonic and handle obstacle states (critical logic for line following)
    handleUltrasonic(); 
    
    // Handle bump sensors (critical logic for reversing safety)
    handleBumpSensors();

    // Run the primary control logic based on the determined state
    switch (currentState) {
      case STATE_IDLE:
      case STATE_BUMP_STOPPED:
        stopMotors();
        break;
        
      case STATE_LINE_FOLLOWING:
        lineFollowing(); 
        break;
        
      case STATE_SIDEWAYS_TEST: 
        sidewaysMovement();
        break;
        
      case STATE_OBSTACLE_REVERSING:
        // Already set to REVERSE_SPEED in handleUltrasonic() or handleBumpSensors()
        // We ensure motors are running the reverse speed just in case.
        setMotorSpeeds(REVERSE_SPEED, REVERSE_SPEED);
        break;
        
      // STATE_BUMP_RECOVERING state is removed
    }
  }
}
