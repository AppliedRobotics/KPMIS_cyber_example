#include <SPI.h>
#include <DxlMaster2.h>
#include <Wire.h>
#include <ICM20948_WE.h>
// Note: ardu_drive.hpp and ardu_uno_keyboard.hpp are removed

#define ICM20948_ADDR 0x68
#define PC_SERIAL Serial

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

void setup(void)
{
  // Initialize I2C communication for the IMU
  Wire.begin();
  
  // Initialize Serial communication for output
  Serial.begin(1000000); // Using the same baud rate as the original code

  // Initialize the IMU
  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
  }
  else{
    Serial.println("ICM20948 is connected");
  }

  // Calibrate the IMU - essential for accurate gyro readings
  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
  Serial.println("Done!"); 

  // Set IMU parameters (e.g., Gyroscope Range, Digital Low-Pass Filter, Sample Rate)
  // These settings are typical and may need fine-tuning for specific applications
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_2000); // Set to max range
  myIMU.setGyrDLPF(ICM20948_DLPF_6);    
  myIMU.setGyrSampleRateDivider(10);
}

void loop(void)
{
  // Read all sensor data from the chip
  myIMU.readSensor();

  // --- Read and Print Angular Velocities (Gyroscope) ---
  xyzFloat gyrVal;
  
  // Get and print angular velocity values in degrees/second
  myIMU.getGyrValues(&gyrVal); 
  Serial.println("Angular velocity (deg/s - X, Y, Z axes):");
  Serial.print("X (Roll Rate): ");
  Serial.print(gyrVal.x);
  Serial.print("   Y (Pitch Rate): ");
  Serial.print(gyrVal.y);
  Serial.print("   Z (Yaw Rate): ");
  Serial.println(gyrVal.z);

  Serial.println("*************************************");
 
  // Wait before the next reading
  delay(500);
}