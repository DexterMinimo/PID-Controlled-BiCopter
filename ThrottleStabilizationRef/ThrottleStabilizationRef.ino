#include <Wire.h>
#include <MPU6050.h>



// Set the minimum and maximum values for the ESCs
#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000

// Set the target pitch angle for stabilization (in degrees)
#define TARGET_PITCH_ANGLE 5

//// Create an instance of the MPU6050 class for the IMU sensor
MPU6050 mpu;
//
void setup() {
 Serial.begin(115200);
  Wire.begin();
//
//  // Initialize the MPU6050 sensor
mpu.initialize();
//
//  // Set the full scale range for the gyro and accelerometer to +/- 250 degrees/sec and +/- 2g, respectively
//  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
//  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//
//  // Set the digital low pass filter (DLPF) to reduce noise in the sensor readings
//  mpu.setDLPFMode(MPU6050_DLPF_BW_20);

  // Set the PWM frequency for the motor control pins to 490 Hz
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  TCCR2B = TCCR2B & 0b11111000 | 0x01;
}

void loop() {
  // Read the accelerometer and gyro data from the MPU6050 sensor
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Calculate the pitch angle from the accelerometer data
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  // Calculate the error between the target pitch angle and the current pitch angle
  float error = TARGET_PITCH_ANGLE - pitch;

  // Calculate the adjustment to be made to the throttle
  float adjustment = error * 0.1;

  // Calculate the new throttle value for the motors
  int throttle1 = constrain(MIN_THROTTLE + adjustment, MIN_THROTTLE, MAX_THROTTLE);
  int throttle2 = constrain(MIN_THROTTLE - adjustment, MIN_THROTTLE, MAX_THROTTLE);

  // Set the duty cycle of the ESCs to the throttle values for the motors
  analogWrite(9, 200); // Motor 1 (Left)
  analogWrite(10, 200); // Motor 2 (Right)

  delay(100);
}
