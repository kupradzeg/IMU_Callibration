// Include required libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// Initialize variables
float roll_angle_magnitude; // Magnitude of roll angle from accelerometer
float pitch_angle_magnitude; // Magnitude of pitch angle from accelerometer
float filtered_roll_angle_previous = 0; // Filtered previous roll angle
float filtered_roll_angle_new; // Filtered new roll angle
float filtered_pitch_angle_previous = 0; // Filtered previous pitch angle
float filtered_pitch_angle_new; // Filtered new pitch angle
float gyro_roll_angle = 0; // Roll angle from gyroscope
float gyro_pitch_angle = 0; // Pitch angle from gyroscope
float roll_angle; // Final roll angle after combining accelerometer and gyroscope data
float pitch_angle; // Final pitch angle after combining accelerometer and gyroscope data
float roll_angle_radians; // Roll angle in radians
float pitch_angle_radians; // Pitch angle in radians
float Xm; // Magnetometer X component
float Ym; // Magnetometer Y component
float heading; // Magnetometer heading angle
float dt; // Time since last loop iteration
unsigned long previous_millis;

#define BNO055_SAMPLERATE_DELAY_MS (100)

// Create an instance of the BNO055 sensor
Adafruit_BNO055 imu_sensor = Adafruit_BNO055();

void setup() {
// Initialize serial communication
Serial.begin(115200);

// Initialize the BNO055 sensor
imu_sensor.begin();
delay(1000);
int8_t temp = imu_sensor.getTemp();
imu_sensor.setExtCrystalUse(true);

// Initialize previous millis value for loop timing
previous_millis = millis();
}

void loop() {
// Get calibration data from the sensor
uint8_t system, gyro, accel, mg = 0;
imu_sensor.getCalibration(&system, &gyro, &accel, &mg);

// Get accelerometer, gyroscope, and magnetometer data from the sensor
imu::Vector<3> acc = imu_sensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> gyr = imu_sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
imu::Vector<3> mag = imu_sensor.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

// Calculate roll and pitch angles from accelerometer data
roll_angle_magnitude = -atan2(acc.x()/9.8, acc.z()/9.8) / 2 / 3.141592654 * 360;
pitch_angle_magnitude = -atan2(acc.y()/9.8, acc.z()/9.8) / 2 / 3.141592654 * 360;

// Filter the roll and pitch angles from accelerometer data
filtered_pitch_angle_new = 0.95 * filtered_pitch_angle_previous + 0.05 * pitch_angle_magnitude;
filtered_roll_angle_new = 0.95 * filtered_roll_angle_previous + 0.05 * roll_angle_magnitude;

// Calculate roll and pitch angles from gyroscope data
dt = (millis() - previous_millis) / 1000.0;
previous_millis = millis();
roll_angle = (roll_angle + gyr.y() * dt) * 0.95 + roll_angle_magnitude * 0.05;
pitch_angle = (pitch_angle - gyr.x() * dt) * 0.95 + pitch_angle_magnitude * 0.05;

// Update the roll and pitch angle from gyroscope data
gyro_roll_angle += gyr.y() * dt;
gyro_pitch_angle -= gyr.x() * dt;

// Convert roll and pitch angles to radians for magnetometer calculations
roll_angle_radians = roll_angle / 360.0 * (2 * 3.14);
pitch_angle_radians = pitch_angle / 360.0 * (2 * 3.14);

// Calculate the X and Y components of the magnetometer data
Xm = mag.x() * cos(roll_angle_radians) - mag.y() * sin(pitch_angle_radians) * sin(roll_angle_radians) + mag.z() * cos(pitch_angle_radians) * sin(roll_angle_radians);
Ym = mag.y() * cos(pitch_angle_radians) + mag.z() * sin(pitch_angle_radians);

// Calculate the heading angle from the magnetometer data
heading = atan2(Ym, Xm) / (2 * 3.14) * 360;

// Print the roll angle, pitch angle, heading, and calibration data
Serial.print("Roll angle: ");
Serial.print(roll_angle);
Serial.print(" degrees, Pitch angle: ");
Serial.print(pitch_angle);
Serial.print(" degrees, Heading: ");
Serial.print(heading);
Serial.print(" degrees, Accelerometer calibration: ");
Serial.print(accel);
Serial.print(", Gyroscope calibration: ");
Serial.print(gyro);
Serial.print(", Magnetometer calibration: ");
Serial.print(mg);
Serial.print(", System calibration: ");
Serial.println(system);

// Update the filtered roll and pitch angles
filtered_roll_angle_previous = filtered_roll_angle_new;
filtered_pitch_angle_previous = filtered_pitch_angle_new;

// Wait for the specified delay before starting the next loop iteration
delay(BNO055_SAMPLERATE_DELAY_MS);
}