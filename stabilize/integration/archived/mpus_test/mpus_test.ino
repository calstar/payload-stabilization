
/*
Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
by Dejan, https://howtomechatronics.com
*/
#include <Wire.h>
const int MPU1 = 0x68; // MPU6050 I2C address
const int MPU2 = 0x69; // MPU6050 I2C address
#define ADO_HIGH_PIN 7
float AccX1, AccY1, AccZ1;
float GyroX1, GyroY1, GyroZ1;
float accAngleX1, accAngleY1, gyroAngleX1, gyroAngleY1, gyroAngleZ1;
float roll1, pitch1, yaw1;
float AccErrorX1, AccErrorY1, GyroErrorX1, GyroErrorY1, GyroErrorZ1;

float AccX2, AccY2, AccZ2;
float GyroX2, GyroY2, GyroZ2;
float accAngleX2, accAngleY2, gyroAngleX2, gyroAngleY2, gyroAngleZ2;
float roll2, pitch2, yaw2;
float AccErrorX2, AccErrorY2, GyroErrorX2, GyroErrorY2, GyroErrorZ2;

float elapsedTime, currentTime, previousTime;
int c = 0;
void setup() {
Serial.begin(115200);
//digitalWrite(ADO_HIGH_PIN, HIGH);
Serial.println('0');
Wire.begin(); // Initialize comunication
Serial.println('1');
Wire.beginTransmission(MPU1); // Start communication with MPU6050 // MPU=0x68
Serial.println('2');
Wire.write(0x6B); // Talk to the register 6B
Serial.println('3');
Wire.write(0x00); // Make reset - place a 0 into the 6B register
Serial.println('4');
Wire.endTransmission(true); //end the transmission
Serial.println('e');

Wire.begin(); // Initialize comunication
Wire.beginTransmission(MPU2); // Start communication with MPU6050 // MPU=0x68
Wire.write(0x6B); // Talk to the register 6B
Wire.write(0x00); // Make reset - place a 0 into the 6B register
Wire.endTransmission(true); //end the transmission

/*
// Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
Wire.beginTransmission(MPU);
Wire.write(0x1C); //Talk to the ACCEL_CONFIG register (1C hex)
Wire.write(0x10); //Set the register bits as 00010000 (+/- 8g full scale range)
Wire.endTransmission(true);
// Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
Wire.beginTransmission(MPU);
Wire.write(0x1B); // Talk to the GYRO_CONFIG register (1B hex)
Wire.write(0x10); // Set the register bits as 00010000 (1000deg/s full scale)
Wire.endTransmission(true);
delay(20);
*/
// Call this function if you need to get the IMU error values for your module
//calculate_IMU_error();
delay(20);
}
void loop() {
// === Read acceleromter data === //
//Serial.println('5');
Wire.beginTransmission(MPU1);
//Serial.println('6');
Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
Wire.endTransmission(false);
//Serial.println('h');
Wire.requestFrom(MPU1, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
//Serial.println('s');
//For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
AccX1 = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
AccY1 = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
AccZ1 = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
// Calculating Roll and Pitch from the accelerometer data
accAngleX1 = (atan(AccY1 / sqrt(pow(AccX1, 2) + pow(AccZ1, 2))) * 180 / PI) + 0.27;// AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
accAngleY1 = (atan(-1 * AccX1 / sqrt(pow(AccY1, 2) + pow(AccZ1, 2))) * 180 / PI) + 0.35;// AccErrorY ~(-1.58)
// === Read gyroscope data === //
previousTime = currentTime; // Previous time is stored before the actual time read
currentTime = millis(); // Current time actual time read
elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
Wire.beginTransmission(MPU1);
Wire.write(0x43); // Gyro data first register address 0x43
Wire.endTransmission(false);
Wire.requestFrom(MPU1, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
GyroX1 = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
GyroY1 = (Wire.read() << 8 | Wire.read()) / 131.0;
GyroZ1 = (Wire.read() << 8 | Wire.read()) / 131.0;
// Correct the outputs with the calculated error values
GyroX1 = GyroX1 - 2.20; // + 0.56; // GyroErrorX ~(-0.56)
GyroY1 = GyroY1 - 5.64;// - 5.91; // - 2; // GyroErrorY ~(2)
GyroZ1 = GyroZ1 - 0.20;// + 0.08; // + 0.79; // GyroErrorZ ~ (-0.8)
// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
gyroAngleX1 = gyroAngleX1 + GyroX1 * elapsedTime; // deg/s * s = deg
gyroAngleY1 = gyroAngleY1 + GyroY1 * elapsedTime;
yaw1 = yaw1 + GyroZ1 * elapsedTime;
// Complementary filter - combine acceleromter and gyro angle values
roll1 = 0.96 * gyroAngleX1 + 0.04 * accAngleX1;
pitch1 = 0.96 * gyroAngleY1 + 0.04 * accAngleY1;

// Do the same for MPU2 //
// === Read acceleromter data === //

Wire.beginTransmission(MPU2);
Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
Wire.endTransmission(false);
Wire.requestFrom(MPU2, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
//For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
AccX2 = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
AccY2 = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
AccZ2 = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
// Calculating Roll and Pitch from the accelerometer data
accAngleX2 = (atan(AccY2 / sqrt(pow(AccX2, 2) + pow(AccZ2, 2))) * 180 / PI) - 1.90;// - 0.49;// AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
accAngleY2 = (atan(-1 * AccX2 / sqrt(pow(AccY2, 2) + pow(AccZ2, 2))) * 180 / PI) - 2.80;// - 3.07;// AccErrorY ~(-1.58)
// === Read gyroscope data === //
previousTime = currentTime; // Previous time is stored before the actual time read
currentTime = millis(); // Current time actual time read
elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
Wire.beginTransmission(MPU2);
Wire.write(0x43); // Gyro data first register address 0x43
Wire.endTransmission(false);
Wire.requestFrom(MPU2, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
GyroX2 = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
GyroY2 = (Wire.read() << 8 | Wire.read()) / 131.0;
GyroZ2 = (Wire.read() << 8 | Wire.read()) / 131.0;
// Correct the outputs with the calculated error values
GyroX2 = GyroX2 - 0.43;// - 0.28;// - 2.03; // + 0.56; // GyroErrorX ~(-0.56)
GyroY2 = GyroY2 - 0.07;// - 0.24;// - 5.91; // - 2; // GyroErrorY ~(2)
GyroZ2 = GyroZ2 - 0.67;// - 0.46;// + 0.08; // + 0.79; // GyroErrorZ ~ (-0.8)
// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
gyroAngleX2 = gyroAngleX2 + GyroX2 * elapsedTime; // deg/s * s = deg
gyroAngleY2 = gyroAngleY2 + GyroY2 * elapsedTime;
yaw2 = yaw2 + GyroZ2 * elapsedTime;
// Complementary filter - combine acceleromter and gyro angle values
roll2 = 0.96 * gyroAngleX2 + 0.04 * accAngleX2;
pitch2 = 0.96 * gyroAngleY2 + 0.04 * accAngleY2;

// Print the values on the serial monitor
/*
Serial.print("1. ");
Serial.print(roll1);
Serial.print("/");
Serial.print(pitch1);
Serial.print("/");
Serial.println(yaw1);
*/
Serial.print("2. ");
Serial.print(roll2);
Serial.print("/");
Serial.print(pitch2);
Serial.print("/");
Serial.println(yaw2);

delay(10);
//uncomment to get error values and add to /subtract from the values above based on error 
//calculate_IMU_error();

}

void calculate_IMU_error() {
// We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
// Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
// Read accelerometer values 200 times
c = 0;
while (c < 200) {
Wire.beginTransmission(MPU1);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU1, 6, true);
AccX1 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
AccY1 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
AccZ1 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
// Sum all readings
AccErrorX1 = AccErrorX1 + ((atan((AccY1) / sqrt(pow((AccX1), 2) + pow((AccZ1), 2))) * 180 / PI));
AccErrorY1 = AccErrorY1 + ((atan(-1 * (AccX1) / sqrt(pow((AccY1), 2) + pow((AccZ1), 2))) * 180 / PI));
c++;
}
//Divide the sum by 200 to get the error value
AccErrorX1 = AccErrorX1 / 200;
AccErrorY1 = AccErrorY1 / 200;
c = 0;
// Read gyro values 200 times
while (c < 200) {
Wire.beginTransmission(MPU1);
Wire.write(0x43);
Wire.endTransmission(false);
Wire.requestFrom(MPU1, 6, true);
GyroX1 = Wire.read() << 8 | Wire.read();
GyroY1 = Wire.read() << 8 | Wire.read();
GyroZ1 = Wire.read() << 8 | Wire.read();
// Sum all readings
GyroErrorX1 = GyroErrorX1 + (GyroX1 / 131.0);
GyroErrorY1 = GyroErrorY1 + (GyroY1 / 131.0);
GyroErrorZ1 = GyroErrorZ1 + (GyroZ1 / 131.0);
c++;
}
//Divide the sum by 200 to get the error value
GyroErrorX1 = GyroErrorX1 / 200;
GyroErrorY1 = GyroErrorY1 / 200;
GyroErrorZ1 = GyroErrorZ1 / 200;

c = 0;
while (c < 200) {
Wire.beginTransmission(MPU2);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU2, 6, true);
AccX2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
AccY2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
AccZ2 = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
// Sum all readings
AccErrorX2 = AccErrorX2 + ((atan((AccY2) / sqrt(pow((AccX2), 2) + pow((AccZ2), 2))) * 180 / PI));
AccErrorY2 = AccErrorY2 + ((atan(-1 * (AccX2) / sqrt(pow((AccY2), 2) + pow((AccZ2), 2))) * 180 / PI));
c++;
}
//Divide the sum by 200 to get the error value
AccErrorX2 = AccErrorX2 / 200;
AccErrorY2 = AccErrorY2 / 200;
c = 0;
// Read gyro values 200 times
while (c < 200) {
Wire.beginTransmission(MPU2);
Wire.write(0x43);
Wire.endTransmission(false);
Wire.requestFrom(MPU2, 6, true);
GyroX2 = Wire.read() << 8 | Wire.read();
GyroY2 = Wire.read() << 8 | Wire.read();
GyroZ2 = Wire.read() << 8 | Wire.read();
// Sum all readings
GyroErrorX2 = GyroErrorX2 + (GyroX2 / 131.0);
GyroErrorY2 = GyroErrorY2 + (GyroY2 / 131.0);
GyroErrorZ2 = GyroErrorZ2 + (GyroZ2 / 131.0);
c++;
}
//Divide the sum by 200 to get the error value
GyroErrorX2 = GyroErrorX2 / 200;
GyroErrorY2 = GyroErrorY2 / 200;
GyroErrorZ2 = GyroErrorZ2 / 200;


// Print the error values on the Serial Monitor
Serial.print("AccErrorX1: ");
Serial.println(AccErrorX1);
Serial.print("AccErrorY1: ");
Serial.println(AccErrorY1);
Serial.print("GyroErrorX1: ");
Serial.println(GyroErrorX1);
Serial.print("GyroErrorY1: ");
Serial.println(GyroErrorY1);
Serial.print("GyroErrorZ1: ");
Serial.println(GyroErrorZ1);

Serial.print("AccErrorX2: ");
Serial.println(AccErrorX2);
Serial.print("AccErrorY2: ");
Serial.println(AccErrorY2);
Serial.print("GyroErrorX2: ");
Serial.println(GyroErrorX2);
Serial.print("GyroErrorY2: ");
Serial.println(GyroErrorY2);
Serial.print("GyroErrorZ2: ");
Serial.println(GyroErrorZ2);
}
