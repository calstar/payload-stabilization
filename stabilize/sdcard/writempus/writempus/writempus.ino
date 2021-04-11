#include <Wire.h>
const int MPU1 = 0x68; // MPU6050 I2C address
const int MPU2 = 0x69; // MPU6050 I2C address

float yprs[6];
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

#include <SD.h>
#include <SPI.h>
  
int CS_PIN = 10 ; // CS pin
String Data ="" ; 
File myFile;
int counter = 0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); // Initialize comunication
  Wire.beginTransmission(MPU1); // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B); // Talk to the register 6B
  Wire.write(0x00); // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); //end the transmission
  
  Wire.begin(); // Initialize comunication
  Wire.beginTransmission(MPU2); // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B); // Talk to the register 6B
  Wire.write(0x00); // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); //end the transmission
  
  Serial.begin(9600);
  pinMode(CS_PIN, OUTPUT);
  initializeSD();                
  //createFile("Sample.csv");
  myFile = SD.open("Sample3.csv", FILE_WRITE);
  if (myFile) {
    Serial.println("File created successfully.");
  } else {
    Serial.println("Error while creating file.");
  }
}

void initializeSD() {
  Serial.println("Initializing SD card...");
  pinMode(CS_PIN, OUTPUT);
  if (SD.begin()) {
    Serial.println("SD card is ready to use.");
  } else {
    Serial.println("SD card initialization failed");
  }
}  
 /*
 int createFile(char filename[]) {
  myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {
    Serial.println("File created successfully.");
    return 1;
  } else {
    Serial.println("Error while creating file.");
    return 0;
  }
 }  
 */
void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU1);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU1, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
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
  
  yprs[0] = yaw1;
  yprs[1] = pitch1;
  yprs[2] = roll1;
  yprs[3] = yaw2;
  yprs[4] = pitch2;
  yprs[5] = roll2;

  if (myFile) {
    Data= String(yprs[0])+","+String(yprs[1])+","+String(yprs[2])+","+String(yprs[3])+","+String(yprs[4])+","+String(yprs[5])+","+"test1";
             
    myFile.println(Data);  // Write this Data to SD
    
    Serial.println(Data);
    if (counter % 10 == 0) {
      myFile.close();
      myFile = SD.open("Sample3.csv", FILE_WRITE);
    }
    delay(10);
    counter++;
  } else {
    Serial.println("Couldn't write to file");\
  }
}
