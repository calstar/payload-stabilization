#include "I2Cdev.h"
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if  I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL
#define OUTPUT_READABLE_WORLDACCEL

#define INTERRUPT1_PIN 2
#define LED_PIN 13
#define SERVO1_PIN 4
#define SERVO2_PIN 5
#define SERVO3_PIN 6

bool blinkState = false;

bool dmp1Ready = false;
uint8_t mpu1IntStatus;
uint8_t dev1Status;
uint16_t packet1Size;
uint16_t fifo1Count;
uint8_t fifo1Buffer[64];

Servo servo1;
Servo servo2;
Servo servo3;

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];
float servo1init = 85;
float servo2init = 95;
float servo3init = 95;
float ang = 60;

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

int relay = 9;
bool launched = false;
bool unsafe = false;
bool completed = false;

volatile bool mpu1Interrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmp1DataReady() {
    mpu1Interrupt = true;
}




void setup() {
  // put your setup code here, to run once:
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  mpu1.initialize();
  pinMode(INTERRUPT1_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu1.testConnection() ? F("MPU1 connection successful") : F("MPU1 connection failed"));

  Serial.println(F("\nSend any character to begin DMP programming and demo: "));

  Serial.println(F("Initializing MPU1 DMP..."));
  dev1Status = mpu1.dmpInitialize();

  if (dev1Status == 0) {
    mpu1.CalibrateAccel(6);
    mpu1.CalibrateGyro(6);
    mpu1.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu1.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT1_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT1_PIN), dmp1DataReady, RISING);
    mpu1IntStatus = mpu1.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmp1Ready = true;
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(dev1Status);
      Serial.println(F(")"));
  }
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo1.write(servo1init);
  servo2.write(servo2init);
  servo3.write(servo3init);
  delay(100);
  digitalWrite(relay, LOW);
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!launched) {
    
  }
}
