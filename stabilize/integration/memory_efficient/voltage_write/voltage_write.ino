#include <SD.h>
#include "Wire.h"

int VOLTAGE_PIN = A0;

float voltage;
int CS_PIN = 10;
String Data1 = "";
String Data2 = "";
File myFile;
String myFileName = "Voltage.csv";
int counter = 0;
int relay = 9;

void setup() {
  // put your setup code here, to run once:
  digitalWrite(relay, HIGH);
  Serial.begin(115200);
  Serial.println("HI");
  initializeSD();
  myFile = SD.open(myFileName, FILE_WRITE);
  Serial.println(myFile);
  if (myFile) {
    Serial.println("File created successfully.");
    myFile.println(String("Start"));
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

void loop() {
  // put your main code here, to run repeatedly:
  voltage = analogRead(VOLTAGE_PIN) * 5.0/1023;
  Serial.print("Voltage = ");
  Serial.println(voltage);
  if (myFile) {
        myFile.println(voltage);
        if (counter >= 50) {
          myFile.close();
          myFile = SD.open(myFileName, FILE_WRITE);
          //delay(10);
        }
        counter++;
      } else {
        Serial.println("Couldn't write to file");
      }
  delay(10);
}
