//#include <Wire.h>

//float elapsedTime, currentTime, previousTime;
//int c = 0;

#include <SD.h>
//#include <SPI.h>
  
int CS_PIN = 10 ; // CS pin
String Data ="" ; 
File myFile;
int counter = 0;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  pinMode(CS_PIN, OUTPUT);
  initializeSD();                
  //createFile("Sample.csv");
  myFile = SD.open("Sample2.csv", FILE_WRITE);
  Serial.println(myFile);
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

  if (myFile) {
    Data= String(1)+","+String(2)+","+String(3)+","+String(4)+","+String(5)+","+String(6)+","+"test1";
             
    myFile.println(Data);  // Write this Data to SD
    
    Serial.println(Data);
    if (counter >= 10) {
      myFile.close();
      return;//myFile = SD.open("Sample.csv", FILE_WRITE);
    }
    delay(10);
    counter++;
  } else {
    Serial.println("Couldn't write to file");\
  }
}
