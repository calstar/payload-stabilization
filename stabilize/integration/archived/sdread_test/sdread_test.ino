
  #include <SD.h>
  #include <SPI.h>
  
  File myFile;  
  int CS_PIN = 10 ; // CS pin
  
  void setup() {
  Serial.begin(9600);
  pinMode(CS_PIN, OUTPUT);
  
  initializeSD();  
  Read_Data() ;    
  }
  void loop() {  }  //==============
  void initializeSD() {
  Serial.println("Initializing SD card...");
  pinMode(CS_PIN, OUTPUT);
  if (SD.begin()) {
    Serial.println("SD card is ready to use.");
  } else {
    Serial.println("SD card initialization failed");
    return;
  }
 }  // =============================
  void Read_Data() {
    myFile = SD.open("Sample3.csv"); 
  if (myFile) {
    Serial.println("Sample3.csv:");   
    // read file until no more
    while (myFile.available()) {
     long Data = myFile.read() ;
     Serial.write(Data);
    }
   
    myFile.close();   // close the file:
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
    }
 } // =================
