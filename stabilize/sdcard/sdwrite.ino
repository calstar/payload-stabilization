<strong>
  
  #include <SD.h>
  #include <SPI.h>
  
  int CS_PIN = 10 ; // CS pin
  int Sensor0 = 0 ;
  int Sensor1 = 1 ;
  String Data ="" ; 
  File myFile;
  void setup() {
  Serial.begin(9600);
  pinMode(CS_PIN, OUTPUT);
    initializeSD();                
    createFile("Sample2.csv");    
   
    for( int i=0 ; i<6; i++ )  {     
     writeToFile() ;              
    }   
     myFile.close();            
  } 
  
 void initializeSD() {
  Serial.println("Initializing SD card...");
  pinMode(CS_PIN, OUTPUT);
  if (SD.begin()) {
    Serial.println("SD card is ready to use.");
  } else {
    Serial.println("SD card initialization failed");
    return;
  }
 }  
 
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
 
 int writeToFile() {  
  if (myFile) {
    Sensor0 = analogRead(A0) ; 
    Sensor1 = analogRead(A1) ; 
    Data= String(Sensor0)+","+String(Sensor1) ;
             
    myFile.println(Data);  // Write this Data to SD
    
    Serial.println(Data);
    delay(100);    
    return 1;
  } else {
    Serial.println("Couldn't write to file");
    return 0;
  }
 }  
   
  void loop() {  }</strong>
