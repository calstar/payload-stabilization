<strong>
  
  #include <SD.h>
  #include <SPI.h>
  
  int CS_PIN = 10 ; // CS pin
  String Data ="" ; 
  File myFile;
  
  void setupSD() {
  Serial.begin(9600);
  pinMode(CS_PIN, OUTPUT);
    initializeSD();                
    createFile("Sample.csv");    
   /*
    for( int i=0 ; i<1000; i++ )  {     
     writeToFile() ;              
    } */  
     //myFile.close();            
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
 
 int writeToFile(myFile, yprs) {  
  if (myFile) {
    Data= String(yprs[0])+","+String(yprs[1])+","+String(yprs[2])+","+String(yprs[3])+","+String(yprs[4])+","+String(yprs[5]);
             
    myFile.println(Data);  // Write this Data to SD
    
    Serial.println(Data);
    delay(10);
    return 1;
  } else {
    Serial.println("Couldn't write to file");\
    return 0;
  }
 }  
</strong>
