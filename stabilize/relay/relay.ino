int relay = 5; //13;
bool launched = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200); //(115200); // can change this based on what's needed
  pinMode (relay, OUTPUT);
  
  // bool active = false;
}

void loop() {
  // put your main code here, to run repeatedly:
   
  if (launched) {
    digitalWrite(relay, HIGH);
    launched = false;
    Serial.print(false);
  } else {
    digitalWrite(relay, LOW);
    launched = true;
    Serial.print(true);
  }
  delay(1000);
}
