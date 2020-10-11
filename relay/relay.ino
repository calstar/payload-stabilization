int relay = 13;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // can change this based on what's needed
  pinMode (relay, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool launched = false;
  // bool active = false;
  if (!launched) {
    digitalWrite(relay, HIGH);
    launched = true;
  }
  
}
