void setup() {
  setupMPUs();
  setupSDs();

}

File myFile;

void loop() {
  update();
  writeToFile(myFile, yprs);
}
