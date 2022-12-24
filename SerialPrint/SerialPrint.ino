void
setup() {
  Serial.begin(38400);
  Serial.print("Press any key: ");
}

void
loop() {
  if(Serial.available()) {
    Serial.println(Serial.read());
  }
}
