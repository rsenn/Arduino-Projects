  const int resistorPin = A0; // initalizes the pin to attach the resistors
  float resistorValue = 0; // variable to store the resistors read values
  float Vout = 0; // variable to store the V out from the resistor
  float referenceResistor = 4700; // set your reference resistor value here
  float measuredResistance = 0;
  float mappedResistorValue = 0;
  int Vin = 5.08;
  
  void setup() {
  Serial.begin(9600);
  
  }
  
  void loop() {
  resistorValue = analogRead(resistorPin);
  
  mappedResistorValue = map(resistorValue, 0, 1023, 0, 255);
  Serial.println(resistorValue);
  Vout = ( resistorValue * Vin) / 1023.0;
  measuredResistance = (Vout * 4700) / (Vin- Vout);
  Serial.println(measuredResistance);
  delay(250);
  
  }
