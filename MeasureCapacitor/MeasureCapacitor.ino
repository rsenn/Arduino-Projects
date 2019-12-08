/*
    MeasureCapacitor

    Measures the capacitance between D7 and A2.
    Prints the result to the Serial Monitor.

    This example code is in the public domain.
*/
#include <Capacitor.h>

// Capacitor under test.
// Note that for electrolytics the first pin (in this case D7)
// should be positive, the second (in this case A2) negative.
Capacitor cap1(7,A2);

void setup() {
  Serial.begin(9600);
}

void loop() {
  float c = cap1.Measure();
  Serial.print(c);  // Measure the capacitance (in pF), print to Serial Monitor
  Serial.println("pF");
  delay(1000);                     // Wait for 1 second, then repeat
}
