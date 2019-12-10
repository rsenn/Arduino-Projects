/*
    MeasureCapacitor

    Measures the capacitance between D7 and A2.
    Prints the result to the Serial Monitor.

    This example code is in the public domain.
*/
#include <Capacitor.h>
#include <LiquidCrystal.h>


// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(12, 11, 5,4,3,2);


// Capacitor under test.
// Note that for electrolytics the first pin (in this case D7)
// should be positive, the second (in this case A2) negative.
Capacitor cap1(7,A2);

void setup() {
   // set up the LCD's number of columns and rows:
  lcd.begin(20, 2);
  Serial.begin(9600);
}

void print(float val) {
  Serial.print(val);
  lcd.write(val);
}

void println(const char* str) {
  Serial.println(str);
  lcd.write(str);
  lcd.write("\n");
}

void loop() {
  float c = cap1.Measure();
  Serial.print(c);  // Measure the capacitance (in pF), print to Serial Monitor
  Serial.println("pF");
  delay(1000);                     // Wait for 1 second, then repeat
}
