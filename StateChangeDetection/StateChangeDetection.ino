const int buttonPin = 3;
const int ledPin = 13;
const int NUM_ANALOG = 2;
int buttonPushCounter = 0;
int buttonState = 0;
int lastButtonState = 0;
char buttonStates[4] = {0, 0, 0, 0};
char lastButtonStates[4] = {0, 0, 0, 0};
#include <AnalogButtons.h>
#define ANALOG_PIN A1

void
setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  analogReference(DEFAULT); 
  Serial.begin(38400);
}

int
getButtonBits() {
  int i;
  int ret = 0;
  int changed = 0;
  for(i = 0; i < 4; i++) {
    buttonState = buttonStates[i] = digitalRead(buttonPin + i);
    lastButtonState = lastButtonStates[i];
    if(buttonState != lastButtonState) {
      changed = 1;
      if(buttonState == LOW)
        buttonPushCounter++;
      lastButtonStates[i] = buttonStates[i];
    }
  }
  ret = 0;
  for(i = 0; i < 4; i++) {
    ret |= (buttonStates[i] == LOW ? 1 : 0) << i;
  }
  return ret;
}
void b1Click() { Serial.println("button 1 clicked"); } void b1Hold() { Serial.println("button 1 held"); }
void b2Click() { Serial.println("button 2 clicked"); } void b2Hold() { Serial.println("button 2 held"); }
void b3Click() { Serial.println("button 3 clicked"); } void b3Hold() { Serial.println("button 3 held"); }
void b4Click() { Serial.println("button 4 clicked"); } void b4Hold() { Serial.println("button 4 held"); }


AnalogButtons analogButtons(ANALOG_PIN, INPUT);

Button b1 = Button(1001, &b1Click, &b1Hold);
Button b2 = Button(500, &b2Click, &b2Hold);
Button b3 = Button(250, &b3Click, &b3Hold);
Button b4 = Button(125, &b4Click, &b4Hold);



void
loop() {
  int i;
  int changed = 0;
  unsigned  int an[3];
  static int prevAn[3], prevButtons;
  static float prevVoltage[3];
  int buttons = getButtonBits();
  if(1 || prevButtons != buttons) {
    for(i = 0; i < NUM_ANALOG; i++) {
      an[i] = analogRead(A1 + i);
      
      int state = buttonStates[i] == LOW;
      if(i > 0)
        Serial.print(" ");
      Serial.print(state ? "X" : ".");
    }
        for(i = 0; i < NUM_ANALOG; i++) {
      int value = an[i];
      float voltage = value * 5.0 / 1024;
      float v = voltage;
      const int RANGE_SIZE = 8;
      int range = (float)an[i] * RANGE_SIZE / 1024;
      changed = fabs(prevVoltage[i] - voltage) > 0.1;
      if(1) {
        Serial.print(" ");
        Serial.print("A");Serial.print(i+1);
        Serial.print("=");
   /*     if(i == 0 )
      Serial.print(range);
      else */
      Serial.print(v);
      }
          prevAn[i] = an[i];
          prevVoltage[i]  = voltage;

    }
  
    Serial.println("");
    prevButtons = buttons;
  }
  if(buttonPushCounter % 4 == 0) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
  delay(50);
}
