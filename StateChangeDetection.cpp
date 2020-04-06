#include <AnalogButtons.h>
#define ANALOG_PIN A1

#include <MultiButtons.h>

const int buzzer = 9; // buzzer to arduino pin 9

unsigned int frequency = 1000;
unsigned int beeps = 10;

const int buttonPin = 3;
const int ledPin = 13;
const int NUM_ANALOG = 1;
int buttonPushCounter = 0;
int buttonState = 0;
int lastButtonState = 0;
char buttonStates[4] = {0, 0, 0, 0};
char lastButtonStates[4] = {0, 0, 0, 0};
char steerState[4] = {0, 0, 0, 0};
static int prevAn[NUM_ANALOG];

const char* buttonNames[] = {"up", "down", "left", "right"};
// Declare voltage ranges for each button
int voltageRanges[][2] = {{1023, 700}, {500, 699}, {200, 499}, {0, 199}};
int btnCount = sizeof(voltageRanges) / sizeof(voltageRanges[0]);
// Must declare buttonHandler callback function and used variables
// before generate MultiButtons object
//
//
void
buttonHandler(MultiButtons* mb, int btnIndex) {
  /*
   * NOTE: you may call method in MultiButtons object by 'mb->theMethod();'
   */
  int btnID = btnIndex + 1;
  Serial.print("Button pressed: ");
  Serial.println(buttonNames[btnIndex]);
  mb->setTriggerEdge(BTN_TRIGGER_EDGE_RELEASE);
}

MultiButtons mb(ANALOG_PIN, btnCount, voltageRanges, buttonHandler, 1024, BTN_TRIGGER_EDGE_PRESS);
/*MultiButtons mbr(ANALOG_PIN, btnCount, voltageRanges, buttonRelease, 1024, BTN_TRIGGER_EDGE_RELEASE);*/

void
setup() {
  // pinMode(ANALOG_PIN, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  analogReference(DEFAULT);
  Serial.begin(38400);
  mb.begin();

  pinMode(buzzer, OUTPUT); // Set buzzer - pin 9 as an output
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
// void b1Click() {Serial.println("button 1 clicked"); } void b1Hold() {Serial.println("button 1 held"); } void
// b2Click() {Serial.println("button 2 clicked"); } void b2Hold() {Serial.println("button 2 held"); } void b3Click()
// {Serial.println("button 3 clicked"); } void b3Hold() {Serial.println("button 3 held"); } void b4Click()
// {Serial.println("button 4 clicked"); } void b4Hold() {Serial.println("button 4 held"); } AnalogButtons
// analogButtons(ANALOG_PIN, INPUT); Button b1 = Button(1001, &b1Click, &b1Hold); Button b2 = Button(500, &b2Click,
// &b2Hold); Button b3 = Button(250, &b3Click, &b3Hold); Button b4 = Button(125, &b4Click, &b4Hold);

void
printAnalog(int an[], int num) {
  static float prevVoltage[3];

  int i;
  for(i = 0; i < num; i++) {
    unsigned long value = an[i];
    float voltage = value * 5.0 / 1024;
    float v = voltage;
    const int RANGE_SIZE = 8;
    int range = (float)an[i] * RANGE_SIZE / 1024;
    int cond = fabs(prevVoltage[i] - voltage) > 0.1;
    if(1 || cond) {
      Serial.print(" ");
      Serial.print("A");
      Serial.print(i + 1);
      Serial.print("=");
      /*     if(i == 0 )
         Serial.print(range);
         else */
      Serial.print(value);
    }
    prevAn[i] = an[i];
    prevVoltage[i] = voltage;
  }

  Serial.println("");
}

void
loop() {
  int i;
  int changed = 0;
  unsigned int an[3];
  static int prevButtons;

  mb.loop();
  int buttons = getButtonBits();
  int action = prevButtons != buttons || fabs(prevAn[0] - an[0]) > 50;
  if(action) {
    for(i = 0; i < 4; i++) {

      const int bit = (buttons >> i) & 1;

      /*  if(i > 0)
          Serial.print(" ");*/
      Serial.print(bit ? "X" : ".");
    }
    for(i = 0; i < NUM_ANALOG; i++) {
      an[i] = analogRead(ANALOG_PIN);
    }

    printAnalog(an, NUM_ANALOG);

    tone(buzzer, 3000);
    delay(25);
    noTone(buzzer);
    delay(25);

    prevButtons = buttons;
  }

  delay(50);
  noTone(buzzer);
}
