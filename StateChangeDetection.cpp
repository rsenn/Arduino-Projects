const int buttonPin = 3;
const int ledPin = 13;
int buttonPushCounter = 0;
int buttonState = 0;
int lastButtonState = 0;
char buttonStates[4] = {0, 0, 0, 0};
char lastButtonStates[4] = {0, 0, 0, 0};
void
setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
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
    lastButtonState = buttonState;
    ret = 0;
    for(i = 0; i < 4; i++) {
      ret |= (buttonStates[i] == LOW ? 1 : 0) << i;
    }
    return ret;
  }
  void loop() {
    int i;
    int changed = 0;
    static int prevButtons;
    int buttons = getButtonBts();
    if(prevButtons != buttons) {
      for(i = 0; i < 4; i++) {
        int state = buttonStates[i] == LOW;
        if(i > 0)
          Serial.print(" ");
        Serial.print(state ? "1" : "0");
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