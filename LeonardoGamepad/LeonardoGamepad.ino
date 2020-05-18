


#include "HID-Project.h"


// this constant won't change:
const int buttonPin = 3; // the pin that the pushbutton is attached to
const int ledPin = 13;   // the pin that the LED is attached to

// Variables will change:
int buttonPushCounter = 0; // counter for the number of button presses
int buttonState = 0;       // current state of the button
int lastButtonState = 0;   // previous state of the button
char buttonStates[4] = {0, 0, 0, 0};
char lastButtonStates[4] = {0, 0, 0, 0};

void
setup() {
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  // initialize the LED as an output:
  pinMode(ledPin, OUTPUT);
  // initialize serial communication:
//  Serial.begin(38400);


  // Sends a clean report to the host. This is important on any Arduino type.
  Gamepad.begin();
}

void
loop() {
  int i;
  int changed = 0;
  // read the pushbutton input pin:
  for(i = 0; i < 4; i++) {

    buttonState = buttonStates[i] = digitalRead(buttonPin + i);
    lastButtonState = lastButtonStates[i];

    // compare the buttonState to its previous state
    if(buttonState != lastButtonState) {
      changed = 1;

      // if the state has changed, increment the counter
      if(buttonState == LOW) {
        // if the current state is HIGH then the button went from off to on:
        buttonPushCounter++;
        // Serial.print("1");
        /*Serial.print("number of button pushes: ");
          Serial.println(buttonPushCounter);*/
      } else {
        // if the current state is LOW then the button went from on to off:
        // Serial.print("0");
      }
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
  for(i = 0; i < 4; i++) lastButtonStates[i] = buttonStates[i];

  /* if(changed) {
     for(i = 0; i < 4; i++) {
       if(i > 0)
         Serial.print(" ");
       Serial.print(buttonStates[i]);
     }

     Serial.println("");
   }
  */
  // turns on the LED every four button pushes by checking the modulo of the
  // button push counter. the modulo function gives you the remainder of the
  // division of two numbers:
  if(buttonPushCounter % 4 == 0) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  // Functions above only set the values.
  // This writes the report to the host.
  Gamepad.write();

}
