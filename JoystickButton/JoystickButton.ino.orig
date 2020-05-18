// Simple example application that shows how to read four Arduino
// digital pins and map them to the USB Joystick library.
//
// The digital pins 9, 10, 11, and 12 are grounded when they are pressed.
//
// NOTE: This sketch file is for use with Arduino Leonardo and
//       Arduino Micro only.
//
// by Matthew Heironimus
// 2015-11-20
//--------------------------------------------------------------------

#include <Joystick.h>

void setup() {
  delay(3000);
  // Initialize Button Pins
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);

  // Initialize Joystick Pins
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  // Initialize Joystick Library
  Joystick.begin();
}

// Constant that maps the phyical pin to the joystick button.
const int pinToButtonMap = 9;
const int pinToJoystickMap = 4;

// Last state of the button
int lastButtonState[4] = {0,0,0,0};
int lastJoystickState[4] = {0,0,0,0};

void loop() {

  // Read pin values
  for (int index = 0; index < 4; index++)
  {
    int currentButtonState = !digitalRead(index + pinToButtonMap);
    if (currentButtonState != lastButtonState[index])
    {
      Joystick.setButton(index, currentButtonState);
      lastButtonState[index] = currentButtonState;
    }
  }

  boolean change = false;
  for (int index = 0; index < 4; index++)
  {
    int currentJoystickState = !digitalRead(index + pinToJoystickMap);
    if (currentJoystickState != lastJoystickState[index])
    {
      lastJoystickState[index] = currentJoystickState;
      change = true;
    }
  }

  if (change) {
    if ((lastJoystickState[0] == 1) and (lastJoystickState[1] == 0)) {
      Joystick.setXAxis(-127);
    }
    else {
      if ((lastJoystickState[0] == 0) and (lastJoystickState[1] == 1)) {
        Joystick.setXAxis(127);
      }
      else {
        Joystick.setXAxis(0);
      }
    }

    if ((lastJoystickState[2] == 1) and (lastJoystickState[3] == 0)) {
      Joystick.setYAxis(-127);
    }
    else {
      if ((lastJoystickState[2] == 0) and (lastJoystickState[3] == 1)) {
        Joystick.setYAxis(127);
      }
      else {
        Joystick.setYAxis(0);
      }
    }
    delay(20);
  }
 

//  delay(50);
}

