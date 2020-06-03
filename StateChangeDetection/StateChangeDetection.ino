#include <AnalogButtons.h>
#include <Keyboard.h>
#include <HID_Buttons.h> // Must import AFTER Keyboard.h

#include <EEPROM.h>

#define ANALOG_PIN A1
#define PROM_ADDR 2
#include <MultiButtons.h>

#include <avr/wdt.h>

#define soft_reset()                                                                                                   \
  do {                                                                                                                 \
    wdt_enable(WDTO_15MS);                                                                                             \
    for(;;) {                                                                                                          \
    }                                                                                                                  \
  } while(0)

int OUTPUT_SERIAL = 1;

// debounce by insisting the state is consistent this many reads in a row
#define MAX_DEBOUNCE_COUNT 12

const int buzzer = 9; // buzzer to arduino pin 9

unsigned int frequency = 1000;
unsigned int beeps = 10;

const int buttonPin = 3;
const int ledPin = 13;
const int NUM_ANALOG = 1;

char steerState[4] = {0, 0, 0, 0};

enum {
  KEYCODE_UP = 0,
  KEYCODE_DOWN = 1,
  KEYCODE_LEFT = 2,
  KEYCODE_RIGHT = 3,
  KEYCODE_START = 4,
  KEYCODE_SELECT = 5,
  KEYCODE_BUTTON1 = 6,
  KEYCODE_BUTTON2 = 7,
  KEYCODE_BUTTON3 = 8,
  KEYCODE_BUTTON4 = 9,
  KEYCODE_BUTTON5 = 10,
  KEYCODE_BUTTON6 = 11,
};

static uint8_t keyboardCodes[] = {
    KEY_UP_ARROW,    // Up
    KEY_DOWN_ARROW,  // Down
    KEY_LEFT_ARROW,  // Left
    KEY_RIGHT_ARROW, // Right
    KEY_RETURN,      // Start
    KEY_RIGHT_SHIFT, // Select

    // Button 1-6
    'x', // A
    'z', // B
    's', // X
    'a', // Y
    'd', // L
    'c', // R
};
static uint8_t debounceCounts[12];
static uint8_t debounceState[12];

static int prevAn[NUM_ANALOG];

const char* buttonNames[] = {"up", "down", "left", "right"};
// Declare voltage ranges for each button
int voltageRanges[][2] = {{1023, 700}, {500, 699}, {200, 499}, {0, 199}};
int btnCount = sizeof(voltageRanges) / sizeof(voltageRanges[0]);

// Must declare buttonHandler callback function and used variables
// before generate MultiButtons object
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

  OUTPUT_SERIAL = !!EEPROM.read(PROM_ADDR);

  if(OUTPUT_SERIAL) {
    Serial.begin(38400);

    while(!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Start in serial mode");
  } else {
    Keyboard.begin();
  }
  mb.begin();

  pinMode(buzzer, OUTPUT); // Set buzzer - pin 9 as an output

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, OUTPUT_SERIAL ? LOW : HIGH);

  for(int i = 0; i < sizeof(debounceState) / sizeof(debounceState[0]); i++) debounceState[i] = 0;
}

/**
 * @brief      { function_description }
 */
void
shutdown() {
  if(OUTPUT_SERIAL)
    Serial.end();
  else
    Keyboard.end();
}

/**
 * @brief      Restarts
 *
 * @param[in]  mode  The mode
 */
void
restart(int mode) {
  if(OUTPUT_SERIAL && mode == 0)
    Serial.println("Restart in HID mode");

  shutdown();
  EEPROM.write(PROM_ADDR, !!mode);
  delay(1000);

  setup();
}

/**
 * @brief      Counts the number of bits.
 *
 * @param[in]  val   The value
 *
 * @return     Number of bits.
 */
int
countBits(unsigned int val) {
  int c;

  for(c = 0; val; val >>= 1) c += val & 1;
  return c;
}

/**
 * @brief      Gets button bits.
 *
 * @return     The button bits.
 */
int
getButtonBits() {
  int i, ret = 0, changed = 0;
  int flag = 0;

  static char state[4] = {0, 0, 0, 0}, lastState[4] = {0, 0, 0, 0};

  for(i = 0; i < 4; i++) {
    flag = state[i] = digitalRead(buttonPin + i);
    if(flag != lastState[i]) {
      changed = 1;
      lastState[i] = state[i];
    }
  }
  ret = 0;
  for(i = 0; i < 4; i++) {
    ret |= (state[i] == LOW ? 1 : 0) << i;
  }
  return ret;
}

/**
 * @brief      Prints analog values
 *
 * @param      an    { parameter_description }
 * @param[in]  num   The number
 */
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
      Serial.print(value);
    }
    prevAn[i] = an[i];
    prevVoltage[i] = voltage;
  }
  Serial.println("");
}

void
setButtonState(int n, int state) {
  int oldState = debounceState[n];
  debounceState[n] = state;
  if(!OUTPUT_SERIAL) {
    if(state && !oldState)
      Keyboard.press(keyboardCodes[n]);
    if(!state && oldState)
      Keyboard.release(keyboardCodes[n]);
  }
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
  uint8_t* counts = &debounceCounts[KEYCODE_BUTTON1];

  if(countBits(buttons) >= 3)
    restart(!OUTPUT_SERIAL);

  if(action) {
    for(i = 0; i < 4; i++) {

      const int prevBit = (prevButtons >> i) & 1;
      const int bit = (buttons >> i) & 1;

      if(bit) {
        if(counts[i] < MAX_DEBOUNCE_COUNT)
          counts[i]++;
        if(counts[i] == MAX_DEBOUNCE_COUNT)
          setButtonState(i + KEYCODE_BUTTON1, 1);
      } else {
        if(counts[i] > 0)
          counts[i]--;
        if(counts[i] == 0)
          setButtonState(i + KEYCODE_BUTTON1, 0);
      }

      if(OUTPUT_SERIAL)
        Serial.print(debounceState[i + KEYCODE_BUTTON1] ? "X" : ".");
    }
    for(i = 0; i < NUM_ANALOG; i++) {
      an[i] = analogRead(ANALOG_PIN);
    }

    if(OUTPUT_SERIAL) {
      printAnalog(an, NUM_ANALOG);

      tone(buzzer, 3000);
      delay(25);
      noTone(buzzer);
      delay(25);
    }

    prevButtons = buttons;
  }

  if(OUTPUT_SERIAL) {
    delay(50);
    noTone(buzzer);
  }
}
