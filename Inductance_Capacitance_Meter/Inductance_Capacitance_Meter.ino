 #include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <elapsedMillis.h>
#include <limits.h>
#include <Capacitor.h>
#include <FreqCounter.h>

#include <Nokia_LCD.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <RotaryEncoder.h>

#define ADAFRUIT 0

#if ADAFRUIT
// Hardware SPI (faster, but must use certain hardware pins):
// SCK is LCD serial clock (SCLK) - this is pin 13 on Arduino Uno
// MOSI is LCD DIN - this is pin 11 on an Arduino Uno
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 lcd = Adafruit_PCD8544(5, 7, 6);
// Note with hardware SPI MISO and SS pins aren't used but will still be read
// and written to during SPI transfer.  Be careful sharing these pins!
#else
#define clearDisplay clear
#define display getCursorX
Nokia_LCD lcd(13 /*SCLK*/, 11 /*DN(MOSI)*/, 5 /*D/C*/, 7 /*SCE*/, 6 /*RST*/);
#endif

volatile bool busy = false, ledState = false;
elapsedMillis timeElapsed;
uint16_t interval = 250;

volatile int32_t count = 0;
volatile uint32_t numInterrupts;
// Capacitor under test.
// Note that for electrolytics the first pin (in this case D7)
// should be positive, the second (in this case A2) negative.
Capacitor cap1(2, A2);

char buffer[128];

uint32_t freq;
uint8_t mode;
enum { FREQUENCY, CAPACITANCE, VOLTAGE, INDUCTANCE };

const char* const modeNames[] = {"freq", "cap.", "volt", "indu"};

const uint16_t Timer1_Init = 34286, Timer2_Init = 10;

volatile uint16_t Timer1_Overflow = 0, Timer1_Overflow2 = 0, Timer2_Overflow = 0;

// Variables holding three timestamps
volatile uint16_t Capture1_Time, Capture2_Time, Capture3_Time;
volatile uint8_t Capture_Flag;

float capacitance, inductance;

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH 16

static const unsigned char PROGMEM logo16_glcd_bmp[] = {0b00000000, 0b11000000, 0b00000001, 0b11000000, 0b00000001, 0b11000000, 0b00000011,
                                                        0b11100000, 0b11110011, 0b11100000, 0b11111110, 0b11111000, 0b01111110, 0b11111111,
                                                        0b00110011, 0b10011111, 0b00011111, 0b11111100, 0b00001101, 0b01110000, 0b00011011,
                                                        0b10100000, 0b00111111, 0b11100000, 0b00111111, 0b11110000, 0b01111100, 0b11110000,
                                                        0b01110000, 0b01110000, 0b00000000, 0b00110000};

RotaryEncoder encoder(A0, A1);

// Define some constants.
// at 500ms, there should be no acceleration.
constexpr static const unsigned long kAccelerationLongCutoffMillis = 500;
// at 4ms, we want to have maximum acceleration
constexpr static const unsigned long kAccelerationShortCutffMillis = 4;
// linear acceleration: incline
constexpr static const float m = -0.16;
// linear acceleration: y offset
constexpr static const float c = 84.03;
static int32_t pos, offset;

/**
 * @brief      Initialize Timer 1.
 */
void
InitTimer1(void) {
  noInterrupts();

  TCCR1A = 0; // Set Initial Timer value
  TCCR1B = 0;
  TCNT1 = Timer1_Init;
  TCCR1B |= (1 << ICES1);                // First capture on rising edge
  TIMSK1 |= (1 << ICIE1) | (1 << TOIE1); // Enable input capture and overflow interrupts
  TCCR1B |= (1 << CS10);                 // Start timer without prescaller

  interrupts(); // Enable global interrutps
}

/**
 * @brief      Initialize Timer 2.
 */
void
InitTimer2(void) {
  // TIMER2_OVF_vect (Timer2 overflow) is fired with freq:
  // Freq_OVF = 16000000/(scale*(255-Timer2_Init)) Hz
  // Square wave( _-_-_ ) on pin OVF_Pin has:
  // Freq_PIN = FreqOVF/2
  TCCR2B = 0x00;       // No clock source (Timer/Counter stopped)
  TCNT2 = Timer2_Init; // Register : the Timer/Counter (TCNT2) and Output Compare
  // Register (OCR2A and OCR2B) are 8-bit Reset Timer Count
  TCCR2A = 0x00; // TCCR2A - Timer/Counter Control Register A
  // All bits to zero -> Normal operation
  TCCR2B &= 0b11111000;
  TCCR2B |= 0b00000001;   // Prescale 128 (Timer/Counter started)
  TIMSK2 |= (1 << TOIE2); // TIMSK2 - Timer/Counter2 Interrupt Mask Register
  // Bit 0 - TOIE2: Timer/Counter2 Overflow Interrupt Enable
}

volatile uint32_t x, y, z, r;
volatile int32_t ax = 0, bx = 0;

/**
 * @brief      Prints inductance.
 *
 * @param[in]  us         duration
 * @param[in]  freq       frequency
 * @param[in]  dutyCycle  duty cycle
 */
void
printInductance(float us, float freq, float dutyCycle) {

  capacitance = 1e-06;                                              // Using 1uF Capacitor
  inductance = 1.0 / (capacitance * freq * freq * 4 * M_PI * M_PI); // Inductance Equation
  inductance *= 1e+06;                                              // note that this is the same as saying inductance = inductance*1E6

  if(inductance > 1000000 || inductance < 1) {
    Serial.println("  Out of Range  ");
  } else {
    Serial.print("    ");
    if(inductance > 1000) {
      inductance = inductance / 1000;
      Serial.print(inductance, 2);
      Serial.println(" mH");
    } else {
      Serial.print(inductance, 0);
      Serial.println(" uH");
    }
  }
  /*Serial.print("T= ");
  Serial.print(us);
  Serial.print("us");
  Serial.print("&D= ");
  Serial.print(dutyCycle);
  Serial.println("%");*/
}

/**
 * Timer 1 overflow service routine
 */
ISR(TIMER1_OVF_vect) { Timer1_Overflow++; }

/**
 * Timer 2 overflow service routine
 */
ISR(TIMER2_OVF_vect) {
  TCNT2 = Timer2_Init;
  Timer2_Overflow++;

  if(Timer2_Overflow & 0b100) {
    ledState = !ledState;
    digitalWrite(13, ledState);
  }
}

/**
 * Timer 1 capture service routine
 */
ISR(TIMER1_CAPT_vect) {
  if(Capture_Flag == 0) {
    Capture1_Time = ICR1; // save captured timestamp
    Timer1_Overflow = 0;
    TCCR1B &= ~(1 << ICES1); // change capture on falling edge
  }
  if(Capture_Flag == 1) {
    Capture2_Time = ICR1;
    Timer1_Overflow2 = Timer1_Overflow;
    TCCR1B |= (1 << ICES1); // change capture on rising edge
  }
  if(Capture_Flag == 2) {
    Capture3_Time = ICR1;
    TIMSK1 &= ~((1 << ICIE1) | (1 << TOIE1)); // stop input capture and overflow interrupts
  }
  Capture_Flag++; // increment Capture_Flag
}

void
handleButton() {
  pos = 0;

  //  Serial.println("button pressed");
}

void
setBusy(bool state = true) {
  busy = state;
}

void
print(const char* b) {
  Serial.print(b);
  lcd.print(b);
}

void
println(const char* b) {
  Serial.println(b);
  lcd.print(b);
  lcd.display();
}

/**
 * @brief      Measure frequency on Pin 5
 */
void
measureFrequency() {
  FreqCounter::f_comp = 8; // Set compensation to 12
  FreqCounter::start(100); // Start counting with gatetime of 100ms

  while(FreqCounter::f_ready == 0) // wait until Timer2_Overflow ready
    freq = FreqCounter::f_freq;    // read result
  dtostrf(freq, 3, 2, buffer);

  lcd.setCursor(0, 4);

  print("f = ");
  print(buffer); // print result
  println(" Hz");
}

/**
 * @brief      Measure voltage on ADC channels
 *
 * @param[in]  numChannels  The number of channels
 */
void
measureVoltage(int numChannels) {
  const float mul = (5.0 / 1023.0);

  lcd.setCursor(0, 3);
  lcd.print("U =");

  for(int i = 0; i < numChannels; ++i) {
    float voltage = (float)analogRead(A0 + i) * mul;

    dtostrf(voltage, 3, 2, buffer);

    /*   Serial.print(i > 0 ? " A" : "A");
     Serial.print(i);
      Serial.print("=");*/
    if(i == 0) {
      lcd.print(" ");
      lcd.print(buffer);
      lcd.print(" V");
    }
    if(i > 0)
      Serial.print(" ");
    Serial.print(buffer);
  }
  println("");
}

void
drawColon(int b = 1) {
  char bitmap[] = {
      (((count >> 1) + 1) & 1 == b) ? 0b00000100100 : 0b00000100000,
      (((count >> 1) + 1) & 1 == b) ? 0b00000100100 : 0b00000100000,
      0b0000000,
  };
  lcd.draw(bitmap, sizeof(bitmap), false);
}

void
drawZero() {
  char bitmap[] = {
      0b00111110,
      0b01000001,
      0b01000001,
      0b01000001,
      0b00111110,
  };
  lcd.draw(bitmap, sizeof(bitmap), false);
}

void
drawChar(char c) {
  char s[] = {c, 0};
  if(c == '0')
    drawZero();
  else
    lcd.print(s);
  char b = 0;
  lcd.draw(&b, 1, false);
}

void
showTime(const char* label, int32_t secs) {
  uint32_t h, m, s;

  lcd.print(" ");

  while(*label) {
    drawChar((char)*label);
    label++;
  }

  lcd.print(" ");
  secs %= 86400;
  if(secs < 0)
    secs += 86400;

  s = secs;

  m = s / 60;
  m = s / 60 % 60;
  h = s / 3600;
  s = s % 60;

  if(h < 10)
    drawChar('0');
  else
    drawChar('0' + (h % 24) / 10);
  drawChar('0' + (h % 10));

  drawColon();
  if(m < 10)
    drawChar(0x30);
  else
    drawChar(0x30 + ((m % 60) / 10));

  drawChar('0' + (m % 10));
  if(1) {
    drawColon();

    drawChar('0' + s / 10);
    drawChar('0' + (s % 10));
  }
  drawChar(' ');
}

/**
 * @brief      Measure the capacitance (in pF), print to Serial Monitor
 */
void
measureCapacitance() {
  float c = cap1.Measure();
  const char* unit = " pF";
  if(c >= 1000) {
    c /= 1000;
    unit = " nF";
  }

  dtostrf(c, 3, 2, buffer);

  lcd.setCursor(0, 2);

  lcd.print("C = ");
  print(buffer);
  println(unit);
}

void
animateProgress() {
  bool animTime = (timeElapsed / (busy ? 100 : 1000)) & 1;
  static int animStep;

  // const char* animChars[4] = {"[ - ]","[ \\ ]","[ | ]","[ / ]"};
  const char* animChars[] = {
      "  ..o     ",
      "   ..o    ",
      "    ..o   ",
      "     ..o  ",
      "      ..o ",
      "       o..",
      "      o.. ",
      "     o..  ",
      "    o..   ",
      "   o..    ",
      "  o..     ",
      " ..o      ",
  };
  const int animSteps = sizeof(animChars) / sizeof(animChars[0]);
  digitalWrite(13, animTime & 1);

  lcd.setCursor(0, 0);
  lcd.print(animChars[animStep++ % animSteps]);
  lcd.display();

  delay(100);
}

void
processLine(char* s) {
  uint32_t seconds = 0;
  while(*s) {
    if(*s >= '0' && *s <= '9') {
      unsigned long l = strtoul(s, &s, 10);

      if(l != ULONG_MAX) {
        seconds *= 60;
        seconds += l;
        continue;
      }
    }

    s++;
  }
  count = (seconds % 86400) * 4;
}

void
drawLineH(int x1, int x2, int row, int y) {
  int x;
  lcd.setCursor(x1, row);
  char bit = 1 << (y % 8);

  for(x = x1 & (~0b111); x < x2; x += 8) {
    char bitmap[] = {bit, bit, bit, bit, bit, bit, bit, bit};

    lcd.draw(bitmap, sizeof(bitmap), false);
  }
}

void
processChar(char c) {
  static int dumpPos;
  static char inputString[10 + 1];
  int i;

  drawLineH(5, 72, 1, 5);
  lcd.setCursor(8, 1);
  inputString[dumpPos] = c;
  inputString[dumpPos + 1] = '\0';

  if(c == '\n')
    inputString[dumpPos] = ' ';
  dumpPos++;
  if(dumpPos > 10)
    dumpPos = 0;

  for(i = dumpPos + 1; i < 10; i++) inputString[i] = ' ';
  inputString[10] = '\0';

  if(c == '\n')
    dumpPos = 0;

  lcd.print(inputString);
  processLine(inputString);
}

/**
 * @brief      Setup routine
 */
void
setup() {
  x = 0;
  y = 0;
  z = 0;
  r = 0;
  offset = 0;
  pos = 0;
  numInterrupts = 0;
  Serial.begin(38400);

  pinMode(8, INPUT);
  pinMode(9, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  /*  pinMode(A0, INPUT);
    pinMode(A1, INPUT);*/

  analogWrite(3, 120);
  InitTimer1();
  // InitTimer2();X

  lcd.begin();         // Initialize the screen
  lcd.setContrast(50); // Set the contrast; Good values are usualy between 40 and 60
  lcd.clearDisplay();  // clears the screen and buffer
  lcd.display();
  delay(200);

  /*  Serial.print("display dimensions: ");
    Serial.print(lcd.width());
    Serial.print("x");
    Serial.println(lcd.width());
  */
  // lcd.drawBitmap(30, 16, logo16_glcd_bmp, 16, 16, 1);
  lcd.display();
  delay(200);

  /* lcd.invertDisplay(true);
   delay(1000);
   lcd.invertDisplay(false);
   delay(1000);*/

  // lcd.clearDisplay();

  lcd.setCursor(0, 0);
  lcd.print("Meep Meep! ====>>");
  lcd.display();
  delay(200);

  // Serial.println("Ind.Meter 1uH-1H");
  Serial.println("Connect Inductor to Pin D8, D9");
  Serial.println("Connect Capacitor to Pin 2, A2");
  Serial.println("Connect Frequency to Pin D5");
  Serial.println("Connect Voltage to Pin A0 - A3");

  attachInterrupt(digitalPinToInterrupt(A2), handleButton, FALLING);

  lcd.clear();
  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
}

void
updateEncoder() {
  int buttonState = digitalRead(A2);
  static RotaryEncoder::Direction lastMovementDirection = RotaryEncoder::Direction::NOROTATION;
  encoder.tick();

  int newPos = encoder.getPosition();
  if(pos != newPos) {

    // compute linear acceleration
    RotaryEncoder::Direction currentDirection = encoder.getDirection();
    if(currentDirection == lastMovementDirection && currentDirection != RotaryEncoder::Direction::NOROTATION &&
       lastMovementDirection != RotaryEncoder::Direction::NOROTATION) {
      // ... but only of the direction of rotation matched and there
      // actually was a previous rotation.
      unsigned long deltat = encoder.getMillisBetweenRotations();

      if(deltat < kAccelerationLongCutoffMillis) {
        if(deltat < kAccelerationShortCutffMillis) {
          // limit to maximum acceleration
          deltat = kAccelerationShortCutffMillis;
        }

        float ticksActual_float = m * deltat + c;
        // Round by adding 1
        // Then again remove 1 to determine the actual delta to the encoder
        // value, as the encoder already ticked by 1 tick in the correct
        // direction. Thus, just cast to an integer type.
        long deltaTicks = (long)ticksActual_float;

        // Adjust sign: Needs to be inverted for counterclockwise operation
        if(currentDirection == RotaryEncoder::Direction::COUNTERCLOCKWISE) {
          deltaTicks = -(deltaTicks);
        }

        newPos = newPos + deltaTicks;
        encoder.setPosition(newPos);
      }
    }
    Serial.print("pos = ");
    Serial.print(newPos);
    Serial.println();
    if(buttonState == LOW) {
      int8_t seconds = count / 4 + offset % 60;

      offset = newPos * 60;
      int32_t value = count / 4 + offset;
      offset -= value % 60;
      offset += seconds;
    } else
      pos = newPos;
  } // if
}

void
loop() {
  lcd.setCursor(0, 0);

  updateEncoder();
  if(1) {
    x = digitalRead(A0) == LOW;
    digitalWrite(3, x);
    y = digitalRead(A1) == LOW;
    digitalWrite(4, y);
    z = digitalRead(A2) == LOW;
    lcd.print(z);
    lcd.print(" ");

    /*
        lcd.print(" ");
        lcd.print(y);------------------------------------------------------------
        lcd.print(" ");*/
    lcd.print((int)pos);
    /*
    lcd.print(" ");
    lcd.print(bx);*/

    r = 0;
    x = 0;
    y = 0;
  }
  while(Serial.available()) processChar((char)Serial.read());

  if(timeElapsed > interval) {

    while(timeElapsed >= interval) {
      timeElapsed -= interval;
      count++;
      /*count %= 86400*4;
      if(count < 0)
        count += 86400*4;*/

      lcd.setCursor(0, 1);
      showTime("ZEIT", count / 4 + offset);
      lcd.setCursor(0, 2);
      showTime(" ON ", 7 * 60 * 60);
      lcd.setCursor(0, 3);
      showTime(" OFF", 23 * 60 * 60);
    }
    if(!busy) {

      mode++;
      if(mode == 3)
        mode = 0;

      Serial.print(modeNames[mode]);
      Serial.println(" measurement");
      mode = -1;

      switch(mode) {
        case FREQUENCY:
          measureFrequency();
          timeElapsed = 0;
          break;

        case CAPACITANCE:
          measureCapacitance();
          timeElapsed = 0;
          break;

        case VOLTAGE:
          measureVoltage(7);
          timeElapsed = 0;
          break;

        case INDUCTANCE:
          setBusy();
          digitalWrite(9, HIGH);
          delay(5); // give some time to charge inductor.
          digitalWrite(9, LOW);
          delayMicroseconds(400); // some delay to make it stable
          break;
      }
    }
  } else {
    // calculate duty cycle if all timestamps captured
    if(Capture_Flag == 3) {
      uint32_t R2 = (Timer1_Overflow * 65536) + Capture3_Time;
      uint32_t R1 = Capture1_Time;
      uint32_t F1 = (Timer1_Overflow2 * 65536) + Capture2_Time;

      float ton = (F1 - R1) * 62.5e-03;
      float us = (R2 - R1) * 62.5e-03;
      float duty = (float(F1 - R1) / float(R2 - R1)) * 100;
      float freq = (1 / us) * 1000000;

      //    printInductance(ton, freq, duty);

      // delay(500);
      // clear flag
      Capture_Flag = 0;
      // clear overflow counters;
      Timer1_Overflow = 0;
      Timer1_Overflow2 = 0;
      // clear interrupt flags to avoid any pending interrupts
      TIFR1 = (1 << ICF1) | (1 << TOV1);
      // enable input capture and overflow interrupts
      TIMSK1 |= (1 << ICIE1) | (1 << TOIE1);
      timeElapsed = 0;

      setBusy(false);
    }
  }

  // animateProgress();
}
