#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <elapsedMillis.h>
#include <Capacitor.h>
#include <FreqCounter.h>

#include <Nokia_LCD.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

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
  TCCR2A = 0x00;       // TCCR2A - Timer/Counter Control Register A
                       // All bits to zero -> Normal operation
  TCCR2B &= 0b11111000;
  TCCR2B |= 0b00000001;   // Prescale 128 (Timer/Counter started)
  TIMSK2 |= (1 << TOIE2); // TIMSK2 - Timer/Counter2 Interrupt Mask Register
                          // Bit 0 - TOIE2: Timer/Counter2 Overflow Interrupt Enable
}

/**
 * @brief      Setup routine
 */
void
setup() {
  Serial.begin(38400);
  pinMode(8, INPUT);
  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(3, OUTPUT);
  analogWrite(3, 120);
  InitTimer1();
  // InitTimer2();

  lcd.begin();         // Initialize the screen
  lcd.setContrast(50); // Set the contrast; Good values are usualy between 40 and 60

  lcd.clearDisplay(); // clears the screen and buffer
  lcd.display();
  delay(2000);

  /*  Serial.print("display dimensions: ");
    Serial.print(lcd.width());
    Serial.print("x");
    Serial.println(lcd.width());
  */
  // lcd.drawBitmap(30, 16, logo16_glcd_bmp, 16, 16, 1);
  lcd.display();
  delay(2000);

  // invert the display
  /* lcd.invertDisplay(true);
   delay(1000);
   lcd.invertDisplay(false);
   delay(1000);*/

  // lcd.clearDisplay();

  lcd.setCursor(0, 0);
  lcd.print("Meep Meep! ====>>");
  lcd.display();
  delay(2000);

  // Serial.println("Ind.Meter 1uH-1H");
  Serial.println("Connect Inductor to Pin D8, D9");
  Serial.println("Connect Capacitor to Pin 2, A2");
  Serial.println("Connect Frequency to Pin D5");
  Serial.println("Connect Voltage to Pin A0 - A3");
}

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
loop() {

  if(!busy) {
    if(timeElapsed > interval) {
      mode++;
      if(mode == 3)
        mode = 0;

      Serial.print(modeNames[mode]);
      Serial.println(" measurement");

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

      printInductance(ton, freq, duty);

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
  analogWrite(3, 120);
  pinMode(3, OUTPUT);

  animateProgress();
}
