#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <Nokia_LCD.h>
#include <elapsedMillis.h>
#include <Capacitor.h>
#include <FreqCounter.h>

Nokia_LCD lcd(13 /*SCLK*/, 11 /*DN(MOSI)*/, 5 /*D/C*/, 7 /*SCE*/, 6 /*RST*/);

volatile bool busy = false, ledState = false;
elapsedMillis timeElapsed;
uint16_t interval = 250;

// Capacitor under test.
// Note that for electrolytics the first pin (in this case D7)
// should be positive, the second (in this case A2) negative.
Capacitor cap1(8, A2);

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
  InitTimer1();
  // InitTimer2();

  lcd.begin();         // Initialize the screen
  lcd.setContrast(50); // Set the contrast; Good values are usualy between 40 and 60
  lcd.clear();         // Clear the screen
  delay(1000);

  lcd.setCursor(2, 1);
  lcd.print("Meep Meep!");

  // Serial.println("Ind.Meter 1uH-1H");
  Serial.println("Connect Inductor to Pin D8, D9");
  Serial.println("Connect Capacitor to Pin D7, A2");
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
    ledState = !ledstate;
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

/**
 * @brief      Measure frequency on Pin 5
 */
void
measureFrequency() {
  FreqCounter::f_comp = 8; // Set compensation to 12
  FreqCounter::start(100); // Start counting with gatetime of 100ms

  while(FreqCounter::f_ready == 0) // wait until Timer2_Overflow ready
    freq = FreqCounter::f_freq;    // read result

  Serial.print(freq); // print result
  Serial.println("Hz");
}

/**
 * @brief      Measure voltage on ADC channels
 *
 * @param[in]  numChannels  The number of channels
 */
void
measureVoltage(int numChannels) {
  const float mul = (5.0 / 1023.0);
  
  for(int i = 0; i < numChannels; ++i) {
    float voltage = (float)analogRead(A0 + i) * mul;

    Serial.print(i > 0 ? " A" : "A");
    Serial.print(i);
    Serial.print("=");
    Serial.print(voltage);
    Serial.print("V");
  }

  Serial.println("");
}

/**
 * @brief      Measure the capacitance (in pF), print to Serial Monitor
 */
void
measureCapacitance() {
  float c = cap1.Measure();

  Serial.print(c);
  Serial.println("pF");
}

void
loop() {
  digitalWrite(13, (timeElapsed / (busy ? 100 : 1000)) & 1);

  if(!busy) {
    if(timeElapsed > interval) {
      mode++;
      if(mode == 3)
        mode = 0;

      Serial.print(modeNames[mode]);
      Serial.print(" measurement: ");

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
          measureVoltage(8);
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
}
