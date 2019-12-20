

// Code Created by Arafa Microsys (Eng.Hossam Arafa)
// www.youtube.com/arafamicrosystems
// www.facebook.com/arafa.microsys
// Please, if you don't subscribe to the channel subscribe for supporting us to provide more Special Episodes

#include <avr/io.h>
#include <avr/interrupt.h>

volatile bool busy = false;
volatile bool ledState = false;
 const int TCNT2init = 10;

#include <MsTimer2.h>
#include <elapsedMillis.h>

elapsedMillis timeElapsed; // declare global if you don't want it reset every time loop runs
unsigned int interval = 3000;
/* volatile unsigned int blinkIntervals[] = { 1000, 1000, 100, 100, 100, 100 };
volatile unsigned int blinkIndex = 0; */
int counter = 0;

#include <Capacitor.h>

// Capacitor under test.
// Note that for electrolytics the first pin (in this case D7)
// should be positive, the second (in this case A2) negative.
Capacitor cap1(7, A2);

#include <FreqCounter.h>
unsigned long frq;

struct {
  int mode;
} state;

enum { FREQUENCY, CAPACITANCE, VOLTAGE, INDUCTANCE };

const char* modes[] = {"frequency", "capacitance", "voltage", "inductance"};
// Counts overflovs
volatile uint16_t Tovf, Tovf1;

// Variables holding three timestamps
volatile uint16_t Capt1, Capt2, Capt3;

// capture Flag
volatile uint8_t Flag;
float last;
float capacitance, inductance;
int mark;


void
serprint(float us, float Freq, float Duty) // Function Received Freq and Duty Cycle to print them
{
  capacitance = 1.E-6;                                                    // Using 1uF Capacitor
  inductance = 1. / (capacitance * Freq * Freq * 4. * 3.14159 * 3.14159); ////Inductance Equation
  inductance *= 1E6; // note that this is the same as saying inductance = inductance*1E6
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

  /*
  Serial.print("T= ");
  Serial.print(us);
  Serial.print("us");
  Serial.print("&D= ");
  Serial.print(Duty);
  Serial.println("%");
  */
}

// Initialize timer
void
InitTimer1(void) {
  noInterrupts();
  // Set Initial Timer value
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 34286;

  // First capture on rising edge
  TCCR1B |= (1 << ICES1);
  // Enable input capture and overflow interrupts
  TIMSK1 |= (1 << ICIE1) | (1 << TOIE1);
  // Start timer without prescaller
  TCCR1B |= (1 << CS10);
  // Enable global interrutps
  interrupts();
}
unsigned int Hz = 16000;

float timerFreq = -(1) * (Hz - 2000000) / Hz;

void
InitTimer2(void) {
 // TIMER2_OVF_vect (Timer2 overflow) is fired with freq: 
 // Freq_OVF = 16000000/(scale*(255-TCNT2init)) Hz
 // Square wave( _-_-_ ) on pin OVF_Pin has:
 // Freq_PIN = FreqOVF/2 

 TCCR2B = 0x00; // No clock source (Timer/Counter stopped) 
 
 TCNT2 = TCNT2init; // Register : the Timer/Counter (TCNT2) and Output Compare Register (OCR2A and OCR2B) are 8-bit
                    // Reset Timer Count
 
 TCCR2A = 0x00; // TCCR2A - Timer/Counter Control Register A
                // All bits to zero -> Normal operation
 
TCCR2B = TCCR2B & B11111000 | B00000001; // Prescale 128 (Timer/Counter started)
 TCCR2B &= ~(1<<CS21);          // CS22=1 CS21=0 CS20=1 -> prescale = 128
 
 TIMSK2 |= (1<<TOIE2); // TIMSK2 - Timer/Counter2 Interrupt Mask Register
 // Bit 0 - TOIE2: Timer/Counter2 Overflow Interrupt Enable
 
}

ISR(TIMER2_OVF_vect) {
   TCNT2 = TCNT2init;

  
  counter++;
  if(counter % 3 == 0) {
    counter = 0;
  
  ledState ^= 1;
  digitalWrite(13, ledState);

  }
}

ISR(TIMER1_OVF_vect) {
  // increment overflow counter
  Tovf++;
  /*   if(Tovf >= blinkInterval) {
     Tovf = 0;
   */
  //    ledState = !ledState;
  //    digitalWrite(13, Tovf & 1);
  //  }
}

// capture ISR
ISR(TIMER1_CAPT_vect) {
  if(Flag == 0) {
    // save captured timestamp
    Capt1 = ICR1;
    Tovf = 0;
    // change capture on falling edge
    TCCR1B &= ~(1 << ICES1);
  }
  if(Flag == 1) {
    Capt2 = ICR1;
    Tovf1 = Tovf;
    // change capture on rising edge
    TCCR1B |= (1 << ICES1);
  }
  if(Flag == 2) {
    Capt3 = ICR1;
    // stop input capture and overflow interrupts
    TIMSK1 &= ~((1 << ICIE1) | (1 << TOIE1));
  }
  // increment Flag
  Flag++;
}

void
setBusy(bool state = true) {
  busy = state;

  // blinkInterval = state ? 100 : 500;
  /*   MsTimer2::stop();
    MsTimer2::set(state ? 150 : 800, periodicTimer); // 500ms period
    MsTimer2::start(); */
}

void
setup() {
  Serial.begin(38400);
  pinMode(8, INPUT);
  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);

  InitTimer1();
  InitTimer2();

  setBusy(false);

  // Serial.println("Ind.Meter 1uH-1H");
  Serial.println("Connect Inductor to Pin D8, D9");
  Serial.println("Connect Capacitor to Pin D7, A2");
  Serial.println("Connect Frequency to Pin D5");
  Serial.println("Connect Voltage to Pin A0 - A3");
}

void
measureFrequency() {

  FreqCounter::f_comp = 8;         // Set compensation to 12
  FreqCounter::start(100);         // Start counting with gatetime of 100ms
  while(FreqCounter::f_ready == 0) // wait until counter ready

    frq = FreqCounter::f_freq; // read result
  Serial.print(frq);           // print result
  Serial.println("Hz");
}

void
measureVoltage(int numChannels) {
  const float mul = (5.0 / 1023.0);
  for(int i = 0; i < numChannels; ++i) {
    float voltage = (float)analogRead(A0 + i) * mul;
    Serial.print("A");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(voltage);
    Serial.println("V");
  }
}

void
measureCapacitance() {
  float c = cap1.Measure();
  Serial.print(c); // Measure the capacitance (in pF), print to Serial Monitor
  Serial.println("pF");
}

void
loop() {
      Serial.print("Counter: ");
      Serial.println(counter);

  if(!busy) {

    if(timeElapsed > interval) {

      state.mode++;
      state.mode %= 3;

      Serial.print("Doing ");
      Serial.print(modes[state.mode]);
      Serial.println(" measurement...");

      if(state.mode == FREQUENCY) {
        measureFrequency();
        timeElapsed = 0;

      } else if(state.mode == CAPACITANCE) {
        measureCapacitance();
        timeElapsed = 0;

      } else if(state.mode == INDUCTANCE) {
        setBusy();

        digitalWrite(9, HIGH);
        delay(5); // give some time to charge inductor.
        digitalWrite(9, LOW);
        delayMicroseconds(400); // some delay to make it stable

      } else if(state.mode == VOLTAGE) {
        measureVoltage(8);
        timeElapsed = 0;
      }
    }
  } else {

    // calculate duty cycle if all timestamps captured
    if(Flag == 3) {

      uint32_t R2 = (Tovf * 65536) + Capt3;
      uint32_t R1 = Capt1;
      uint32_t F1 = (Tovf1 * 65536) + Capt2;
      float ton = (F1 - R1) * 62.5e-3;
      float us = (R2 - R1) * 62.5e-3;
      float Dutycycle = (float(F1 - R1) / float(R2 - R1)) * 100;
      float freq = (1 / us) * 1000000;
      serprint(ton, freq, Dutycycle);
      // delay(500);
      // clear flag
      Flag = 0;
      // clear overflow counters;
      Tovf = 0;
      Tovf1 = 0;
      // clear interrupt flags to avoid any pending interrupts
      TIFR1 = (1 << ICF1) | (1 << TOV1);
      // enable input capture and overflow interrupts
      TIMSK1 |= (1 << ICIE1) | (1 << TOIE1);

      timeElapsed = 0;
      setBusy(false);
    }
  }
}
