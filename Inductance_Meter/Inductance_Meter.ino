
// Code Created by Arafa Microsys (Eng.Hossam Arafa)
// www.youtube.com/arafamicrosystems
// www.facebook.com/arafa.microsys
// Please, if you don't subscribe to the channel subscribe for supporting us to provide more Special Episodes

#include <avr/io.h>
#include <avr/interrupt.h>

#include <elapsedMillis.h>

elapsedMillis timeElapsed; // declare global if you don't want it reset every time loop runs
unsigned int interval = 1000;

#include <Capacitor.h>

// Capacitor under test.
// Note that for electrolytics the first pin (in this case D7)
// should be positive, the second (in this case A2) negative.
Capacitor cap1(7, A2);

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
serprint(float us, float Freq, float Duty) { // Function Received Freq and Duty Cycle to print them
  capacitance = 1.E-6;                       // Using 1uF Capacitor
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
  TCNT1 = 0;
  // First capture on rising edge
  TCCR1B |= (1 << ICES1);
  // Enable input capture and overflow interrupts
  TIMSK1 |= (1 << ICIE1) | (1 << TOIE1);
  // Start timer without prescaller
  TCCR1B |= (1 << CS10);
  // Enable global interrutps
  interrupts();
}

ISR(TIMER1_OVF_vect) {
  // increment overflow counter
  Tovf++;
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
// Overflow ISR

void
setup() {
  Serial.begin(9600);
  pinMode(8, INPUT);
  pinMode(9, OUTPUT);
  InitTimer1();
  Serial.println("Ind.Meter 1uH-1H");
  Serial.println("Connect Inductor");
}

void
loop() {
  if(mark == 10000) { // mark used to give some delay without using the delay function
    digitalWrite(9, HIGH);
    delay(5); // give some time to charge inductor.
    digitalWrite(9, LOW);
    delayMicroseconds(400); // some delay to make it stable
    mark = 0;
  }
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
  }
  mark++;

  if(timeElapsed > interval) {
    float c = cap1.Measure();
    Serial.print(c); // Measure the capacitance (in pF), print to Serial Monitor
    Serial.println("pF");
    timeElapsed = 0; // reset the counter to 0 so the counting starts over...
  }
}
