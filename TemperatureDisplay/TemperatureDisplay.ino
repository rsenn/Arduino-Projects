#include <Adafruit_MAX31855.h>

static const int CLK = 13; // SCK
static const int DO = 12; // MISO
static const int CS = 11; // SS

Adafruit_MAX31855 thermocouple(CLK, CS, DO);

#define SOFTWARESERIAL 1
//#define NEOSWSERIAL 1

#if SOFTWARESERIAL
#include <SoftwareSerial.h>

SoftwareSerial ss(3, 2);

#elif NEOSWSERIAL
#include <NeoSWSerial.h>

NeoSWSerial ss(3, 2);
#else
auto& ss = Serial;
#endif

volatile uint32_t newlines = 0UL;

void
handleRxChar(uint8_t c) {
  if(c == '\n')
    newlines++;
}

void
displayInteger(uint16_t n) {
  uint8_t digits[4]={0,0,0,0xb0};

  digits[2] = n % 10;
  n /= 10;
  digits[1] = n % 10;
  n /= 10;
  digits[0] = n % 10; 

  for(int i = 0; i < 4; i++) {
    if(digits[i])
      break;
    digits[i] = 0x78;
  }

  ss.write(digits, 4);
  
  ss.write('\x7d');
  ss.write('\x65');
}

void
setDecimalPoint(uint8_t dp) {
  uint8_t data[4] = {0x77, (uint8_t)(1 << dp), 0x77, (uint8_t)(1 << dp)};
  ss.write(data, 2);
}

void
setup() {
  Serial.begin(38400);
  
  ss.begin(9600);
  ss.write((uint8_t)0x7a);
  ss.write((uint8_t)100);
  //ss.write('\0');
  setDecimalPoint(1);

  // wait for MAX chip to stabilize
  delay(500);
  if(!thermocouple.begin()) {
    ss.write("xxxE", 4);
    while(1) delay(10);
  }
}

static uint16_t counter;

void
loop() {
  displayInteger(counter++);
  delay(500);

  if(counter % 2 == 0) {
    Serial.print("TemperatureDisplay ");
    Serial.print(counter);
    Serial.println("");
  }
}
