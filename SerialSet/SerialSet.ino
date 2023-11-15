#define PGC A0
#define PGD A1
#define MCLR A3

int pinModes[4]= { -1, -1,-1,-1};
int pinStates[4]= { -1, -1,-1,-1};

void
setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  pinMode(13, OUTPUT);

  setPin(0, HIGH);
  setPin(1, HIGH);
  setPin(3, HIGH);
}

void
setPin(int no, int level) {
  pinModes[no]=OUTPUT;
  pinMode(no+A0,level? INPUT_PULLUP:OUTPUT);
if(level==LOW)
 digitalWrite(no+A0, LOW);
 pinStates[no] =level;
}

void
printPin(int no) {
  Serial.print("Pin ");
  Serial.print(((const char* [4]){"PGC", "PGD", 0, "MCLR"})[no]);
}

 const char*
levelString(int level) {
  return ((const char* [2]){"LOW", "HIGH"})[!!level];
}

void
loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()) {
    String line = Serial.readStringUntil('\n');
    int pin = -1;
    for(int i = 0; i < line.length(); ++i) {

      switch(line[i]) {
        case 'B':
        #define _BAUD 57600                    // Baud rate (9600 is default)
#define _UBRR (F_CPU / 16) / _BAUD - 1 // Used for UBRRL and UBRRH
Serial.print("UBRR: ");
Serial.println((unsigned long)(UBRR0H << 8) | (UBRR0L));
Serial.print("UCSR0A: ");
Serial.println((unsigned long)UCSR0A, BIN);
Serial.print("UCSR0B: ");
Serial.println((unsigned long)UCSR0B, BIN);
break;
        case '0':
        case '1':
        case '2':
        case '3': pin = ((int)(unsigned int)(unsigned char)line[i]) - 0x30; break;

        case 'C': pin = 0; break;
        case 'D': pin = 1; break;
        case 'P':
        case 'M': pin = 3; break;

        case 'X': digitalWrite(13, HIGH); break;
        case 'x': digitalWrite(13, LOW); break;

        case '+':
        case '-':
        case 'H':
        case 'L':
          if(pin != -1) {
            int level=line[i] == 'H' || line[i] == '+' ? HIGH : LOW;
            setPin(pin, level);
            
            printPin(pin);
            Serial.print(" set to ");
            Serial.println(levelString(level));
          }
          break;

        case 'I':
        case '<':
        case 'O':
        case '>':
          if(pin != -1) {
            int dir = line[i] == 'I' || line[i] == '<' ? INPUT : OUTPUT;

            pinMode(pin + A0, pinModes[pin]= dir);

            printPin(pin);
            Serial.print(" mode ");
            Serial.println(((const char* [2]){"INPUT", "OUTPUT"})[dir]);
          }
          break;

          case 'W': _delay_ms(100);
          break;

        case 'R':
        case '?':
          if(pin != -1) {
            int level = digitalRead(pin + A0);

            printPin(pin);
            Serial.print(" is ");
            Serial.println(levelString(level));
          }
          break;

        case ' ': break;

        default:
          Serial.print("Char: ");
          Serial.println((long)line[i]);
          break;
      }
    }
    /*    Serial.print("Line: ");
      Serial.println(line);*/
  }
}
