#define PGC A0
#define PGD A1
#define MCLR A3

void
setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  pinMode(PGC, OUTPUT);
  pinMode(PGD, INPUT);
  pinMode(MCLR, OUTPUT);
  digitalWrite(MCLR, HIGH);
  digitalWrite(PGD, HIGH);
  digitalWrite(PGC, HIGH);
}

void
loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()) {
    String line = Serial.readStringUntil('\n');
    int pin = -1;
    for(int i = 0; i < line.length(); ++i) {

      switch(line[i]) {
        case '0':
        case '1':
        case '2':
        case '3': pin = ((int)(unsigned int)(unsigned char)line[i]) - 0x30; break;

        case 'H':
        case 'L':
          if(pin != -1) {
            pinMode(pin + A0, OUTPUT);
            digitalWrite(pinn + A0, line[i] == 'H' ? HIGH : LOW);

            Serial.print("Pin ");
            Serial.print((long)pi);
            Serial.print(" set to ");
            Serial.println((long)line[i] == 'H');
          }
          pin = -1;
          break;

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
