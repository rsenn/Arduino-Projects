/*
  SerialScope
 
 	Draws a graph of the analog signal on the Serial.

 	1. read the pin
 	2. map the analog readings to a range [a,b]
 	3. draw the range [a,b] to the range [0,SERIAL_COLUMNS] on the Serial
 	4. waits the samplign time and repeat from 1

 */

#define SERIAL_COLUMNS 78
#define ZERO_CHAR 'O'
#define BAR_CHAR '#'
#define END_CHAR '|'

/*
	Modify the following to suite your needs!
*/
int pin = A2;						//the pin to read
int scope_range_min = -1000;		//minimum value of the range [a]
int scope_range_max = 1000;			//maximum value of the range [b]
int sampling = 50;					//sampling time

char buf[SERIAL_COLUMNS+1];
unsigned long thetime = 0;
int sensorValue = 0; 
int zero_pos = 0; 					//position of the zero in [a,b] relative to [0,SERIAL_COLUMNS]


int convert_to_serial(int value){
  // map it to the wanted range for the input
  int tempValue = map(value, 0, 1023, scope_range_min, scope_range_max);
  // now, map it to the serial line length
  return map(tempValue,scope_range_min,scope_range_max,0,SERIAL_COLUMNS);
}

void prepare_line(int val){
  memset(buf,' ', sizeof(buf));
  buf[zero_pos]=ZERO_CHAR;
  if(zero_pos>val) {
    //for(int i=val;i<zero_pos;i++) buf[i]=BAR_CHAR;
    memset(buf+val,BAR_CHAR,zero_pos-val);
  } else if(zero_pos<val){
    //for(int i=val;i>zero_pos;i--) buf[i]=BAR_CHAR;  
    memset(buf+zero_pos+1,BAR_CHAR,val-zero_pos);
  }
  buf[SERIAL_COLUMNS]=END_CHAR;
}

void serial_scope(int value){
  int val = convert_to_serial(value);
  prepare_line(val);
  Serial.println(buf);
}

void setup() {
  Serial.begin(115200);
  pinMode(pin,INPUT);
  
  //finds the zero of the scope graph
  if(scope_range_max <0) zero_pos=SERIAL_COLUMNS;
  else if(scope_range_min>0) zero_pos=0;
  else zero_pos = map(0,scope_range_min,scope_range_max,0,SERIAL_COLUMNS);
  
}

void loop() {
  sensorValue = analogRead(pin);
  thetime = millis();
  serial_scope(sensorValue);
  thetime = millis()-thetime;
  delay(thetime>sampling ? 0:(sampling-thetime));
}
