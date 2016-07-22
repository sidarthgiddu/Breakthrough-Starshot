int pwm1 = 100;
int pwm2 = 50;
int pwm3 = 200;
bool p1 = true;
bool p2 = true;
bool p3 = true;

int cycleLength = 600; //Microseconds
unsigned long LastCycle = 0;
unsigned long NextCycle = cycleLength;
bool ADCS = true;
bool startCycle = true;

int dur1 = map(pwm1, 0, 255, 0, cycleLength);
int dur2 = map(pwm2, 0, 255, 0, cycleLength);
int dur3 = map(pwm3, 0, 255, 0, cycleLength);


void setup() {
  // put your setup code here, to run once:
  //pinMode(11, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  
//  pinMode(13, OUTPUT);

 
  //digitalWrite(A1, HIGH);
  //digitalWrite(A4, HIGH);
  //
  delay(3000);
  Serial.println(dur1);
  delay(3000);
}



void loop() {
  // put your main code here, to run repeatedly:
  if (ADCS) {
    long ms = micros();
    if (startCycle) {
      digitalWrite(A1, HIGH);
      digitalWrite(A2, HIGH);
      digitalWrite(A3, HIGH);
      //while (1);
      startCycle = false;
      NextCycle = ms + cycleLength;
      LastCycle = ms;
      p1 = true;
      p2 = true;
      p3 = true;
    }
    if (p1 && (ms - LastCycle >= dur1)) {
      //Serial.println("Here");
      digitalWrite(A1, LOW);
      p1 = false;
    }
    if (p2 && (ms - LastCycle >= dur2)) {
      digitalWrite(A2, LOW);
      p2 = false;
    }
    if (p3 && (ms - LastCycle >= dur3)) {
      digitalWrite(A3, LOW);
      p3 = false;
    }
    if (ms > NextCycle) {
      //Serial.print("End");
      startCycle = true;
    }
  }
}
