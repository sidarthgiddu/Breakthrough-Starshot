int k = 25;
int cycle = 0;
void setup() {
  // put your setup code here, to run once:
  
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT); 
  pinMode(5, OUTPUT); 
  pinMode(6, OUTPUT);
  for (int i = 9; i <= 24; i++) {
    pinMode(i, OUTPUT);   // sets the LED on
  }
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  cycle++;
  for (int i = 0; i < k; i++) {
    digitalWrite(i, HIGH);   // sets the LED on
  }
  delay(200);
  for (int i = 0; i < k; i++) {             // waits for a second
    digitalWrite(i, LOW);    // sets the LED off
  }
  delay(200);
  Serial.println(cycle);
}
