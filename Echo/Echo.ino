void echo() {
  delay(200);
  while(Serial.available()){
    Serial1.print((char)Serial.read());
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(19200);
  Serial1.print("AT&K0\r");
  Serial1.print("AT\r");
  delay(100);
  Serial1.print("AT+SBDIX\r");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  //Serial1.print("AT\r");
  while(Serial1.available()){
    Serial.print((char)Serial1.read());
  }
  if(Serial.available()){
    echo();
  }
  delay(1000);
  Serial.print('.');
}
