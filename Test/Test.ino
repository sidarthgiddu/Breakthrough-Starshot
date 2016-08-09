void setup() {
  // put your setup code here, to run once:
 Serial1.begin(19200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial1.print("AT\r");
  delay(100);
  while(Serial1.available()){
    Serial.print((char)Serial1.read());
  }

}
