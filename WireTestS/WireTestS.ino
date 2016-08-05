

#include <Wire.h>

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600); 
  pinMode(13,OUTPUT);pinMode(8,OUTPUT);
}

bool l = true;

void loop() {
  delay(100);
  Serial.println(millis());
  if (l){
    digitalWrite(13,HIGH);
  } else {
    digitalWrite(13,LOW);
  }
  l = !l;
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  digitalWrite(8,HIGH);
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}
