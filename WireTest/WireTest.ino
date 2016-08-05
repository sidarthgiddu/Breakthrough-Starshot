#include <Wire.h>

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  pinMode(13,OUTPUT);pinMode(8,OUTPUT);
  Serial.begin(9600);
}

byte x = 0;
bool l = true;
void loop() {
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write("x is ");        // sends five bytes
  Wire.write(x);              // sends one byte
  digitalWrite(8,HIGH);
  int err = Wire.endTransmission();    // stop transmitting
  digitalWrite(8,LOW);
  Serial.println(err);
  x++;
  delay(500);
  if (l){
    digitalWrite(13,HIGH);
  } else {
    digitalWrite(13,LOW);
  }
  l = !l;
}
