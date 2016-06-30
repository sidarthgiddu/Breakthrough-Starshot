#include <Wire.h>

void setup() {
  pinMode(13,OUTPUT);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
}

int getTempDegrees(int TempPin) {
 // Returns the temperature of the sensor at pin senseTemp
 int temp = analogRead(TempPin);
 temp = map(temp, 0, 543, -50, 125);
 return temp;
}
int getLightLvl(int LightPin) {
 //Returns the lighting of the sensor at pin senselight (10k resistor)
 int light = analogRead(LightPin);
 light = map(light, 0, 775, 100, 0);
 //2.5 / 3.3 *1023
 return light;
}

void loop() {
  digitalWrite(13,HIGH);
  delay(50);
  digitalWrite(13,LOW);
  delay(20);
  digitalWrite(13,HIGH);
  delay(50);
  digitalWrite(13,LOW);
  delay(1000);
  Serial.println("No Command");
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  while (1 <= Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
}
