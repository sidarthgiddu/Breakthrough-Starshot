#include <IridiumSBD.h>

const int RBSleep = 22;


bool ISBDCallback()
{
   unsigned ledOn = (bool)((millis() / 500) % 2);
   digitalWrite(13, ledOn); // Blink LED every second
    //Wire.println("CallBack");
   return true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  pinmode(RBSleep,OUTPUT);
  digitalWrite(RBSleep,HIGH);


}

void loop() {
  // put your main code here, to run repeatedly:

}
