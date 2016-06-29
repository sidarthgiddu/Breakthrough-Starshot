int senseTemp =A0;
int senseLight =A1;
int senseSolar =A2;
int senseCurrent =A3;

String data;
int cycle = 0;

void setup() {
   
  Serial.begin(9600);
  
}

void loop() {
  
  String tempString = String(getTempDegrees(senseTemp));
  data += tempString;
  cycle = cycle + 1;
  delay(5000);

  if (cycle > 20){
    Serial.println(data);
    int data[100];
    cycle = 0;
  }
}

int getTempDegrees(int TempPin) {
  // Returns the temperature of the sensor at pin senseTemp
  int temp = analogRead(TempPin);
  temp = map(temp, 0, 543, -50, 125);
  return temp;
}
int getLightlvl(int LightPin) {
  //Returns the lighting of the sensor at pin senselight (10k resistor)
  int light = analogRead(LightPin);
  light = map(light, 0, 775, 100, 0);
  //2.5 / 3.3 *1023
  return light;
}
int getCurrentAmp(int CurrentPin){
  //Returns Amperage of current sensors at senseCurrent 1v/amp (10k resistor)
  int current = analogRead(CurrentPin);
  current = map(current, 0, 1, 0, 1);
  //250 ma min, 430 ma max. 20k resistor might work better...
  return current;
}

