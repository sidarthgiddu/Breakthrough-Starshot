

int RocRecive = 0;
int RocTransmit = 1;
int TempRead = A0;
int CurrentRead = A1;
int GyroRead = A2;
int MagRead = A3;

String inputString = "";
boolean stringComplete = false;

//Very incomplete

void setup() {
Serial.begin(9600);
inputString.reserve(200);
//String bit capacity
}

void loop() {
if (stringComplete) {
  Serial.println(inputString);
  inputString ="";
  stringComplete = false;
  //Clears string
  
  tempSensor = analogRead(A0);
  currentSensor = analogRead(A1);
  gyroSensor = analogRead(A2);
  magnetSensor = analogRead(A3);
  Serial.print("Temp, (TempSensor) ");
  Serial.print("Current, (CurrentSensor) ");
  Serial.print("Gyro, (GyroSensor) ");
  Serial.println("Mag, (MagnetSensor) ");
  }
}
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}



