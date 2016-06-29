#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


long lastTime = millis();
Adafruit_LSM9DS0 imu = Adafruit_LSM9DS0();
int waitTime = 2000;
int cycle = 0;
int ledState = LOW;
int recentSlaveCom = 0;

//Pinout Numbers
const int DoorSens = 13;
const int Battery = 14;
const int RBRx = 0; const int RBTx = 1; const int RBSleep = 22;
const int RB_RI = 23; const int RB_RTS = 24; const int RB_CTS = 6;
const int SDApin = 20;
const int SCLpin = 21;

//Downlink Test Placeholders
int DLTime = 3000;
int lastDLTime = 0;
float placeHolderBattery = 0.3;
float placeHolderSolarXPlus = .1;
float placeHolderSolarXMinus = .2;
float placeHolderSolarYPlus = .3;
float placeHolderSolarYMinus = .4;
float placeHolderSolarZPlus = .5;
float placeHolderSolarZMinus = .6;
int placeHolderDoorSense = 1;
float placeHolderLightSense = 5.5;
float placeHolderAnalogTemp = 21.0;
int placeHoldernumPhotos = 10;

//
//  int DC = 1
//  int IDLE_COM = 2
//
//  //Downlink Flags
//  const int HEALTH = 1
//  const int GPS = 2
//  const int IMAGE = 3
//  const int LOW_POWER = 4;
//  const int current_DLFlags[2] = {0,0} //Distribute Later
//
//  //Operating Parameters
//  int DL_ATTEMPT_TIME = 10*60*1000 //Millis to try to downlink

class floatTuple
{
  public:
    float x;
    float y;
    float z;

    floatTuple(float a, float b, float c) {
      x = a;
      y = b;
      z = c;
    }
    void print() {
      Serial.print(x); Serial.print(" ");
      Serial.print(y); Serial.print(" ");
      Serial.print(z); Serial.println(" ");
    }
};

class sensorDataDownlink
{
    //floatTuple gyro;
    float Mag[3];
    float Gyro[3];
    int ImuTemp;
    float Battery;
    float SolarXPlus;
    float SolarXMinus;
    float SolarYPlus;
    float SolarYMinus;
    float SolarZPlus;
    float SolarZMinus;
    int DoorSense;
    float LightSense;
    float AnalogTemp;
    int numPhotos;

  public:
    sensorDataDownlink(floatTuple g, floatTuple M, int IT, float B, float SolarXP, float SolarXM,
                       float SolarYP, float SolarYM, float SolarZP, float SolarZM, int DS, float LS, float AT, int nP) {
      Gyro[0] = g.x; Gyro[1] = g.y; Gyro[2] = g.z;
      Mag[0] = M.x; Mag[1] = M.y; Mag[2] = M.z;
      ImuTemp = IT;
      Battery = B;
      SolarXPlus = SolarXP;
      SolarXMinus = SolarXM;
      SolarYPlus = SolarYP;
      SolarYMinus = SolarYM;
      SolarZPlus = SolarZP;
      SolarZMinus = SolarZM;
      DoorSense = DS;
      LightSense = LS;
      AnalogTemp = AT;
      numPhotos = nP;
    }
    String toString() {
      //Produces ASCII Output for Downlink
      String output = "";
      output += "Sensor Downlink |";
      output += "GX:" + String(Gyro[0]) + "|GY:" + String(Gyro[1]) + "|GyroZ:" + String(Gyro[2]) + "|";
      output += "MX:" + String(Mag[0]) + "|GyroY:" + String(Mag[1]) + "|GyroZ:" + String(Mag[2]) + "|";
      output += "IT:" + String(ImuTemp) + "|";
      output += "B:" + String(Battery) + "|";
      output += "SX+:" + String(SolarXPlus) + "|SX-:" + String(SolarXMinus) +
                "|SY+:" + String(SolarYPlus) + "|SY-:" + String(SolarYMinus) +
                "|SZ+:" + String(SolarZPlus) + "|SZ-:" + String(SolarZMinus) + "|";
      output += "DS:" + String(DoorSense) + "|";
      output += "LS:" + String(AnalogTemp) + "|";
      output += "nP:" + String(numPhotos) + "|";
      return output;
    }
};


int getTempDegrees(int TempPin) {
  // Returns the temperature of the sensor at pin senseTemp
  int temp = analogRead(TempPin);
  temp = map(temp, 0, 543, -50, 125);
  return temp;
}

void configureSensor()
{
  //set magnetometer range to +-2 gauss
  imu.setupMag(imu.LSM9DS0_MAGGAIN_2GAUSS);
  //set gyro range to +-245 degrees per second
  imu.setupGyro(imu.LSM9DS0_GYROSCALE_245DPS);
}

floatTuple getMagData(Adafruit_LSM9DS0 imu, int wT)
{
  int k = 0;
  int sumx = 0;
  int sumy = 0;
  int sumz = 0;
  long endTime = millis() + long(wT);
  while (millis() < endTime) {
    imu.read();
    sumx = sumx + (int)imu.magData.x;
    sumy = sumy + (int)imu.magData.y;
    sumz = sumz + (int)imu.magData.z;
    k++;
  } //32768
  floatTuple mData = floatTuple(sumx * (2.0 / 32768 / k), sumy * (2.0 / 32768 / k), sumz * (2.0 / 32768 / k));
  return mData;
}

floatTuple getGyroData(Adafruit_LSM9DS0 imu, int wT)
{
  int k = 0;
  int sumx = 0;
  int sumy = 0;
  int sumz = 0;
  long endTime = millis() + long(wT);
  while (millis() < endTime) {
    imu.read();
    sumx = sumx + (int)imu.gyroData.x;
    sumy = sumy + (int)imu.gyroData.y;
    sumz = sumz + (int)imu.gyroData.z;
    k++;
  } //32768
  floatTuple gData = floatTuple(sumx * (245.0 / 32768 / k), sumy * (245.0 / 32768 / k), sumz * (245.0 / 32768 / k));
  return gData;
}

int getImuTempData(Adafruit_LSM9DS0 imu, int wT)
{
  int k = 0;
  int sum = 0;
  long endTime = millis() + long(wT);
  while (millis() < endTime) {
    imu.read();
    sum = sum + (int)imu.temperature;
    k++;
  }
  return sum * (1.0 / k / 8.0); // 8 per Degree C
}


void initalizePinOut() {
  const int DoorSens = 13; pinMode(DoorSens, INPUT);
  const int DoorTrig = 5; pinMode(DoorSens, INPUT);
  const int Battery = A0; pinMode(Battery, INPUT);
  const int RBRx = 0; //RockBlock Serial Into FCom
  const int RBTx = 1; //RockBlock Serial Out of FCom
  const int RBSleep = 22; pinMode(RBSleep, OUTPUT);
  const int RB_RI = 23; pinMode(RBSleep, OUTPUT);
  const int RB_RTS = 24; pinMode(RBSleep, OUTPUT);
  const int RB_CTS = 6; pinMode(RBSleep, OUTPUT);
  const int SDApin = 20; //I2C Data
  const int SCLpin = 21; //I2C Clock
  const int SolarXPlus = A1; pinMode(SolarXPlus, INPUT); //Solar Current X+
  const int SolarXMinus = A2; pinMode(SolarXMinus, INPUT); //Solar Current X-
  const int SolarYPlus = A3; pinMode(SolarYPlus, INPUT); //Solar Current Y+
  const int SolarYMinus = A4; pinMode(SolarYMinus, INPUT); //Solar Current Y-
  const int SolarZPlus = A5; pinMode(SolarZPlus, INPUT); //Solar Current Z+
  const int SolarZMinus = 9; pinMode(SolarZMinus, INPUT); //Solar Current Z-
}


void setup()
{
  //Testing Code
  Serial.begin(9600); //Will Use for RockBlock
  //pinMode(12, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(12), forceSerialOff, LOW);
  //attachInterrupt(digitalPinToInterrupt(9), sendCommandToSlave, LOW);

  //Try to initialize and warn if we couldn't detect the chip
  while (!imu.begin());

  initalizePinOut();
  configureSensor(); //runs configureSensor function
  pinMode(A3, INPUT); //Temperature Sensor
  Wire.begin(); //Start i2c as master

}

//Testing Forced Terminator
void forceSerialOff() {
  digitalWrite(13, LOW);
  Serial.println("Force Terminated");
  Serial.end();
  while (1) {
    delay(200);
  }
}

//Testing Slave Communication
void sendCommandToSlave() { //Interrupt Pin
  if (!recentSlaveCom) {
    Wire.beginTransmission(8); // transmit to device #8
    Serial.println("Command Sent");
    Wire.write("Command Sent");   // sends String
    Wire.endTransmission();    // stop transmitting
    //delay(1000);
    recentSlaveCom += 5;
  }
}

int getCurrentAmp(int CurrentPin) { //NEEDS RESCALE
  //Returns Amperage of current sensors at senseCurrent 1v/amp (10k resistor)
  int current = analogRead(CurrentPin);
  current = map(current, 0, 1, 0, 1);
  //250 ma min, 430 ma max. 20k resistor might work better...
  return current;
}

void loop()
{

  //Testing IM
  //floatTuple mag = getMagData(imu, waitTime);
  //floatTuple gyro = getGyroData(imu, waitTime);
  //Serial.print("Gyro: "); gyro.print();
  //Serial.print("Mag:  "); mag.print();
  //Serial.println(temp2);

  //Blinker for Testing
  if (millis() - lastTime >= 100) {
    lastTime = millis();
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(13, ledState);
  }

  //Testing IMU and Sensor Downlink String Generator
  if (millis() - lastDLTime >= DLTime) {
    lastDLTime = millis();
    floatTuple mag = getMagData(imu, waitTime);
    floatTuple gyro = getGyroData(imu, waitTime);
    int temp1 = getImuTempData(imu, waitTime);

    sensorDataDownlink DL =
      sensorDataDownlink(gyro, mag, temp1, placeHolderBattery, placeHolderSolarXPlus, placeHolderSolarXMinus,
                         placeHolderSolarYPlus, placeHolderSolarYMinus, placeHolderSolarZPlus, placeHolderSolarZMinus,
                         placeHolderDoorSense, placeHolderLightSense, placeHolderAnalogTemp, placeHoldernumPhotos);
    Serial.println("Downlink String: ");
    String DLS = DL.toString();
    Serial.println(DLS);
    Serial.println(DLS.length());
    Serial.println(millis() - lastDLTime);
    lastDLTime = millis();
    
    
  }

  //Testing Iterators
  recentSlaveCom--;
  cycle++;
  //Serial.print("C: ");
  //Serial.println(cycle);
}












