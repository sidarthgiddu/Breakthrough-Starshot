#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

////Constant Initialization
long ledLastTime = millis();
int cycle = 0;
int ledState = LOW;
int manualTimeout = 10 * 1000;
int SlaveResets = 0;
int deployTimeOut = 30 * 1000;

//IMU and Sensor Test
bool SensorFetch = false;
bool imuWorking = true;
bool masterUseIMU = true;
Adafruit_LSM9DS0 imu = Adafruit_LSM9DS0();
int waitTime = 100;

//Slave Communication Test
bool slaveWorking = true;
int recentSlaveCom = 0;
bool TestSCom = true;
int lastSComTime = 0;
int lastSComAttempt = 0;
int SComTime = 4000;
int SlaveResetTimeOut = 30 * 1000;
int slaveDataSize = 100;

//Serial Command Test
int popTime = 4000;
int lastPopTime = 0;

//Commanded Action Flags
bool commandedSC = false;
bool commandedDL = false;

//Pinout Numbers
const int DoorSens = 13;
const int DoorTrig = 5;
const int Battery = A0;
const int RBRx = 0; //RockBlock Serial Into FCom
const int RBTx = 1; //RockBlock Serial Out of FCom
const int RBSleep = 22;
const int RB_RI = 23;
const int RB_RTS = 24;
const int RB_CTS = 6;
const int SDApin = 20; //I2C Data
const int SCLpin = 21; //I2C Clock
const int SolarXPlus = A1; //Solar Current X+
const int SolarXMinus = A2; //Solar Current X-
const int SolarYPlus = A3; //Solar Current Y+
const int SolarYMinus = A4; //Solar Current Y-
const int SolarZPlus = A5; //Solar Current Z+
const int SolarZMinus = 9; //Solar Current Z-
const int SlaveReset = 10; //Slave Fault Handing (via Hard Reset)
const int DoorMagEnable = 11; //Allow Door Magnetorquer to work

//Downlink Test Placeholders
int DLTime = 6000;
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

class commandBuffer {
  public:
    int commandStack[200][2];
    int openSpot;
    commandBuffer() {
      commandStack[200][2] = { -1};
      openSpot = 0;
    }
    void print() {
      int i = 0;
      Serial.print("cBuf = [");
      int endT = millis() + manualTimeout;
      while (i < 200 && millis() < endT) {
        if (commandStack[i][0] == -1 && commandStack[i][1] == -1) {
          break;
        }
        Serial.print(commandStack[i][0]);
        Serial.print(":");
        Serial.print(commandStack[i][1]);
        Serial.print("|");
        i++;
      }
      Serial.println("]");
    }
};
commandBuffer cBuf;

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
    int IMUWorking;
    int SlaveWorking;
    int Resets;

  public:
    sensorDataDownlink(floatTuple g, floatTuple M, int IT, float B, float SolarXP, float SolarXM,
                       float SolarYP, float SolarYM, float SolarZP, float SolarZM, int DS, float LS,
                       float AT, int nP, bool IMUW, bool SW, int R) {
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
      IMUWorking = IMUW;
      SlaveWorking = SW;
      Resets = R;
    }
    String toString() {
      //Produces JSON Output in ASCII  for Downlink
      String output = "";
      output += "{";
      output += "GX:" + String(Gyro[0]) + ",GY:" + String(Gyro[1]) + ",GZ:" + String(Gyro[2]) + ",";
      output += "MX:" + String(Mag[0]) + ",MY:" + String(Mag[1]) + ",MZ:" + String(Mag[2]) + ",";
      output += "IT:" + String(ImuTemp) + ",";
      output += "B:" + String(Battery) + ",";
      output += "SX+:" + String(SolarXPlus) + ",SX-:" + String(SolarXMinus) +
                ",SY+:" + String(SolarYPlus) + ",SY-:" + String(SolarYMinus) +
                ",SZ+:" + String(SolarZPlus) + ",SZ-:" + String(SolarZMinus) + ",";
      output += "DS:" + String(DoorSense) + ",";
      output += "LS:" + String(AnalogTemp) + ",";
      output += "nP:" + String(numPhotos) + ",";
      output += "IW:" + String(IMUWorking) + ",";
      output += "SW:" + String(SlaveWorking) + ",";
      output += "Rs:" + String(Resets) + "}";
      return output;
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//IMU Code

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Helper Functions

void initalizePinOut() {
  const int DoorSens = 13; pinMode(DoorSens, INPUT);
  const int DoorTrig = 5; pinMode(DoorTrig, OUTPUT);
  const int Battery = A0; pinMode(Battery, INPUT);
  const int RBRx = 0; //RockBlock Serial Into FCom
  const int RBTx = 1; //RockBlock Serial Out of FCom
  const int RBSleep = 22; pinMode(RBSleep, OUTPUT);
  const int RB_RI = 23; pinMode(RB_RI, INPUT);
  const int RB_RTS = 24; pinMode(RB_RTS, INPUT);
  const int RB_CTS = 6; pinMode(RB_CTS, INPUT);
  const int SDApin = 20; //I2C Data
  const int SCLpin = 21; //I2C Clock
  const int SolarXPlus = A1; pinMode(SolarXPlus, INPUT); //Solar Current X+
  const int SolarXMinus = A2; pinMode(SolarXMinus, INPUT); //Solar Current X-
  const int SolarYPlus = A3; pinMode(SolarYPlus, INPUT); //Solar Current Y+
  const int SolarYMinus = A4; pinMode(SolarYMinus, INPUT); //Solar Current Y-
  const int SolarZPlus = A5; pinMode(SolarZPlus, INPUT); //Solar Current Z+
  const int SolarZMinus = 9; pinMode(SolarZMinus, INPUT); //Solar Current Z-
  const int SlaveReset = 10; pinMode(SolarZMinus, INPUT); //Slave Fault Handing (via Hard Reset)
  const int DoorMagEnable = 11; pinMode(DoorMagEnable, OUTPUT); //Allow Door Magnetorquer to work
}

int getCurrentAmp(int CurrentPin) { //NEEDS RESCALE
  //Returns Amperage of current sensors at senseCurrent 1v/amp (10k resistor)
  int current = analogRead(CurrentPin);
  current = map(current, 0, 1, 0, 1);
  //250 ma min, 430 ma max. 20k resistor might work better...
  return current;
}

//Determine Remaining RAM
extern "C" char *sbrk(int i);
int freeRam () {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////Parser Functions

void buildBuffer(String com)
{
  int commandData;
  int commandType;
  String comRemaining = com;
  bool loop = true;
  while (loop) {
    commandType = (com.substring(0, com.indexOf(","))).toInt();
    commandData = (com.substring(com.indexOf(",") + 1, com.indexOf("."))).toInt();
    cBuf.commandStack[cBuf.openSpot][0] = commandType;
    cBuf.commandStack[cBuf.openSpot][1] = commandData;
    if (com.indexOf(".") == com.length() - 1) {
      loop = false;
      Serial.println("Finished Adding Commands");
    } else {
      com = com.substring(com.indexOf(".") + 1);
    }
    cBuf.openSpot++;
  }
}

boolean isInputValid(String input) {
  //Check if incoming command is valid
  int lastPunc = 0; //1 if ",", 2 if ".", 0 Otherwise
  bool valid = true;
  int q = 0;
  int l = input.length();
  int endT = manualTimeout + millis();
  while (q < l) {
    char currentChar = input[q];
    q++;

    if (millis() > endT) {
      valid = false;
      break;
    }

    if (isPunct(currentChar)) {
      if (currentChar == (',')) {
        //Check if last was a period
        //Serial.println("Comma Found");
        if (lastPunc == 0 || lastPunc == 2) {
          //Serial.println("Comma OK");
          lastPunc = 1;
        } else {
          //Serial.println("2 Commas");
          valid = false;
          break;
        }
      } else if (currentChar == ('.')) {
        //Serial.println("Period Found");
        if (lastPunc == 1) {
          //Serial.println("Period ok");
          lastPunc = 2;
        } else {
          //Serial.println("2 Periods or No prior comma");
          valid = false;
          break;
        }
      } else {
        //Serial.println("Invalid Punc");
        valid = false;
        break;
      }
    } else if (isAlpha(currentChar)) {
      //Serial.println("Alpha");
      valid = false;
      break;
    } else if (isSpace(currentChar)) {
      //Serial.println("Space");
      valid = false;
      break;
    }

    //Detect no ending period
    if (q == input.length() - 1) {
      if (input[q] != '.') {
        //Serial.println("No Ending");
        valid = false;
        break;
      }
    }
    //Null Character in the middle
    if (currentChar == '\0' && q != input.length() - 1) {
      valid = false;
      break;
    }
  }
  return valid;
}

void popCommand() {
  //Process an Incoming Command
  Serial.println ("Executing Command:");
  if (cBuf.openSpot > 0) {
    Serial.println (cBuf.openSpot - 1);
    int currentCommand[2] = {cBuf.commandStack[cBuf.openSpot - 1][0], cBuf.commandStack[cBuf.openSpot - 1][1]};
    Serial.print(currentCommand[0]);
    Serial.print(":");
    Serial.println(currentCommand[1]);
    cBuf.commandStack[cBuf.openSpot - 1][0] = -1;
    cBuf.commandStack[cBuf.openSpot - 1][1] = -1;
    cBuf.openSpot --;
    //Place the Command ID in the "#"
    switch (currentCommand[1]) {
      case (11):
        deployTimeOut = (currentCommand[2]);
        break;
      case (12):
        manualTimeout = (currentCommand[2]);
        break;
    }

  } else {
    Serial.println("No Command");
  }
}

void readSerialAdd2Buffer() {
  //Modify for RB Communication


  if (Serial.available() > 0) {
    Serial.println("Recieving Command");
    String comString = "";
    while (Serial.available() > 0) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      comString += inChar;
    }
    if (isInputValid(comString)) {
      Serial.println("Command is Valid");
      buildBuffer(comString);
      Serial.println("Built Command Buffer Successfully");
    } else {
      Serial.println("Invalid Command");
    }
  }
}

void sendSCommand(char data[]) {
  Wire.beginTransmission(8); // transmit to device #8
  Serial.print("Command Sent to Slave: ");
  Serial.println(data);
  Wire.write(data);   // sends String
  Wire.endTransmission();    // stop transmitting
}

String requestFromSlave() {
  Serial.println("Requesting");
  Wire.requestFrom(8, 100, true); // request 16 bytes from slave device #8
  delay(50);
  String res = "";
  int endTime = millis() + manualTimeout;
  Serial.println("Here");
  while (Wire.available() && millis() < endTime) { // slave may send less than requested
    res += (char)Wire.read(); // receive a byte as character
  }
  return res;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//// Main Loop

void setup()
{

  initalizePinOut();
  digitalWrite(SlaveReset, HIGH); //Enable Slave

  //Testing Code
  Serial.begin(9600); //Will Use for RockBlock
  delay(1000);
  //pinMode(12, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(12), forceSerialOff, LOW);
  //attachInterrupt(digitalPinToInterrupt(9), sendCommandToSlave, LOW);
  for (int i = 0 ; i < 10; i++) {
    Serial.println("Here1");
  }
  //Try to initialize and warn if we couldn't detect the chip
  int endT = millis() + manualTimeout;

  while (!imu.begin() && millis() < endT) {
    Serial.println("IMU Not working");
  }


  cBuf = commandBuffer();

  configureSensor(); //runs configureSensor function
  //pinMode(A3, INPUT); //Temperature Sensor
  Wire.begin(); //Start i2c as master

  for (int i = 0 ; i < 10; i++) {
    Serial.println("Here");
  }
}

void loop() {
  //Initalize Per-Loop Variables
  String SlaveResponse = "";
  floatTuple mag = floatTuple(0, 0, 0);
  floatTuple gyro = floatTuple(0, 0, 0);
  float currents[6] = {999};

  if (SensorFetch) {
    if (imuWorking){
    mag = getMagData(imu, waitTime);
    gyro = getGyroData(imu, waitTime);
    }
    currents[0] = getCurrentAmp(SolarXPlus);
    currents[1] = getCurrentAmp(SolarXMinus);
    currents[2] = getCurrentAmp(SolarYPlus);
    currents[3] = getCurrentAmp(SolarYMinus);
    currents[4] = getCurrentAmp(SolarZPlus);
    currents[5] = getCurrentAmp(SolarZMinus);
  }
  //readSerialAdd2Buffer();
  //if (millis() - lastPopTime >= popTime) {
  //  popCommand();
  //}

  //Blinker for Testing
  if (millis() - ledLastTime >= 300) {
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(13, ledState);
    Serial.print("Running: ");
    Serial.println(millis() - ledLastTime);
    ledLastTime = millis();
  }

  //Testing IMU and Sensor Downlink String Generator
  if (millis() - lastDLTime >= DLTime || commandedDL) {
    floatTuple mag = floatTuple(0, 0, 0);
    floatTuple gyro = floatTuple(0, 0, 0);
    if (imuWorking) {
      mag = getMagData(imu, waitTime);
      gyro = getGyroData(imu, waitTime);
    }
    int temp1 = 99;//getImuTempData(imu, waitTime);

    sensorDataDownlink DL = sensorDataDownlink(gyro, mag, temp1, placeHolderBattery,
                            placeHolderSolarXPlus, placeHolderSolarXMinus,
                            placeHolderSolarYPlus, placeHolderSolarYMinus,
                            placeHolderSolarZPlus, placeHolderSolarZMinus,
                            placeHolderDoorSense, placeHolderLightSense,
                            placeHolderAnalogTemp, placeHoldernumPhotos,
                            imuWorking, slaveWorking, SlaveResets);
    //Send Data to RockBlock via Serial
    Serial.println("Downlink String: ");
    String DLS = DL.toString();
    Serial.println(DLS);
    Serial.println(DLS.length());
    lastDLTime = millis();
  }
  //
  //  //Test Slave Communication
  //  //char SCommand[] = "1,1.";
  if (TestSCom) {
    if (millis() - lastSComAttempt >= SComTime || commandedSC) {
      lastSComAttempt = millis();
      Serial.print("Slave Status Report: ");
      //sendSCommand(SCommand);
      SlaveResponse = requestFromSlave();
      Serial.println(SlaveResponse);

      if (!SlaveResponse.equals("")) {
        lastSComTime = millis(); //Reset Timeout if Com is successful

        //Valid Reply From Slave:
        Serial.println("Valid Reply");
      } else {
        //No Reply From Slave
        if (millis() - lastSComTime > SlaveResetTimeOut) {

          //No Communication for (SlaveResetTimeOut) ms
          slaveWorking = false;

          if (!slaveWorking) {
            digitalWrite(SlaveReset, LOW);
            delay(50);
            digitalWrite(SlaveReset, HIGH);
            SlaveResets++;
            //delay(3000);
            //Wire.beginTransmission(8); // transmit to device #8
            //Wire.write("Reset");   // sends String
            //Wire.endTransmission();    // stop transmitting

            Serial.println("Slave Reset");
          }
        } else {
          Serial.print("No Reply From Slave for ");
          Serial.print((millis() - lastSComTime) / 1000.0);
          Serial.println(" seconds");
        }
      }
    }
  }



  //Testing Iterators
  recentSlaveCom--;
  cycle++;
  //Serial.print("C: ");
  //Serial.println(cycle);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//bool deploy () {
//  Serial.println("Executing Deployment");
//  int numImages = 10;
//  String imageBuffer[numImages];
//  int gData[1000][3];
//  int mData[1000][3];
//  int pData[1000];
//  int i = 0;
//  int dT = 1.5 * 60 * 1000;
//
//  digitalWrite(DoorTrig, HIGH);
//  int ForceEndTime = millis() + deployTimeOut;
//  while (digitalRead(DoorSens) && millis() < ForceEndTime)
//
//    if (millis() > ForceEndTime) {
//      return false;
//    }
//
//  int endTime = millis() + (dT);
//  int imuTime = 200;
//  int imuI = 0;
//  int photoTime = 1000; //Set with variable
//  int nextImuT = millis() + imuTime;
//  int nextPhotoT = millis() + photoTime;
//  int photoI = 0;
//
//
//  while (millis() < endTime) {
//    if (millis() > nextImuT) {
//      //Imu gathering code
//      imuI++;
//      nextImuT = millis() + imuTime;
//      //placeholdergetPhotoresistorData;
//      //placeholdergetMagData;
//      //placeholdergetGyroData;
//    }
//    if (millis() > nextPhotoT) {
//      //takePic();
//      photoI++;
//      nextPhotoT = millis() + imuTime;
//    }
//  }
//  Serial.print("Photoresistor: ");
//  //Serial.println(pData);
//  Serial.print("Magnetometer: ");
//  //Serial.println(Mdata);
//  Serial.print("Gyroscope: ");
//  //  Serial.println(Gdata);
//
//  if (photoI + 1 == numImages)
//  {
//    Serial.println ("Image Capture Succesful");
//  } else {
//    Serial.println ("Error Occured during Image Capture");
//  }
//  return true;
//}
//
//
//







