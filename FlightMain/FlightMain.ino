#include <SD.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


//State Machine Definition
#define DORMANT_CRUISE 1
#define INITALIZATION  2
#define INIT_SLEEP     3
#define NORMAL_OPS     4
#define ECLIPSE        5
#define DEPLOY_ARMED   6
#define DEPLOY_VERIF   7
#define DEPLOY_DOWN_LK 8
#define LOW_POWER      9
#define SAFE_MODE     10
const int IMAGE_REQUEST = 888;
bool WireConnected = true;
bool LED_NOT_DOOR = false;

////Constant Initialization
unsigned long cruiseEnd = 30 * 60 * 1000;
unsigned long ledLastTime = millis();
long cycle = 0;
int ledState = LOW;
unsigned long manualTimeout = 10 * 1000;
int SlaveResets = 0;
unsigned long deployTimeOut = 30 * 1000;

//State Machine Transition Flags and Times
unsigned long eclipseEntry;
unsigned long lowPowerEntry;
unsigned long normOpEntry;
unsigned long initEntry;
unsigned long initSleepEntry;
unsigned long deployArmedEntry;
unsigned long deployVEntry;
unsigned long forceExitEclipseTime = 50 * 60 * 1000;
unsigned long forceLPEclipseTime = 180 * 60 * 1000;
unsigned long lastAccelTime;

int LightThreshold = 80; //0-100
float LV_Threshold = 3.2;
float HV_Threshold = 3.4;
float EclipseAmp_Threshold = 0.01;
int LT_Threshold = -10; //C
int HT_Threshold = 60; //C

//RockBlock Test
unsigned long lastRBCheck = 0;
long RBCheckTime = 6000;

//IMU and Sensor Test
bool SensorFetch = false;
bool masterUseIMU = true;
Adafruit_LSM9DS0 imu = Adafruit_LSM9DS0();
int imuSensorDwell = 100; //Averaging Time Non-BLOCKING!
unsigned long lastIMUTime = 0;
int IMUrecords = 0;
float gyroThresholdY = 3;
float gyroThresholdX = 3;

//Slave Communication Test
bool slaveWorking = true;
long recentSlaveCom = 0;
bool TestSCom = true;
long lastSComTime = 0;
long lastSComAttempt = 0;
int SComTime = 2000;
unsigned long SlaveResetTimeOut = 30 * 1000;
int slaveDataSize = 100;

//ADCS Test
unsigned long LastSpinCheckT = 0;
unsigned long SpinCheckTime = 5 * 60 * 1000;
float OmegaThreshold = 30; //Degrees per second

//Serial Command Test
int popTime = 4000;
unsigned long lastPopTime = 0;

//RockBlock Test
//IridiumSBD iSBD = IridiumSBD(Serial, 22); //RBSleep Pin
unsigned long SBDCallBackStartTime = 0;
unsigned long RBForcedTimeout = 30 * 1000;

//Commanded Action Flags
bool commandedSC = false;
bool commandedDL = false;

//Pinout Numbers
//TO DO
const int DoorSensePin = 13;
const int DoorTrig = 5;
const int BatteryPin = A0;
const int RBRx = 0; //RockBlock Serial Into FCom
const int RBTx = 1; //RockBlock Serial Out of FCom
const int RBSleep = 22;
const int RB_RI = 23;
const int RB_RTS = 24;
const int RB_CTS = 6;
const int SDApin = 20; //I2C Data
const int SCLpin = 21; //I2C Clock
const int SolarXPlusPin = A1; //Solar Current X+
const int SolarXMinusPin = A2; //Solar Current X-
const int SolarYPlusPin = A3; //Solar Current Y+
const int SolarYMinusPin = A4; //Solar Current Y-
const int SolarZPlusPin = A5; //Solar Current Z+
const int SolarZMinusPin = 9; //Solar Current Z-
const int SlaveReset = 10; //Slave Fault Handing (via Hard Reset)
const int DoorMagEnable = 11; //Allow Door Magnetorquer to work

//Downlink Test Placeholders
long DLTime = 6137;
long lastDLTime = 0;
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

//Deployment Test
bool DA_Initialize = true;

//Image Downlink Test
#define chipSelect 4
#define DLSize 320

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
      Serial.print(x); Serial.print(F(" "));
      Serial.print(y); Serial.print(F(" "));
      Serial.print(z); Serial.println(F(" "));
    }
};

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void printArray(uint8_t * arr, int s) {
  if (s) {
    for (int i = 0; i < s; i++) {
      //    for (int j = 0; j < 8; j++) {
      //      if (arr[i] < pow(2, j))
      //        Serial.print(B0);
      //    }
      //      if (i % DLSize == 0) {
      //        Serial.println(" ");
      //      }
      print_binary(arr[i], 8);
      Serial.print(" ");
    }
    Serial.println("");
  }
}

class RAMImage {
  public:
    uint8_t a0[DLSize];
    uint8_t a1[DLSize];
    uint8_t a2[DLSize];
    uint8_t a3[DLSize];
    uint8_t a4[DLSize];
    uint8_t a5[DLSize];
    uint8_t a6[DLSize];
    uint8_t a7[DLSize];
    uint8_t a8[DLSize];
    uint8_t a9[DLSize];
    uint8_t a10[DLSize];
    uint8_t a11[DLSize];
    uint8_t a12[DLSize];
    uint8_t a13[DLSize];
    uint8_t a14[DLSize];
    uint8_t a15[DLSize];

    int sizeArray[16] = {0};
    int finalIndex = 0;
    String Filename;

    int photoSize() {
      int sum = 0;
      for (int i = 0; i < 16; i++) {
        sum += sizeArray[i];
      }
      return sum;
    }

    void printRI() {
      delay(100);
      Serial.println("\nRAM Loaded Image: " + Filename);
      Serial.println("   Segments: " + String(finalIndex + 1));
      int sum = 0; for (int i = 0; i <= finalIndex; i++) {
        sum += sizeArray[i];
      }
      Serial.println("   Total Size: " + String(sum));
      Serial.println("\nBinary Form:\n");

      printArray(a0, sizeArray[0]);
      printArray(a1, sizeArray[1]);
      printArray(a2, sizeArray[2]);
      printArray(a3, sizeArray[3]);
      printArray(a4, sizeArray[4]);
      printArray(a5, sizeArray[5]);
      printArray(a6, sizeArray[6]);
      printArray(a7, sizeArray[7]);
      printArray(a8, sizeArray[8]);
      printArray(a9, sizeArray[9]);
      printArray(a10, sizeArray[10]);
      printArray(a11, sizeArray[11]);
      printArray(a12, sizeArray[12]);
      printArray(a13, sizeArray[13]);
      printArray(a14, sizeArray[14]);
      printArray(a15, sizeArray[15]);
    }

    RAMImage() {
      sizeArray[16] = {0};
      finalIndex = 0;
      Filename = "";

      a0[DLSize] = {0};
      a1[DLSize] = {0};
      a2[DLSize] = {0};
      a3[DLSize] = {0};
      a4[DLSize] = {0};
      a5[DLSize] = {0};
      a6[DLSize] = {0};
      a7[DLSize] = {0};
      a8[DLSize] = {0};
      a9[DLSize] = {0};
      a10[DLSize] = {0};
      a11[DLSize] = {0};
      a12[DLSize] = {0};
      a13[DLSize] = {0};
      a14[DLSize] = {0};
      a15[DLSize] = {0};
      //Blank
    }

    void store(uint8_t * data, int dataSize) {
      //Stores an entire Image, splitting it into segments
      //Filename = "Transfered Image";
      int dsize = dataSize;
      int index = 0;
      while (true) {
        if (dsize - DLSize > 0) {
          sizeArray[index] = DLSize;
          dsize -= DLSize;
          //Serial.println(sizeArray[index]);
        } else {
          sizeArray[index] = dsize;
          finalIndex = index;
          Serial.println(sizeArray[index]);
          break;
        }
        index++;
      }

      if (dataSize >= 0) {
        for (int i = 0; i < min(dataSize, DLSize); i++) {
          a0[i] = data[i];
        }
      }
      if (dataSize >= DLSize * 1) {
        for (int i = 0; i < min(dataSize - DLSize, DLSize); i++) {
          a1[i] = data[(DLSize * 1) + i];
        }
      }
      if (dataSize >= DLSize * 2) {
        for (int i = 0; i < min(dataSize - DLSize * 2, DLSize); i++) {
          a2[i] = data[(DLSize * 2) + i];
        }
      }
      if (dataSize >= DLSize * 3) {
        for (int i = 0; i < min(dataSize - DLSize * 3, DLSize); i++) {
          a3[i] = data[(DLSize * 3) + i];
        }
      }
      if (dataSize >= DLSize * 4) {
        for (int i = 0; i < min(dataSize - DLSize * 4, DLSize); i++) {
          a4[i] = data[(DLSize * 4) + i];
        }
      }
      if (dataSize >= DLSize * 5) {
        for (int i = 0; i < min(dataSize - DLSize * 5, DLSize); i++) {
          a5[i] = data[(DLSize * 5) + i];
        }
      }
      if (dataSize >= DLSize * 6) {
        for (int i = 0; i < min(dataSize - DLSize * 6, DLSize); i++) {
          a6[i] = data[(DLSize * 6) + i];
        }
      }
      if (dataSize >= DLSize * 7) {
        for (int i = 0; i < min(dataSize - DLSize * 7, DLSize); i++) {
          a7[i] = data[(DLSize * 7) + i];
        }
      }
      if (dataSize >= DLSize * 8) {
        for (int i = 0; i < min(dataSize - DLSize * 8, DLSize); i++) {
          a8[i] = data[(DLSize * 8) + i];
        }
      }
      if (dataSize >= DLSize * 9) {
        for (int i = 0; i < min(dataSize - DLSize * 9, DLSize); i++) {
          a9[i] = data[(DLSize * 9) + i];
        }
      }
      if (dataSize >= DLSize * 10) {
        for (int i = 0; i < min(dataSize - DLSize * 10, DLSize); i++) {
          a10[i] = data[(DLSize * 10) + i];
        }
      }
      if (dataSize >= DLSize * 11) {
        for (int i = 0; i < min(dataSize - DLSize * 11, DLSize); i++) {
          a11[i] = data[(DLSize * 11) + i];
        }
      }
      if (dataSize >= DLSize * 12) {
        for (int i = 0; i < min(dataSize - DLSize * 12, DLSize); i++) {
          a12[i] = data[(DLSize * 12) + i];
        }
      }
      if (dataSize >= DLSize * 13) {
        for (int i = 0; i < min(dataSize - DLSize * 13, DLSize); i++) {
          a13[i] = data[(DLSize * 13) + i];
        }
      }
      if (dataSize >= DLSize * 14) {
        for (int i = 0; i < min(dataSize - DLSize * 14, DLSize); i++) {
          a14[i] = data[(DLSize * 14) + i];
        }
      }
      if (dataSize >= DLSize * 15) {
        for (int i = 0; i < min(dataSize - DLSize * 15, DLSize); i++) {
          a15[i] = data[(DLSize * 15) + i];
        }
      }
    }

    void store(uint8_t * data, int dataSize, int index) {
      //Stores a segment in the RAMImage
      int bytes = min(DLSize, dataSize);
      if (index > finalIndex) {
        finalIndex = index;
      }
      switch (index) {
        case 0:
          for (int i = 0; i < bytes; i++) {
            a0[i] = data[i];
          }
          break;
        case 1:
          for (int i = 0; i < bytes; i++) {
            a1[i] = data[i];
          }
          break;
        case 2:
          for (int i = 0; i < bytes; i++) {
            a2[i] = data[i];
          }
          break;
        case 3:
          for (int i = 0; i < bytes; i++) {
            a3[i] = data[i];
          }
          break;
        case 4:
          for (int i = 0; i < bytes; i++) {
            a4[i] = data[i];
          }
          break;
        case 5:
          for (int i = 0; i < bytes; i++) {
            a5[i] = data[i];
          }
          break;
        case 6:
          for (int i = 0; i < bytes; i++) {
            a6[i] = data[i];
          } break;
        case 7:
          for (int i = 0; i < bytes; i++) {
            a7[i] = data[i];
          } break;
        case 8:
          for (int i = 0; i < bytes; i++) {
            a8[i] = data[i];
          } break;
        case 9:
          for (int i = 0; i < bytes; i++) {
            a9[i] = data[i];
          } break;
        case 10:
          for (int i = 0; i < bytes; i++) {
            a10[i] = data[i];
          } break;
        case 11:
          for (int i = 0; i < bytes; i++) {
            a11[i] = data[i];
          } break;
        case 12:
          for (int i = 0; i < bytes; i++) {
            a12[i] = data[i];
          } break;
        case 13:
          for (int i = 0; i < bytes; i++) {
            a13[i] = data[i];
          } break;
        case 14:
          for (int i = 0; i < bytes; i++) {
            a14[i] = data[i];
          } break;
        case 15:
          for (int i = 0; i < bytes; i++) {
            a15[i] = data[i];
          } break;
      }
    }
    uint8_t * get(int index) {
      switch (index) {
        case 0: return a0;
        case 1: return a1;
        case 2: return a2;
        case 3: return a3;
        case 4: return a4;
        case 5: return a5;
        case 6: return a6;
        case 7: return a7;
        case 8: return a8;
        case 9: return a9;
        case 10: return a10;
        case 11: return a11;
        case 12: return a12;
        case 13: return a13;
        case 14: return a14;
        case 15: return a15;
      }
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//IMU Code

floatTuple getMagData(Adafruit_LSM9DS0 imu) {
  floatTuple mData = floatTuple((int)imu.magData.x * (2.0 / 32768),
                                (int)imu.magData.y * (2.0 / 32768),
                                (int)imu.magData.z * (2.0 / 32768));
  return mData;
}

floatTuple getGyroData(Adafruit_LSM9DS0 imu) {
  floatTuple gData = floatTuple((int)imu.gyroData.x * (245.0 / 32768),
                                (int)imu.gyroData.y * (245.0 / 32768),
                                (int)imu.gyroData.z * (245.0 / 32768));
  return gData;
}

int getImuTempData(Adafruit_LSM9DS0 imu) {
  return (int)imu.temperature ;// * (1.0 / 8.0); // 8 per Degree C
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
      Serial.print(F("cBuf = ["));
      int endT = millis() + manualTimeout;
      while (i < 200 && millis() < endT) {
        if (commandStack[i][0] == -1 && commandStack[i][1] == -1) {
          break;
        }
        Serial.print(commandStack[i][0]);
        Serial.print(F(":"));
        Serial.print(commandStack[i][1]);
        Serial.print(F("|"));
        i++;
      }
      Serial.println(F("]"));
    }
};
commandBuffer cBuf;

class masterStatus {
  public:
    //Adafruit_LSM9DS0 imu;

    RAMImage imageR;
    int currentSegment;
    int RequestingImageStatus;

    bool hardwareAvTable[10];//Hardware Avaliability Table
    //[Imu, SX+,SX-,SY+, SY-, SZ+, SZ-,Temp,DoorSense,LightSense]
    //Fix PopCommand Av Swap Limit if changed

    int State;
    int NextState;
    float Mag[3];
    float Gyro[3];
    float Accel[3];
    float MagAcc[3];
    float GyroAcc[3];
    float AccelAcc[3];
    int TempAcc;
    int ImuTemp;
    float Battery;
    float BatteryAcc;
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
    int SlaveWorking;
    int Resets;

    bool PayloadDeployed;
    int missionStatus; //0=incomplete, 1=success, 2=failure
    int deploySetting; //0=DoorSensor OR Light, 1 = Just Light, 2 = Just DoorSensor


    float * IMUData[3];
    float * LIGHTData;
    float * AccelData[3];
    int dataIndex;
    int accelIndex;

    bool ADCS_Active;
    int MResets;
    int CurXDir; //-1 or 1 for Coil Current Direction
    int CurXPWM; // 0 to 255 for Coil Current Level
    int CurYDir; //-1 or 1 for Coil Current Direction
    int CurYPWM; // 0 to 255 for Coil Current Level
    int CurZDir; //-1 or 1 for Coil Current Direction
    int CurZPWM; // 0 to 255 for Coil Current Level

    //ROCKBLOCKVARIABLES

    bool AttemptingLink;
    int MOStatus;
    int MOMSN;
    int MTStatus;
    int MTMSN;
    int MTLength;
    int MTQueued;
    String SBDRT;
    int LastMsgType;
    int LastSMsgType;
    //LastMsgType values:
    //0 = inval
    //1 = ok
    //2 = ring
    //3 = error
    //4 = ready


    masterStatus( ) {
      State = 4; //Normal Ops
      NextState = State;
      PayloadDeployed = false;
      deploySetting = 0;
      //imu = Adafruit_LSM9DS0(); //imu.begin();

      bool hardwareAvTable[10] = {true}; // Hardware Avaliability Table
      //[Imu, SX+,SX-,SY+, SY-, SZ+, SZ-,Temp,DoorSense,LightSense]

      Gyro[3] = {0};
      Mag[3] = {0};
      GyroAcc[3] = {0};
      MagAcc[3] = {0};
      TempAcc = 0;
      ImuTemp = 0;
      Battery = 3.8;
      BatteryAcc = 0;
      SolarXPlus = 0;
      SolarXMinus = 0;
      SolarYPlus = 0;
      SolarYMinus = 0;
      SolarZPlus = 0;
      SolarZMinus = 0;
      DoorSense = 0;
      LightSense = 0;
      AnalogTemp = 0;
      numPhotos = 0;

      SlaveWorking = 1;
      Resets = 0;
      missionStatus = 0;


      imageR = RAMImage();
      currentSegment = 0;
      RequestingImageStatus = 0;

      AttemptingLink = false;
      MOStatus = 0;
      MOMSN = 0;
      MTStatus = 0;
      MTMSN = 0;
      MTLength = 0;
      MTQueued = 0;
      LastMsgType = 0;
      LastSMsgType = 0;

      ADCS_Active = 1;
      MResets = 0;
      CurXDir = 0;
      CurXPWM = 0;
      CurYDir = 0;
      CurYPWM = 0;
      CurZDir = 0;
      CurZPWM = 0;

      IMUData[3][360] = {0};
      LIGHTData[360] = {0};
      dataIndex = 0;
      accelIndex = 0;
    }
    void updateSensors() {
      if (hardwareAvTable[0]) {
        //Serial.println("Read1");
        imu.read();
        //Serial.println("Read2");
        floatTuple M = getMagData(imu);
        floatTuple g = getGyroData(imu);
        GyroAcc[0] += g.x; GyroAcc[1] += g.y; GyroAcc[2] += g.z;
        MagAcc[0] += M.x; MagAcc[1] += M.y; MagAcc[2] += M.z;
        TempAcc += getImuTempData(imu);
        //AccelAcc
      }
      SolarXPlus = getCurrentAmp(1); //X+
      SolarXMinus = getCurrentAmp(2); //X-
      SolarYPlus = getCurrentAmp(3); //Y+
      SolarYMinus = getCurrentAmp(4); //Y-
      SolarZPlus = getCurrentAmp(5); //Z+
      SolarZMinus = getCurrentAmp(6); //Z-

      //Serial.println(analogRead(BatteryPin));
      BatteryAcc += 2 * fmap((float)analogRead(BatteryPin), 0.0, 1023.0, 0.0, 3.3); //TODO x2??
      //Serial.println(BatteryAcc);
      DoorSense = digitalRead(DoorSensePin);
    }

    //    void configureSensor()
    //    {
    //      if (hardwareAvTable[0]) {
    //        //set magnetometer range to +-2 gauss
    //        imu.setupMag(imu.LSM9DS0_MAGGAIN_2GAUSS);
    //        //set gyro range to +-245 degrees per second
    //        imu.setupGyro(imu.LSM9DS0_GYROSCALE_245DPS);
    //        //imu.setupAccel(imu.LSM9DS0_ACCEL_MG_LSB_2G);
    //      }
    //    }

    String toString() {
      //Produces JSON Output in ASCII  for Downlink
      String output = "";
      output += "{";
      output += "S:" + String(State) + ",";
      output += "PS:" + String(imageR.photoSize()) + ",";
      output += "GX:" + String(Gyro[0]) + ",GY:" + String(Gyro[1]) + ",GZ:" + String(Gyro[2]) + ",";
      output += "MX:" + String(Mag[0]) + ",MY:" + String(Mag[1]) + ",MZ:" + String(Mag[2]) + ",";
      output += "IT:" + String(ImuTemp) + ",";
      output += "AT:" + String(AnalogTemp) + ",";
      output += "B:" + String(Battery) + ",";
      output += "SX+:" + String(SolarXPlus) + ",SX-:" + String(SolarXMinus) +
                ",SY+:" + String(SolarYPlus) + ",SY-:" + String(SolarYMinus) +
                ",SZ+:" + String(SolarZPlus) + ",SZ-:" + String(SolarZMinus) + ",";
      output += "DS:" + String(DoorSense) + ",";
      output += "LS:" + String(LightSense) + ",";
      output += "nP:" + String(numPhotos) + ",";
      output += "IW:" + String(hardwareAvTable[0]) + ",";
      output += "SW:" + String(SlaveWorking) + ",";
      output += "Rs:" + String(Resets) + ",";
      output += "AA:" + String(ADCS_Active) + ",";
      output += "XD:" + String(CurXDir) + ",";
      output += "XP:" + String(CurXPWM) + ",";
      output += "YD:" + String(CurYDir) + ",";
      output += "YP:" + String(CurYPWM) + ",";
      output += "ZD:" + String(CurZDir) + ",";
      output += "ZP:" + String(CurZPWM) + "}";
      return output;
    }
};
masterStatus masterStatusHolder;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Helper Functions

void initalizePinOut() {
  //const int DoorSens = 13; pinMode(DoorSens, INPUT); //WRONG
  if (LED_NOT_DOOR) {
    pinMode(DoorSensePin, OUTPUT); //Red LED and Door Sensor (-_-)
  } else {
    pinMode(DoorSensePin, INPUT_PULLUP);
  }
  pinMode(DoorTrig, OUTPUT);
  pinMode(BatteryPin, INPUT);
  //RockBlock Serial Into FCom
  //RockBlock Serial Out of FCom
  pinMode(RBSleep, OUTPUT);
  pinMode(RB_RI, INPUT);
  pinMode(RB_RTS, INPUT);
  pinMode(RB_CTS, INPUT);
  //I2C Data
  //I2C Clock
  pinMode(SolarXPlusPin, INPUT); //Solar Current X+
  pinMode(SolarXMinusPin, INPUT); //Solar Current X-
  pinMode(SolarYPlusPin, INPUT); //Solar Current Y+
  pinMode(SolarYMinusPin, INPUT); //Solar Current Y-
  pinMode(SolarZPlusPin, INPUT); //Solar Current Z+
  pinMode(SolarZMinusPin, INPUT); //Solar Current Z-
  pinMode(SolarZMinusPin, INPUT); //Slave Fault Handing (via Hard Reset)
  pinMode(DoorMagEnable, OUTPUT); //Allow Door Magnetorquer to work

}

float getCurrentAmp(int panel) {
  //Returns Amperage of current sensors at senseCurrent 4v/amp (40k resistor)
  //1v/A @10k
  //8.2V/A @82k
  float current;
  switch (panel) {
    case 1:
      if (masterStatusHolder.hardwareAvTable[1]) {
        current = analogRead(SolarXPlusPin);
      } else {
        current = 0;
      } break;
    case 2:
      if (masterStatusHolder.hardwareAvTable[2]) {
        current = analogRead(SolarXMinusPin);
      } else {
        current = 0;
      } break;
    case 3:
      if (masterStatusHolder.hardwareAvTable[3]) {
        current = analogRead(SolarYPlusPin);
      } else {
        current = 0;
      } break;
    case 4:
      if (masterStatusHolder.hardwareAvTable[4]) {
        current = analogRead(SolarYMinusPin);
      } else {
        current = 0;
      } break;
    case 5:
      if (masterStatusHolder.hardwareAvTable[5]) {
        current = analogRead(SolarZPlusPin);
      } else {
        current = 0;
      } break;
    case 6:
      if (masterStatusHolder.hardwareAvTable[6]) {
        current = analogRead(SolarZMinusPin);
      } else {
        current = 0;
      } break;
  }
  //0.12195
  //Serial.println(current);
  current = fmap((float)current, 0.0, 1023.0, 0, .40244); //3.3V=.825A
  // 8.2V/A with a 82k Resistor
  return current;
}


float getTotalAmperage() {
  //Check the amps from each solar panel and returns total amperage
  float TotalCurrent = 0;
  for (int i = 1 ; i <= 6; i++) {
    TotalCurrent += getCurrentAmp(i);
  }
  return TotalCurrent;
}

//Determine Remaining RAM
extern "C" char *sbrk(int i);
int freeRam () {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}

volatile bool stall = true;
void waitForInterrupt() {
  stall = false;
  //noInterrupts();
}

void SensorDataCollect() { //TODO
  masterStatusHolder.updateSensors();
  IMUrecords++;
  //Request Light/Temp Data From Slave
  if (millis() - lastIMUTime > imuSensorDwell) {
    if (masterStatusHolder.hardwareAvTable[0]) {
      masterStatusHolder.Gyro[0] = masterStatusHolder.GyroAcc[0] / ((float)IMUrecords);
      masterStatusHolder.Gyro[1] = masterStatusHolder.GyroAcc[1] / ((float)IMUrecords);
      masterStatusHolder.Gyro[2] = masterStatusHolder.GyroAcc[2] / ((float)IMUrecords);
      masterStatusHolder.Mag[0] = masterStatusHolder.MagAcc[0] / ((float)IMUrecords);
      masterStatusHolder.Mag[1] = masterStatusHolder.MagAcc[1] / ((float)IMUrecords);
      masterStatusHolder.Mag[2] = masterStatusHolder.MagAcc[2] / ((float)IMUrecords);
      masterStatusHolder.ImuTemp = masterStatusHolder.TempAcc / ((float)IMUrecords);
      masterStatusHolder.TempAcc = 0;
      masterStatusHolder.GyroAcc[0] = 0; masterStatusHolder.GyroAcc[1] = 0; masterStatusHolder.GyroAcc[2] = 0;
      masterStatusHolder.MagAcc[0] = 0; masterStatusHolder.MagAcc[1] = 0; masterStatusHolder.MagAcc[2] = 0;
    }
    masterStatusHolder.Battery = masterStatusHolder.BatteryAcc / ((float)IMUrecords); masterStatusHolder.BatteryAcc = 0;
    lastIMUTime = millis();
    IMUrecords = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////Parser Functions

void buildBuffer(String com) {
  int commandData;
  int commandType;
  String comRemaining = com;
  bool loop = true;
  while (loop) {
    commandType = (com.substring(0, com.indexOf(","))).toInt();
    commandData = (com.substring(com.indexOf(",") + 1, com.indexOf("!"))).toInt();
    cBuf.commandStack[cBuf.openSpot][0] = commandType;
    cBuf.commandStack[cBuf.openSpot][1] = commandData;
    if (com.indexOf("!") == com.length() - 1) {
      loop = false;
      Serial.println(F("Finished Adding Commands"));
    } else {
      com = com.substring(com.indexOf("!") + 1);
    }
    cBuf.openSpot++;
  }
}

boolean isInputValid(String input) {
  //Check if incoming command is valid
  int lastPunc = 0; //1 if ",", 2 if "!", 0 Otherwise
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
      } else if (currentChar == ('!')) {
        if (input[q - 2] == ',') {
          //Serial.println("No Second Command Number");
          valid = false;
          break;
        }
        //Serial.println("Excl Found");
        if (lastPunc == 1) {
          //Serial.println("Period ok");
          lastPunc = 2;
        } else {
          //Serial.println("2 Excl or No prior comma");
          valid = false;
          break;
        }
      } else if (currentChar == ('-')) {
        //Serial.println("Hypen Found");
        if (input[q - 2] == ',') { //q incremented after value capture
          //Serial.println("Negative Sign ok");
        } else {
          //Serial.println("Hyphen in wrong place");
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

    //Detect no ending exclamation point
    if (q == input.length() - 1) {
      if (input[q] != '!') {
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

void popCommands() {
  //Process All the Incoming Commands
  while (cBuf.openSpot > 0) { //Manual Timeout
    //Serial.println ("Executing Command:");
    if (cBuf.openSpot > 0) {
      Serial.println (cBuf.openSpot - 1);
      int currentCommand[2] = {cBuf.commandStack[cBuf.openSpot - 1][0], cBuf.commandStack[cBuf.openSpot - 1][1]};
      //Serial.print(currentCommand[0]);
      //Serial.print(":");
      //Serial.println(currentCommand[1]);
      cBuf.commandStack[cBuf.openSpot - 1][0] = -1;
      cBuf.commandStack[cBuf.openSpot - 1][1] = -1;
      cBuf.openSpot --;
      //Place the Command ID in the "#"

      //Commands
      switch (currentCommand[0]) {
        case (91): //Arm Deployment
          masterStatusHolder.NextState = DEPLOY_ARMED;
          DA_Initialize = true;
        case (92): //Set Deploy Timeout (seconds)
          if (currentCommand[1] >= 2000) {
            deployTimeOut = (currentCommand[1]) * 1000;
          }
          break;
        case (93): //Set Manual Function Timeout (millis)
          if (currentCommand[1] >= 2000) {
            manualTimeout = (currentCommand[1]);
          }
          break;

        case (94): { //Toggle ADCS
            String com = "91," + String(currentCommand[1]) + "!";
            sendSCommand(com);
            break;
          }
        case (95): { //Set Master-Slave Command Time (millis)
            if (currentCommand[1] > 0) {
              SComTime = currentCommand[1];
            }
            break;
          }
        case (96): //Test Force Deploy
          digitalWrite(DoorTrig, HIGH);
          break;
        case (97): //Check System Time
          long t = millis();
          Serial.println("<<System Time: "String(t % (long)60 * 60000) + ":" +
                         String(t % (long)60000) + ":" + String(t % (long)1000) + ">>");
          break;
        case (51): //Take Photos
          sendSCommand("101,1!");
          break;
        case (52): { //Set PhotoBurst Time
            String com = "102," + String(currentCommand[1]) + "!";
            sendSCommand(com);
            break;
          }
        case (53): { //Get # of Available Photos
            Serial.println("Photos Available: " + String(masterStatusHolder.numPhotos));
            break;
          }
        case (54): //Wipe SD Card
          Serial.println("Wiping SD Card");
          sendSCommand("107,1!");
          break;
        case (61):
          masterStatusHolder.MResets = (currentCommand[1]);
          break;
        case (62):
          masterStatusHolder.AnalogTemp = (currentCommand[1]);
          break;
        case (63):
          masterStatusHolder.LightSense = (currentCommand[1]);
          break;
        case (64):
          masterStatusHolder.CurXDir = (currentCommand[1]);
          break;
        case (65):
          masterStatusHolder.CurYDir = (currentCommand[1]);
          break;
        case (66):
          masterStatusHolder.CurZDir = (currentCommand[1]);
          break;
        case (67):
          masterStatusHolder.CurXPWM = (currentCommand[1]);
          break;
        case (68):
          masterStatusHolder.CurYPWM = (currentCommand[1]);
          break;
        case (69):
          masterStatusHolder.CurZPWM = (currentCommand[1]);
          break;
        case (610):
          masterStatusHolder.numPhotos = (currentCommand[1]);
          break;
        case (71):
          masterStatusHolder.MOStatus = (currentCommand[1]);
          break;
        case (72):
          masterStatusHolder.MOMSN = (currentCommand[1]);
          break;
        case (73):
          masterStatusHolder.MTStatus = (currentCommand[1]);
          break;
        case (74):
          masterStatusHolder.MTMSN = (currentCommand[1]);
          break;
        case (75):
          masterStatusHolder.MTLength = (currentCommand[1]);
          break;
        case (76):
          masterStatusHolder.MTQueued = (currentCommand[1]);
          break;
        case (77):
          masterStatusHolder.SBDRT = (currentCommand[1]);
          break;
        case (81): {
            masterStatusHolder.imageR = RAMImage();
            masterStatusHolder.RequestingImageStatus = 1;
            masterStatusHolder.NextState = IMAGE_REQUEST;
            String com = "103," + String(currentCommand[1]) + "!";
            sendSCommand(com);
            char filename[9];
            strcpy(filename, "A0000.JPG");
            filename[1] = '0' + currentCommand[1] / 1000;
            filename[2] = '0' + currentCommand[1] % 1000 / 100;
            filename[3] = '0' + currentCommand[1] % 1000 % 100 / 10;
            filename[4] = '0' + currentCommand[1] % 1000 % 100 % 10;
            masterStatusHolder.imageR.Filename = filename;
            break;
          }
        case (83): {
            masterStatusHolder.RequestingImageStatus = 0;
            masterStatusHolder.NextState = NORMAL_OPS;
            //DANGER
            break;
          }
        case (111): { //Force SBDIX
            Serial.println("Sent SBDIX");
            sendSBDIX(false); //Does NOT sent Attempting Link to True
            break;
          }
        case (112): { //Force Ping
            Serial.println("Sent AT\\r");
            Serial1.print(F("AT\r"));
            break;
          }
        case (113): { //Force Disable Flow Control
            Serial.println("Sent AT&K0\\r");
            Serial1.print(F("AT&K0\r"));
            break;
          }
        case (114): {  //Force Diable RING //TODO
            //            Serial.println("Sent ");
            //            Serial1.print();
            break;
          }
        case (120):  //Set Broken Components
          if (currentCommand[1] <= 10) {
            masterStatusHolder.hardwareAvTable[currentCommand[1]] =
              !masterStatusHolder.hardwareAvTable[currentCommand[1]];
          }
      }
    } else {
      //Serial.println("No Command");
    }
  }
}

void readSerialAdd2Buffer() {
  //Testing
  if (Serial.available() > 0) {
    Serial.println("Reading Testing Command");
    String comString = "";
    while (Serial.available() > 0) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      comString += inChar;
    }
    Serial.println("TCommand: " + comString);
    if (isInputValid(comString)) {
      Serial.println("Testing Command is Valid");
      buildBuffer(comString);
      popCommands();
    } else {
      Serial.println("Invalid Testing Command");
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Slave Functions

/* Supported Commands Slave Can Recieved
  Send Gyro X to Slave: "11,<(String)(float)>!"
  Send Gyro Y to Slave: "12,<(String)(float)>!"
  Send Gyro Z to Slave: "13,<(String)(float)>!"
  Send Mag X to Slave: "21,<(String)(float)>!"
  Send Mag Y to Slave: "22,<(String)(float)>!"
  Send Mag Z to Slave: "23,<(String)(float)>!"
  Z Torquer On for Heat: "41,1!" //TODO
  Z Torquer Off for Heat: "41,0!" //TODO
  Activate ACDS: "91,1!"
  Deactivate ACDS: "91,0!"
  Photo Burst Start: "101,1!"
  Photo Burst Duration: "101,<int>!" (Seconds NOT Millis)
*/

void sendSCommand(String data) {
  Serial.print("Command Sent to Slave: <");
  Serial.println(data + ">");
  char com[data.length() + 1];
  data.toCharArray(com, data.length() + 1);
  if (WireConnected) {
    Wire.beginTransmission(11); // transmit to device #8
    Wire.write(com);   // sends String
    Wire.endTransmission();    // stop transmitting
  }
}

void sectionReadToValue(String s, int * data, int dataSize) {
  for (int i = 0; i < dataSize; i++) {
    data[i] = (s.substring(0, s.indexOf(','))).toInt();
    s = s.substring(s.indexOf(',') + 1);
  }
}

uint8_t segment[4000];
int dataIndex = 0;
bool lastRead = false;
int readSize = 0;
bool requestFromSlave() {
  String res = "";
  bool success = false;
  if (WireConnected) {
    switch (masterStatusHolder.RequestingImageStatus) {
      case (1): { //Get Image Size
          Wire.requestFrom(11, 8, true);
          delay(10);
          String res = "";
          while (Wire.available()) {
            res += (char)Wire.read();
          }
          readSize = res.toInt();
          Serial.println("Incoming Photo Size: " + String(readSize));
          if (readSize > 7) {
            masterStatusHolder.RequestingImageStatus = 2;
            sendSCommand("105,1!");
          } else if (readSize == 0) {
            Serial.println("Image Didn't Exist");
            //Update HardwareAv
            masterStatusHolder.RequestingImageStatus = 0;
          } else {
            Serial.println("SD Card Not Working Transfer Aborted");
            //Update HardwareAv
            masterStatusHolder.RequestingImageStatus = 0;
          }
          break;
        }
      case (2): { //Get Image Data
          Serial.print("Requesting Image Data: ");
          (Wire.requestFrom(11, 32, true)); // request 64 bytes from slave device #8
          delay(10);
          int i = 0;
          //Serial.println("AV: " + String(Wire.available()));
          while (Wire.available()) {
            segment[dataIndex] = (uint8_t)Wire.read();
            //Serial.println(segment[dataIndex]);
            if (dataIndex >= readSize) {
              break;
            }
            dataIndex++;
            i++;
          }
          //printArray(segment, dataIndex);
          Serial.println(dataIndex);
          //Serial.println("i: "+String(i));
          if (dataIndex >= readSize - 1 && dataIndex > 16) { //if (i < 16) {
            Serial.println("Last");
            lastRead = true;
            //printArray(segment, dataIndex);
            masterStatusHolder.RequestingImageStatus = 0;
            masterStatusHolder.imageR.store(segment, dataIndex);
            dataIndex = 0;

            //Will reset Slave from ImageTransmit Mode

          } else {
            //Serial.println("Not Last");
          }
          break;
        }
      case (0): {
          Wire.requestFrom(11, 40, true); // request 10 bytes from slave device #8
          //delay(50);
          int endTime = millis() + manualTimeout;
          //Serial.println("Here");

          //Read and Reformat
          //  ADCS_Active;
          if (Wire.available()) {
            success = true;
          }
          String res = "";
          while (Wire.available()) {
            res += (char)Wire.read();
          }
          res = res.substring(0, res.indexOf('|'));
          if (masterStatusHolder.State != DEPLOY_ARMED) {
            Serial.println("SSVs Updated: " + res);
          }
          int data[10];
          sectionReadToValue(res, data, 10);
          masterStatusHolder.MResets = data[0];
          masterStatusHolder.AnalogTemp = data[1];
          masterStatusHolder.LightSense = data[2];
          masterStatusHolder.CurXDir = data[3];
          masterStatusHolder.CurYDir = data[4];
          masterStatusHolder.CurZDir = data[5];
          masterStatusHolder.CurXPWM = data[6];
          masterStatusHolder.CurYPWM = data[7];
          masterStatusHolder.CurZPWM = data[8];
          masterStatusHolder.numPhotos = data[9];

          break;
        }
    }
    return success;
  }
}

String buildIMUDataCommand() {
  // ex. gyro data: "11,3.653!12,2.553!13,-10!"
  String res = "";
  //Sends Info x1000
  res += "11," + String((long int)(1000 * masterStatusHolder.Gyro[0])) + "!";
  res += "12," + String((long int)(1000 * masterStatusHolder.Gyro[1])) + "!";
  res += "13," + String((long int)(1000 * masterStatusHolder.Gyro[2])) + "!";
  res += "21," + String((long int)(1000 * masterStatusHolder.Mag[0])) + "!";
  res += "22," + String((long int)(1000 * masterStatusHolder.Mag[1])) + "!";
  res += "23," + String((long int)(1000 * masterStatusHolder.Mag[2])) + "!";
  return res;
}

void sendIMUToSlave() {
  String SCommand = buildIMUDataCommand();
  char SComCharA[SCommand.length() + 1];
  SCommand.toCharArray(SComCharA, SCommand.length() + 1);
  sendSCommand(SComCharA);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// RockBlock Uplink/Downlink Functions
//

int curComL = 0;

String rocResponseRead() {
  long start = millis();
  //Serial.print(Serial1.available());
  while (!Serial1.available() && (millis() - start > 4000));
  delay(10);
  String responseString = "";

  while (Serial1.available() > 0) {
    //NEED TO DETECT \r or Command End
    responseString += (char)Serial1.read();

    if (millis() - start > 4000) {
      Serial.println(F("Com Timeout"));
      break;
    }
  }
  //Serial.print(responseString);
  return responseString;
}

bool rockOKParse() {
  String input = rocResponseRead();
  bool valid = false;
  //Serial.print(input);
  if (input[2] == 'O' && input[3] == 'K') {
    valid = true;
  }
  return valid;
}

void RBDATA() {
  int swnumber = 0;
  String ReceivedMessage = rocResponseRead(); //determines case
  Serial.print("<" + ReceivedMessage + ">");
  //Serial.print("ReceivedMessage:");
  //Serial.println(ReceivedMessage);
  int plus = ReceivedMessage.indexOf('+');
  int colon = ReceivedMessage.indexOf(':');
  int S_SBDRING = ReceivedMessage.indexOf('S');
  int E_ERROR = ReceivedMessage.indexOf('E');
  int R_ERROR = ReceivedMessage.lastIndexOf('R');
  int O_OK = ReceivedMessage.indexOf('O');
  int K_OK = ReceivedMessage.indexOf('K');
  int space = ReceivedMessage.indexOf(' ');
  int carReturn = ReceivedMessage.lastIndexOf('\r');
  int R_READY = ReceivedMessage.indexOf('R');
  int Y_READY = ReceivedMessage.lastIndexOf('Y');

  String Ring;
  String OK;
  String error;
  String nomessage;
  String invalid;
  int LengthOfMessage = ReceivedMessage.length();
  //  Serial.print("Length of Message:");
  //  Serial.println(ReceivedMessage.length());
  //  Serial.print("Substring:");
  //  Serial.println(ReceivedMessage.substring(plus, colon));

  if (ReceivedMessage.substring(plus, colon).equals(F("+SBDIX"))) {
    swnumber = 1;
  }
  else if (ReceivedMessage.substring(plus, colon).equals(F("+SBDRT"))) {
    swnumber = 2;
  }
  else if (ReceivedMessage.substring(S_SBDRING).equals(F("SBDRING"))) {
    swnumber = 3;
  }
  else if (ReceivedMessage.substring(E_ERROR, R_ERROR).equals(F("ERRO"))) {
    swnumber = 5;
  }
  else if (ReceivedMessage.substring(R_READY, Y_READY).equals(F("READ"))) {
    swnumber = 7;
  }
  else if (ReceivedMessage.length() == 0) {
    swnumber = 6;
  }
  else if (ReceivedMessage.substring(O_OK, K_OK + 1).equals(F("OK"))) {
    if (swnumber == 0) {
      swnumber = 4;
    }
  } else {
    swnumber = 0;
  }

  String DATA = ReceivedMessage.substring(colon + 1);
  int DATALength = DATA.length();
  String SBDRTDATA = DATA.substring(2, (DATALength - 6));
  int SBDRTDATALength = SBDRTDATA.length();

  int firstcomma, secondcomma, thirdcomma, fourthcomma, fifthcomma;
  String firstnumber, secondnumber, thirdnumber, fourthnumber, fifthnumber, sixthnumber;

  switch (swnumber) {
    case 1: //SBDIX command
      //Serial.println("case 1");
      firstcomma  = DATA.indexOf(',');
      secondcomma = DATA.indexOf(',', firstcomma + 1);
      thirdcomma = DATA.indexOf(',', secondcomma + 1);
      fourthcomma = DATA.indexOf(',', thirdcomma + 1);
      fifthcomma = DATA.indexOf(',', fourthcomma + 1);
      firstnumber = DATA.substring(1, firstcomma);
      //Serial.println(firstnumber);
      secondnumber = DATA.substring(firstcomma + 2, secondcomma);
      //Serial.println(secondnumber);
      thirdnumber = DATA.substring(secondcomma + 2, thirdcomma);
      //Serial.println(thirdnumber);
      fourthnumber = DATA.substring(thirdcomma + 2, fourthcomma);
      //Serial.println(fourthnumber);
      fifthnumber = DATA.substring(fourthcomma + 2, fifthcomma);
      //Serial.println(fifthnumber);
      sixthnumber = DATA.substring(fifthcomma + 2, LengthOfMessage - 17);
      // Serial.print("sixthnumber:");
      //Serial.println(sixthnumber);
      //Valid Command, Invalid -> false
      masterStatusHolder.MOStatus = firstnumber.toInt();
      masterStatusHolder.MOMSN = secondnumber.toInt();
      masterStatusHolder.MTStatus = thirdnumber.toInt();
      masterStatusHolder.MTMSN = fourthnumber.toInt();
      masterStatusHolder.MTLength = fifthnumber.toInt();
      masterStatusHolder.MTQueued = sixthnumber.toInt();

      //Safe to do here????
      masterStatusHolder.AttemptingLink = false;

      break;

    case 2: //SBDRT command
      //Valid Command From Ground
      if (isInputValid(SBDRTDATA)) {
        buildBuffer(SBDRTDATA);
        popCommands();
      } else {
        //Invalid Uplink
        masterStatusHolder.LastMsgType = 0;

      }
      break;
    case 3://SBDRING
      //Message is waiting the Buffer
      masterStatusHolder.LastMsgType = 2;
      masterStatusHolder.LastSMsgType = 2;
      //return "";
      break;
    case 4: //OK
      masterStatusHolder.LastMsgType = 1;
      masterStatusHolder.LastSMsgType = 1;
      //return "";
      break;
    case 5: // ring
      masterStatusHolder.LastMsgType = 3;
      masterStatusHolder.LastSMsgType = 3;
      //return "";
      break;
    case 6: // blank msg //TODO
      masterStatusHolder.LastMsgType = 0;
      //return "";
      break;
    case 7: // ready
      masterStatusHolder.LastMsgType = 4;
      masterStatusHolder.LastSMsgType = 4;
      break;
    case 0: // invalid
      masterStatusHolder.LastMsgType = 0;
      break;
      //0 = inval
      //1 = ok
      //2 = ring
      //3 = error
      //4 = ready
  }
}

int lastRBcheck = 0;

bool responsePing() {
  bool ping = false;
  Serial1.print(F("AT\r"));
  if (rockOKParse()) {
    ping = true;
  }
  return ping;
}

void sendSBDIX(bool AL) {
  Serial1.print(F("AT+SBDIX")); // \r?
  if (AL) {
    masterStatusHolder.AttemptingLink = true;
  }
}

//int startTest() {
//  // while millis < 6mins
//  bool go = true;
//  int failType = 0;
//  Serial.println("Here Starttest");
//  Serial1.print("ATE0\r");
//  if (rocOKParse() || true) {
//    Serial.println("Stage0 Pass");
//    Serial1.print("AT\r");
//    if (rocOKParse()) {
//      Serial.println("Stage1 Pass");
//      Serial1.print("AT + SBDIX\r");
//      if (rocOKParse()) {
//        Serial.println("Stage2 Pass");
//        Serial1.print("AT + SBDWT = ");
//        Serial1.print(downlinkData);
//        Serial1.print("\r");
//        if (rocOKParse()) {
//          Serial.println("Stage3 Pass");
//          go = false;
//
//        }
//      }
//    }
//  }
//  return failType;
//
//}

int downlinkSegment(int segIndex) {
  // 0 For Success, 1 for No Ready, 2 For No OK
  uint8_t * Data = masterStatusHolder.imageR.get(segIndex);
  int DataSize = masterStatusHolder.imageR.sizeArray[segIndex];

  Serial.println("Begining Downlink");

  Serial1.print("AT+SBDWB=");
  Serial1.print(DataSize);
  Serial1.print("\r");

  //  /////////////////////////////////
  //  Serial.println("Response 1 >>");
  //  while (Serial1.available()) {
  //    Serial.print((char)Serial1.read());
  //  }
  //  Serial.println("\n<<Response 1");
  //  /////////////////////////////////

  RBDATA();
  if (!(masterStatusHolder.LastMsgType == 4)) {
    //No Ready Recieved
    return 1;
  }

  uint16_t checksum = 0;
  for (int i = 0; i < DataSize; ++i)
  {
    Serial1.write(Data[i]);
    checksum += (uint16_t)Data[i];
  }
  //printArray(imageBuffer[0], DataSize);
  Serial1.write(checksum >> 8);
  Serial1.write(checksum & 0xFF);
  delay(100);

  //  /////////////////////////////////Ok?
  //  Serial.println("Response 2 >> ");
  //  while (Serial1.available()) {
  //    Serial.print((char)Serial1.read());
  //  }
  //  Serial.println("\n<< Response 2");
  //  /////////////////////////////////

  RBDATA();
  if (!(masterStatusHolder.LastMsgType == 1)) {
    //No Ok Recieved
    return 2;
  }

  return 0;
}


void print_binary(int v, int num_places) {
  int mask = 0, n;
  for (n = 1; n <= num_places; n++) {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask;  // truncate v to specified number of places
  while (num_places) {
    if (v & (0x0001 << num_places - 1)) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
    --num_places;
  }
}

//void buildImageBuffer(String Filename) { //Rename to masterstatusholderimagebuffer in future...
//  File IMGFile = SD.open(Filename, FILE_READ);
//  uint8_t jpglen = IMGFile.size();
//  //int segments = ((8*jpglen) / 320) + 1;
//  //String imageBuffer[segments];
//  int index = 0;
//  int i = 0;
//  Serial.println("Starting Segmentation");
//  while (IMGFile.available()) {
//    //Serial.println("Available: " + String(IMGFile.available()));
//    int bytesToRead = min(320, IMGFile.available());
//    uint8_t segment[bytesToRead - 1];
//    for (int z = 0; z < bytesToRead; z++) {
//      segment[z] = 0;
//    }
//    while (i < bytesToRead) {
//      segment[i] = (uint8_t)IMGFile.read();
//      i++;
//    }
//    //Serial.print("Current Segment " + String(index) + ": ");
//    printArray(segment, i);
//    //Serial.println("");
//    //Serial.println("Here1");
//    //    imageBuffer[index] = segment;
//    masterStatusHolder.imageR.store(segment, i, index);
//    masterStatusHolder.imageR.sizeArray[index] = bytesToRead;
//
//    i = 0;
//    index = index + 1;
//    //Serial.println("Here2");
//  }
//  IMGFile.close():
//  Serial.println("\nDone");
//  masterStatusHolder.imageR.finalIndex = index - 1;
//  return;
//}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*  Watchdog timer support for Arduino Zero
  by Richard Hole  December 19, 2015
*/

//setupWDT( 11 );

static void   WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}

//============= resetWDT ===================================================== resetWDT ============
void resetWDT() {
  // reset the WDT watchdog timer.
  // this must be called before the WDT resets the system
  WDT->CLEAR.reg = 0xA5; // reset the WDT
  WDTsync();
}

//============= systemReset ================================================== systemReset ============
void systemReset() {
  // use the WDT watchdog timer to force a system reset.
  // WDT MUST be running for this to work
  WDT->CLEAR.reg = 0x00; // system reset via WDT
  WDTsync();
}

//============= setupWDT ===================================================== setupWDT ============
void setupWDT( uint8_t period) {
  // initialize the WDT watchdog timer

  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required

  WDT->CONFIG.reg = min(period, 11); // see Table 17-5 Timeout Period (valid values 0-11)

  WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable watchdog
  WDTsync();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Stall() {
  stall = true;
  long start = millis();
  Serial.println("Awaiting Input");
  if (LED_NOT_DOOR) {
    while (millis() - start < 8000) { //(stall) {
      digitalWrite(13, HIGH);
      delay(40);
      digitalWrite(13, LOW);
      delay(40);
      Serial.print(".");
    }
  } else {
    while (millis() - start < 8000) { //(stall) {
      delay(80);
      Serial.print(".");
    }
  }
}

//// Main Loop

void setup() {

  digitalWrite(12, HIGH);
  attachInterrupt(digitalPinToInterrupt(12), waitForInterrupt, LOW);
  Serial.begin(9600);
  Serial1.begin(19200);
  Wire.begin(); //Start i2c as master
  //while (!Serial);
  Stall();

  Serial.println("\nStarting IMU");

  if (!imu.begin()) {
    Serial.println("IMU Failed");
    masterStatusHolder.hardwareAvTable[0] = false;
  }

  Serial.println("Successful Start");

  initalizePinOut();
  digitalWrite(SlaveReset, HIGH); //Enable Slave
  delay(1000);
  Stall();

  cBuf = commandBuffer();
  masterStatusHolder = masterStatus();
  masterStatusHolder.deploySetting = 1; //Just Light

  //Try to initialize and warn if we couldn't detect the chip
  int endT = millis() + manualTimeout;

  masterStatusHolder.RequestingImageStatus = 0;
  masterStatusHolder.State = NORMAL_OPS;
  masterStatusHolder.NextState = NORMAL_OPS;
  Serial.println(F("Setup Done"));
}

//Testing Variables
bool downlinkJustStaged = false;
//bool ImageDownlink_Active = true;
const int DOWNLINK_TESTING = 999;
//const int IMAGE_REQUEST = 888;
int Requests = 0;
long linkTime = 0;
int trys = 0;

void loop() {
  readSerialAdd2Buffer(); //Testing Command Input
  //masterStatusHolder.State = IMAGE_REQUEST;
  switch (masterStatusHolder.State) {
    case (777): //Stall State
      Serial.println("Idling in Stall State");
      Stall();
      masterStatusHolder.NextState = NORMAL_OPS;//NORMAL_OPS;
      masterStatusHolder.RequestingImageStatus = 0;
      break;
    case (IMAGE_REQUEST): {
        //Serial.println("Here");
        //Serial.println(masterStatusHolder.RequestingImage);
        //delay(100);
        switch (masterStatusHolder.RequestingImageStatus) {
          case (2): {
              //Serial.println("Image Data Requested");
              requestFromSlave();
              Requests++;
              if (Requests > 800) {
                Serial.println("Image Transfer Fail");
                Requests = 0;
                lastRead = false;
                masterStatusHolder.RequestingImageStatus = 0;
                masterStatusHolder.NextState = NORMAL_OPS;//DOWNLINK_TESTING ?
                sendSCommand("104,1!");
              }
              //delay(5);
              if (lastRead) {
                //          Wire.flush();
                //          Wire.end();
                //          Wire.begin();
                masterStatusHolder.RequestingImageStatus = 0;
                Requests = 0;
                lastRead = false;
                masterStatusHolder.NextState = NORMAL_OPS;//DOWNLINK_TESTING; ?
                sendSCommand("104,1!");
                Serial.println("");
                masterStatusHolder.imageR.printRI();
                //Stall();
                Serial.println("");
                break;
              }
              break;
            }
          case (1): {
              delay(10);
              requestFromSlave();
              break;
            }
          case (0): {
              //Shouldn't be here
              //Serial.println("State 0");
              masterStatusHolder.NextState = NORMAL_OPS;
              break;
            }
        }
        break;
      }

    case (DOWNLINK_TESTING):
      if (masterStatusHolder.currentSegment <= masterStatusHolder.imageR.finalIndex) {
        if (masterStatusHolder.MOStatus == 0) { //|| masterStatusHolder.currentSegment == 0) {
          //Attempt Segment Downlink
          if (!masterStatusHolder.AttemptingLink) {
            int error = downlinkSegment(masterStatusHolder.currentSegment);
            if (error == 0) {
              //Success
              masterStatusHolder.MOStatus = 13;
              downlinkJustStaged = true;
              Serial.println("Segment " + String(masterStatusHolder.currentSegment) + " Staged");
            } else {
              //Fail
              Serial.println("Error: " + String(error));
            }
          } else {
            Serial.print("Still Attempting Link: ");
            Serial.println(linkTime / 1000.0);
            delay(50);
          }
        } else {
          if (downlinkJustStaged) {
            //SBDIX Call after SBDWB
            Serial.print("SBDIX Sent");
            sendSBDIX(true); //Attempting Link is true after this

            linkTime = millis();
            //downlinkJustStaged = true;
            downlinkJustStaged = false;
          }
          if (trys % 50 == 0) {
            Serial.println("");
          }
          trys++;

          RBDATA(); //Can Reset MOStatus to 0
          delay(200);
        }
      } else {
        masterStatusHolder.NextState = NORMAL_OPS;
      }
      //TODO Timeout
      break;

    case (NORMAL_OPS):

      //Collect Sensor Data
      SensorDataCollect();
      if (millis() - lastRBCheck >= RBCheckTime) {
        Serial.print("<R");
        Serial1.println("AT\r");
        Serial.print(String(rockOKParse()) + ">");
        //        //delay(10);
        //        RBDATA();
        //        Serial1.print(F("AT&K0\r"));
        //        delay(10);
        //        RBDATA();
        lastRBCheck = millis();
      }

      //Testing IMU and Sensor Downlink String Generator
      if (millis() - lastDLTime >= DLTime || commandedDL) {
        //Send Data to RockBlock via Serial
        String DLS = masterStatusHolder.toString();
        Serial.println(F(""));
        Serial.println(DLS);
        //Serial.println(DLS.length());
        lastDLTime = millis();
      }

      //Test Slave Communication
      if (TestSCom && WireConnected) {
        unsigned long t = millis();
        if (millis() - lastSComAttempt >= SComTime) {
          lastSComAttempt = millis();
          Serial.print("\nSlave IMU transfer: ");
          sendIMUToSlave();
          bool SlaveResponse = requestFromSlave();

          if (SlaveResponse) {
            //Serial.print(".");
            lastSComTime = millis(); //Reset Timeout if Com is successful
          } else {
            //No Reply From Slave
            //            if (millis() - lastSComTime > SlaveResetTimeOut) {
            //
            //              //No Communication for (SlaveResetTimeOut) ms
            //              slaveWorking = false;
            //
            //              if (!slaveWorking) {
            //                digitalWrite(SlaveReset, LOW);
            //                delay(50);
            //                digitalWrite(SlaveReset, HIGH);
            //                SlaveResets++;
            //                //delay(1000);
            //                Serial.println("Slave Reset");
            //              }
            //            } else {
            Serial.print(F("No Reply From Slave for "));
            Serial.print((millis() - lastSComTime) / 1000.0);
            Serial.println(F(" seconds"));
            //            }
          }
        }
      }

      //ADCS
      if (millis() - LastSpinCheckT > SpinCheckTime) {
        Serial.println("Here3");
        LastSpinCheckT = millis();
        if (masterStatusHolder.Gyro[0] > OmegaThreshold || masterStatusHolder.Gyro[1] > OmegaThreshold) {
          sendSCommand("91,1!"); //Activate Torquers
          masterStatusHolder.ADCS_Active = true;
        } else {
          sendSCommand("91,0!"); //Deactivate Torquers
          masterStatusHolder.ADCS_Active = false;
        }
      }

      //Eclipse Detection
      //        if (getTotalAmperage() < EclipseAmp_Threshold) {
      //          masterStatusHolder.NextState = ECLIPSE;
      //          //torquers off
      //          eclipseEntry = millis();
      //        }
      //        //Low Power Detection
      //        if (masterStatusHolder.Battery * 2 < LV_Threshold) {
      //          masterStatusHolder.NextState = LOW_POWER;
      //          lowPowerEntry = millis();
      //        }

      //Blinker for Testing
      if (millis() - ledLastTime >= 100) {
        if (LED_NOT_DOOR) {
          //unsigned long t = millis() - ledLastTime;
          if (ledState == LOW) {
            ledState = HIGH;
          } else {
            ledState = LOW;
          }
          digitalWrite(13, ledState);
        }
        Serial.print(F("."));
        //        Serial.print("Running: ");
        //        Serial.print(t);
        //        Serial.print(" -> Cycle Stretch of: ");
        //        Serial.println(t - 100);
        ledLastTime = millis();
      }
      break;

    case (DORMANT_CRUISE):

      //30 min Dormant Cruise
      if (millis() > cruiseEnd) {
        masterStatusHolder.NextState = INITALIZATION;
        initEntry = millis();
      } else {
        delay(10000);
      }
      break;

    case (INITALIZATION):
      {
        if (millis() - initEntry < 100) {
          sendSCommand("91,1!");
        }

        //Collect Sensor Data
        SensorDataCollect();

        //Listen to Status Reports
        if (masterStatusHolder.Gyro[0] < gyroThresholdX && masterStatusHolder.Gyro [1] < gyroThresholdY) {
          masterStatusHolder.NextState = NORMAL_OPS;
        }

        //Check battery ->> INIT_SLEEP
        if (masterStatusHolder.Battery < LV_Threshold) {
          masterStatusHolder.NextState = INIT_SLEEP;
        }

        if (millis() - initEntry > (long)2700000) {
          //call downlink function
          //TODO
        }
        break;
      }

    case (INIT_SLEEP): {
        //Check Time
        if (millis() - initSleepEntry > (long)60 * 45 * 1000) {
          masterStatusHolder.NextState = INITALIZATION;
        }
        //Check battery ->> INITALIZATION
        if (masterStatusHolder.Battery > HV_Threshold) {
          masterStatusHolder.NextState = INITALIZATION;
        }
      }
      break;

    case (ECLIPSE):
      {

        //Check Battery
        //Check Solar Current
        //Check Time
        //Magtorquers off?

        if (masterStatusHolder.Battery < LV_Threshold) {
          masterStatusHolder.NextState = LOW_POWER;
        }
        //Check Solar Current
        //Check Time
        float EclipseAmps = getTotalAmperage();
        if (EclipseAmps > .1 || millis() - eclipseEntry > forceExitEclipseTime) {
          masterStatusHolder.NextState = NORMAL_OPS;
          normOpEntry = millis();
        } else {
          delay(10000);
        }
        break;
      }

    case (DEPLOY_ARMED): { //TODO Broken Door Sensor or Light Sensor Fallback
        SensorDataCollect();
        if (millis() - lastSComAttempt >= 5) {
          lastSComAttempt = millis();
          sendIMUToSlave();
          bool SlaveResponse = requestFromSlave();
          if (SlaveResponse) {
            lastSComTime = millis(); //Reset Timeout if Com is successful
          }
        }
        Serial.print("<" + String(digitalRead(DoorSensePin)) + String(masterStatusHolder.LightSense) + ">");
        if (cycle % 29 == 0) {
          Serial.println("");
        }
        if (DA_Initialize) {
          digitalWrite(DoorTrig, HIGH); //Activate Nichrome
          DA_Initialize = false;
        }
        switch (masterStatusHolder.deploySetting) {
          case (0): //Use Door OR Light
            if ((masterStatusHolder.hardwareAvTable[8] && digitalRead(DoorSensePin)) ||
                (masterStatusHolder.hardwareAvTable[9] && masterStatusHolder.LightSense > LightThreshold)) { //wait for door sensor
              //Door is open
              Serial.print(F("Door Release, Start Image Capture"));
              digitalWrite(DoorTrig, LOW); //deactivate nichrome wire
              delay(200);
              sendSCommand("101,1!"); //Trigger Camera
              masterStatusHolder.NextState = DEPLOY_VERIF;
              deployVEntry = millis();
            }
            break;
          case (1): //Just Use Light
            if (masterStatusHolder.hardwareAvTable[9] && masterStatusHolder.LightSense > LightThreshold) { //wait for door sensor
              //Door is open
              Serial.print(F("Door Release, Start Image Capture"));
              digitalWrite(DoorTrig, LOW); //deactivate nichrome wire
              delay(200);
              sendSCommand("101,1!"); //Trigger Camera
              masterStatusHolder.NextState = DEPLOY_VERIF;
              deployVEntry = millis();
            }
            break;
          case (2): //Just Use Door
            if (masterStatusHolder.hardwareAvTable[8] && digitalRead(DoorSensePin)) { //wait for door sensor
              //Door is open
              Serial.print(F("Door Release, Start Image Capture"));
              digitalWrite(DoorTrig, LOW); //deactivate nichrome wire
              delay(200);
              sendSCommand("101,1!"); //Trigger Camera
              masterStatusHolder.NextState = DEPLOY_VERIF;
              deployVEntry = millis();
            }
            break;
        }
        if (millis() - deployArmedEntry > (long)(60 * 3 * 1000)) {
          digitalWrite(DoorTrig, LOW);
          masterStatusHolder.missionStatus = 3;
        }
        break;
      }

    case (DEPLOY_VERIF):
      Serial.print("$");
      if (cycle % 100 == 0) {
        Serial.println("");
      }
      delay(1);
      //bool SlaveResponse = requestFromSlave(); //Need inputisvalid
      lastAccelTime = millis();

      //      if (millis() - lastAccelTime >= 1000 &&  masterStatusHolder.accelIndex <= 40){ //take accelerometer data every .5s for 20s
      //        masterStatusHolder.AccelData[0][masterStatusHolder.accelIndex] = masterStatusHolder.Accel[0];
      //        masterStatusHolder.AccelData[1][masterStatusHolder.accelIndex] = masterStatusHolder.Accel[1];
      //        masterStatusHolder.AccelData[2][masterStatusHolder.accelIndex] = masterStatusHolder.Accel[2];
      //        lastAccelTime = millis();
      //        masterStatusHolder.accelIndex++;
      //      }

      if (masterStatusHolder.LightSense > LightThreshold) { //LightSensor Trigger
        masterStatusHolder.PayloadDeployed == true;
        masterStatusHolder.missionStatus = 2;
      } else {
        //masterStatusHolder.PayloadDeployed == false;
      }

      if (millis() - deployVEntry > 5000) {
        masterStatusHolder.NextState = NORMAL_OPS;
      }
      break;

    case (LOW_POWER):
      masterStatusHolder.updateSensors(); //Fix Later
      if (masterStatusHolder.Battery * 2 >= HV_Threshold) {
        masterStatusHolder.NextState = NORMAL_OPS;
      } else {
        delay(10000);
      }
      break;


  }
  masterStatusHolder.State = masterStatusHolder.NextState;
  //Testing Iterators
  cycle++;
  //Serial.print("C : ");
  //Serial.println(cycle);
  if (masterStatusHolder.State == NORMAL_OPS && (millis() % (long)60000 == 0)) {
    long t = millis();
    Serial.println("<<System Time: "String(t % (long)60 * 60000) + ":" +
                   String(t % (long)60000) + ":" + String(t % (long)1000) + ">>");
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////












