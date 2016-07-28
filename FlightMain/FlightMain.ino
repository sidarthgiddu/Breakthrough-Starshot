
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

////Constant Initialization
unsigned long cruiseEnd = 30 * 60 * 1000;
unsigned long ledLastTime = millis();
long cycle = 0;
int ledState = LOW;
unsigned long manualTimeout = 10 * 1000;
int SlaveResets = 0;
unsigned long deployTimeOut = 30 * 1000;
bool hardwareAvTable[8] = {true}; //Hardware Avaliability Table
//[Imu, SX+,SX-,SY+, SY-, SZ+, SZ-,Temp]

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

float LV_Threshold = 3.2;
float HV_Threshold = 3.8;
float EclipseAmp_Threshold = 0.01;
int LT_Threshold = -10; //C
int HT_Threshold = 60; //C

//RockBlock Test
unsigned long lastRBCheck = 0;
long RBCheckTime = 5 * 60 * 1000;

//IMU and Sensor Test
bool SensorFetch = false;
bool imuWorking = true;
bool masterUseIMU = true;
///Adafruit_LSM9DS0 imu = Adafruit_LSM9DS0();
int imuSensorDwell = 50;
float gyroThresholdY = 3;
float gyroThresholdX = 3;

//Slave Communication Test
bool slaveWorking = true;
long recentSlaveCom = 0;
bool TestSCom = true;
long lastSComTime = 0;
long lastSComAttempt = 0;
int SComTime = 10;
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
const int DoorSens = 13;
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
const int SolarXPlus = A1; //Solar Current X+
const int SolarXMinus = A2; //Solar Current X-
const int SolarYPlus = A3; //Solar Current Y+
const int SolarYMinus = A4; //Solar Current Y-
const int SolarZPlus = A5; //Solar Current Z+
const int SolarZMinus = 9; //Solar Current Z-
const int SlaveReset = 10; //Slave Fault Handing (via Hard Reset)
const int DoorMagEnable = 11; //Allow Door Magnetorquer to work

//Downlink Test Placeholders
long DLTime = 6005;
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//IMU Code

floatTuple getMagData(Adafruit_LSM9DS0 imu, int wT) {
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

floatTuple getGyroData(Adafruit_LSM9DS0 imu, int wT) {
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

int getImuTempData(Adafruit_LSM9DS0 imu, int wT) {
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

class masterStatus {
  public:
    Adafruit_LSM9DS0 imu;

    int State;
    int NextState;
    float Mag[3];
    float Gyro[3];
    float Accel[3];
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
    bool PayloadDeployed;
    int missionStatus; //0=incomplete, 1=success, 2=failure


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


    masterStatus(int S = NORMAL_OPS, floatTuple g = floatTuple(0, 0, 0) , floatTuple M = floatTuple(0, 0, 0), int IT = 0,
                 float B = 0, float SolarXP = 0, float SolarXM = 0, float SolarYP = 0, float SolarYM = 0,
                 float SolarZP = 0, float SolarZM = 0, int DS = 0, float LS = 0,
                 float AT = 0, int nP = 0, bool IMUW = true, bool SW = true, int R = 0, int MR = 0,
                 int XD = 0, int XP = 0, int YD = 0, int YP = 0, int ZD = 0, int ZP = 0, bool ADCS = false,
                 bool pd = false, int mS = 0 ) {

      State = S;
      NextState = State;
      PayloadDeployed = pd;
      imu = Adafruit_LSM9DS0();
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
      missionStatus = mS;

      ADCS_Active = ADCS;
      MResets = MR;
      CurXDir = XD;
      CurXPWM = XP;
      CurYDir = YD;
      CurYPWM = YP;
      CurZDir = ZD;
      CurZPWM = ZP;

      IMUData[3][360] = {0};
      LIGHTData[360] = {0};
      dataIndex = 0;
      accelIndex = 0;
    }
    void updateSensors(int wT) {
      if (imuWorking) {
        floatTuple M = getMagData(imu, wT);
        floatTuple g = getGyroData(imu, wT);
        Gyro[0] = g.x; Gyro[1] = g.y; Gyro[2] = g.z;
        Mag[0] = M.x; Mag[1] = M.y; Mag[2] = M.z;
      }
      SolarXPlus = getCurrentAmp(1); //X+
      SolarXMinus = getCurrentAmp(2); //X-
      SolarYPlus = getCurrentAmp(3); //Y+
      SolarYMinus = getCurrentAmp(4); //Y-
      SolarZPlus = getCurrentAmp(5); //Z+
      SolarZMinus = getCurrentAmp(6); //Z-

      Battery = analogRead(BatteryPin);

      //Request Light/Temp Data From Slave

    }
    void configureSensor()
    {
      //set magnetometer range to +-2 gauss
      imu.setupMag(imu.LSM9DS0_MAGGAIN_2GAUSS);
      //set gyro range to +-245 degrees per second
      imu.setupGyro(imu.LSM9DS0_GYROSCALE_245DPS);
      //imu.setupAccel(imu.LSM9DS0_ACCEL_MG_LSB_2G);
    }
    String toString() {
      //Produces JSON Output in ASCII  for Downlink
      String output = "";
      output += "{";
      output += "S:" + String(State) + ",";
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
      output += "IW:" + String(IMUWorking) + ",";
      output += "SW:" + String(SlaveWorking) + ",";
      output += "Rs:" + String(Resets) + ",";
      output += "AA:" + String(ADCS_Active) + ",";
      output += "XD" + String(CurXDir) + ",";
      output += "XP" + String(CurXPWM) + ",";
      output += "YD" + String(CurYDir) + ",";
      output += "YP" + String(CurYPWM) + ",";
      output += "ZD" + String(CurZDir) + ",";
      output += "ZP" + String(CurZPWM) + ",";
      return output;
    }
};
masterStatus masterStatusHolder;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Helper Functions

void initalizePinOut() {
  ///const int DoorSens = 13; pinMode(DoorSens, INPUT); //WRONG
  pinMode(13, OUTPUT); //Red LED

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

float getCurrentAmp(int panel) {
  //Returns Amperage of current sensors at senseCurrent 4v/amp (40k resistor)
  float current;
  switch (panel) {
    case 1:
      if (hardwareAvTable[1]) {
        current = analogRead(SolarXPlus);
      } else {
        current = 0;
      } break;
    case 2:
      if (hardwareAvTable[2]) {
        current = analogRead(SolarXMinus);
      } else {
        current = 0;
      } break;
    case 3:
      if (hardwareAvTable[3]) {
        current = analogRead(SolarYPlus);
      } else {
        current = 0;
      } break;
    case 4:
      if (hardwareAvTable[4]) {
        current = analogRead(SolarYMinus);
      } else {
        current = 0;
      } break;
    case 5:
      if (hardwareAvTable[5]) {
        current = analogRead(SolarZPlus);
      } else {
        current = 0;
      } break;
    case 6:
      if (hardwareAvTable[6]) {
        current = analogRead(SolarZMinus);
      } else {
        current = 0;
      } break;
  }
  current = map(current, 0, 1023, 0, .825); //3.3V=.825A
  // 4V/A with a 40k Resistor
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
      Serial.println("Finished Adding Commands");
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
        //Serial.println("Excl Found");
        if (lastPunc == 1) {
          //Serial.println("Period ok");
          lastPunc = 2;
        } else {
          //Serial.println("2 Excl or No prior comma");
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
      switch (currentCommand[1]) {
        case (11):
          deployTimeOut = (currentCommand[2]);
          break;
        case (12):
          manualTimeout = (currentCommand[2]);
          break;
        case (61):
          masterStatusHolder.MResets = (currentCommand[2]);
          break;
        case (62):
          masterStatusHolder.AnalogTemp = (currentCommand[2]);
          break;
        case (63):
          masterStatusHolder.LightSense = (currentCommand[2]);
          break;
        case (64):
          masterStatusHolder.CurXDir = (currentCommand[2]);
          break;
        case (65):
          masterStatusHolder.CurYDir = (currentCommand[2]);
          break;
        case (66):
          masterStatusHolder.CurZDir = (currentCommand[2]);
          break;
        case (67):
          masterStatusHolder.CurXPWM = (currentCommand[2]);
          break;
        case (68):
          masterStatusHolder.CurYPWM = (currentCommand[2]);
          break;
        case (69):
          masterStatusHolder.CurZPWM = (currentCommand[2]);
          break;
        case (610):
          masterStatusHolder.numPhotos = (currentCommand[2]);
          break;
      }

    } else {
      //Serial.println("No Command");
    }
  }
}

void readSerialAdd2Buffer() {
  //Modify for RB Communication
  if (Serial.available() > 0) {
    //Serial.println("Recieving Command");
    String comString = "";
    while (Serial.available() > 0) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      comString += inChar;
    }
    if (isInputValid(comString)) {
      //Serial.println("Command is Valid");
      buildBuffer(comString);
      popCommands();
    } else {
      //Serial.println("Invalid Command");
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Slave Functions

/* Supported Commands
  Send Gyro X to Slave: "11,<(String)(float)>!"
  Send Gyro Y to Slave: "12,<(String)(float)>!"
  Send Gyro Z to Slave: "13,<(String)(float)>!"
  Send Mag X to Slave: "21,<(String)(float)>!"
  Send Mag Y to Slave: "22,<(String)(float)>!"
  Send Mag Z to Slave: "23,<(String)(float)>!"
  Z Torquer On for Heat: "41,1!"
  Z Torquer Off for Heat: "41,0!"
  Activate ACDS: "51,1!"
  Deactivate ACDS: "51,0!"
  Take a Picture if ready: "61,1!"
*/

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

String buildIMUDataCommand() {
  // ex. gyro data: "11,3.653!12,2.553!13,-10!"
  String res = "";
  res += "11," + String(masterStatusHolder.Gyro[0]) + "!";
  res += "12," + String(masterStatusHolder.Gyro[1]) + "!";
  res += "13," + String(masterStatusHolder.Gyro[2]) + "!";
  res += "21," + String(masterStatusHolder.Mag[0]) + "!";
  res += "22," + String(masterStatusHolder.Mag[1]) + "!";
  res += "23," + String(masterStatusHolder.Mag[2]) + "!";
  return res;
}

void sendIMUToSlave() {
  String SCommand = buildIMUDataCommand();
  char SComCharA[SCommand.length()];
  SCommand.toCharArray(SComCharA, SCommand.length());
  sendSCommand(SComCharA);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// RockBlock Uplink/Downlink Functions
//

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

  //Try to initialize and warn if we couldn't detect the chip
  int endT = millis() + manualTimeout;

  while (!masterStatusHolder.imu.begin() && millis() < endT) {
    Serial.println("IMU Not working");
  }


  cBuf = commandBuffer();
  masterStatusHolder = masterStatus();

  masterStatusHolder.configureSensor(); //runs configureSensor function
  //pinMode(A3, INPUT); //Temperature Sensor
  Wire.begin(); //Start i2c as master

  //  for (int i = 0 ; i < 10; i++) {
  //    Serial.println("Finished Setup");
  //  }
}

void loop() {
  switch (masterStatusHolder.State) {
    case (NORMAL_OPS):
      {

        //Collect Sensor Data
        //if (SensorFetch) {
        masterStatusHolder.updateSensors(imuSensorDwell);
        //Request Light/Temp Data From Slave
        //}

        if (millis() - lastRBCheck >= RBCheckTime) {
          //Do RockBlock Stuff
        }

        //Testing IMU and Sensor Downlink String Generator
        if (millis() - lastDLTime >= DLTime || commandedDL) {
          //Send Data to RockBlock via Serial
          Serial.println("Downlink String: ");
          String DLS = masterStatusHolder.toString();


          Serial.println(DLS);
          Serial.println(DLS.length());
          lastDLTime = millis();
        }

        //Test Slave Communication
        if (TestSCom) {
          if (millis() - lastSComAttempt >= SComTime || commandedSC) {
            lastSComAttempt = millis();
            //Serial.print("Slave Status Report: "); //Stalls here with Wire?
            sendIMUToSlave();
            String SlaveResponse = requestFromSlave();

            if (!SlaveResponse.equals("")) {
              lastSComTime = millis(); //Reset Timeout if Com is successful
              if (isInputValid(SlaveResponse)) {
                //Serial.println(Valid Reply From Slave);
                buildBuffer(SlaveResponse);
              } else {
                //Serial.println("Invalid Response");
              }
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
              Serial.print("No Reply From Slave for ");
              Serial.print((millis() - lastSComTime) / 1000.0);
              Serial.println(" seconds");
              //            }
            }
          }
        }

        //ADCS
        if (millis() - LastSpinCheckT > SpinCheckTime) {
          float spinMagnitude = 0;
          for (int i = 0; i < 3; i++) {
            spinMagnitude = pow(masterStatusHolder.Gyro[i], 2);
          }
          spinMagnitude = sqrt(spinMagnitude);
          if (spinMagnitude > OmegaThreshold) {
            sendSCommand("91,1!"); //Activate Torquers
            masterStatusHolder.ADCS_Active = true;
          } else {
            sendSCommand("91,0!"); //Activate Torquers
            masterStatusHolder.ADCS_Active = true;
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
        if (millis() - ledLastTime >= 500) {
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
        break;
      }

    case (DORMANT_CRUISE):
      {
        //30 min Dormant Cruise
        if (millis() > cruiseEnd) {
          masterStatusHolder.NextState = INITALIZATION;
          initEntry = millis();
        } else {
          delay(10000);
        }
        break;
      }

    case (INITALIZATION):
      {
        //Initiate Detumble "41,1!"-->"51,1!"?
        char data[] = {'5', '1', ',', '1', '!'};
        sendSCommand(data);
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
        float Amps = getTotalAmperage();
        if (Amps > .1 || millis() - eclipseEntry > forceExitEclipseTime) {
          masterStatusHolder.NextState = NORMAL_OPS;
          normOpEntry = millis();
        } else {
          delay(10000);
        }
        break;

      }

    case (DEPLOY_ARMED):
      if (DA_Initialize) {
        char data[] = {'6', '1', ',', '1', '!'};
        sendSCommand(data); //Prep Camera
        digitalWrite(24, HIGH); //Activate Nichrome
        DA_Initialize = false;
      }
      if (masterStatusHolder.DoorSense == LOW) { //wait for door sensor
        //Door is open
        //sendSCommand(); //Trigger Camera
        digitalWrite(DoorTrig, LOW); //deactivate nichrome wire
        masterStatusHolder.NextState = DEPLOY_VERIF;
        deployVEntry = millis();

      } else {
        if (millis() - deployArmedEntry > (long)60 * 6 * 1000) {
          digitalWrite(DoorTrig, LOW);
          masterStatusHolder.missionStatus = 3;
        }
      }

      break;


    case (DEPLOY_VERIF):
      buildBuffer(requestFromSlave());
      lastAccelTime = millis();

      if (millis() - lastAccelTime >= 1000 &&  masterStatusHolder.accelIndex <= 40) //take accelerometer data every .5s for 20s
      {
        masterStatusHolder.AccelData[0][masterStatusHolder.accelIndex] = masterStatusHolder.Accel[0];
        masterStatusHolder.AccelData[1][masterStatusHolder.accelIndex] = masterStatusHolder.Accel[1];
        masterStatusHolder.AccelData[2][masterStatusHolder.accelIndex] = masterStatusHolder.Accel[2];
        lastAccelTime = millis();
        masterStatusHolder.accelIndex++;
      }
      if (masterStatusHolder.LightSense > placeHolderLightSense ) { //LightSensor Trigger
        masterStatusHolder.PayloadDeployed == true;
        masterStatusHolder.missionStatus = 2;
      }
      else {
        masterStatusHolder.PayloadDeployed == false;
      }

      break;


    case (DEPLOY_DOWN_LK): {
        //Upon Request Downlink Image
        //Downlink Data
      }
      break;

    case (LOW_POWER):
      masterStatusHolder.updateSensors(1);
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







