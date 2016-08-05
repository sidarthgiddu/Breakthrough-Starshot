#include <Wire.h>
#include <MatrixMathDOUBLE.h>
#include <Adafruit_VC0706.h>
#include <SPI.h>
#include <SD.h>

////Constant Initialization
bool TestReset = false;

int MasterFaultTime = 5 * 60 * 1000;
int lastMasterCom = 0;
int resets = 0;
int manualTimeoutS = 10 * 1000;
bool imuWorking = true;
int lightLevel = 0;
int tempBattery = 0;
//int numPhotos = 0;
//int photoInterval = 0;

////Magnetorquer States
////Maybe FloatTuples
//int CurXDir = 0; //-1 or 1 for Coil Current Direction
//int CurXPWM = 0; // 0 to 255 for Coil Current Level
//int CurYDir = 0; //-1 or 1 for Coil Current Direction
//int CurYPWM = 0; // 0 to 255 for Coil Current Level
//int CurZDir = 0; //-1 or 1 for Coil Current Direction
//int CurZPWM = 0; // 0 to 255 for Coil Current Level

//Slave Pinouts
const int MasterReset = A4;
const int CamRx = 0; //Camera Serial into SCom
const int CamTx = 1; //Camera Serial out of SCom
const int SDApin = 20; //I2C Data
const int SCLpin = 21; //I2C Clock
const int TempS = A0;  //Analog Temp Sensor
const int LightS = A1; //Analog Light Sensor

const int CX1 = 22; //Coil X Input 1
const int CX2 = 23; //Coil X Input 2
const int CY1 = 5; //Coil Y Input 1
const int CY2 = 6; //Coil Y Input 2
const int CZ1 = 9; //Coil Z Input 1
const int CZ2 = 10; //Coil Z Input 2

const int CX_PWM = 11; //Coil X PWM Control
const int CY_PWM = 12; //Coil Y PWM Control
const int CZ_PWM = 13; //Coil Z PWM Control
const int CXY_Enable = A5; //Coil X and Y Enable
const int CZ_Enable = 24; //Coil Z Enable

//Software PWM Values
bool p1 = true;
bool p2 = true;
bool p3 = true;
int cycleLength = 600; //Microseconds
unsigned long LastCycle = 0;
unsigned long NextCycle = cycleLength;
bool ADCS = true;
bool startCycle = true;
bool ADCS_exit = false;

//Camera
#define chipSelect 4
#define smallImage VC0706_160x120
#define mediumImage VC0706_320x240
#define largeImage VC0706_640x480

//SD Card
#define chipSelect 4
#define DLSize 320

Adafruit_VC0706 cam = Adafruit_VC0706(&Serial1);

int getTempDegrees() {
  // Returns the temperature of the sensor at pin senseTemp
  int temp = map(analogRead(TempS), 0, 543, -50, 125); //round?
  return temp;
}

int getLightLvl() {
  //Returns the lighting of the sensor at pin senselight (10k resistor)
  int light = map(analogRead(LightS), 0, 775, 100, 0); //2.5 / 3.3 *1023
  return light;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Determine Remaining RAM
extern "C" char *sbrk(int i);
int freeRam () {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
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

void printArray(uint8_t * arr, int s) {
  if (s) {
    for (int i = 0; i < s; i++) {
      //    for (int j = 0; j < 8; j++) {
      //      if (arr[i] < pow(2, j))
      //        Serial.print(B0);
      //    }
      print_binary(arr[i], 8);
      Serial.print(" ");
    }
    Serial.println("");
  }
}

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

    void printRI() {
      Serial.println("RAM Loaded Image: " + Filename);
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
      //Blank
    }

    void store(uint8_t * data, int dataSize, int index) {
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


class slaveStatus
{
  public:
    bool ADCS_Active;
    int Temp;
    int Light;
    int mtX;
    int mtY;
    int mtZ;
    int Resets;
    int numPhotos;
    int CurXDir; //-1 or 1 for Coil Current Direction
    int CurXPWM; // 0 to 255 for Coil Current Level
    int CurYDir; //-1 or 1 for Coil Current Direction
    int CurYPWM; // 0 to 255 for Coil Current Level
    int CurZDir; //-1 or 1 for Coil Current Direction
    int CurZPWM; // 0 to 255 for Coil Current Level
    double gyro[3];
    double mag[3];
    int durX;
    int durY;
    int durZ;
    bool CameraBurst;
    int imageSize;
    unsigned long burstStart;
    unsigned long burstDuration;
    RAMImage imageR;
    bool ImageRequested;

    //Iridium Network Status Attributes





    slaveStatus(float t = 0, int L = 0, int r = 0, int n = 0, int XD = 0, int XP = 0,
                int YD = 0, int YP = 0, int ZD = 0, int ZP = 0,
                floatTuple g = floatTuple(0, 0, 0), floatTuple M = floatTuple(0, 0, 0),
                int IS = smallImage, long BD = 120000) {

      imageR = RAMImage();
      ImageRequested = false;

      ADCS_Active = false;
      Temp = t;
      Light = L;
      Resets = r;
      numPhotos = n;
      CurXDir = XD;
      CurXPWM = XP;
      CurYDir = YD;
      CurYPWM = YP;
      CurZDir = ZD;
      CurZPWM = ZP;
      gyro[0] = g.x; gyro[1] = g.y; gyro[2] = g.z;
      mag[0] = M.x; mag[1] = M.y; mag[2] = M.z;
      durX = 0;
      durY = 0;
      durZ = 0;
      imageSize = IS;
      CameraBurst = false;
      burstDuration = BD; //Default = 120s
    }
    void pwmWrite(int x, int y, int z) {
      durX = map(x, 0, 255, 0, cycleLength);
      durY = map(y, 0, 255, 0, cycleLength);
      durZ = map(z, 0, 255, 0, cycleLength);
    }
    String toString() {
      String res = "";
      //Update Order Below and Update Master Accordingly
      res += "61," + String(resets); //Resets
      res += "!62," + String(Temp); //Temp
      res += "!63," + String(Light); //Light
      res += "!64," + String(CurXDir);
      res += "!65," + String(CurYDir);
      res += "!66," + String(CurZDir);
      res += "!67," + String(CurXPWM);
      res += "!68," + String(CurYPWM);
      res += "!69," + String(CurZPWM);
      res += "!610," + String(numPhotos);
      res += "!";
      return res;
    }
    void WriteStatus() {
      //Copy Order Above and Update Master Accordingly
      Wire.write((uint8_t)resets);
      Wire.write((uint8_t)Temp);
      Wire.write((uint8_t)Light);
      Wire.write((uint8_t)CurXDir);
      Wire.write((uint8_t)CurYDir);
      Wire.write((uint8_t)CurZDir);
      Wire.write((uint8_t)CurXPWM);
      Wire.write((uint8_t)CurYPWM);
      Wire.write((uint8_t)CurZPWM);
      Wire.write((uint8_t)numPhotos);
      //10Byte Transmission
    }
    void print() {
      Serial.println(toString());
    }
    void updatePassive() {
      //Update Internal Information
      Temp = getTempDegrees();
      Light = getLightLvl();
    }
    void updateTorquers(floatTuple Dir, floatTuple PWM) {
      CurXDir = Dir.x;
      CurXPWM = PWM.x;
      CurYDir = Dir.y;
      CurYPWM = PWM.y;
      CurZDir = Dir.z;
      CurZPWM = PWM.z;
      /// AnalogWrite ....
      Serial.print("Direction: ");
      Dir.print();
      Serial.print("PWM Values: ");
      PWM.print();
    }
};
slaveStatus StatusHolder;

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
      int endT = millis() + manualTimeoutS;
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

////////////////////////////////
////////////////////////////////
////////////////////////////////
//======================================== ADCS DATA ===================================================================
double mass = 1.33;
double zeroMain = 0.0, six = 6.0;
double Bfield[3];
double w[3];
double Inertia[3][3] = {{mass / six, zeroMain, zeroMain},
  {zeroMain, mass / six, zeroMain},
  {zeroMain, zeroMain, mass / six}
}; // Inertia initialization

double E = 1; //0.0001;

void runADCS(double* Bvalues, double* gyroData, double Kp, double Kd) {

  //////////////freeRam ();
  //double J[9] = {0,Bvalues[2],(-1.0)*Bvalues[1],(-1.0)*Bvalues[2], 0, Bvalues[0], Bvalues[1], (-1.0)*Bvalues[0], 0};

  //Matrix.Print((double*)Bvalues, 3, 1, "Magn");
  //Matrix.Print((double*)gyroData, 3, 1, "Gyro");

  Matrix.Copy((double*)Bvalues, 1, 3, (double*)Bfield); // create new field to scale for the pseudo-inverse
  //Matrix.Scale((double*)Bfield, 3, 1, E); // scale duplicated Bfield array with E for pseudo-inverse
  //Matrix.Print((double*)Bfield, 3, 1, "Scaled Bfield");
  double Jnew[4][3] = {0, Bvalues[2], (-1.0)*Bvalues[1],
    {(-1.0)*Bvalues[2], 0, Bvalues[0]},
    {Bvalues[1], (-1.0)*Bvalues[0], 0},
    {Bfield[0], Bfield[1], Bfield[2]}
  };
  ////Serial.println(freeRam ());
  double Jtranspose[3][4];
  double Jp[3][3]; // Jproduct here, but Jpseudo-inverse later
  double Jppinv[3][4];

  Matrix.Transpose((double*)Jnew, 4, 3, (double*)Jtranspose); // transpose(Jnew)
  //Serial.println ("Step 1 complete");
  Matrix.Multiply((double*)Jtranspose, (double*)Jnew, 3, 4, 3, (double*)Jp); //transpose(Jnew)*Jnew=Anew
  //Serial.println ("Step 2 complete");
  Matrix.Invert((double*)Jp, 3); // inverse(transpose(Jnew)*Jnew)=Bnew
  //Serial.println ("Step 3 complete");
  Matrix.Multiply((double*)Jp, (double*)Jtranspose, 3, 3, 4, (double*)Jppinv); // Bnew*transpose(Jnew)=Cnew
  //Serial.println ("Step 4 complete");

  // redefine Jp as Jpseudo-inverse
  Jp[0][0] = Jppinv[0][0]; Jp[0][1] = Jppinv[0][1]; Jp[0][2] = Jppinv[0][2]; // rescaled up
  Jp[1][0] = Jppinv[1][0]; Jp[1][1] = Jppinv[1][1]; Jp[1][2] = Jppinv[1][2];
  Jp[2][0] = Jppinv[2][0]; Jp[2][1] = Jppinv[2][1]; Jp[2][2] = Jppinv[2][2];
  Matrix.Scale((double*)Jp, 3, 3, 1000000.0);
  //////////////Serial.println(freeRam ());
  double OmegaError[3], BfieldError[3], ErrorSum[3], current[3];
  double Omegacmd[3] = {0, 0, 1};
  double Bmagnitude = sqrt(Bvalues[0] * Bvalues[0] + Bvalues[1] * Bvalues[1] + Bvalues[2] * Bvalues[2]);
  double Bcmd[3] = {0, 0, 1};
  double A = 0.532;

  Matrix.Scale((double*)Bfield, 3, 1, 1 / Bmagnitude);

  Matrix.Subtract((double*) Bfield, (double*) Bcmd, 3, 1, (double*) BfieldError);
  //Serial.println ("Step 5 complete");
  Matrix.Subtract((double*) gyroData, (double*) Omegacmd, 3, 1, (double*) OmegaError);
  //Serial.println ("Step 6 complete");
  //////////////Serial.println(freeRam ());

  Matrix.Scale((double*)BfieldError, 3, 1, (Kp / A)); // scale error with proportional gain (updates array)

  //Serial.println ("Step 7 complete");
  Matrix.Scale((double*)OmegaError, 3, 1, (Kd / A)); // scale error with derivative gain (updates array)
  //Serial.println ("Step 8 complete");

  Matrix.Add((double*)BfieldError, (double*)OmegaError, 3, 1, (double*) ErrorSum);
  //Serial.println ("Step 9 complete"); delay(50);
  Matrix.Scale((double*) ErrorSum, 3, 1, -1.0); // prep error for multiplication with the Jpinv matrix
  //Serial.println ("Step 10 complete"); delay(50);

  //Matrix.Print((double*)ErrorSum, 3, 1, "MATRIX TO CHECK");
  //Matrix.Print((double*)Jp, 3, 3, "Jpinv");

  //Serial.println(freeRam ()); delay(50);
  Matrix.Multiply((double*) Jp, (double*) ErrorSum, 3, 3, 1, (double*) current);
  //Serial.println ("Step 11 complete");

  //////////////Serial.println(freeRam ());
  //delete (Jp); delete  (Jtranspose); delete  (Jppinv);
  //delete (gyroData); delete (Bvalues); delete (Jnew);
  //delete (OmegaError); delete (BfieldError); delete (Omegacmd); delete (Bcmd);
  //Serial.println("Calling outputPWM"); delay(50);
  //Matrix.Print((double*)current, 3, 1, "Current");
  outputPWM((double*) current, 3);
  //Serial.println("outputPWM ran"); delay(50);
  //delete (ErrorSum);
}

void outputPWM(double* I, int length) {
  float Imax = 0.2;
  //String I1 = I[0].toString();
  //String I2 = I[1].toString();
  //String I3 = I[2].toString();
  //float If[3] = {I1.toFloat(), I2.toFloat(), I3.toFloat()};
  //delete I,I1,I2,I3;

  for (int i = 0; i < length; i++) {
    if (abs(I[i]) > Imax) {
      //Serial.println("saturated");
      I[i] = Imax * sgn(I[i]);
    }
    //Serial.print(I[i]); Serial.println(" ");
  }

  // CREATE PWM OUT SIGNAL

  //StatusHolder.pwmWrite(I[0] / Imaxf * 255,I[1] / Imaxf * 255,I[2] / Imaxf * 255);
  /// ...
  //analogWrite(CX_PWM, I[0] / Imaxf * 255);
  //analogWrite(CY_PWM, I[1] / Imaxf * 255);
  //analogWrite(CZ_PWM, I[2] / Imaxf * 255);

  floatTuple PWMvaluesForTorquers = floatTuple(I[0] / Imax * 255, I[1] / Imax * 255, I[2] / Imax * 255);
  floatTuple PWMdirectionsForTorquers = floatTuple(sgn(I[0]), sgn(I[1]), sgn(I[2]));
  StatusHolder.updateTorquers(PWMdirectionsForTorquers, PWMvaluesForTorquers);
}

static inline double sgn(double val) {
  if (val < 0.0) return -1.0;
  if (val == 0.0) return 0.0;
  return 1.0;
}
// Placeholder Test Data
double gData[3] = {0.2, 0.04, -0.1};
double mData[3] = {28.87, 28.87, 28.87}; //in 1000s of nT (or *10^3 nT)
double Kp = 0.001;
double Kd = 0.001;


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
  int endT = manualTimeoutS + millis();
  if (input.equals("")) {
    return false; //Recently Added
  }
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

void PopCommands() {
  //Process an Incoming Command
  while (cBuf.openSpot > 0) { //Manual Timeout
    Serial.println ("Executing Command:");
    if (cBuf.openSpot > 0) {
      int currentCommand[2] = {cBuf.commandStack[cBuf.openSpot - 1][0],
                               cBuf.commandStack[cBuf.openSpot - 1][1]
                              };
      Serial.print(currentCommand[0]);
      Serial.print(":");
      Serial.println(currentCommand[1]);
      cBuf.commandStack[cBuf.openSpot - 1][0] = -1;
      cBuf.commandStack[cBuf.openSpot - 1][1] = -1;
      cBuf.openSpot--;

      //Switch Case on Command[0]
      switch (currentCommand[0]) {
        case (11): //Update Gyro X
          StatusHolder.gyro[0] = currentCommand[1] / 1000.0;
          break;
        case (12): //Update Gyro Y
          StatusHolder.gyro[1] = currentCommand[1] / 1000.0;
          break;
        case (13): //Update Gyro Z
          StatusHolder.gyro[2] = currentCommand[1] / 1000.0;
          break;
        case (21): //Update Mag X
          StatusHolder.mag[0] = currentCommand[1] / 1000.0;
          break;
        case (22): //Update Mag Y
          StatusHolder.mag[1] = currentCommand[1] / 1000.0;
          break;
        case (23): //Update Mag Z
          StatusHolder.mag[2] = currentCommand[1] / 1000.0;
          break;
        case (91): //Activate/Deactive ADCS (1=On,0=Off);
          StatusHolder.ADCS_Active = currentCommand[1];
          break;
        case (101): //Start PhotoBurst
          StatusHolder.CameraBurst = true;
          StatusHolder.burstStart = millis();
          break;
        case (102): //Set PhotBurst Duration
          StatusHolder.burstDuration = currentCommand[1] * 1000;
          break;
        case (103): //Request Photo #<currentCommand[1]>
          char filename[8];
          strcpy(filename, "0000.JPG");
          filename[0] = '0' + currentCommand[1] / 1000;
          filename[1] = '0' + currentCommand[1] % 1000 / 100;
          filename[2] = '0' + currentCommand[1] % 1000 % 100 / 10;
          filename[3] = '0' + currentCommand[1] % 1000 % 100 % 10;
          buildImageBuffer(filename);
          break;
      }
    } else {
      Serial.println(F("No Command"));
    }
  }
  Serial.println(F("Done"));
}

void commandParser(int nBytes) {
  //Need nBytes?
  digitalWrite(8, HIGH);
  String command = "";
  while (Wire.available()) {
    char c = Wire.read(); // receive byte as a character
    command += c;        // print the character
  }
  Serial.println("Com: " + command);
  //Parse Command
  if (isInputValid(command)) {
    Serial.println(F("Command is Valid"));
    buildBuffer(command);
    Serial.println(F("Built Command Buffer Successfully"));
    PopCommands();
  } else {
    Serial.println(F("Invalid Command"));
  }
  digitalWrite(8, LOW);
  Serial.println(F("Here"));
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void requestEvent() {
  //Serial.println("Data Request");
  digitalWrite(8, HIGH);
  if (StatusHolder.ImageRequested) {
    //Image Transfer
  } else {
    //General Communication
    StatusHolder.WriteStatus(); //Check Format
  }
  lastMasterCom = millis();
  digitalWrite(8, LOW);
}

void initalizePinOut() {
  const int MasterReset = A4; pinMode(MasterReset, OUTPUT); //Reset Master Core
  digitalWrite(MasterReset, HIGH);
  const int CamRx = 0; //Camera Serial into SCom
  const int CamTx = 1; //Camera Serial out of SCom
  const int SDApin = 20; //I2C Data
  const int SCLpin = 21; //I2C Clock
  const int TempS = A0; pinMode(TempS, INPUT); //Analog Temp Sensor
  const int LightS = A1; pinMode(LightS, INPUT); //Analog Light Sensor

  const int CX1 = 22; pinMode(CX1, OUTPUT); //Coil X Input 1
  const int CX2 = 23; pinMode(CX2, OUTPUT); //Coil X Input 2
  const int CY1 = 5; pinMode(CY1, OUTPUT); //Coil Y Input 1
  const int CY2 = 6; pinMode(CY2, OUTPUT); //Coil Y Input 2
  const int CZ1 = 9; pinMode(CZ1, OUTPUT); //Coil Z Input 1
  const int CZ2 = 10; pinMode(CZ2, OUTPUT); //Coil Z Input 2

  const int CX_PWM = 11; pinMode(CX_PWM, OUTPUT); //Coil X PWM Control
  const int CY_PWM = 12; pinMode(CY_PWM, OUTPUT); //Coil Y PWM Control
  const int CZ_PWM = 13; pinMode(CZ_PWM, OUTPUT); //Coil Z PWM Control
  const int CXY_Enable = A5; pinMode(CXY_Enable, OUTPUT); //Coil X and Y Enable
  const int CZ_Enable = 24; pinMode(CZ_Enable, OUTPUT); //Coil Z Enable

  const int LED = 13;  pinMode(LED, OUTPUT);
}


int takePic(int ImageSize) {
  // Try to locate the camera
  cam.begin(); //Need to ensure it worked
  cam.setImageSize(ImageSize);//cam.setImageSize(VC0706_160x120);

  if (!cam.takePicture()) {
    return 0;
  } else {
    StatusHolder.numPhotos++;
    //Serial.println("Picture taken!");
  }

  // Create an image with the name IMAGExx.JPG //ALTER THIS TO STORE THE NEXT AVAILABLE FILENAME
  char filename[8];
  strcpy(filename, "0000.JPG");
  for (int i = 0; i < 9999; i++) {
    filename[0] = '0' + StatusHolder.numPhotos / 1000;
    filename[1] = '0' + StatusHolder.numPhotos % 1000 / 100;
    filename[2] = '0' + StatusHolder.numPhotos % 1000 % 100 / 10;
    filename[3] = '0' + StatusHolder.numPhotos % 1000 % 100 % 10;
    if (!SD.exists(filename)) {
      break;
    } else {
      StatusHolder.numPhotos++; //If loss of power resets it but photos were taken prior
    }
  }

  File imgFile = SD.open(filename, FILE_WRITE); //Open Image File
  uint16_t jpglen = cam.frameLength(); //Image Size in Bytes
  uint16_t bytesWritten = 0;

  //int32_t time = millis();
  pinMode(8, OUTPUT);
  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    uint8_t *databuffer;
    uint8_t bytesToRead = min(64, jpglen); // 64 Byte Reads
    databuffer = cam.readPicture(bytesToRead);
    bytesWritten += imgFile.write(databuffer, bytesToRead); //Returns bytes written if needed
    if (++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
      //Reset Timer Here
      wCount = 0;
    }
    jpglen -= bytesToRead;
  }
  imgFile.close();
  //time = millis() - time;
  return bytesWritten;
}

void buildImageBuffer(String Filename) { //Rename to masterstatusholderimagebuffer in future...
  if (!SD.exists(Filename)) { //File does not exist
    return;
  }

  File IMGFile = SD.open(Filename, FILE_READ);
  uint8_t jpglen = IMGFile.size();
  //int segments = ((8*jpglen) / 320) + 1;
  //String imageBuffer[segments];
  int index = 0;
  int i = 0;
  Serial.println("Starting Segmentation");

  while (IMGFile.available()) {
    //Serial.println("Available: " + String(IMGFile.available()));
    int bytesToRead = min(320, IMGFile.available());
    uint8_t segment[bytesToRead - 1];
    for (int z = 0; z < bytesToRead; z++) {
      segment[z] = 0;
    }
    while (i < bytesToRead) {
      segment[i] = (uint8_t)IMGFile.read();
      i++;
    }
    //Serial.print("Current Segment " + String(index) + ": ");
    printArray(segment, i);
    //Serial.println("");
    //Serial.println("Here1");
    //    imageBuffer[index] = segment;
    StatusHolder.imageR.store(segment, i, index);
    StatusHolder.imageR.sizeArray[index] = bytesToRead;

    i = 0;
    index = index + 1;
    //Serial.println("Here2");
  }

  IMGFile.close();
  Serial.println("\nDone");
  StatusHolder.imageR.finalIndex = index - 1;
  return;
}


int ledState = HIGH;
long ledLastTime = 0;
long lastADCSTime = 0;

void setup() {
  Serial.begin(9600);
  delay(1000);

  initalizePinOut();
  slaveStatus StatusHolder = slaveStatus();

  StatusHolder.ADCS_Active = true;

  //digitalWrite(MasterReset, HIGH); //Enable Master
  Wire.begin(8);
  Wire.onReceive(commandParser);
  Wire.onRequest(requestEvent);

  //Reset Indication
  pinMode(8, OUTPUT);
  for (int j = 0; j < 10; j++) {
    digitalWrite(8, HIGH);
    delay(140);
    digitalWrite(8, LOW);
    delay(140);
  }
  //Forced Stall
  //  pinMode(12, INPUT_PULLUP);
  //  attachInterrupt(digitalPinToInterrupt(12), forcedStall, LOW);

  //Camera Setup
  SD.begin(chipSelect);
  //  if (!SD.begin(chipSelect)) {
  //    Serial.println("Card failed, or not present");
  //    // don't do anything more:
  //    return;
  //  }

  //Image Transfer Test
  //  buildImageBuffer("0000.JPG");
  //  Serial.println("\n\nPrint Start");
  //  StatusHolder.imageR.printRI();
  //  Serial.println("\nPrint End");
}

bool SoftwarePWM = false;
bool disableT = false;
void loop() {
  StatusHolder.updatePassive();
  StatusHolder.ADCS_Active = false;
  //Test ADCS
  if (StatusHolder.ADCS_Active) {
    if (millis() - lastADCSTime >= 2000) {
      if (millis() - lastADCSTime >= 2100) {
        //Serial.println("happy");
        runADCS(mData, gData, Kp, Kd);
        //runADCS(StatusHolder.mag, StatusHolder.gyro, Kp, Kd); //placeholders
        //Serial.println("still happy");
        //Serial.print("X axis: "); Serial.print(StatusHolder.CurXDir, 20); Serial.print(" "); Serial.println(StatusHolder.CurXPWM, 20);
        //Serial.print("Y axis: "); Serial.print(StatusHolder.CurYDir, 20); Serial.print(" "); Serial.println(StatusHolder.CurYPWM, 20);
        //Serial.print("Z axis: "); Serial.print(StatusHolder.CurZDir, 20); Serial.print(" "); Serial.println(StatusHolder.CurZPWM, 20);
        lastADCSTime = millis();
        disableT = true;
      } else {
        //torquers off
        //analogWrite(CX_PWM, 0);
        //analogWrite(CY_PWM, 0);
        //analogWrite(CZ_PWM, 0);
        if (disableT) {
          floatTuple PWMvaluesForTorquers = floatTuple(0, 0, 0);
          floatTuple PWMdirectionsForTorquers = floatTuple(0, 0, 0);
          StatusHolder.updateTorquers(PWMdirectionsForTorquers, PWMvaluesForTorquers);
          disableT = false;
          //Request new Gyro Data
        }
      }
    }

    //SoftwarePWM
    if (SoftwarePWM) {
      unsigned long ms = micros();
      if (startCycle) {
        digitalWrite(A1, HIGH);
        digitalWrite(A2, HIGH);
        digitalWrite(A3, HIGH);
        startCycle = false;
        NextCycle = ms + cycleLength;
        LastCycle = ms;
        p1 = true;
        p2 = true;
        p3 = true;
      }
      if (p1 && (ms - LastCycle >= StatusHolder.durX)) {
        //Serial.println("Here");
        digitalWrite(CX_PWM, LOW);
        p1 = false;
      }
      if (p2 && (ms - LastCycle >= StatusHolder.durY)) {
        digitalWrite(CY_PWM, LOW);
        p2 = false;
      }
      if (p3 && (ms - LastCycle >= StatusHolder.durZ)) {
        digitalWrite(CZ_PWM, LOW);
        p3 = false;
      }
      if (ms > NextCycle) {
        //Serial.print("End");
        startCycle = true;
      }
    } else {
      if (ADCS_exit) {
        StatusHolder.CurXDir = 0;
        StatusHolder.CurYDir = 0;
        StatusHolder.CurZDir = 0;
        digitalWrite(CX_PWM, LOW);
        digitalWrite(CY_PWM, LOW);
        digitalWrite(CZ_PWM, LOW);
        ADCS_exit = false;
      }
    }
  }

  if (StatusHolder.CameraBurst) {
    if (millis() <= StatusHolder.burstStart + StatusHolder.burstDuration) {
      takePic(StatusHolder.imageSize);
      StatusHolder.numPhotos++;
    } else {
      StatusHolder.CameraBurst = false;
    }
  }

  //Blinker for Testing
  if (millis() - ledLastTime >= 477) {
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(13, ledState);
    Serial.print("S Running: ");
    Serial.println(millis() - ledLastTime);
    ledLastTime = millis();
  }

  //Reset Master if No Communication for 5 min
  //  if (TestReset && (millis() - lastMasterCom > MasterFaultTime)) {
  //    digitalWrite(MasterReset, LOW);
  //    resets++;
  //    delay(100);
  //    digitalWrite(MasterReset, HIGH);
  //    lastMasterCom = millis(); //
  //  }

}


////////////////////////////////////////////////////////////////////////////////


