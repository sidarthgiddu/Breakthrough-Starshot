#include <Wire.h>
#include <MatrixMathDOUBLE.h>
#include <Adafruit_VC0706.h>
#include <SPI.h>
#include <SD.h>

////Constant Initialization
int MasterFaultTime = 5 * 60 * 1000;
int lastMasterCom = 0;
uint8_t resets = 0;
int manualTimeoutS = 10 * 1000;
bool imuWorking = true;
uint8_t lightLevel = 0;
uint8_t tempBattery = 0;
int sensRecords = 0;
long lastSensAvg = 0;
int SensAvgTime = 100;

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
long lastCamCheckTime = 0;
int camCheckTime = 1000;

//SD Card
#define chipSelect 4
#define DLSize 320
bool SDActive = false;

Adafruit_VC0706 cam = Adafruit_VC0706(&Serial1);

int getTempDegrees() {
  // Returns the temperature of the sensor at pin senseTemp
  int temp = map(analogRead(TempS), 0, 543, -50, 125); //round?
  return temp;
}

int getLightLvl() { //TODO
  //Returns the lighting of the sensor at pin senselight (10k resistor)
  int light = map(analogRead(LightS), 0, 1023, 0, 100); //2.5 / 3.3 *1023
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
      if (i % DLSize == 0) {
        Serial.println("");
      }
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

class RAMImageSlave {
  public:
    uint8_t a[DLSize * 16];

    long photosize;
    String Filename;
    volatile int currentIndex;

    void printRI() {
      Serial.println("RAM Loaded Image: " + Filename);
      Serial.println("   Total Size: " + String(photosize));
      Serial.println("   Current Index: " + String(currentIndex));
      Serial.println("\nBinary Form:");
      printArray(a, photosize);
    }

    RAMImageSlave() {
      int photosize = 0;
      String Filename = "";
      int currentIndex = 0;
    }

    void store(uint8_t * data, int dataSize) {
      //Can't Handle images over DLSize*16 (~5kB);
      for (int i = 0; i < dataSize; i++) {
        a[i] = data[i];
      }
    }
};


class slaveStatus
{
  public:
    bool ADCS_Active;
    int Temp;
    int Light;
    int TempAcc;
    int LightAcc;
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
    RAMImageSlave imageR;
    volatile int ITStatus;
    bool SDActive;

    //Iridium Network Status Attributes

    slaveStatus(float t = 0, int L = 0, int r = 0, int n = 0, int XD = 0, int XP = 0,
                int YD = 0, int YP = 0, int ZD = 0, int ZP = 0,
                floatTuple g = floatTuple(0, 0, 0), floatTuple M = floatTuple(0, 0, 0),
                int IS = smallImage, long BD = 10 * 1000) {

      imageR = RAMImageSlave();
      ITStatus = 0;

      ADCS_Active = true; //false;
      SDActive = false;
      Temp = t;
      TempAcc = 0;
      Light = L;
      LightAcc = 0;
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
      burstDuration = BD; //Default = 10s
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
      String r1 = String(resets) + "," + String(Temp) + "," + String(Light) + "," +
                  String(CurXDir) + "," + String(CurYDir) + "," + String(CurZDir) + "," +
                  String(CurXPWM) + "," + String(CurYPWM) + "," + String(CurZPWM) + "," +
                  String(numPhotos) + "," + String(CameraStatus) + "," + "|,,,";
      char r2[r1.length()];
      r1.toCharArray(r2, r1.length());
      Wire.write(r2, r1.length());
    }
    void print() {
      Serial.println(toString());
    }
    void updatePassive() {
      //Update Internal Information
      TempAcc += getTempDegrees();
      LightAcc += getLightLvl();
      sensRecords++;
      if (millis() - lastSensAvg > SensAvgTime) {
        Temp = TempAcc / sensRecords; TempAcc = 0;
        Light = LightAcc / sensRecords; LightAcc = 0;
        sensRecords = 0;
      }
    }
    void updateTorquers(floatTuple Dir, floatTuple PWM) {
      CurXDir = Dir.x;
      CurXPWM = PWM.x;
      CurYDir = Dir.y;
      CurYPWM = PWM.y;
      CurZDir = Dir.z;
      CurZPWM = PWM.z;
      /// AnalogWrite ....
      //Serial.print("Direction: ");
      //Dir.print();
      //Serial.print("PWM Values: ");
      //PWM.print();
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
  double OmegaError[3], BfieldError[3], ErrorSum[3], current[3], OmegaHat[3];
  double Omegacmd[3] = {0, 0, 1};
  double Bmagnitude = sqrt(Bvalues[0] * Bvalues[0] + Bvalues[1] * Bvalues[1] + Bvalues[2] * Bvalues[2]);
  double OmegaMagnitude = sqrt(gyroData[0] * gyroData[0] + gyroData[1] * gyroData[1] + gyroData[2] * gyroData[2]);
  double Bcmd[3] = {0, 0, 1};
  double A = 0.532;

  Matrix.Copy((double*)gyroData, 3, 1, (double*)OmegaHat);
  Matrix.Scale((double*)OmegaHat, 3, 1, 1 / OmegaMagnitude);
  Matrix.Scale((double*)Bfield, 3, 1, 1 / Bmagnitude);

  Matrix.Subtract((double*) gyroData, (double*) Omegacmd, 3, 1, (double*) OmegaError);
  //Serial.println ("Step 6 complete");
  //////////////Serial.println(freeRam ());

  /////// BfieldError NEW VERSION
  //Matrix.Print((double*)Bfield, 3, 1, "Normalized Bfield");
  double upperbnd = 1.2349, lowerbnd = 0.8098;
  double theta = sqrt((Bfield[0] - OmegaHat[0]) * (Bfield[0] - OmegaHat[0])
                      + (Bfield[1] - OmegaHat[1]) * (Bfield[1] - OmegaHat[1]));
  //Serial.println ("theta is: "+String(theta));
  double ex = (Bfield[0] - OmegaHat[0]) / theta;
  double ey = (Bfield[1] - OmegaHat[1]) / theta;
  theta = ey / ex;

  // test //////// RBF!!!!!!!!!!!! ======================================== ////////////////////////
  theta = 1; // <==================   REMOVE THIS BEFORE FLIGHT //TODO
  //Serial.println ("theta is: "+ String(theta));

  if ((theta <= upperbnd) && (theta >= lowerbnd)) {
    BfieldError[0] = Bfield[1] * OmegaHat[2] - Bfield[2] * OmegaHat[1];
    BfieldError[1] = Bfield[2] * OmegaHat[0] - Bfield[0] * OmegaHat[2];
    BfieldError[2] = Bfield[0] * OmegaHat[1] - Bfield[1] * OmegaHat[0];
    //Matrix.Print((double*)BfieldError, 3, 1, "Updated BfieldError");
    Matrix.Scale((double*)BfieldError, 3, 1, (Kp / A));
    // Matrix.Print((double*)BfieldError, 3, 1, "Scaled BfieldError");
  } else {
    BfieldError[0] = 0;
    BfieldError[1] = 0;
    BfieldError[2] = 0;
  }
  /////// END of BfieldError NEW VERSION

  /////// BfieldError OLD VERSION
  /*
     Matrix.Subtract((double*) Bfield, (double*) Bcmd, 3, 1, (double*) BfieldError);
    //Serial.println ("Step 5 complete");
    Matrix.Scale((double*)BfieldError, 3, 1, (Kp / A)); // scale error with proportional gain (updates array)
  */
  /////// END of BfieldError OLD VERSION

  //Serial.println ("Step 7 complete");

  Matrix.Add((double*)BfieldError, (double*)OmegaError, 3, 1, (double*) ErrorSum);
  //Serial.println ("Step 9 complete"); delay(50);

  Matrix.Scale((double*)OmegaError, 3, 1, (Kd / A)); // scale error with derivative gain (updates array)
  //Serial.println ("Step 8 complete");

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

void setNumPhotos() {
  if (SDActive) {
    StatusHolder.numPhotos = 0;
    for (int i = 0; i < 10000; i++) {
      char filename[9];
      strcpy(filename, "A0000.JPG");
      filename[1] = '0' + i / 1000;
      filename[2] = '0' + i % 1000 / 100;
      filename[3] = '0' + i % 1000 % 100 / 10;
      filename[4] = '0' + i % 1000 % 100 % 10;
      if (SD.exists(filename)) {
        StatusHolder.numPhotos++;
      } else {
        break;
      }
    }
  }
}

void clearSDCard(int i) {
  StatusHolder.imageR = RAMImageSlave();
  int notExists = 0;
  for (i; i < 10000; i++) {
    char filename[9];
    strcpy(filename, "A0000.JPG");
    filename[1] = '0' + i / 1000;
    filename[2] = '0' + i % 1000 / 100;
    filename[3] = '0' + i % 1000 % 100 / 10;
    filename[4] = '0' + i % 1000 % 100 % 10;
    if (SD.exists(filename)) {
      Serial.print("Removed: ");
      Serial.println(filename);
      SD.remove(filename);
      notExists = 0;
    } else {
      notExists++;
    }
    if (notExists > 3) {
      return;
    }
  }
  return;
}

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
      //Serial.println("Finished Adding Commands");
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
    Serial.println("Empty");
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
          lastPunc = 1; //,
        } else {
          //Serial.println("2 Commas");
          valid = false;
          break;
        }
      } else if (currentChar == ('!')) {
        //Serial.println("Exclamation Found");
        if (lastPunc == 1) {
          //Serial.println("Exclamation ok");
          lastPunc = 2; //!
        } else {
          //Serial.println("2 Periods or No prior comma");
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
  Serial.print ("Executing Commands: ");
  while (cBuf.openSpot > 0) { //Manual Timeout
    if (cBuf.openSpot > 0) {
      int currentCommand[2] = {cBuf.commandStack[cBuf.openSpot - 1][0],
                               cBuf.commandStack[cBuf.openSpot - 1][1]
                              };
      Serial.print(currentCommand[0]);
      Serial.print(":");
      Serial.print(currentCommand[1]);
      Serial.print("  ");
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
          Serial.print("Burst Start: ");
          Serial.println(String(StatusHolder.burstDuration / 1000.0) + "s");
          break;
        case (102): //Set PhotBurst Duration
          Serial.println("Burst Duration Set");
          StatusHolder.burstDuration = currentCommand[1] * 1000;
          break;
        case (103): //Request Photo #<currentCommand[1]>
          char filename[9];
          strcpy(filename, "A0000.JPG");
          filename[1] = '0' + currentCommand[1] / 1000;
          filename[2] = '0' + currentCommand[1] % 1000 / 100;
          filename[3] = '0' + currentCommand[1] % 1000 % 100 / 10;
          filename[4] = '0' + currentCommand[1] % 1000 % 100 % 10;
          StatusHolder.imageR = RAMImageSlave();
          if (SD.exists(filename)) {
            buildImageBuffer(filename);
          } else {
            StatusHolder.imageR.Filename = "N\\A";
            StatusHolder.imageR.photosize = 0;
            StatusHolder.imageR.currentIndex = 0;
          }
          Serial.println("\nImage");
          StatusHolder.imageR.printRI();
          StatusHolder.ITStatus = 1; //Stage Image
          break;
        case (104): //Image Successfully Recieved
          StatusHolder.ITStatus = 0; //Transmit Success
          break;
        case (105): //Image Retrival
          StatusHolder.ITStatus = 2; //Send Image
          break;
        case (106): {
            File root = SD.open("/");
            root.rewindDirectory();
            Serial.println("");
            Serial.println("Files: ");
            printDirectory(root, 0);
            root.rewindDirectory();
            //root.close();
            break;
          }
        case (107): {
            Serial.println("Wiping SD Card");
            clearSDCard(0);
            File root = SD.open("/");
            root.rewindDirectory();
            root.close();
            StatusHolder.numPhotos = 0;
            break;
          }
        case (108):
          StatusHolder.ITStatus = 0;
          break;
        case (109):
          setNumPhotos();
          Serial.println("Photos Stored: " + String(StatusHolder.numPhotos));
          break;
      }
    } else {
      Serial.println(F("No Command"));
    }
  }
  Serial.println(F("Success"));
}

void commandParser(int nBytes) {
  if (nBytes <= 1) {
    Serial.println("PCALL");
    return;
  }
  digitalWrite(8, HIGH);
  String command = "";
  while (Wire.available()) {
    command += (char)Wire.read();
  }
  //Serial.println("Com: " + command);
  //Parse Command
  Serial.print("\nCommand: <" + command);
  Serial.println("> = " + String(nBytes) + "");
  if (isInputValid(command)) {
    Serial.println(F("Commands are Valid"));
    buildBuffer(command);
    //Serial.println(F("Built Command Buffer Successfully"));
    PopCommands();
  } else {
    Serial.println(F("Invalid Commands"));
  }
  digitalWrite(8, LOW);
  //Serial.println(F("Here"));
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool test = true;
volatile int totalBytesSent = 0;
void requestEvent() {
  //Serial.println("Data Request");
  digitalWrite(8, HIGH);
  switch (StatusHolder.ITStatus) { //Image Request
    case (1): {
        if (SDActive) {
          Serial.println("Image Size Request");
          String s = String(StatusHolder.imageR.photosize);
          for (int i = 0; i < s.length(); i++) {
            Wire.write(s.charAt(i));
          }
          if (StatusHolder.imageR.photosize == 0) {
            StatusHolder.ITStatus = 0;
          }
        } else {
          //Card Not Working
          Serial.println("SD Card Fail");
          Wire.write('7');
        }
        break;
      }
    case (2): {
        if (SDActive) {
          Serial.println("Image Data Request");
          int bytesSent = 0;
          //uint8_t buf1[16];
          int i;
          for (i = 0; i < 32; i++) {
            bytesSent += Wire.write(StatusHolder.imageR.a[StatusHolder.imageR.currentIndex]);
            Serial.println(StatusHolder.imageR.a[StatusHolder.imageR.currentIndex]);
            StatusHolder.imageR.currentIndex++;
            if (StatusHolder.imageR.currentIndex >= StatusHolder.imageR.photosize) {
              break;
            }
          }

          Serial.println("bytesSent: " + String(bytesSent));
          totalBytesSent += bytesSent;
          if (StatusHolder.imageR.currentIndex >= StatusHolder.imageR.photosize) {
            Wire.flush();
            //      Wire.end();
            //      Wire.begin(11);
            Serial.println("Finished Data, Sent: " + String(totalBytesSent));
            StatusHolder.ITStatus = 0;
            StatusHolder.imageR.currentIndex = 0;
            totalBytesSent = 0;
            test = false;
            return;
          }

          //      Serial.println("Image Data Request");
          //      for (int i = 0; i < StatusHolder.imageR.photosize; i++) {
          //        totalBytesSent += Wire.write(StatusHolder.imageR.a[i]);
          //        Serial.println(StatusHolder.imageR.a[i]);
          //        Serial.println(StatusHolder.imageR.photosize-i);
          //      }
          //      Serial.println("Finished Data, Sent: " + String(totalBytesSent));
          //      StatusHolder.ImageRequested = false;
          //      StatusHolder.imageR.currentIndex = 0;
          //      totalBytesSent = 0;
          //      test = false;
          //      return;

          break;
        } else {
          Serial.println("No SD Card, Shouldn't Be here");
        }
      }
    case (0): { //General Communication
        if (!StatusHolder.CameraBurst) {
          setNumPhotos();
        }
        StatusHolder.WriteStatus(); //Check Format
        break;
      }
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
    //Serial.println("TCommand: " + comString);
    if (isInputValid(comString)) {
      Serial.print("Testing Command is Valid. ");
      buildBuffer(comString);
      PopCommands();
    } else {
      Serial.println("Invalid Testing Command");
    }
  }
}

bool camStatus = false;
String takePic(int ImageSize) {
  // Try to locate the camera
  if (!camStatus) {
    Serial.println("Camera Fail");
    return "";
  }
  //cam.begin(); //Need to ensure it worked
  cam.setImageSize(ImageSize);//cam.setImageSize(VC0706_160x120);

  if (!cam.takePicture()) {
    Serial.println("Take Failure");
    return "";
  } else {
    StatusHolder.numPhotos++;
    Serial.print("Picture taken! ");
  }

  // Create an image with the name Axxxx.JPG //ALTER THIS TO STORE THE NEXT AVAILABLE FILENAME
  char filename[9];
  strcpy(filename, "A0000.JPG");
  for (int i = 0; i < 9999; i++) {
    filename[1] = '0' + i / 1000;
    filename[2] = '0' + i % 1000 / 100;
    filename[3] = '0' + i % 1000 % 100 / 10;
    filename[4] = '0' + i % 1000 % 100 % 10;
    if (!SD.exists(filename)) {
      Serial.print("File created: ");
      Serial.println(filename);
      break;
    }
  }
  File imgFile = SD.open(filename, FILE_WRITE); //Open Image File
  uint16_t jpglen = cam.frameLength(); //Image Size in Bytes
  uint16_t bytesWritten = 0;

  //int32_t time = millis();
  pinMode(8, OUTPUT);
  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  long startT = millis();
  while (jpglen > 0 && 1) { //(millis() - startT <= 3000)) {
    uint8_t *databuffer;
    uint8_t bytesToRead = min(64, jpglen); // 64 Byte Reads
    databuffer = cam.readPicture(bytesToRead);
    bytesWritten += imgFile.write(databuffer, bytesToRead); //Returns bytes written if needed
    if (++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
      //Reset Timer Here
      wCount = 0;
    }
    jpglen -= bytesToRead;
    //Serial.println(jpglen);
  }
  //  if (millis() - startT <= 3000) {
  //    Serial.println("Photo Write Timeout");
  //  }
  imgFile.close();
  //time = millis() - time;
  camStatus = false;
  return filename;
}


int getPhotosStored() {
  int i = 0;
  char filename[9];
  strcpy(filename, "A0000.JPG");
  for (i; i < 9999; i++) {
    filename[1] = '0' + StatusHolder.numPhotos / 1000;
    filename[2] = '0' + StatusHolder.numPhotos % 1000 / 100;
    filename[3] = '0' + StatusHolder.numPhotos % 1000 % 100 / 10;
    filename[4] = '0' + StatusHolder.numPhotos % 1000 % 100 % 10;
    if (!SD.exists(filename)) {
      break;
    }
  }
  return i;
}

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void buildImageBuffer(String Filename) { //Rename to masterstatusholderimagebuffer in future...
  if (SDActive) {
    if (!SD.exists(Filename)) { //File does not exist
      Serial.println("Image Does Not Exist");
      StatusHolder.ITStatus = 0;
      StatusHolder.imageR.Filename = "Image Didn't Exist";
      StatusHolder.imageR.photosize = 0;
      StatusHolder.imageR.currentIndex = 0;
      return;
    }

    File IMGFile = SD.open(Filename, FILE_READ);
    StatusHolder.imageR.photosize = IMGFile.size();
    int index = 0;
    int i = 0;
    Serial.println("Starting Segmentation");

    while (IMGFile.available()) {
      //Use Single Array
      StatusHolder.imageR.a[i] = (uint8_t)IMGFile.read();
      i++;
    }

    IMGFile.close();
    Serial.println("\nDone");
    StatusHolder.imageR.Filename = Filename;
    StatusHolder.imageR.currentIndex = 0;
  } else {
    Serial.println("No Card, Image Not Loaded");
  }
  return;
}


int ledState = HIGH;
long ledLastTime = 0;
long lastADCSTime = 0;
File root;
void setup() {
  Serial.begin(9600);
  //while (!Serial);

  delay(100);
  Serial.println("Starting SD");
  SDActive = true;
  if (!SD.begin(4))
  {
    Serial.println("SD Failed");
    SDActive = false;
  }

  //  int trys = 0;
  //  while (true) {
  //    if (SD.begin(chipSelect)) {
  //      Serial.println("\nSD Card Active");
  //      SDActive = true;
  //      break;
  //    } //else if (trys > 150) {
  //    //      Serial.println("\nCard failed, or not present");
  //    //      //SDActive = false;
  //    //      break;
  //    //    }
  //    Serial.print("!");
  //    if (trys % 50 == 0) {
  //      Serial.println();
  //    }
  //    delay(10);
  //    trys++;
  //  }

  initalizePinOut();
  slaveStatus StatusHolder = slaveStatus();

  StatusHolder.ADCS_Active = true;

  //digitalWrite(MasterReset, HIGH); //Enable Master
  Wire.begin(11);
  Wire.onReceive(commandParser);
  Wire.onRequest(requestEvent);

  //Reset Indication
  pinMode(8, OUTPUT);
  if (SDActive) {
    for (int j = 0; j < 10; j++) {
      digitalWrite(8, HIGH);
      delay(50);
      digitalWrite(8, LOW);
      delay(50);
    }
  }

  setNumPhotos();
  Serial.println("Photos Stored: " + String(StatusHolder.numPhotos));
  Serial.println(SDActive);
}

bool SoftwarePWM = false;
bool disableT = false;
void loop() {
  readSerialAdd2Buffer();
  //Serial.println(SDActive);
  StatusHolder.updatePassive();
  //StatusHolder.ADCS_Active = false;
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
  } else {
    //ADCS Off
    floatTuple PWMvaluesForTorquers = floatTuple(0, 0, 0);
    floatTuple PWMdirectionsForTorquers = floatTuple(0, 0, 0);
    StatusHolder.updateTorquers(PWMdirectionsForTorquers, PWMvaluesForTorquers);
  }

  if (StatusHolder.CameraBurst) {
    //if (cam.begin()) {
    //Serial.print("Cam Ok");
    if (millis() <= StatusHolder.burstStart + StatusHolder.burstDuration) {
      digitalWrite(8, HIGH);
      String n = takePic(StatusHolder.imageSize);
      digitalWrite(8, LOW);
    } else {
      Serial.println("Burst Ended");
      StatusHolder.CameraBurst = false;
    }
    // } else {
    // Serial.println("Fail");
    //}
  }

  if (millis() - lastCamCheckTime >= camCheckTime) {
    camStatus = cam.begin();
    Serial.print("<C" + String(camStatus) + ">");
    lastCamCheckTime = millis();
  }

  //Blinker for Testing
  if (millis() - ledLastTime >= 100) {
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    //    if (!SDActive) {
    //      SD.begin();
    //      //Serial.println("Retry SD: "+String(SD.begin()));
    //    }
    digitalWrite(13, ledState);
    //StatusHolder.imageR.printRI();
    Serial.print(".");
    //    Serial.print("S Running: ");
    //    Serial.print(millis() - ledLastTime);
    //    Serial.println(": " + String(millis() / 1000));
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


