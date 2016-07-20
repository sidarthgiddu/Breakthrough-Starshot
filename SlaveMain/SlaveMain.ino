#include <Wire.h>
#include <MatrixMathDOUBLE.h>

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
    float gyro[3];
    float mag[3];

    slaveStatus(float t = 0, int L = 0, int r = 0, int n = 0, int XD = 0, int XP = 0,
                int YD = 0, int YP = 0, int ZD = 0, int ZP = 0,
                floatTuple g = floatTuple(0, 0, 0), floatTuple M = floatTuple(0, 0, 0)) {
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
    }
    String toString() {
      String res = "";
      //
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
////======================================== ADCS DATA ===================================================================
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
  
  Matrix.Print((double*)Bvalues, 3, 1, "Magn");
  Matrix.Print((double*)gyroData, 3, 1, "Gyro");
  
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
  Serial.println ("Step 1 complete");
  Matrix.Multiply((double*)Jtranspose, (double*)Jnew, 3, 4, 3, (double*)Jp); //transpose(Jnew)*Jnew=Anew
  Serial.println ("Step 2 complete");
  Matrix.Invert((double*)Jp, 3); // inverse(transpose(Jnew)*Jnew)=Bnew
  Serial.println ("Step 3 complete");
  Matrix.Multiply((double*)Jp, (double*)Jtranspose, 3, 3, 4, (double*)Jppinv); // Bnew*transpose(Jnew)=Cnew
  Serial.println ("Step 4 complete");
  
  // redefine Jp as Jpseudo-inverse
  Jp[0][0] = Jppinv[0][0]; Jp[0][1] = Jppinv[0][1]; Jp[0][2] = Jppinv[0][2]; // rescaled up
  Jp[1][0] = Jppinv[1][0]; Jp[1][1] = Jppinv[1][1]; Jp[1][2] = Jppinv[1][2];
  Jp[2][0] = Jppinv[2][0]; Jp[2][1] = Jppinv[2][1]; Jp[2][2] = Jppinv[2][2];
  Matrix.Scale((double*)Jp,3,3,1000000.0);
  //////////////Serial.println(freeRam ());
  double OmegaError[3], BfieldError[3], ErrorSum[3], current[3];
  double Omegacmd[3] = {0, 0, 1};
  double Bmagnitude = sqrt(Bvalues[0]*Bvalues[0]+Bvalues[1]*Bvalues[1]+Bvalues[2]*Bvalues[2]);
  double Bcmd[3] = {0, 0, 1};
  double A = 0.532;
  
  Matrix.Scale((double*)Bfield, 3, 1, 1/Bmagnitude);
  
  Matrix.Subtract((double*) Bfield, (double*) Bcmd, 3, 1, (double*) BfieldError);
  Serial.println ("Step 5 complete");
  Matrix.Subtract((double*) gyroData, (double*) Omegacmd, 3, 1, (double*) OmegaError);
  Serial.println ("Step 6 complete");
  //////////////Serial.println(freeRam ());
  
  Matrix.Scale((double*)BfieldError, 3, 1, (Kp/A)); // scale error with proportional gain (updates array)
  
  Serial.println ("Step 7 complete");
  Matrix.Scale((double*)OmegaError, 3, 1, (Kd/A)); // scale error with derivative gain (updates array)
  Serial.println ("Step 8 complete");

  Matrix.Add((double*)BfieldError, (double*)OmegaError, 3, 1, (double*) ErrorSum);
  Serial.println ("Step 9 complete"); delay(50);
  Matrix.Scale((double*) ErrorSum, 3, 1, -1.0); // prep error for multiplication with the Jpinv matrix
  Serial.println ("Step 10 complete"); delay(50);

  Matrix.Print((double*)ErrorSum, 3, 1, "MATRIX TO CHECK");
  Matrix.Print((double*)Jp, 3, 3, "Jpinv");
  
  Serial.println(freeRam ()); delay(50);
  Matrix.Multiply((double*) Jp, (double*) ErrorSum, 3, 3, 1, (double*) current);
  Serial.println ("Step 11 complete");
  
  //////////////Serial.println(freeRam ());
  //delete (Jp); delete  (Jtranspose); delete  (Jppinv);
  //delete (gyroData); delete (Bvalues); delete (Jnew);
  //delete (OmegaError); delete (BfieldError); delete (Omegacmd); delete (Bcmd);
  Serial.println("Calling outputPWM"); delay(50);
  Matrix.Print((double*)current, 3, 1, "Current");
  outputPWM((double*) current, 3);
  Serial.println("outputPWM ran"); delay(50);
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
      Serial.println("saturated");
      I[i] = Imax * sgn(I[i]);
    }
    Serial.print(I[i]); Serial.println(" ");
  }

  // CREATE PWM OUT SIGNAL
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
double mData[3] = {28.87,28.87,28.87}; //in 1000s of nT (or *10^3 nT)
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
        case (11):
          StatusHolder.gyro[0] = currentCommand[1];
          break;
        case (12):
          StatusHolder.gyro[1] = currentCommand[1];
          break;
        case (13):
          StatusHolder.gyro[2] = currentCommand[1];
          break;
        case (21):
          StatusHolder.mag[0] = currentCommand[1];
          break;
        case (22):
          StatusHolder.mag[1] = currentCommand[1];
          break;
        case (23):
          StatusHolder.mag[2] = currentCommand[1];
          break;
        case (91):
          StatusHolder.ADCS_Active = currentCommand[1];
          break;

      }
    } else {
      Serial.println("No Command");
    }
  }
}

void commandParser(int nBytes) {
  //Need nBytes?
  String command = "";
  while (1 <= Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    command += c;        // print the character
  }
  //Parse Command
  if (isInputValid(command)) {
    Serial.println("Command is Valid");
    buildBuffer(command);
    Serial.println("Built Command Buffer Successfully");
    PopCommands();

  } else {
    Serial.println("Invalid Command");
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void requestEvent() {
  //Serial.println("Data Request");
  String r = StatusHolder.toString();
  char response[r.length()];
  r.toCharArray(response, r.length());
  Wire.write(response);
  lastMasterCom = millis();
}

void initalizePinOut() {
  const int MasterReset = A4; pinMode(MasterReset, OUTPUT); //Reset Master Core
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

void forcedStall() {
  digitalWrite(8, HIGH);
  int test[5] = {4};
  int x = test[78];
  Serial.println(x);
  Serial.println("Didnt crash1");
  int y = 12.0 / 0.0;
  Serial.println(y);
  Serial.println("Didnt crash2");
  int z;
  int crash = z + 4;
  Serial.println(z);
  Serial.println("Didnt crash3");
  while (1);
  digitalWrite(8, LOW);
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  initalizePinOut();
  slaveStatus StatusHolder = slaveStatus();

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
}

int ledState = HIGH;
long ledLastTime = 0;
long lastADCSTime = 0;

void loop() {

  StatusHolder.updatePassive(); StatusHolder.ADCS_Active = true;
 //Test ADCS
  if (StatusHolder.ADCS_Active && millis() - lastADCSTime >= 2000) {
    if (millis() - lastADCSTime >= 2100) {
      Serial.println("happy");
      runADCS(mData, gData, Kp, Kd); //placeholders
      Serial.println("still happy");
      //Serial.print("X axis: "); Serial.print(StatusHolder.CurXDir, 20); Serial.print(" "); Serial.println(StatusHolder.CurXPWM, 20);
      //Serial.print("Y axis: "); Serial.print(StatusHolder.CurYDir, 20); Serial.print(" "); Serial.println(StatusHolder.CurYPWM, 20);
      //Serial.print("Z axis: "); Serial.print(StatusHolder.CurZDir, 20); Serial.print(" "); Serial.println(StatusHolder.CurZPWM, 20);
      lastADCSTime = millis();
    } else {
      //torquers off
      //analogWrite(CX_PWM, 0);
      //analogWrite(CY_PWM, 0);
      //analogWrite(CZ_PWM, 0);
      //floatTuple PWMvaluesForTorquers = floatTuple(0,0,0;
      //floatTuple PWMdirectionsForTorquers = floatTuple(0,0,0);
      //StatusHolder.updateTorquers(PWMdirectionsForTorquers, PWMvaluesForTorquers);
    }
  }

  //Reset Master if No Communication for 5 min
  //  if (TestReset && (millis() - lastMasterCom > MasterFaultTime)) {
  //    digitalWrite(MasterReset, LOW);
  //    resets++;
  //    delay(100);
  //    digitalWrite(MasterReset, HIGH);
  //    lastMasterCom = millis(); //
  //  }

  //Blinker for Testing
  if (millis() - ledLastTime >= 477) {
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(13, ledState);
    //Serial.print("S Running: ");
    //Serial.println(millis() - ledLastTime);
    ledLastTime = millis();
  }

}


////////////////////////////////////////////////////////////////////////////////


