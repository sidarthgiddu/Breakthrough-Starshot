#include <Wire.h>
#include "BigNumber.h"
#include <MatrixMath.h>

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

    slaveStatus(float t, int L, int r, int n, int XD, int XP,
                int YD, int YP, int ZD, int ZP, floatTuple g, floatTuple M) {
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
      res += "{MRsts:" + String(resets);
      res += ",T:" + String(Temp);
      res += ",L:" + String(Light);
      res += ",XD:" + String(CurXDir);
      res += ",YD:" + String(CurYDir);
      res += ",ZD:" + String(CurZDir);
      res += ",XP:" + String(CurXPWM);
      res += ",YP:" + String(CurYPWM);
      res += ",ZP:" + String(CurZPWM);
      res += ",nP:" + String(numPhotos);
      res += "GX:" + String(gyro[0]) + ",GY:" + String(gyro[1]) + ",GZ:" + String(gyro[2]) + ",";
      res += "MX:" + String(mag[0]) + ",MY:" + String(mag[1]) + ",MZ:" + String(mag[2]) + ",";
      res += "}||||";
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
slaveStatus StatusHolder = slaveStatus(0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                       floatTuple(0, 0, 0), floatTuple(0, 0, 0));

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

BigNumber mass = 1.33;
BigNumber zero=0.0, six=6.0;
BigNumber Bfield[3];
BigNumber w[3];
BigNumber Inertia[3][3] = {{mass / six, zero, zero},
  {zero, mass / six, zero},
  {zero, zero, mass / six}
}; // Inertia initialization
BigNumber E = 1e-4;

void runADCS(float* Magfield, float* omega, BigNumber Kp, BigNumber Kd) {
  BigNumber gyroData[3]={omega[0],omega[1],omega[2]};
  BigNumber Bvalues[3]={Magfield[0],Magfield[1],Magfield[2]};
  
  BigNumber J[9] = {0, Bvalues[2], -Bvalues[1], -Bvalues[2], 0, Bvalues[0], Bvalues[1], -Bvalues[0], 0};

  Matrix.Copy((BigNumber*)Bvalues, 1, 3, (BigNumber*)Bfield); // create new field to scale for the pseudo-inverse
  Matrix.Scale((BigNumber*)Bfield, 3, 1, E); // scale duplicated Bfield array with E for pseudo-inverse

  BigNumber Jnew[4][3] = {{J[0], J[1], J[2]},
    {J[3], J[4], J[5]},
    {J[6], J[7], J[8]},
    {Bfield[0]*E, Bfield[1]*E, Bfield[2]*E}
  };

  BigNumber Jtranspose[3][4];
  BigNumber Jproduct[3][3];
  BigNumber Jppinv[3][4];

  Matrix.Transpose((BigNumber*)Jnew, 4, 3, (BigNumber*)Jtranspose); // transpose(Jnew)
  Matrix.Multiply((BigNumber*)Jtranspose, (BigNumber*)Jnew, 3, 4, 3, (BigNumber*)Jproduct); //transpose(Jnew)*Jnew=Anew
  Matrix.Invert((BigNumber*)Jproduct, 3); // inverse(transpose(Jnew)*Jnew)=Bnew
  Matrix.Multiply((BigNumber*)Jproduct, (BigNumber*)Jtranspose, 3, 3, 4, (BigNumber*)Jppinv); // Bnew*transpose(Jnew)=Cnew

  BigNumber Jpinv[3][3] = {{Jppinv[0][0], Jppinv[0][1], Jppinv[0][2]},
    {Jppinv[1][0], Jppinv[1][1], Jppinv[1][2]},
    {Jppinv[2][0], Jppinv[2][1], Jppinv[2][2]}
  };

  BigNumber current[3][1];
  BigNumber OmegaError[3][1], BfieldError[3][1], ErrorSum[3][1];
  BigNumber Omegacmd[3][1] = {0, 0, 1};
  BigNumber Bcmd[3][1] = {0, 0, 1};
  BigNumber A = 0.532;

  Matrix.Subtract((BigNumber*) Bvalues, (BigNumber*) Bcmd, 3, 1, (BigNumber*) BfieldError);
  Matrix.Subtract((BigNumber*) gyroData, (BigNumber*) Omegacmd, 3, 1, (BigNumber*) OmegaError);

  Matrix.Scale((BigNumber*)BfieldError, 3, 1, Kp / A); // scale error with proportional gain (updates array)
  Matrix.Scale((BigNumber*)OmegaError, 3, 1, Kd / A); // scale error with derivative gain (updates array)

  Matrix.Add((BigNumber*)BfieldError, (BigNumber*)OmegaError, 3, 1, (BigNumber*) ErrorSum);
  Matrix.Scale((BigNumber*) ErrorSum, 3, 1, -1.0); // prep error for multiplication with the Jpinv matrix
  Matrix.Multiply((BigNumber*) Jpinv, (BigNumber*) ErrorSum, 3, 3, 1, (BigNumber*) current);

  Matrix.Print((BigNumber*) Jproduct, 3, 3, "check");

  outputPWM((BigNumber*) current, 3);
}

void outputPWM(BigNumber* I, int length) {
  float Imax = 2.0;
  String I1=I[0].toString();
  String I2=I[1].toString();
  String I3=I[2].toString();
  float If[3]={I1.toFloat(),I2.toFloat(),I3.toFloat()};
  free (&I[0]);
  free (&I[1]);
  free (&I[2]);

  for (int i = 0; i < length; i++) {
    if (abs(If[i]) > Imax) {
      I[i] = Imax * sgn(I[i]);
    }
  }

  // CREATE PWM OUT SIGNAL
  //analogWrite(CX_PWM, If[1] / Imaxf * 255);
  //analogWrite(CY_PWM, If[2] / Imaxf * 255);
  //analogWrite(CZ_PWM, If[3] / Imaxf * 255);

  floatTuple PWMvaluesForTorquers = floatTuple(If[1] / Imax * 255, If[2] / Imax * 255, If[3] / Imax * 255);
  floatTuple PWMdirectionsForTorquers = floatTuple(sgn(If[0]), sgn(If[1]), sgn(If[2]));
  StatusHolder.updateTorquers(PWMdirectionsForTorquers, PWMvaluesForTorquers);

}

static inline float sgn(float val) {
  if (val < 0.0) return -1.0;
  if (val == 0.0) return 0.0;
  return 1.0;
}
// Placeholder Test Data
float gData[3] = {0.2, 0.04, -0.1};
float mData[3] = {0.00002, 0.0004, -0.0009};
BigNumber Kp = 1e-3;
BigNumber Kd = 1e-3;

////////////////////////////////
////////////////////////////////
////////////////////////////////
////////////////////////////////

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

void loopPopCommand() {
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
      }
    } else {
      Serial.println("No Command");
    }
  }
}


////////////////////////////////
////////////////////////////////
////////////////////////////////
////////////////////////////////


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
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

    //popCommand

  } else {
    Serial.println("Invalid Command");
  }
}


void requestEvent() {
  Serial.println("Data Request");
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



  //while(1);
  digitalWrite(8, LOW);

}

void setup() {
  Serial.begin(9600);
  delay(1000);

  initalizePinOut();

  //digitalWrite(MasterReset, HIGH); //Enable Master
  Wire.begin(8);
  // join i2c bus with address #8
  Wire.onReceive(commandParser);
  Wire.onRequest(requestEvent);

  //int endT = millis() + manualTimeoutS;
  //while (!imu.begin() && millis() < endT);
  //if (!imu.begin()) {
  //  imuWorking = false;
  //}

  //Reset Indication
  pinMode(8, OUTPUT);
  for (int j = 0; j < 10; j++) {
    digitalWrite(8, HIGH);
    delay(140);
    digitalWrite(8, LOW);
    delay(140);
  }

  //Forced Stall
  pinMode(12, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(12), forcedStall, LOW);

}

int ledState = HIGH;
long ledLastTime = 0;
long lastADCSTime = 0;
void loop() {
  StatusHolder.updatePassive();
  //StatusHolder.updateTorquers(floatTuple TorquerOutput);

  //popCommand();

  //Test ADCS
  if (millis() - lastADCSTime >= 3000) {
    runADCS(mData, gData, Kp, Kd); //placeholders
    Serial.print("X axis: "); Serial.print(StatusHolder.CurXDir,20); Serial.print(" "); Serial.println(StatusHolder.CurXPWM,20);
    Serial.print("Y axis: "); Serial.print(StatusHolder.CurYDir,20); Serial.print(" "); Serial.println(StatusHolder.CurYPWM,20);
    Serial.print("Z axis: "); Serial.print(StatusHolder.CurZDir,20); Serial.print(" "); Serial.println(StatusHolder.CurZPWM,20);
    lastADCSTime = millis();
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


