#include <Wire.h>
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
    bool ADCS_Active;
    int Temp;
    int Light;
    int mtX;
    int mtY;
    int mtZ;
    int Resets;
    int numPhotos;
    int CurXDir; //-1 or 1 for Coil Current Direction, 0 is off
    int CurXPWM; // 0 to 255 for Coil Current Level, 0 is off
    int CurYDir; //-1 or 1 for Coil Current Direction, 0 is off
    int CurYPWM; // 0 to 255 for Coil Current Level, 0 is off
    int CurZDir; //-1 or 1 for Coil Current Direction, 0 is off
    int CurZPWM; // 0 to 255 for Coil Current Level, 0 is off
    float gyro[3];
    float mag[3];
    float No_Torquer_gyro[3];
    float No_Torquer_mag[3];
    int format; //HEX or DEC

    slaveStatus(int formant, float t = 0, int L = 0, int r = 0, int n = 0, int XD = 0, int XP = 0,
                int YD = 0, int YP = 0, int ZD = 0, int ZP = 0, bool ADCS = false,
                floatTuple g = floatTuple(0, 0, 0), floatTuple M = floatTuple(0, 0, 0)) {
      //Maybe Zeros wont work for ADCS at the start
      ADCS_Active = ADCS;
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
      No_Torquer_gyro[0] = g.x; No_Torquer_gyro[1] = g.y; No_Torquer_gyro[2] = g.z;
      No_Torquer_mag[0] = M.x; No_Torquer_mag[1] = M.y; No_Torquer_mag[2] = M.z;

    }
    String toString() {
      String res = "";
      res += "{MRsts:" + String(resets);
      res += ",T:" + String(Temp, format);
      res += ",L:" + String(Light, format);
      res += ",XD:" + String(CurXDir, format);
      res += ",YD:" + String(CurYDir, format);
      res += ",ZD:" + String(CurZDir, format);
      res += ",XP:" + String(CurXPWM, format);
      res += ",YP:" + String(CurYPWM, format);
      res += ",ZP:" + String(CurZPWM, format);
      res += ",nP:" + String(numPhotos, format);
      res += "GX:" + String(gyro[0], format) + ",GY:" + String(gyro[1], format) + ",GZ:" + String(gyro[2], format) + ",";
      res += "MX:" + String(mag[0], format) + ",MY:" + String(mag[1], format) + ",MZ:" + String(mag[2], format) + ",";
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
slaveStatus StatusHolder = slaveStatus(DEC); //DEC for Decimal Output


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

float mass = 1.33;
float Bfield[3];
float w[3];
float Inertia[3][3] = {{mass / 6, 0, 0},
  {0, mass / 6, 0},
  {0, 0, mass / 6}
}; // Inertia initialization
float E = 1e-4;

void runADCS(float* Bvalues, float* gyroData, float Kp, float Kd) {
  float J[9] = {0, Bvalues[2], -Bvalues[1], -Bvalues[2], 0, Bvalues[0], Bvalues[1], -Bvalues[0], 0};

  Matrix.Copy((float*)Bvalues, 1, 3, (float*)Bfield); // create new field to scale for the pseudo-inverse
  Matrix.Scale((float*)Bfield, 3, 1, E); // scale duplicated Bfield array with E for pseudo-inverse

  float Jnew[4][3] = {{J[0], J[1], J[2]},
    {J[3], J[4], J[5]},
    {J[6], J[7], J[8]},
    {Bfield[0]*E, Bfield[1]*E, Bfield[2]*E}
  };

  float Jtranspose[3][4];
  float Jproduct[3][3];
  float Jppinv[3][4];

  Matrix.Transpose((float*)Jnew, 4, 3, (float*)Jtranspose); // transpose(Jnew)
  Matrix.Multiply((float*)Jtranspose, (float*)Jnew, 3, 4, 3, (float*)Jproduct); //transpose(Jnew)*Jnew=Anew
  Matrix.Invert((float*)Jproduct, 3); // inverse(transpose(Jnew)*Jnew)=Bnew
  Matrix.Multiply((float*)Jproduct, (float*)Jtranspose, 3, 3, 4, (float*)Jppinv); // Bnew*transpose(Jnew)=Cnew

  float Jpinv[3][3] = {{Jppinv[0][0], Jppinv[0][1], Jppinv[0][2]},
    {Jppinv[1][0], Jppinv[1][1], Jppinv[1][2]},
    {Jppinv[2][0], Jppinv[2][1], Jppinv[2][2]}
  };

  float current[3][1];
  float OmegaError[3][1], BfieldError[3][1], ErrorSum[3][1];
  float Omegacmd[3][1] = {0, 0, 1};
  float Bcmd[3][1] = {0, 0, 1};
  float A = 0.532;

  Matrix.Subtract((float*) Bvalues, (float*) Bcmd, 3, 1, (float*) BfieldError);
  Matrix.Subtract((float*) gyroData, (float*) Omegacmd, 3, 1, (float*) OmegaError);

  Matrix.Scale((float*)BfieldError, 3, 1, Kp / A); // scale error with proportional gain (updates array)
  Matrix.Scale((float*)OmegaError, 3, 1, Kd / A); // scale error with derivative gain (updates array)

  Matrix.Add((float*)BfieldError, (float*)OmegaError, 3, 1, (float*) ErrorSum);
  Matrix.Scale((float*) ErrorSum, 3, 1, -1.0); // prep error for multiplication with the Jpinv matrix
  Matrix.Multiply((float*) Jpinv, (float*) ErrorSum, 3, 3, 1, (float*) current);

  Matrix.Print((float*) Jproduct, 3, 3, "check");

  outputPWM((float*) current, 3);
}

void outputPWM(float* I, int length) {
  float Imax = 2.0;

  for (int i = 0; i < length; i++) {
    if (abs(I[i]) > Imax) {
      I[i] = Imax * sgn(I[i]);
    }
  }

  // CREATE PWM OUT SIGNAL
  //analogWrite(CX_PWM, I[1] / Imax * 255);
  //analogWrite(CY_PWM, I[2] / Imax * 255);
  //analogWrite(CZ_PWM, I[3] / Imax * 255);

  floatTuple PWMvaluesForTorquers = floatTuple(I[1] / Imax * 255, I[2] / Imax * 255, I[3] / Imax * 255);
  floatTuple PWMdirectionsForTorquers = floatTuple(sgn(I[0]), sgn(I[1]), sgn(I[2]));
  StatusHolder.updateTorquers(PWMdirectionsForTorquers, PWMvaluesForTorquers);


}

static inline float sgn(float val) {
  if (val < 0.0) return -1.0;
  if (val == 0.0) return 0.0;
  return 1.0;
}
// Placeholder Test Data
float gData[3] = {0.2, 0.04, -0.1};
float mData[3] = {0.00002 * 1000, 0.0004 * 1000, -0.0009 * 1000};
float Kp = 1e-3;
float Kd = 1e-3;

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
        case (51):
          StatusHolder.ADCS_Active = currentCommand[1];
          break;
      }
    } else {
      Serial.println("No Command");
    }
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void commandParser(int nBytes) {
  lastMasterCom = millis();
  String command = "";
  while (1 <= Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    command += c;        // print the character
  }
  //Parse Command
  if (isInputValid(command)) {
    //Serial.println("Command is Valid");
    buildBuffer(command);
    //Serial.println("Built Command Buffer Successfully");

    //popCommand

  } else {
    //Serial.println("Invalid Command");
  }
}


void requestEvent() {
  lastMasterCom = millis();
  Serial.println("Data Request");
  String r = StatusHolder.toString();
  char response[r.length()];
  r.toCharArray(response, r.length());
  Wire.write(response);
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

  digitalWrite(MasterReset, HIGH); //Enable Master
  Wire.begin(8);
  // join i2c bus with address #8
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
  pinMode(12, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(12), forcedStall, LOW);

}

int ledState = HIGH;
long ledLastTime = 0;
long lastADCSTime = 0;
int ADCS_CycleTime = 100; //Time to leave coils on before updating Inputs
int ADCS_CharactTime = 50; //Max Time to Zero Current in Coils w/0V Applied


void loop() {
  StatusHolder.updatePassive();

  //Test ADCS
  if (StatusHolder.ADCS_Active) {
    if  (millis() - lastADCSTime >= ADCS_CycleTime) {
      if (millis() - lastADCSTime >= ADCS_CycleTime + ADCS_CharactTime) {
        //runADCS(mData, gData, Kp, Kd); //placeholder
        runADCS(StatusHolder.mag, StatusHolder.gyro, Kp, Kd); //Activate Coils
        lastADCSTime = millis();
      } else {
        //disableTorquers(); //Let Sensors Get Valid Mag Data
      }
    }
  }

  //Reset Master if No Communication for 5 min //Maybe Not Feasible
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


