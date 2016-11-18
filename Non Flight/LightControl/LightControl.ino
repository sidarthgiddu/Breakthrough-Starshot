#include <Adafruit_NeoPixel.h>

#define BarLength 20
#define BarsPerUnit 8
uint8_t layout[3] = {1, 1, 1};
#define NUM_COLUMNS 3

#define Strip0Pin 8
#define Strip1Pin 9
#define Strip2Pin 10

#define Strip0Len (BarLength*BarsPerUnit*1)
#define Strip1Len (BarLength*BarsPerUnit*1)
#define Strip2Len (BarLength*BarsPerUnit*1)

Adafruit_NeoPixel Strip0;
Adafruit_NeoPixel Strip1;
Adafruit_NeoPixel Strip2;

bool ActiveBars0[Strip0Len / BarLength] = {0};
bool ActiveBars1[Strip1Len / BarLength] = {0};
bool ActiveBars2[Strip2Len / BarLength] = {0};

uint8_t pulse[] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
  10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
  17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
  25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
  37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
  51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
  69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
  90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
  115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
  144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
  177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
  215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255
};

class intTuple
{
  public:
    uint8_t r;
    uint8_t g;
    uint8_t b;

    intTuple(uint8_t a, uint8_t b, uint8_t c) {
      r = a;
      g = b;
      b = c;
    }
    void print() {
      Serial.println("(");
      Serial.print(r); Serial.print(",");
      Serial.print(g); Serial.print(",");
      Serial.print(b); Serial.println(")");
    }
};

class commandBuffer {
    //Class which holds a stack of currently waiting commands which have not been processed
  public:
    int commandStack[200][2];
    int openSpot;
    commandBuffer() {
      commandStack[200][2] = { -1};
      openSpot = 0;
    }
    void print() {
      //Serial formatting and Serial output
      int i = 0;
      Serial.print(F("cBuf = ["));
      while (i < 200) {
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

Adafruit_NeoPixel * column2Strip(int column) {
  //Indexes Start at 0
  Adafruit_NeoPixel *p;
  switch (column) {
    case (0):
      *p = Strip0;
    case (1):
      *p = Strip1;
    case (2):
      *p = Strip2;
  }
  return p;
}

bool * column2ActiveBars(int column) {
  //Indexes Start at 0
  switch (column) {
    case (0):
      return ActiveBars0;
    case (1):
      return ActiveBars1;
    case (2):
      return ActiveBars2;
  }
}

bool pixelActive(int column, int index) {
  bool * active = column2ActiveBars(column);
  return active[index / BarLength]; //Rounds Down
}

void setBar(int column, int bar, intTuple color) {
  //Bar Indexes Start at 0
  colorRange(column, bar * BarLength, (bar + 1)*BarLength - 1, color);
}

void setUnit(int column, int unit, intTuple color) {
  //Unit Indexes start at 0
  colorRange(column, unit * BarLength * BarsPerUnit, (unit + 1)*BarLength * BarsPerUnit - 1, color);
}

void colorRange(int column, int start, int last, intTuple color) {
  //Colors start to last
  Adafruit_NeoPixel *p = column2Strip(column);
  for (int i = start; i <= last; i++) {
    p->setPixelColor(i, color.r, color.g, color.b);
  }
}

void setPixel(int column, int index, int bar, intTuple color) {
  //  bar*BarLength+(2*(bar % 2 == 0) - 1)*index
  Adafruit_NeoPixel *p = column2Strip(column);
  p->setPixelColor(bar * BarLength + (2 * (bar % 2 == 0) - 1)*index, color.r, color.g, color.b);
}

void setAll(intTuple color) {
  for (int i = 0; i < NUM_COLUMNS; i++) {
    Adafruit_NeoPixel *p = column2Strip(i);
    uint16_t n = p->numPixels();
    for (int j = 0; j < n; j++) {
      p->setPixelColor(j, color.r, color.g, color.b);
    }
  }
}

void showAll() {
  for (int i = 0; i < NUM_COLUMNS; i++) {
    Adafruit_NeoPixel *p = column2Strip(i);
    p->show();
  }
}

boolean isInputValid(String input) {
  //Check if incoming command string <input> is valid
  int lastPunc = 0; //1 if ",", 2 if "!", 0 Otherwise
  bool valid = true;
  int q = 0;
  int l = input.length();
  while (q < l) {
    char currentChar = input[q];
    q++;

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

void buildBuffer(String com) {
  //Check if incoming String <com> is valid set of commands and add it to the CommandBuffer
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
      //Serial.println(F("Finished Adding Commands"));
    } else {
      com = com.substring(com.indexOf("!") + 1);
    }
    cBuf.openSpot++;
  }
}

void popCommands() {
  //Process all the Incoming Commands
  long start = millis();
  while (cBuf.openSpot > 0) {
    //Serial.println (cBuf.openSpot - 1);
    int currentCommand[2] = {cBuf.commandStack[cBuf.openSpot - 1][0], cBuf.commandStack[cBuf.openSpot - 1][1]};
    cBuf.commandStack[cBuf.openSpot - 1][0] = -1;
    cBuf.commandStack[cBuf.openSpot - 1][1] = -1;
    cBuf.openSpot --;

    //Supported Commands
    switch (currentCommand[0]) {
      case (1): { //Full White
          intTuple c = intTuple(255, 255, 255);
          setAll(c);
          showAll();
          break;
        }
      case (2): { //Full Red
          intTuple c = intTuple(255, 0, 0);
          setAll(c);
          showAll();
          break;
        }
      case (3): {//Full Green
          intTuple c = intTuple(0, 255, 0);
          setAll(c);
          showAll();
          break;
        }
      case (4): { //Full Blue
          intTuple c = intTuple(0, 0, 255);
          setAll(c);
          showAll();
          break;
        }
      case (5): //Full Specificed Color
        break;
      case (6): //Column Specified Color
        break;
      case (7): //Set Display Mode (Equalizer, Frozen, etc);
        //0 Frozen
        //1 Pulse at set color
        //2 Equalizer
        break;
    }
  }
}

void readSerialAdd2Buffer() {
  //Read Testing Commands from USB Serial
  if (Serial.available() > 0) {
    //Serial.println("Reading Testing Command");
    String comString = "";
    while (Serial.available() > 0) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      comString += inChar;
    }
    //Serial.println("TCommand: " + comString);
    if (isInputValid(comString)) {
      buildBuffer(comString);
      popCommands();

    } else {
      Serial.println("Invalid Testing Command");
    }
  }
}


volatile int MODE = 0;
void setup() {
  // put your setup code here, to run once:
  Strip0 = Adafruit_NeoPixel(Strip0Len, Strip0Pin);
  Strip1 = Adafruit_NeoPixel(Strip1Len, Strip1Pin);
  Strip2 = Adafruit_NeoPixel(Strip2Len, Strip2Pin);
}

int InternalState = 0;
unsigned long cycle = 0;
float EQ_BPS = 2;
void loop() {
  // put your main code here, to run repeatedly:
  switch (MODE) {
    case (0):
      //Frozen (Do Nothing)
      showAll();
      delay(100);
      break;
    case (1): 
        //Pulse
        for (int i = 0; i < NUM_COLUMNS; i++) {
          Adafruit_NeoPixel *p = column2Strip(i);
          int n = p->numPixels();
          int index = cycle % 512;
          if (index < 256) {
            for (uint16_t k = 0; k < n; i++) {
              if (pixelActive(i, k)) {
                p->setBrightness(pulse[index]);
              }
            }
          } else {
            for (int j = 255; j >= 0 ; j--) {
              for (uint16_t k = 0; k < n; i++) {
                if (pixelActive(i, k)) {
                  p->setBrightness(pulse[255 - index]);
                }
              }
            }
          }
          delay(50);
          p->show();
        }
        break;
      
  }
  cycle++;
}












