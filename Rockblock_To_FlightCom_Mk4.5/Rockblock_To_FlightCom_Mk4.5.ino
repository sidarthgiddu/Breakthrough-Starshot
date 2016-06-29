]]]]]class commandBuffer {
  public:
    int commandStack[100][2];
    int openSpot;

    commandBuffer() {
      //int cB[1000][2] = {0};
      //commandStack = cB;
      openSpot = 0;
    }
    void print() {
      int i = 0;
      Serial.print("cBuf = [");
      while (i < 100) {
        if (commandStack[i][0] == 0 && commandStack[i][1] == 0) {
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

commandBuffer cBuf = commandBuffer();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("T-10");
  pinMode (13, OUTPUT);
}

String command;

int waitTime = 500;
int lastTime = 0;
int popTime = 50000;
int popLast = 0;


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
      Serial.println("Finished Adding88888 Commands");
    } else {
      com = com.substring(com.indexOf(".") + 1);
    }
    cBuf.openSpot++;
  }
}


// boolean true false command valid
boolean isInputValid(String input) {
  int lastPunc = 0; //1 if ",", 2 if ".", 0 Otherwise
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
    }
    if (isAlpha(currentChar)) {
      //Serial.println("Alpha");
      valid = false;
      break;
    }
    if (currentChar == ('0')) {
      //Serial.println("Zero");
      valid = false;
      break;
    }
    if (isSpace(currentChar)) {
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
    if (currentChar == '\0' && q != input.length()-1) {
      //Null Character in the middle
      valid = false;
      break;
    }
  }
  return valid;
}
void loop() {
  if (Serial.available() > 0) {
    Serial.println("Recieving Command");
    String comString = "";
    while (Serial.available() > 0) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      comString += inChar;
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
    }
    if (isInputValid(comString)) {
      Serial.println("Command is Valid");
      buildBuffer(comString);
      Serial.println("Recieved Command Successfully");
    } else {
      Serial.println("Invalid Command");
    }
  }

  if (millis() > waitTime * 5 + lastTime) {
    Serial.println("Waiting");
    cBuf.print();
    lastTime = millis();
  }
  if (millis() > popTime + popLast) {
    Serial.println("Pop");
    popCommand();
    popLast = millis();
  }
}

//1,2.3,4.5,6.

void popCommand() {
  Serial.println ("Executing Command:");
  if (cBuf.openSpot > 0) {
    Serial.println (cBuf.openSpot - 1);
    int currentCommand[2] = {cBuf.commandStack[cBuf.openSpot - 1][0], cBuf.commandStack[cBuf.openSpot - 1][1]};
    Serial.print(currentCommand[0]);
    Serial.print(":");
    Serial.println(currentCommand[1]);
    cBuf.commandStack[cBuf.openSpot - 1][0] = 0;
    cBuf.commandStack[cBuf.openSpot - 1][1] = 0;
    cBuf.openSpot --;
  } else {
    Serial.println("No Command");
  }
}


//psuedo commands led on led off  (input output)


//void serialEvent() { //Add Incoming Command to Buffer
//  Serial.println("Recieving Command");
//  String comString = "";
//  while (Serial.available()) {
//    // get the new byte:
//    char inChar = (char)Serial.read();
//    // add it to the inputString:
//    comString += inChar;
//    // if the incoming character is a newline, set a flag
//    // so the main loop can do something about it:
//  }
//  buildBuffer(comString);
//  Serial.println("Recieved Command Successfully");
//}
