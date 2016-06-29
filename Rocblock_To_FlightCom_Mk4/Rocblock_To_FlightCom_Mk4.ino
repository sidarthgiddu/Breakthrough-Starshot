class commandBuffer {
    String commandStack[];
    int openSpot;

  public:
    command() {
      String cB[1000] = {""};
      commandStack = cB
      openSpot = 0;
    }
};

String comBuffer[1000] = {""};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode (13, OUTPUT);
}
String command;
void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available())
  {

    String comString = Serial.readString();
    Serial.println(comString);
    /*(  if(c == "\n")
      {
      parseCommand
      command = "";
      }
      else {
        command += c;
      }*/
    buildBuffer(comString, comBuffer);

    //Print Buffer to Serial
    //}
    Serial.println("Recieved Commands:");
    for (int i = 0; i < 100 ; i++) {
      //Serial.println("Looped");
      if (comBuffer[i] != "") {
        //Serial.println("Found Command");
        Serial.print("Command ");
        Serial.print(i + 1, DEC);
        Serial.print(": ");
        Serial.println(comBuffer[i]);

      } else {
        break;
      }

    }
    delay(10000);
  }
  delay(100);
  Serial.println("No Command");

}

void serialEvent()
while (Serial.available()) {
  
}




void buildBuffer(String com, String comBuffer[])
{
  String comData;
  String comIndex;
  String comRemaining = com;
  int i = 0;

  while (com != "") {
    comIndex = com.substring(0, com.indexOf(","));
    comRemaining = com.substring(com.indexOf(",") + 1);
    comData = comRemaining.substring(0, comRemaining.indexOf("."));
    com = comRemaining.substring(comRemaining.indexOf(".") + 1);

    String comString = comIndex + "-" + comData;
    comBuffer[i] = comString;
    i++;

  }
  /*string commandBuffer[]= {'1,0' '1,1' '2,0' '2,1'

    // Finds the command we wish to activate
    if(comTarget.equalsIgnoreCase(01))
      {
      else if(comDirection.equalsIgnoreCase("1"))
      {
       //Do stuff
       }
      else if(comDirection.equalsIgnoreCase("0"))
      {
       //Dont do stuff
        }
      }
      else Serial.write ("Command Not Recognized")


  */
}
