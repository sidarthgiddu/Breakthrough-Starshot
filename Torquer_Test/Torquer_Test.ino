const int CX1 = 22; 
const int CX2 = 23;
const int CY1 = 5; 
const int CY2 = 6; 
const int CZ1 = 9; 
const int CZ2 = 10; 

const int CX_PWM = 11;
const int CY_PWM = 12;
const int CZ_PWM = 13;
const int CXY_Enable = A5; 
const int CZ_Enable = 24;
void setup() {
  // put your setup code here, to run once:
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

  digitalWrite(CXY_Enable, HIGH);
  digitalWrite(CZ_Enable, HIGH);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Forward");
  digitalWrite(CX1, HIGH);
  digitalWrite(CX2, LOW);
  digitalWrite(CX_PWM, HIGH);

  digitalWrite(CY1, HIGH);
  digitalWrite(CY2, LOW);
  digitalWrite(CY_PWM, HIGH);

  digitalWrite(CZ1, HIGH);
  digitalWrite(CZ2, LOW);
  digitalWrite(CZ_PWM, HIGH);

//  delay(6000); Serial.println("Reverse");
//
//  digitalWrite(CX1, LOW);
//  digitalWrite(CX2, HIGH);
//  analogWrite(CX_PWM, 255);
//
//  digitalWrite(CY1, LOW);
//  digitalWrite(CY2, HIGH);
//  analogWrite(CY_PWM, 255);
//
//  digitalWrite(CZ1, LOW);
//  digitalWrite(CZ2, HIGH);
//  analogWrite(CZ_PWM, 255);

  delay(6000);
}

