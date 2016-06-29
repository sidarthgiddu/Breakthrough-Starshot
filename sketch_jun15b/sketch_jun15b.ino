

void setup() {
Serial.begin(9600);

}

void loop() {
while (Serial.available() == 0);

int val = Serial.read();

Serial.println(val);
 

}
