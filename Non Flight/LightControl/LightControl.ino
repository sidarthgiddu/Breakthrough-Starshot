#define BarLength 20
#define BarsPerUnit 8
int layout[1] = {2};

#define Chain0Pin 8
#define Chain1Pin 9
#define Chain2Pin 10

class intTuple
{
  public:
    int x;
    int y;
    int z;

    intTuple(int a, int b, int c) {
      x = a;
      y = b;
      z = c;
    }
    void print() {
      Serial.println("(");
      Serial.print(x); Serial.print(",");
      Serial.print(y); Serial.print(",");
      Serial.print(z); Serial.println(")");
    }

};

void setBar(int column, int bar, intTuple color) {
  colorRange(column, bar * BarLength, (bar + 1)*BarLength - 1, intTuple color);
}

void setUnit(int column, int unit, intTuple color) {
  colorRange(column, unit * BarLength * BarsPerUnit, (unit + 1)*BarLength * BarsPerUnit - 1, color);
}

void colorRange(int column, int start, int last, intTuple color) {
  //Colors start to last
  for (int i = start; i <= last; i++) {
    //write color to index i on column
  }
}

int mapLight(int x, int y) {

  return;
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
