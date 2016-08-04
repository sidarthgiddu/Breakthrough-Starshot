#include <SD.h>
#include <SPI.h>
#define chipSelect 4
//We have 4 values to define into Masterstatusholder (filing cabinet).
bool Downlink_Active; //True/False is downlinking occurring?...
int imgDownlinkArray[15]; //Array will hold at max 15 chunks of our image
int currImgArrayPl; //Which place are we currently writing to in our array
int lastImgArrayPl; //The last place we are writing to in our array - last spot of useful info (may be smaller than 15)
uint16_t jpglen;
File IMGFile;
String Index;
String Filename = "IMAGE067.jpg";
uint8_t  imageBuffer[20];
int finalIndex = 0;
int sizeArray[20] = {0};

void printArray(uint8_t * arr, int s) {
  if (s) {
    for (int i = 0; i < s; i++) {
      //    for (int j = 0; j < 8; j++) {
      //      if (arr[i] < pow(2, j))
      //        Serial.print(B0);
      //    }
      print_binary(arr[i], 8);
      Serial.print(" ");
    }
    Serial.println("");
  }
}

#define DLSize 320
class RAMImage {
  public:
    uint8_t a0[DLSize];
    uint8_t a1[DLSize];
    uint8_t a2[DLSize];
    uint8_t a3[DLSize];
    uint8_t a4[DLSize];
    uint8_t a5[DLSize];
    uint8_t a6[DLSize];
    uint8_t a7[DLSize];
    uint8_t a8[DLSize];
    uint8_t a9[DLSize];
    uint8_t a10[DLSize];
    uint8_t a11[DLSize];
    uint8_t a12[DLSize];
    uint8_t a13[DLSize];
    uint8_t a14[DLSize];
    uint8_t a15[DLSize];

    int sizeArray[16] = {0};
    int finalIndex = 0;
    String Filename;

    void printRI() {
      Serial.println("RAM Loaded Image: " + Filename);
      Serial.println("   Segments: " + String(finalIndex + 1));
      int sum = 0; for (int i = 0; i <= finalIndex; i++) {
        sum += sizeArray[i];
      }
      Serial.println("   Total Size: " + String(sum));
      Serial.println("\nBinary Form:\n");

      printArray(a0, sizeArray[0]);
      printArray(a1, sizeArray[1]);
      printArray(a2, sizeArray[2]);
      printArray(a3, sizeArray[3]);
      printArray(a4, sizeArray[4]);
      printArray(a5, sizeArray[5]);
      printArray(a6, sizeArray[6]);
      printArray(a7, sizeArray[7]);
      printArray(a8, sizeArray[8]);
      printArray(a9, sizeArray[9]);
      printArray(a10, sizeArray[10]);
      printArray(a11, sizeArray[11]);
      printArray(a12, sizeArray[12]);
      printArray(a13, sizeArray[13]);
      printArray(a14, sizeArray[14]);
      printArray(a15, sizeArray[15]);
    }
    RAMImage() {}
    void store(uint8_t * data, int dataSize, int index) {
      int bytes = min(DLSize, dataSize);
      if (index > finalIndex) {
        finalIndex = index;
      }
      switch (index) {
        case 0:
          for (int i = 0; i < bytes; i++) {
            a0[i] = data[i];
          }
          break;
        case 1:
          for (int i = 0; i < bytes; i++) {
            a1[i] = data[i];
          }
          break;
        case 2:
          for (int i = 0; i < bytes; i++) {
            a2[i] = data[i];
          }
          break;
        case 3:
          for (int i = 0; i < bytes; i++) {
            a3[i] = data[i];
          }
          break;
        case 4:
          for (int i = 0; i < bytes; i++) {
            a4[i] = data[i];
          }
          break;
        case 5:
          for (int i = 0; i < bytes; i++) {
            a5[i] = data[i];
          }
          break;
        case 6:
          for (int i = 0; i < bytes; i++) {
            a6[i] = data[i];
          } break;
        case 7:
          for (int i = 0; i < bytes; i++) {
            a7[i] = data[i];
          } break;
        case 8:
          for (int i = 0; i < bytes; i++) {
            a8[i] = data[i];
          } break;
        case 9:
          for (int i = 0; i < bytes; i++) {
            a9[i] = data[i];
          } break;
        case 10:
          for (int i = 0; i < bytes; i++) {
            a10[i] = data[i];
          } break;
        case 11:
          for (int i = 0; i < bytes; i++) {
            a11[i] = data[i];
          } break;
        case 12:
          for (int i = 0; i < bytes; i++) {
            a12[i] = data[i];
          } break;
        case 13:
          for (int i = 0; i < bytes; i++) {
            a13[i] = data[i];
          } break;
        case 14:
          for (int i = 0; i < bytes; i++) {
            a14[i] = data[i];
          } break;
        case 15:
          for (int i = 0; i < bytes; i++) {
            a15[i] = data[i];
          } break;
      }
    }
    uint8_t * get(int index) {
      switch (index) {
        case 0: return a0;
        case 1: return a1;
        case 2: return a2;
        case 3: return a3;
        case 4: return a4;
        case 5: return a5;
        case 6: return a6;
        case 7: return a7;
        case 8: return a8;
        case 9: return a9;
        case 10: return a10;
        case 11: return a11;
        case 12: return a12;
        case 13: return a13;
        case 14: return a14;
        case 15: return a15;
      }
    }
};
RAMImage imageR = RAMImage();

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
void printImageBuffer() {
  //  for (int i = 0; i <= finalIndex; i++) {
  //    printArray(imageBuffer[i], sizeArray[i]);
  //    Serial.println("");
  //  }
}
void buildImageBuffer(String Filename) { //Rename to masterstatusholderimagebuffer in future...
  IMGFile = SD.open(Filename, FILE_READ);
  uint8_t jpglen = IMGFile.size();
  //int segments = ((8*jpglen) / 320) + 1;
  //String imageBuffer[segments];
  int index = 0;
  int i = 0;
  Serial.println("Starting Segmentation");
  while (IMGFile.available()) {
    //Serial.println("Available: " + String(IMGFile.available()));
    int bytesToRead = min(320, IMGFile.available());
    uint8_t segment[bytesToRead - 1];
    for (int z = 0; z < bytesToRead; z++) {
      segment[z] = 0;
    }
    while (i < bytesToRead) {
      segment[i] = (uint8_t)IMGFile.read();
      i++;
    }
    //Serial.print("Current Segment " + String(index) + ": ");
    printArray(segment, i);
    //Serial.println("");
    //Serial.println("Here1");
    //    imageBuffer[index] = segment;
    imageR.store(segment, i, index);
    imageR.sizeArray[index] = bytesToRead;

    i = 0;
    index = index + 1;
    //Serial.println("Here2");
    //Serial.println("");
    //Serial.println("");
  }
  Serial.println("\nDone");
  finalIndex = index - 1;
  Serial.println(finalIndex);
  return;
}

int downlinkSegment(int segIndex) {
  uint8_t * Data = imageR.get(segIndex);
  int DataSize = imageR.sizeArray[segIndex];
  Serial.println("Begining Downlink");
  Serial1.print("AT+SBDWB=");
  Serial1.print(DataSize);
  Serial1.print("\r");

  delay(100);
  //Wait for Ready
  Serial.println("Response 1 >>");
  while (Serial1.available()) {
    Serial.print((char)Serial1.read());
  }

  Serial.println("\n<<Response 1");
  uint16_t checksum = 0;
  for (int i = 0; i < DataSize; ++i)
  {
    Serial1.write(Data[i]);
    checksum += (uint16_t)Data[i];
  }
  //printArray(imageBuffer[0], DataSize);
  Serial1.write(checksum >> 8);
  Serial1.write(checksum & 0xFF);

  delay(100);
  //Ok?
  Serial.println("Response 2 >> ");
  while (Serial1.available()) {
    Serial.print((char)Serial1.read());
  }
  Serial.println("\n<< Response 2");
  return 0;
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(19200);
  delay(2000);

  Serial.println("Initialize SD...");
  SD.begin(chipSelect);
  Serial.println("Success");

  buildImageBuffer(Filename);
  Serial.println("\n\nPrint Start");
  imageR.printRI();
  Serial.println("\nPrint End");
  downlinkSegment(0);
}

void loop() {
  //  Serial.println(Filename);
  delay(1000);
  Serial.println("Success");
}
