#include <Adafruit_VC0706.h>
#include <SPI.h>
#include <SD.h>

#define chipSelect 4 //define SD card pin

File imgFile;
char filename[] = "placeholderFN";

Adafruit_VC0706 cam = Adafruit_VC0706(&Serial1);

int jpglen = 0; //Starting value for writing, currently no bytes written.

int numPhoto = 0;

void setup() {

  Serial.begin(9600);
  delay(1000);
  pinMode(8, OUTPUT); //???designates SS pin so we can use SPI library
  pinMode(13, OUTPUT);
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) { //select SD card pin
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
}

int TakePic() {

  cam.begin();

  cam.setImageSize(VC0706_160x120);

  // You can read the size back from the camera (optional, but maybe useful?)
  //uint8_t imgsize = cam.getImageSize();


  Serial.println("Snapping...");
  delay(1000);

  if (! cam.takePicture()) {
    Serial.println("Failed to snap!");
  } else {
    Serial.println("Picture taken!");
  }
  uint16_t jpglen = cam.frameLength();

  //Serial.println(cam.takePicture());

  return jpglen;
}
File WritePic(File imgFile, uint16_t jpglen) {

  
  while (jpglen > 0) {
    Serial.println("I'm in the WritePic func..");
    // Open the file for writing

    imgFile = SD.open(filename, FILE_WRITE);

    uint8_t bytesToRead = min(64, jpglen);
    Serial.print("bytesToRead");
    Serial.println(bytesToRead);
    Serial.println("Goin' through the buffer...");
    uint8_t *buffer; //pointer. tells you that the buffer thing later should have 8 bits?
    buffer = cam.readPicture(bytesToRead); //buffer is the current 64 bytes (or less) of the picture
    Serial.print("wrote :");
    int x = imgFile.write(buffer, bytesToRead);
    delay(100);

    if (x > 0) {
      digitalWrite(8, HIGH);
      delay(5);
      digitalWrite(8, LOW);
    }
    Serial.println(x); //write those 64 (or less) bytes of the picture to the image file
    Serial.println("I am writing to imgFile...");
    jpglen -= bytesToRead;
    Serial.print(jpglen, DEC);
    Serial.println("bytes left to read.");
  }
  return jpglen, imgFile;
}

char * makeFilename() {
  //Fix later
  char filename[13];
  strcpy(filename, "IMAGE00.JPG");
  for (int i = 0; i < 500; i++) {
    filename[5] = '0' + i / 10;
    filename[6] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write

    if (SD.exists(filename)) {
      Serial.println("file does exist. Let's try again!");
    }
    else {
      delay(1000);
      Serial.print("File (");
      Serial.print(filename);
      Serial.println(") does not exist, so we will pass it on!!");
      return filename;
      break;
    }
  }
}

unsigned long startTime = millis();
int PictureInterval = 30000;

void loop() {
  //If the jpeg length is 0 (if no picture), take a picture
  if (!(millis() - startTime > PictureInterval)) {
    Serial.print("Starting time: ");
    Serial.println(startTime);
    if (jpglen <= 0) {
      Serial.println("jpglen <= 0");
      if (SD.exists(filename)) {
        imgFile.close();
        ++numPhoto; //Increment by one, and return new value.
        Serial.print("number of photos: ");
        Serial.println(numPhoto);
      }
      Serial.println("Taking pic...");
      jpglen = TakePic(); //get file size
      Serial.println("I took the picture!");
      // Create an image with the name IMAGExx.JPG
      char * filename = makeFilename();
      imgFile = SD.open(filename, FILE_WRITE);
      int32_t time = millis(); // keeping track of time to write
      Serial.print(jpglen, DEC); Serial.println(" bytes.");
    }
    else {
      Serial.println("Writing picture...");
      jpglen, imgFile = WritePic(imgFile, jpglen);
      //needs #of bytes to read and write and the file to write them in
      //If there is a picture, write picture to SD card
      Serial.println("Wrote pic.");


    }
  } else {
    digitalWrite(13, HIGH);
    delay(2000);
    digitalWrite(13, LOW);
    delay(2000);

    //    Serial.print("Number of photos:");
    //    Serial.println(numPhoto);

  }
}


