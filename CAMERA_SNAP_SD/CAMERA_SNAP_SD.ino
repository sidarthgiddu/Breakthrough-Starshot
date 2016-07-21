#include <Adafruit_VC0706.h>
#include <SPI.h>
#include <SD.h>

#define chipSelect 4

Adafruit_VC0706 cam = Adafruit_VC0706(&Serial1);

void setup() {
  Serial.begin(9600);

  delay(3000);
  
    if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  TakePic();
  cam.begin();
  TakePic();
  cam.begin();
  TakePic();
  }
void TakePic() {
  // Try to locate the camera


  cam.setImageSize(VC0706_160x120);

  // You can read the size back from the camera (optional, but maybe useful?)
  uint8_t imgsize = cam.getImageSize();

  Serial.println("Snapping...");

  if (! cam.takePicture())
    Serial.println("Failed to snap!");
  else
    Serial.println("Picture taken!");

  // Create an image with the name IMAGExx.JPG //ALTER THIS TO STORE THE NEXT AVAILABLE FILENAME
  char filename[13];
  strcpy(filename, "IMAGE000.JPG");
  for (int i = 0; i < 200; i++) {
    filename[5] = '0' + i / 100;
    filename[6] = '0' + i % 100/10;
    filename[7] = '0' + i % 100 % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  // Open the file for writing
  File imgFile = SD.open(filename, FILE_WRITE);

  // Get the size of the image (frame) taken
  uint16_t jpglen = cam.frameLength();
  Serial.print("Storing ");
  Serial.print(jpglen, DEC);
  Serial.print(" byte image.");

  int32_t time = millis();
  pinMode(8, OUTPUT);
  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(64, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);
    if (++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
      Serial.print('.');
      wCount = 0;
    }
    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
    jpglen -= bytesToRead;
  }
  imgFile.close();

  time = millis() - time;
  Serial.println("done!");
  Serial.print(time); Serial.println(" ms elapsed");
}

void loop() {
}

