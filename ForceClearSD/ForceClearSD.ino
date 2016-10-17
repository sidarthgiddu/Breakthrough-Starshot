#include <SD.h>

const int SDChipSelect = 4; // SS pin on the SD card.
// Other pins connected to normal SPI

File root;
int DeletedCount = 0;
int FolderDeleteCount = 0;
int FailCount = 0;
String rootpath = "/";

void setup()
{
  Serial.begin(9600);
  pinMode(10, OUTPUT);

  SD.begin(SDChipSelect);

  root = SD.open("/");
  delay(2000);

  rm(root, rootpath);

  if( !DeletedCount && !FailCount && !FolderDeleteCount ){

  } 
  else{
    Serial.print("Deleted ");
    Serial.print(DeletedCount);
    Serial.print(" file");
    if( DeletedCount != 1 ){
      Serial.print("s");
    }
    Serial.print(" and ");
    Serial.print(FolderDeleteCount);
    Serial.print(" folder");
    if ( FolderDeleteCount != 1 ){
      Serial.print("s");
    }
    Serial.println(" from SD card.");
    if( FailCount > 0 ){
      Serial.print("Failed to delete ");
      Serial.print(FailCount);
      Serial.print(" item");
      if( FailCount != 1 ){
        Serial.print("s");
      }
    }
  }
}

void loop()
{
  while(1){
    // nada
  }
}

void rm(File dir, String tempPath) {
  while(true) {
    File entry =  dir.openNextFile();
    String localPath;

    Serial.println("");
    if (entry) {
      if ( entry.isDirectory() )
      {
        localPath = tempPath + entry.name() + rootpath + '\0';
        char folderBuf[localPath.length()];
        localPath.toCharArray(folderBuf, localPath.length() );
        rm(entry, folderBuf);


        if( SD.rmdir( folderBuf ) )
        {
          Serial.print("Deleted folder ");
          Serial.println(folderBuf);
          FolderDeleteCount++;
        } 
        else
        {
          Serial.print("Unable to delete folder ");
          Serial.println(folderBuf);
          FailCount++;
        }
      } 
      else
      {
        localPath = tempPath + entry.name() + '\0';
        char charBuf[localPath.length()];
        localPath.toCharArray(charBuf, localPath.length() );

        if( SD.remove( charBuf ) )
        {
          Serial.print("Deleted ");
          Serial.println(localPath);
          DeletedCount++;
        } 
        else
        {
          Serial.print("Failed to delete ");
          Serial.println(localPath);
          FailCount++;
        }

      }
    } 
    else {
      // break out of recursion
      break;
    }
  }
}
