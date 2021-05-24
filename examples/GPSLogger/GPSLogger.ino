/*
  TinyCircuits GPS TinyShield Logging Example
  
  This example logs GPS NMEA sentences to an SD card. If it doesn't detect an SD
  card at startup, it will output data to the Serial Monitor.
  With the Telit SE868 V2 module with Glonass support, some messages come through
  as GN** sentences instead of GP**. These are changed back to GP** before logging
  so that they don't cause problems with programs like Google Earth.
  
  If you see bad data:
  - Make sure the baud rate in the Serial Monitor is the same you see set in the below 
    program for the SerialMonitorInterface - 115200
  - Some GPS modules shipped before 2018 shipped with 4800 baud instead of 9600. Try changing 
    this value in the top of the GPS.h file.

  Written 10 July 2018 By Ben Rose
  Modified 07 January 2019 By Hunter Hykes
   - This update added SoftwareSerialZero files modified for SAMD21 based boards like
     the TinyScreen+, TinyZero or Arduino Zero.
  Modified May 2021 to clarify comments

  https://TinyCircuits.com
*/

#include "GPS.h"
#include <SD.h>

#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#include <SoftwareSerial.h>
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#include "SoftwareSerialZero.h"
#endif

const int chipSelect = 10;
int cardPresent;

// The GPS connection is attached with a software serial port
SoftwareSerial softSerial(GPS_RXPin, GPS_TXPin);

#define Gps_serial softSerial

void setup()
{
  Gps_serial.begin(GPSBaud);
  SerialMonitorInterface.begin(115200);
  while (!SerialMonitorInterface && millis() < 5000); //On TinyScreen+, this will wait until the Serial Monitor is opened or until 5 seconds has passed

  gpsInitPins();
  delay(100);
  SerialMonitorInterface.print("Attempting to wake GPS module.. ");
  gpsOn();
  SerialMonitorInterface.println("done.");
  delay(200);

  //Enable and set interval or disable, per NMEA sentence type
  Gps_serial.print(gpsConfig(NMEA_GGA_SENTENCE, 1));
  Gps_serial.print(gpsConfig(NMEA_GLL_SENTENCE, 0));
  Gps_serial.print(gpsConfig(NMEA_GSA_SENTENCE, 0));
  Gps_serial.print(gpsConfig(NMEA_GSV_SENTENCE, 0));
  Gps_serial.print(gpsConfig(NMEA_RMC_SENTENCE, 1));
  Gps_serial.print(gpsConfig(NMEA_VTG_SENTENCE, 0));
  Gps_serial.print(gpsConfig(NMEA_GNS_SENTENCE, 0));

  SerialMonitorInterface.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (SD.begin(chipSelect)) {
    SerialMonitorInterface.println("card initialized.");
    cardPresent = true;
  } else {
    SerialMonitorInterface.println("Card failed, or not present- continuing with serial output");
    cardPresent = false;
  }
}

void loop() {
  handleGPS();
}

char waitForCharacter() {
  while (!Gps_serial.available());
  return Gps_serial.read();
}

void handleGPS() {
  while (Gps_serial.read() != '$') {
    if(!Gps_serial.available()){
      return;
    }
  }
  int counter = 1;
  char c = 0;
  char buffer[100];
  buffer[0] = '$';
  c = waitForCharacter();
  while (c != '*') {
    buffer[counter++] = c;
    if(c=='$'){//new start
      counter=1;
    }
    c = waitForCharacter();
  }
  buffer[counter++] = c;
  buffer[counter++] = waitForCharacter();
  buffer[counter++] = waitForCharacter();
  buffer[counter++] = '\r';
  buffer[counter++] = '\n';
  buffer[counter] = '\0';

  buffer[1] = 'G';
  buffer[2] = 'P';

  gpsDoChecksum(buffer);

  if (cardPresent) {
    File dataFile = SD.open("gps.txt", FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.write(buffer, counter);
      dataFile.close();
    } else {
      SerialMonitorInterface.println("error opening gps.txt");
      cardPresent = false;
    }
  } else {
    SerialMonitorInterface.print((char *)buffer);
  }
}
