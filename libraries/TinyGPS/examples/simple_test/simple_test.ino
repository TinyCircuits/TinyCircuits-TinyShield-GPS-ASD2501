
/* This sample code demonstrates the normal use of a TinyGPS object,
 * customised for the TinyCircuits GPS module.
*/


#include <TinyGPS.h>
#include "GPS.h"

TinyGPS gps;
char buf[32];

#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#include <SoftwareSerial.h>
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#include "SoftwareSerialZero.h"
#endif

// The GPS connection is attached with a software serial port
SoftwareSerial ss(GPS_RXPin, GPS_TXPin);


void setup()
{
  SerialMonitorInterface.begin(115200);
  ss.begin(GPSBaud);
  while (!SerialMonitorInterface && millis() < 5000); //On TinyScreen+/SAMD21 platform, this will wait until the Serial Monitor is opened or until 5 seconds has passed
  
  SerialMonitorInterface.print("Simple TinyGPS library v. "); SerialMonitorInterface.println(TinyGPS::library_version());
  SerialMonitorInterface.println("by Mikal Hart");
  SerialMonitorInterface.println();
  
  gpsInitPins();
  delay(100);
  SerialMonitorInterface.print("Attempting to wake GPS module.. ");
  gpsOn();
  SerialMonitorInterface.println("done.");
  delay(100);
  while (ss.available())ss.read();
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // SerialMonitorInterface.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  unsigned long age;
  if (newData)
  {
    float flat, flon;
    gps.f_get_position(&flat, &flon, &age);
    SerialMonitorInterface.print("LAT=");
    SerialMonitorInterface.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    SerialMonitorInterface.print(" LON=");
    SerialMonitorInterface.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    SerialMonitorInterface.print(" SAT=");
    SerialMonitorInterface.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    SerialMonitorInterface.print(" PREC=");
    SerialMonitorInterface.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    //GPS mode
    SerialMonitorInterface.print(" Constellations=");
    SerialMonitorInterface.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0 : gps.constellations());
  }

  //satellites in view
  uint32_t* satz = gps.trackedSatellites();
  uint8_t sat_count = 0;
  for(int i=0;i<24;i++)
  {
    if(satz[i] != 0)    //exclude zero SNR sats
    {
      sat_count++;
      byte strength = (satz[i]&0xFF)>>1;
      byte prn = satz[i]>>8;
      sprintf(buf, "PRN %d: ", prn);
      SerialMonitorInterface.print(buf);
      SerialMonitorInterface.print(strength);
      SerialMonitorInterface.println("dB");
    }
  }
  
  //date time
  int year;
  uint8_t month, day, hour, minutes, second, hundredths;
  gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths, &age);
  sprintf(buf,"GPS time: %d/%02d/%02d %02d:%02d:%02d", year, month, day, hour, minutes, second);
  SerialMonitorInterface.println(buf);

  gps.stats(&chars, &sentences, &failed);
  SerialMonitorInterface.print(" CHARS=");
  SerialMonitorInterface.print(chars);
  SerialMonitorInterface.print(" SENTENCES=");
  SerialMonitorInterface.print(sentences);
  SerialMonitorInterface.print(" CSUM ERR=");
  SerialMonitorInterface.println(failed);
  if (chars == 0)
    SerialMonitorInterface.println("** No characters received from GPS: check wiring **");
}