const int GPS_ONOFFPin = A3;
const int GPS_SYSONPin = A2;
const int GPS_RXPin = A1;
const int GPS_TXPin = A0;
const int GPSBaud = 9600;

const int NMEA_GGA_SENTENCE = 0;
const int NMEA_GLL_SENTENCE = 1;
const int NMEA_GSA_SENTENCE = 2;
const int NMEA_GSV_SENTENCE = 3;
const int NMEA_RMC_SENTENCE = 4;
const int NMEA_VTG_SENTENCE = 5;
const int NMEA_GNS_SENTENCE = 7;

char gpsDecToHex(byte val) {
  return (val & 0xF) + (((val & 0xF) < 10) ? '0' : ('A' - 10));
}

void gpsDoChecksum(char * sentence) {
  int c = 0;
  const int maxLen=100;
  while (sentence[c++] != '$' && c < maxLen);
  if (c >= maxLen)return;
  byte checksum = sentence[c++];
  while (sentence[c] != '*' && c < maxLen)
    checksum ^= sentence[c++];
  if (c >= maxLen)return;
  sentence[c + 1] = gpsDecToHex(checksum >> 4);
  sentence[c + 2] = gpsDecToHex(checksum);
}

char command[] = "$PSRF103,00,00,00,01*xx\r\n";
  
char * gpsConfig(int sentenceType, int rate) {
  command[10] = '0' + constrain(sentenceType,0,9);
  command[16] = '0' + constrain(rate,0,9);
  gpsDoChecksum(command);
  return command;
}

void gpsInitPins() {
  pinMode(GPS_SYSONPin, INPUT);
  digitalWrite(GPS_ONOFFPin, LOW);
  pinMode(GPS_ONOFFPin, OUTPUT);
}

void gpsOn() {
  while (digitalRead( GPS_SYSONPin ) == LOW )
  {
    digitalWrite( GPS_ONOFFPin, HIGH );
    delay(5);
    digitalWrite( GPS_ONOFFPin, LOW );
    delay(100);
  }
}

void gpsOff() {
  while (digitalRead( GPS_SYSONPin ) == HIGH )
  {
    digitalWrite( GPS_ONOFFPin, HIGH );
    delay(5);
    digitalWrite( GPS_ONOFFPin, LOW );
    delay(400);
  }
}

