/*
   (c) 2019 by shopofthings.ch
   Beispiel zum Verbinden des GPS Modules mit
   einem ESP32 Modul
 
   Pinverbindung:
   ESP 3.3V > GPS VCC
   ESP GND  > GPS GND
   ESP RX   > GPS TX
   ESP TX   > GPS RX
 
   Achtung: RX wird mit TX bzw. TX mit RX verbunden!
 
   Das GPS Modul hat eine blaue LED. Diese blinkt, wenn Satelliten erreichbar sind. Sind
   keine Satelliten im Sichtfeld, bleibt sie dunkel. Ein Zustand, wo sie immer leuchtet 
   gibt es nicht.
 
  Troubleshooting:
  - Arduino IDE gibt bei Upload Fehler aus: Boot Taste vor dem Upload gedrückt halten, bis Upload beginnt.
  - 
 
*/
 
#include <TinyGPS++.h> // Library über http://arduiniana.org/libraries/tinygpsplus/ downloaden und installieren
#include <HardwareSerial.h> // sollte bereits mit Arduino IDE installiert sein
#include "EEPROM.h" // sollte bereits mit Arduino IDE installiert sein
 
/*
   mit EEPROM können Daten zwischengespeichert werden,
   ein kleiner Speicher sozusagen. Daten sind auch nach einem
   Reset noch da, wo sie hingeschrieben wurden. Um in den
   Speicher schreiben zu können, muss eine Speichergrösse
   angegeben werden. Diese wird in Bytes angegeben.
   Wir verwenden diese, um den letzten GPS-Fix abzuspeichern
   damit er beim neustart als erstes eingelesen wird. Damit
   wird verhindert, dass z.B. bei einer Aufnahme unsere Position
   als erstes auf den 0,0 Punkt springt, der irgendwo im Atlantischen
   Ozean liegt.
 
   Der Speicher bei ESP32 kann zwischen 4 und maximum 1984 bytes liegen
*/
#define EEPROM_SIZE 128
 
/*
   Als nächstes wird das Programmobjekt für das GPS erstellt, wir nennen
   die Variable die darauf zeigt einfach "gps"
*/
TinyGPSPlus gps;
 
/*
   Nun müssen wir eine serielle Verbindung zum GPS Modul erstellen
   ESP32 unterstützt bis zu 3 Hardware Serielle Verbndungen. Deshalb
   müssen wir auch nicht die Softwareserial-Library verwenden.
   Die Zahl in der Klammer ist die uart_nr. Damit werden die drei möglichen
   Verbindungen unterschieden. Für ESP32 kann dieser Wert also 0, 1 oder 2 sein
*/
HardwareSerial SerialGPS(1);
 
/*
   Als nächstes erstellen wir ein Template-Object, wo wir die ganzen Daten
   in einer Variable abspeichern können, die vom GPS Modul gelesen werden
   Im Anschluss erstellen wir gleich so eine neue Variable, "gpsState"
*/
struct GpsDataState_t {
  double originLat = 0;
  double originLon = 0;
  double originAlt = 0;
  double distMax = 0;
  double dist = 0;
  double altMax = -999999;
  double altMin = 999999;
  double spdMax = 0;
  double prevDist = 0;
};
GpsDataState_t gpsState = {};
 
/*
   Mit der nachfolgenden Konstante definieren wir die Ausgabegeschwindigkeit
   im Serial Monitor. Diese wird in Millisekunden angegeben.
   Darunter die dazugehörigen Variablen, um diese Beschränkung zu erreichen
*/
#define TASK_SERIAL_RATE 1000 // ms
uint32_t nextSerialTaskTs = 0;
uint32_t nextOledTaskTs = 0;
 
/*
   Setup Funktion.
   Diese wird einmal beim Systemstart ausgeführt
   Enthält alle Initialisierungen
*/
void setup() {
 
  // Serial ist die Ausgabe im Serial Monitor
  Serial.begin(115200);
 
  /*
      Die Verbindung mit dem GPS Modul. Wir
      void begin(unsigned long baud, uint32_t config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false, unsigned long timeout_ms = 20000UL);
      baud: baudrate gemäss Spzifikation des GPS Moduls, in diesem Fall 9600
      config: default Wert
      rxPin: ein RX-Pin z.B. 16
      txPin: ein RX-Pin z.B. 17
  */
  SerialGPS.begin(9600, SERIAL_8N1, 18, 19);
 
  /*
     EEPROM Speicher initialisieren, wenn noch nicht existent
  */
  while (!EEPROM.begin(EEPROM_SIZE)) {
    
  }
 
  /*
     Die drei Achsrichtungen x, y, z aus dem Speicher
     lesen und hinterlegen
  */
  long readValue;
  EEPROM_readAnything(0, readValue);
  gpsState.originLat = (double)readValue / 1000000;
 
  EEPROM_readAnything(4, readValue);
  gpsState.originLon = (double)readValue / 1000000;
 
  EEPROM_readAnything(8, readValue);
  gpsState.originAlt = (double)readValue / 1000000;
 
}
 
/*
   Hilfsfunktionen um vereinfacht Speicher zu lesen und schreiben
*/
template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}
 
template <class T> int EEPROM_readAnything(int ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return i;
}
 
 
/*
   Loop Funktion
   Diese wird bei jedem Systemtakt ausgeführt
*/
void loop() {
 
  static int p0 = 0;
 
  // GPS Koordinaten von Modul lesen
  gpsState.originLat = gps.location.lat();
  gpsState.originLon = gps.location.lng();
  gpsState.originAlt = gps.altitude.meters();
 
  // Aktuelle Position in nichtflüchtigen ESP32-Speicher schreiben
  long writeValue;
  writeValue = gpsState.originLat * 1000000;
  EEPROM_writeAnything(0, writeValue);
  writeValue = gpsState.originLon * 1000000;
  EEPROM_writeAnything(4, writeValue);
  writeValue = gpsState.originAlt * 1000000;
  EEPROM_writeAnything(8, writeValue);
  EEPROM.commit(); // erst mit commit() werden die Daten geschrieben
 
  gpsState.distMax = 0;
  gpsState.altMax = -999999;
  gpsState.spdMax = 0;
  gpsState.altMin = 999999;
 
  /*
   * Rohdaten von Serieller Verbndung zum GPS-Modul
   * einlesen. Die Daten werden mittels TinyGPS++ verarbeitet
   * Die Daten werden bewusst erst nach der Zuweisung der Variablen
   * gelesen, damit wir noch im nachfolgenden vereinfacht 
   * Berechnungen anstellen können.
   */
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
 
  /*
   * Diverse Berechnungen von Maximum und Minimum-Werten und zurückgelegter Distanz
   * Diese werden aber erst gemacht, wenn mindestens ein Fix mit 4 Satelliten vorhanden
   * ist, allenfalls wäre die Genauigkeit nicht gegeben und es würden falsche
   * Werte berechnet werden.
   */
  if (gps.satellites.value() > 4) {
    gpsState.dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), gpsState.originLat, gpsState.originLon);
 
    if (gpsState.dist > gpsState.distMax && abs(gpsState.prevDist - gpsState.dist) < 50) {
      gpsState.distMax = gpsState.dist;
    }
    gpsState.prevDist = gpsState.dist;
 
    if (gps.altitude.meters() > gpsState.altMax) {
      gpsState.altMax = gps.altitude.meters();
    }
 
    if (gps.speed.kmph() > gpsState.spdMax) {
      gpsState.spdMax = gps.speed.kmph();
    }
 
    if (gps.altitude.meters() < gpsState.altMin) {
      gpsState.altMin = gps.altitude.meters();
    }
  }
 
  /*
     Damit nicht zu viele Daten im Serial Monitor ausgegeben werden,
     beschränken wir die Ausgabe auf die Anzahl Millisekunden
     die wir in der Konstante "TASK_SERIAL_RATE" gespeichert haben
  */
  if (nextSerialTaskTs < millis()) {
    Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
    Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
    Serial.print("ALT=");  Serial.println(gps.altitude.meters());
    Serial.print("Sats=");  Serial.println(gps.satellites.value());
    Serial.print("DST: ");
    Serial.println(gpsState.dist, 1);
    nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
  }
}
