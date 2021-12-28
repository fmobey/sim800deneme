/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cloud-mqtt-broker-sim800l/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

// Select your modem:
#define TINY_GSM_MODEM_SIM800 // Modem is SIM800L

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "internet"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = "";
const char gprsPass[] = "";

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = ""; 

// MQTT details
const char* broker = "194.31.59.188";                    // Public IP address or domain name
const char* mqttUsername = "deneme";  // MQTT username
const char* mqttPassword = "deneme";  // MQTT password
const char* mqtt_id = "deneme";  // MQTT id
const char* topicOutput = "v1/devices/me/telemetry";

const char* topic = "v1/devices/me/telemetry";


// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#include <Wire.h>
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif
#include <TinyGPS++.h> // Library über http://arduiniana.org/libraries/tinygpsplus/ downloaden und installieren
#include <HardwareSerial.h> // sollte bereits mit Arduino IDE installiert sein
#include "EEPROM.h"
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

// BME280 pins

uint32_t lastReconnectAttempt = 0;

//gps
#define EEPROM_SIZE 128
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
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
#define TASK_SERIAL_RATE 1000 // ms
uint32_t nextSerialTaskTs = 0;
uint32_t nextOledTaskTs = 0;
 


#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

long lastMsg = 0;


void mqttCallback(char* topic, byte* message, unsigned int len) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  

  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp/output1, you check if the message is either "true" or "false". 
  // Changes the output state according to the message


  }


boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker without username and password
  //boolean status = mqtt.connect("GsmClientN");

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect(mqtt_id, mqttUsername, mqttPassword);

  if (status == false) {
    SerialMon.println(" fail");
    ESP.restart();
    return false;
  }
  SerialMon.println(" success");
  mqtt.subscribe(topicOutput);
  

  return mqtt.connected();
}


void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);
  
//gps
    SerialGPS.begin(9600, SERIAL_8N1, 18, 19);
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

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  

  
  SerialMon.println("Wait...");

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }



  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
  }
  
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
}
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
     long t = millis();
  gpsState.distMax = 0;
  gpsState.altMax = -999999;
  gpsState.spdMax = 0;
  gpsState.altMin = 999999;
    long now = millis();
  if (!mqtt.connected()) {

    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds

    if (t - lastReconnectAttempt > 1000) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }
  if (now - lastMsg > 200) {
    lastMsg = now;
    
    while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  }
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




 
  if (now - lastMsg > 2000) {
    lastMsg = now;
    
  


    StaticJsonDocument < 256 > JSONbuffer;
    JsonObject veri = JSONbuffer.createNestedObject();


        veri["LAT"] = gps.location.lat();
        veri["LONG"] = gps.location.lng();
        veri["SPEED"] = gps.speed.kmph();
        veri["ALT"] = gps.altitude.meters();
        veri["LONG"] = gps.location.lng();

    char JSONmessageBuffer[200];
    serializeJsonPretty(JSONbuffer, JSONmessageBuffer);
    Serial.println("Sending message to MQTT topic..");
    Serial.println(JSONmessageBuffer);
    if (mqtt.publish("v1/devices/me/telemetry", JSONmessageBuffer) == true) {
      Serial.println("Success sending message");
    } else {
      Serial.println("Error sending message");
    }

    


  mqtt.loop();
}}