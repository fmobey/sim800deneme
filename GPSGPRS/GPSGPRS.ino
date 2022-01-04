/*
Autour: Furkan Metin OĞUZ
Date:2022
**/
 
#include <TinyGPS++.h> 
#include <HardwareSerial.h> 
#include "EEPROM.h" 
#include <Wire.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00
#define EEPROM_SIZE 1024
#define TASK_SERIAL_RATE 1000 // ms
uint32_t nextSerialTaskTs = 0;
uint32_t nextOledTaskTs = 0;
 


// Modeminizi seçin:
#define TINY_GSM_MODEM_SIM800 

#define SerialMon Serial
#define SerialAT Serial1

#define TINY_GSM_DEBUG SerialMon

// varsa, GSM PIN'i ayarlayın
#define GSM_PIN ""
#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient mqtt(client);
HardwareSerial SerialGPS(2);

TinyGPSPlus gps;
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
//batarya
const int giris=100;
float total =0;
float average=0;
float okunandeger[giris];
float deger =0;
float volt =0;
float yuzde =-1;

// Varsa GPRS kimlik bilgileriniz
const char apn[] = "internet"; 
const char gprsUser[] = "";
const char gprsPass[] = "";

// SIM card PIN 
const char simPIN[]   = ""; 

// MQTT detay
const char* broker = "194.31.59.188";                    // Public IP address or domain name
const char* mqttUsername = "deneme";  // MQTT username
const char* mqttPassword = "deneme";  // MQTT password
const char* mqtt_id = "deneme";  // MQTT id
const char* topicOutput = "v1/devices/me/telemetry";

const char* topic = "v1/devices/me/telemetry";




// BME280 pins

uint32_t lastReconnectAttempt = 0;







long lastMsg = 0;


void mqttCallback(char* topic, byte* message, unsigned int len) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  

  Serial.println();

// mqtt üzerinden kilit açıp kapatma
  if (String(topic) == "v1/devices/me/telemetry") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      
    }
    else if(messageTemp == "off"){
      Serial.println("off");
    }
  }
  }


boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

 
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
  //SerialMon.begin(9600, SERIAL_8N1, 12, 13); kilit sistemi kullanılcağı zaman 
  SerialMon.begin(9600);
  delay(10);
  SerialMon.begin(9600, SERIAL_8N1, 13, 12);//dinleme

  

  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  

  
  SerialMon.println("Wait...");

  // Gsm seri port
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);


  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // sim kilit açma
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

  //gps
    SerialGPS.begin(9600, SERIAL_8N1, 18, 19);

  while (!EEPROM.begin(EEPROM_SIZE)) {
    
  }
 
 
  long readValue;
  EEPROM_readAnything(0, readValue);
  gpsState.originLat = (double)readValue / 1000000;
 
  EEPROM_readAnything(4, readValue);
  gpsState.originLon = (double)readValue / 1000000;
 
  EEPROM_readAnything(8, readValue);
  gpsState.originAlt = (double)readValue / 1000000;
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

  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");



    
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 1000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }

  // Modülden GPS koordinatlarını okuma
  gpsState.originLat = gps.location.lat();
  gpsState.originLon = gps.location.lng();
  gpsState.originAlt = gps.altitude.meters();
 
  // Geçerli konumu kalıcı ESP32 belleğine yaz
  long writeValue;
  writeValue = gpsState.originLat * 1000000;
  EEPROM_writeAnything(0, writeValue);
  writeValue = gpsState.originLon * 1000000;
  EEPROM_writeAnything(4, writeValue);
  writeValue = gpsState.originAlt * 1000000;
  EEPROM_writeAnything(8, writeValue);
  EEPROM.commit(); 
 
  gpsState.distMax = 0;
  gpsState.altMax = -999999;
  gpsState.spdMax = 0;
  gpsState.altMin = 999999;

  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
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
 
  long now = millis();
  if (now - lastMsg > 3000) {
    lastMsg = now;

     for(int i=0; i<100;i++){
  deger = analogRead(34);
  
  volt = ((deger/1202)*345)/15;
  
  okunandeger[i]= volt;
  total= total + okunandeger[i];
  
   average= total/giris;
  }
  total=0;

  yuzde = map(average, 31.1, 42.1, 0.0, 100.0 );


    StaticJsonDocument < 256 > JSONbuffer;
    JsonObject GpsData = JSONbuffer.createNestedObject();
    JsonObject BatteryData = JSONbuffer.createNestedObject();

      BatteryData["BatteryVoltage"]=average;
      BatteryData["BatteryPercent"]=yuzde;

        GpsData["LAT"] = gps.location.lat();
        GpsData["LONG"] = gps.location.lng();
        GpsData["SPEED"] = gps.speed.kmph();
        GpsData["ALT"] = gps.altitude.meters();
        GpsData["LONG"] = gps.location.lng();
        
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
}


  }
