
/*
Autour: Furkan Metin OĞUZ
Date:2022
*/
#define TINY_GSM_MODEM_SIM800 

#define SerialMon Serial
#define SerialAT Serial1

#define TINY_GSM_DEBUG SerialMon

#define GSM_PIN ""

const char apn[] = "internet"; 
const char gprsUser[] = "";
const char gprsPass[] = "";


const char simPIN[]   = ""; 


const char* broker = "194.31.59.188";                    
const char* mqttUsername = "deneme";  
const char* mqttPassword = "deneme";  
const char* mqtt_id = "deneme"; 
const char* topicOutput = "v1/devices/me/telemetry";
const char* topic = "v1/devices/me/telemetry";
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#define rxGPS 3
#define txGPS 2
 
long lat, lon;
SoftwareSerial gpsSerial(rxGPS, txGPS);
TinyGPSPlus gps;


#include <Wire.h>
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
TinyGsmClient client(modem);
PubSubClient mqtt(client);

#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22


uint32_t lastReconnectAttempt = 0;





#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

long lastMsg = 0;


void mqttCallback(char* topic, byte* message, unsigned int len) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  

  Serial.println();

 


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

  SerialMon.begin(115200);
  delay(10);
  gpsSerial.begin(115200);

  

  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  

  
  SerialMon.println("Wait...");

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  
  SerialMon.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

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

  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
}

void loop() {
  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }

  long now = millis();
  if (now - lastMsg > 3000) {
    lastMsg = now;
    
  
while (gpsSerial.available())     // check for gps data
  {
    if (gps.encode(gpsSerial.read()))   // encode gps data
    {

    StaticJsonDocument < 256 > JSONbuffer;
    JsonObject veri = JSONbuffer.createNestedObject();


    veri["LAT"] = gps.location.lat();
    veri["LONG"] = gps.location.lng();
    veri["SPEED"] = gps.speed.kms();
    veri["ALT"] = gps.altitude.meters();
    veri["LONG"] = gps.location.lng();
    veri["DATE"] = gps.date.day()+":"+gps.date.month()+":"+gps.date.year();
    veri["CLOCK"] = gps.time.hour()+":"+gps.time.minute()+":"+gps.time.second();

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


}}



