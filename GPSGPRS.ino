/*
Autour: Furkan Metin OĞUZ
Date:2022
*/

#include <Wire.h>

#include <TinyGsmClient.h>

#include <TinyGPS++.h>

#include <SoftwareSerial.h>

#include <PubSubClient.h>

#include <ArduinoJson.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>


//gprs pin
#define GSM_PIN ""
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define I2C_SDA 21
#define I2C_SCL 22
#define IP5306_ADDR 0x75
#define IP5306_REG_SYS_CTL0 0x00

//sim kurulum
const char apn[] = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";
const char simPIN[] = "";

//mqtt kurulum
const char * broker = "194.31.59.188";
const char * mqttUsername = "deneme";
const char * mqttPassword = "deneme";
const char * mqtt_id = "deneme";
const char * topicOutput = "v1/devices/me/telemetry";
const char * topic = "v1/devices/me/telemetry";
//kordinatlar
long lat, lon;
SoftwareSerial gpsSerial(rxGPS, txGPS);
TinyGPSPlus gps;
TinyGsmClient client(modem);
PubSubClient mqtt(client);
uint32_t lastReconnectAttempt = 0;
long lastMsg = 0;

void mqttCallback(char * topic, byte * message, unsigned int len) {
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

  if (GSM_PIN && modem.getSimStatus() != 3) {
    modem.simUnlock(GSM_PIN);
  }

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  } else {
    SerialMon.println(" OK");
  }

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }

  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
}

void loop() {
  //mqtt bağlanma ve yeniden bağlanma
  if (!mqtt.connected()) {
    SerialMon.println("MQTT NOT CONNECT ");
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000 L) {
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


        StaticJsonDocument < 256 > JSONbuffer;
        JsonObject veri = JSONbuffer.createNestedObject();



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