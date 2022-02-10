/*
Autour: Furkan Metin OĞUZ
Date:2022
*/
#include <TinyGPS++.h>

#include <HardwareSerial.h>

#include "EEPROM.h"

#define EEPROM_SIZE 1024

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

#define TASK_SERIAL_RATE 1000
uint32_t nextSerialTaskTs = 0;
uint32_t nextOledTaskTs = 0;

// modem seçimi:
#define TINY_GSM_MODEM_SIM800

#define SerialMon Serial
#define SerialAT Serial1

#define TINY_GSM_DEBUG SerialMon
//konfigurasyonlar
#define GSM_PIN ""
const char apn[] = "internet";

const char gprsUser[] = "";
const char gprsPass[] = "";
const char simPIN[] = "";

const char * broker = "raccoonscooter.com";// MQTT hostname
const char * mqttUsername = "raccoon"; // MQTT username
const char * mqttPassword = "Ze255Wer29tete/-"; // MQTT password
const char * mqtt_id = "831ac06b-709f-480b-89d4-c6fe333a6b49"; // MQTT id
const char * imei = "867372058971479"; //gps,battery gibi verilerin yollandıgı topic
const char * topic = "867372058971479/systemcontroller"; //kilit,restart gibi verilerin okundugu yer

#include <Wire.h>

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS#include <StreamDebugger.h>

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

// Sim modemi kurulumu
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define I2C_SDA 21
#define I2C_SCL 22

uint32_t lastReconnectAttempt = 0;

String lock_data = "";
String imeiString = "başarısız imei";
const int giris = 100;
float total = 0;
float average = 0;
float okunandeger[giris];
float deger = 0;
float volt = 0;
float yuzde = -1;

#define IP5306_ADDR 0x75
#define IP5306_REG_SYS_CTL0 0x00

long lastMsg = 0;

void mqttCallback(char * topic, byte * message, unsigned int length) {

    String messageTemp;
    //gelen mesajları okuma
    for (int i = 0; i < length; i++) {
        messageTemp += (char) message[i];
    }
    //kilit açma
    if (String(topic) == "867372058971479/systemcontroller") {
        if (messageTemp == "lockopen") {

            unsigned long eskiZaman = 0;
            unsigned long yeniZaman;
            yeniZaman = millis();

            if (yeniZaman - eskiZaman > 1000) {
                //kilit pinine yollanan değer
                for (int i = 0; i < 10; i++) {
                    digitalWrite(32, LOW);
                    delay(50);
                    digitalWrite(32, HIGH);
                    delay(50);
                }
                eskiZaman = yeniZaman;
            }
            //restart komutu
        } else if (messageTemp == "scooteropen") {
                       digitalWrite(33, LOW);

        }
        else if (messageTemp == "scooterclose") {
                    digitalWrite(33, HIGH);

        }
        else if (messageTemp == "restart") {
            lock_data = "kapalı";
            ESP.restart();
        }
    }
}

boolean mqttConnect() {

    boolean status = mqtt.connect(mqtt_id, mqttUsername, mqttPassword);

    if (status == false) {
        ESP.restart();
        return false;
    }
    mqtt.subscribe(topic);

    return mqtt.connected();
}

void setup() {
    SerialMon.begin(9600);
    delay(10);

    pinMode(MODEM_PWKEY, OUTPUT);
    pinMode(MODEM_RST, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);
    digitalWrite(MODEM_PWKEY, LOW);
    digitalWrite(MODEM_RST, HIGH);
    digitalWrite(MODEM_POWER_ON, HIGH);

    SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
    //kilit pini
    pinMode(32, OUTPUT);
    digitalWrite(32, HIGH);
    pinMode(33, OUTPUT);
    digitalWrite(33, HIGH);
    //kilit switch
    pinMode(15, INPUT);

    modem.restart();

    String modemInfo = modem.getModemInfo();

    if (GSM_PIN && modem.getSimStatus() != 3) {
        modem.simUnlock(GSM_PIN);
    }
    //imei okuma
    imeiString = modem.getIMEI();

    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        ESP.restart();
    } else {}

    if (modem.isGprsConnected()) {}

    mqtt.setServer(broker, 1883);
    mqtt.setCallback(mqttCallback);

    SerialGPS.begin(9600, SERIAL_8N1, 18, 19);

    while (!EEPROM.begin(EEPROM_SIZE)) {

    }

    long readValue;
    EEPROM_readAnything(0, readValue);
    gpsState.originLat = (double) readValue / 1000000;

    EEPROM_readAnything(4, readValue);
    gpsState.originLon = (double) readValue / 1000000;

    EEPROM_readAnything(8, readValue);
    gpsState.originAlt = (double) readValue / 1000000;
}
template < class T > int EEPROM_writeAnything(int ee,
    const T & value) {
    const byte * p = (const byte * )(const void * ) & value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        EEPROM.write(ee++, * p++);
    return i;
}

template < class T > int EEPROM_readAnything(int ee, T & value) {
    byte * p = (byte * )(void * ) & value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        *
        p++ = EEPROM.read(ee++);
    return i;
}
void loop() {
    //bağlantı yeniden oluşturuluyor
    if (!mqtt.connected()) {
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 1000) {
            lastReconnectAttempt = t;
            if (mqttConnect()) {
                lastReconnectAttempt = 0;
            }
        }
        return;
    }

    gpsState.originLat = gps.location.lat();
    gpsState.originLon = gps.location.lng();
    gpsState.originAlt = gps.altitude.meters();

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
    if (now - lastMsg > 200) {
        lastMsg = now;
        //batarya ölçme

        for (int i = 0; i < 100; i++) {
            deger = analogRead(34);
            //okunan değerin formülü
            volt = ((deger / 1202) * 345) / 15;

            okunandeger[i] = volt;
            total = total + okunandeger[i];

            average = total / giris;
        }
        total = 0;
        //pil konfigurasyonları yapılması
        yuzde = map(average, 31.1, 42.1, 0.0, 100.0);
        //kilidin durumunu kontrol eder
    if (digitalRead(15) == 1){
            lock_data = "kapali";
          }
    else{
            lock_data = "acik";
          }
        StaticJsonDocument < 512 > JSONbuffer;
        JsonObject GpsVerileri = JSONbuffer.createNestedObject();
        JsonObject BatteryData = JSONbuffer.createNestedObject();

        BatteryData["İMEİ"] = imeiString;
        BatteryData["BV"] = average;
        BatteryData["BP"] = yuzde;
        BatteryData["LS"] = lock_data;
        GpsVerileri["LAT"] = gps.location.lat();
        GpsVerileri["LONG"] = gps.location.lng();
        GpsVerileri["SP"] = gps.speed.kmph();
        GpsVerileri["ALT"] = gps.altitude.meters();

        char JSONmessageBuffer[200];
        serializeJsonPretty(JSONbuffer, JSONmessageBuffer);

        if (mqtt.subscribe(topic) == true) {
            //veri yollama başarılıysa
        }
        if (mqtt.publish(imei, JSONmessageBuffer) == true) {} else {}
        mqtt.loop();
    }
}