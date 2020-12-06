#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <pgmspace.h>
#include <Fsm.h>
#include <Wire.h>
#include <Ticker.h>
#include <Types.h>
#include <DebugFlag.h>
#include "config.h"

enum Event {
  EVENT_GOT_IP,
  EVENT_WIFI_DISCONNECTED,
  EVENT_TIME_SYNCED,
  EVENT_MQTT_CONNECTED,
  EVENT_MQTT_DISCONNECTED,
  EVENT_DATA_RECEIVED,
  EVENT_DATA_SENT
  };