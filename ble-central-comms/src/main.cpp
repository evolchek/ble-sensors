#include "main.h"

void onWifiConnecting();
State stateWifiConnecting(NULL, &onWifiConnecting, NULL);
void onTimeSyncing();
State stateTimeSynching(NULL, &onTimeSyncing, NULL);
void onMqttConnecting();
State stateMqttConnecting(NULL, &onMqttConnecting, NULL);
void onIdleEnter();
void onIdle();
void onIdleExit();
State stateIdle(&onIdleEnter, &onIdle, &onIdleExit);
void onSendingData();
State stateSendingData(NULL, &onSendingData, NULL);
Fsm fsm(&stateWifiConnecting);

WiFiClientSecure net;

BearSSL::X509List caCert(caCertStr);
BearSSL::X509List clientCert(clientCertStr);
BearSSL::PrivateKey privKey(privKeyStr);

PubSubClient client(net);

struct {
  struct {
    float temp;
    _uint iteration;
  } spaTemperature;
} sensorData = {
  .spaTemperature = {
    .temp = 0,
    .iteration = 0
  }
};

template<typename... Args> void debugLog(const char *format, Args... args) {
  if (!_debug) return;
  Serial.printf(format, args...);
  Serial.println();
}

void onWifiConnecting() {
  // debugLog("%d", WiFi.status());
}

void onTimeSyncing() {
  const time_t pastTimestamp = 1607144400; // December 5, 2020
  static unsigned long lastTry = 0;
  if (millis() - lastTry > 10000 && time(nullptr) < pastTimestamp) {
    debugLog("Time synchronization");
    configTime(0, 0, "0.ca.pool.ntp.org", "1.ca.pool.ntp.org", "2.ca.pool.ntp.org");
    lastTry = millis();
  } else if (time(nullptr) > pastTimestamp) fsm.trigger(EVENT_TIME_SYNCED);
}

void pubSubErr(_char MQTTErr){
  if (MQTTErr == MQTT_CONNECTION_TIMEOUT)
    debugLog("Connection tiemout");
  else if (MQTTErr == MQTT_CONNECTION_LOST)
    debugLog("Connection lost");
  else if (MQTTErr == MQTT_CONNECT_FAILED)
    debugLog("Connect failed");
  else if (MQTTErr == MQTT_DISCONNECTED)
    debugLog("Disconnected");
  else if (MQTTErr == MQTT_CONNECTED)
    debugLog("Connected");
  else if (MQTTErr == MQTT_CONNECT_BAD_PROTOCOL)
    debugLog("Connect bad protocol");
  else if (MQTTErr == MQTT_CONNECT_BAD_CLIENT_ID)
    debugLog("Connect bad Client-ID");
  else if (MQTTErr == MQTT_CONNECT_UNAVAILABLE)
    debugLog("Connect unavailable");
  else if (MQTTErr == MQTT_CONNECT_BAD_CREDENTIALS)
    debugLog("Connect bad credentials");
  else if (MQTTErr == MQTT_CONNECT_UNAUTHORIZED)
    debugLog("Connect unauthorized");
}

void onMqttConnecting() {
  static unsigned long lastTry = 0;
  if (millis() - lastTry < 10000) return;
  debugLog("Connecting to AWS IoT");
  if (client.connect(THINGNAME)) {
    // debugLog("Subscribing");
    // if (client.subscribe(mqttSubTopic)) fsm.trigger(EVENT_MQTT_CONNECTED);
    // else pubSubErr(client.state());
    fsm.trigger(EVENT_MQTT_CONNECTED);
  } else pubSubErr(client.state());
  lastTry = millis();
}

void onSendingData() {
  StaticJsonDocument<3 * JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(2)> doc;
  JsonObject state = doc.createNestedObject("state");
  JsonObject reported = state.createNestedObject("reported");
  JsonObject spaTemperature = reported.createNestedObject("spaTemperature");
  spaTemperature["temp"] = sensorData.spaTemperature.temp;
  spaTemperature["iteration"] = sensorData.spaTemperature.iteration;
  char json[measureJson(doc) + 1];
  serializeJson(doc, json, sizeof(json));
  debugLog("Payload: %s", json);
  if (!client.publish(mqttPubTopic, json)) pubSubErr(client.state());
  fsm.trigger(EVENT_DATA_SENT);
}

Ticker mockDataReceived([]() -> void {
  debugLog("Mock data received event");
  sensorData.spaTemperature.temp = 123.45;
  sensorData.spaTemperature.iteration = 678;
  fsm.trigger(EVENT_DATA_RECEIVED);
}, 5000, 1);

void onIdleEnter() {
  if (_debug) {
    digitalWrite(LED_BUILTIN, LOW);
  }
  mockDataReceived.start();
}

void onIdle() {
  if (client.connected()) client.loop();
  else fsm.trigger(EVENT_MQTT_DISCONNECTED);
}

void onIdleExit() {
  if (_debug) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

WiFiEventHandler wifiConnectedHandler, gotIpHandler, wifiDisconnectedHandler, dhcpTimeoutHandler;

void setup() {
  if (_debug) {
    Serial.begin(76800);
    unsigned long startedWaiting = millis();
    while(!Serial && (millis() - startedWaiting < 5000)) delay(100);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  fsm.add_transition(&stateWifiConnecting, &stateTimeSynching, EVENT_GOT_IP, NULL);
  fsm.add_transition(&stateTimeSynching, &stateWifiConnecting, EVENT_WIFI_DISCONNECTED, NULL);
  fsm.add_transition(&stateTimeSynching, &stateMqttConnecting, EVENT_TIME_SYNCED, NULL);
  fsm.add_transition(&stateMqttConnecting, &stateIdle, EVENT_MQTT_CONNECTED, NULL);
  fsm.add_transition(&stateMqttConnecting, &stateWifiConnecting, EVENT_WIFI_DISCONNECTED, NULL);
  fsm.add_transition(&stateIdle, &stateSendingData, EVENT_DATA_RECEIVED, NULL);
  fsm.add_transition(&stateIdle, &stateWifiConnecting, EVENT_WIFI_DISCONNECTED, NULL);
  fsm.add_transition(&stateIdle, &stateMqttConnecting, EVENT_MQTT_DISCONNECTED, NULL);
  fsm.add_transition(&stateSendingData, &stateIdle, EVENT_DATA_SENT, NULL);
  fsm.add_transition(&stateSendingData, &stateWifiConnecting, EVENT_WIFI_DISCONNECTED, NULL);
  fsm.add_transition(&stateSendingData, &stateMqttConnecting, EVENT_MQTT_DISCONNECTED, NULL);

  debugLog("Starting WiFi");
  wifiConnectedHandler = WiFi.onStationModeConnected([](const WiFiEventStationModeConnected &event) {
    debugLog("WiFi connected. Channel: %d", event.channel);
  });
  gotIpHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP &event) {
    debugLog("Got IP: %s", event.ip.toString().c_str());
    fsm.trigger(EVENT_GOT_IP);
  });
  wifiDisconnectedHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected &event) {
    debugLog("WiFi disconnected");
    fsm.trigger(EVENT_WIFI_DISCONNECTED);
  });
  dhcpTimeoutHandler = WiFi.onStationModeDHCPTimeout([]() {
    debugLog("DHCP Timeout. Reconnecting");
    WiFi.reconnect();
  });
  WiFi.hostname(THINGNAME);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pass);

  Wire.begin(i2cCommsAddress);

  net.setTrustAnchors(&caCert);
  net.setClientRSACert(&clientCert, &privKey);

  client.setServer(mqttHost, mqttPort);

  Wire.onReceive([](size_t bytesReceived) -> void {
    if (6 != bytesReceived) {
      debugLog("Wrong number of bytes received: %d", bytesReceived);
      return;
    }
    _short temp = 0;
    _uint iteration = 0;
    _uchar data[6];
    for (_uchar i = 0; i < 6; i++) {
      if (Wire.peek() == -1) {
        debugLog("Wire read failure");
        return;
      }
      data[i] = Wire.read();
    }
    temp |= (data[0] << 8);
    temp |= data[1];
    sensorData.spaTemperature.temp = (float) temp / 100;
    iteration |= (data[2] << 24);
    iteration |= (data[3] << 16);
    iteration |= (data[4] << 8);
    iteration |= data[5];
    sensorData.spaTemperature.iteration = iteration;
    fsm.trigger(EVENT_DATA_RECEIVED);
  });
}

void loop() {
  mockDataReceived.update();
  fsm.run_machine();
}