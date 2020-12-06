#include "main.h"

void onScanningEnter();
void onScanning();
void onScanningExit();
State stateScanning(&onScanningEnter, &onScanning, &onScanningExit);
void onConnected();
State stateConnected(NULL, &onConnected, NULL);
Fsm fsm(&stateScanning);

BLEDevice spaTemperature;

template<typename... Args> void debugLog(const char *format, Args... args) {
  if (!_debug) return;
  Serial.printf(format, args...);
  Serial.println();
}

void onScanningEnter() {
  debugLog("Scanning");
  if (_debug) digitalWrite(LEDG, LOW);
  while (!BLE.scanForUuid(spaTemperatureServiceUuid)) BLE.poll();
}

void onScanning() {
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    BLE.stopScan();
    debugLog("Connecting");
    if (peripheral.connect()) {
      debugLog("Connected");
      spaTemperature = peripheral;
      fsm.trigger(EVENT_CONNECTED);
    } else {
      debugLog("Failed to connect!");
      while (!BLE.scanForUuid(spaTemperatureServiceUuid)) BLE.poll();
    }
  }
}

void onScanningExit() {
  if (_debug) digitalWrite(LEDG, HIGH);
}

void onConnected() {
  BLE.poll();
  debugLog("Discovering attributes ...");
  if (spaTemperature.discoverAttributes()) {
    debugLog("Attributes discovered");
  } else {
    debugLog("Attribute discovery failed!");
    fsm.trigger(EVENT_READ_FAILURE);
    return;
  }

  BLECharacteristic characterisic = spaTemperature.characteristic("2A6E");

  if (characterisic) {
    short temp = 0;
    int bytesRead = characterisic.readValue(temp);
    if (bytesRead < 1) {
      debugLog("Bytes read: %d", bytesRead);
      fsm.trigger(EVENT_READ_FAILURE);
      return;
    }
    debugLog("Temperature: %f", (float) temp / 100);
    static _uint iteration = 0;
    debugLog("Iteration: %d", ++iteration);

    // reset watchdog
    nrf_wdt_reload_request_set(NRF_WDT_RR0);

    for (_uchar i = 5; i > 0; i--) {
      debugLog("Sending data to comms");
      Wire.beginTransmission(i2cCommsAddress);
      Wire.write((temp >> 8) & 0xFF);
      Wire.write(temp & 0xFF);
      Wire.write((iteration >> 24) & 0xFF);
      Wire.write((iteration >> 16) & 0xFF);
      Wire.write((iteration >> 8) & 0xFF);
      Wire.write(iteration & 0xFF);
      _uchar result = Wire.endTransmission();
      if (0 != result) debugLog("Error sending data to comms. Code: %d", result);
      if (result <= 2) break;
    }

  } else {
    debugLog("Peripheral does NOT have temperature characteristic");
    fsm.trigger(EVENT_READ_FAILURE);
    return;
  }

  if (spaTemperature.connected()) spaTemperature.disconnect();
  fsm.trigger(EVENT_DISCONNECTED);
}

void setup() {
  if (_debug) {
    Serial.begin(9600);
    unsigned long startedWaiting = millis();
    while(!Serial && (millis() - startedWaiting < 5000)) delay(100);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(LEDR, OUTPUT);
    digitalWrite(LEDR, HIGH);
    pinMode(LEDG, OUTPUT);
    digitalWrite(LEDG, HIGH);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDB, HIGH);
  }

  pinMode(wPin, INPUT_PULLUP);
  // disable watchdog is wPin is grounded
  if (digitalRead(wPin) == HIGH) {
    // configure watchdog
    nrf_wdt_behaviour_set(NRF_WDT_BEHAVIOUR_RUN_SLEEP);
    nrf_wdt_reload_value_set((watchdogTimeout * 32768) / 1000);
    nrf_wdt_reload_request_enable(NRF_WDT_RR0);
    nrf_wdt_task_trigger(NRF_WDT_TASK_START);
  }

  fsm.add_transition(&stateScanning, &stateConnected, EVENT_CONNECTED, NULL);
  fsm.add_transition(&stateConnected, &stateScanning, EVENT_READ_FAILURE, NULL);
  fsm.add_transition(&stateConnected, &stateScanning, EVENT_DISCONNECTED, NULL);

  while (!BLE.begin());
  if (_debug) BLE.debug(Serial);

  debugLog("MAC: %s", BLE.address().c_str());

  Wire.begin();
}

void loop() {
  fsm.run_machine();
}