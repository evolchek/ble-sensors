#include "main.h"

void onSleeping();
State stateSleeping(NULL, &onSleeping, NULL);
void onReadingTempEnter();
void onReadingTemp();
void onReadingTempExit();
State stateReadingTemp(&onReadingTempEnter, &onReadingTemp, &onReadingTempExit);
void onAdvertisingEnter();
void onAdvertising();
void onAdvertisingExit();
State stateAdvertising(&onAdvertisingEnter, &onAdvertising, &onAdvertisingExit);
Fsm fsm(&stateReadingTemp);

Ticker advTimeoutTicker([]() { fsm.trigger(EVENT_ADV_TIMEOUT); }, advertiseFor, 1);

Thermistor* thermistor = new NTC_Thermistor(tPin, r0, tRn, tTn, tB, pow(2, adcResolution) - 1);

BLEShortCharacteristic characteristic("2A6E", BLERead);
BLEService service(bleServiceUuid);

double temperature = 0;
bool tempRead = false;

template<typename... Args> void debugLog(const char *format, Args... args) {
  if (!_debug) return;
  Serial.printf(format, args...);
  Serial.println();
}

_ushort tempReadingIteration = 0;

void onReadingTempEnter() {
  tempReadingIteration = avgReadings;
  temperature = 0;
  digitalWrite(vPin, HIGH);
}

void onReadingTemp() {
  temperature += (thermistor->readCelsius() / avgReadings);
  if (0 == --tempReadingIteration) {
    debugLog("Temperature: %f", temperature);
    fsm.trigger(EVENT_TEMP_READY);
  }
}

void onReadingTempExit() {
  digitalWrite(vPin, LOW);  
}

void onAdvertisingEnter() {
  debugLog("Starting BLE");
  while (!BLE.begin());
  if (_debug) BLE.debug(Serial);
  BLE.setLocalName(bleLocalName);
  BLE.setAppearance(bleAppearance.category | bleAppearance.subCategory);
  BLE.setAdvertisedService(service);
  characteristic.setEventHandler(
    BLERead,
    [](BLEDevice central, BLECharacteristic characteristic) {
        debugLog("Central %s read", central.address().c_str());
        if (central.address() == centralMac) tempRead = true;
      });
  service.addCharacteristic(characteristic);
  BLE.addService(service);
  characteristic.writeValue((short) round(temperature * 100));

  while (!BLE.advertise()) BLE.poll();
  advTimeoutTicker.start();
  debugLog("Advertising");
  if (_debug) {
    digitalWrite(LEDG, LOW);
  }
}

void onAdvertising() {
  if (tempRead) fsm.trigger(EVENT_CHARACTERISTIC_READ);
}

void onAdvertisingExit() {
  BLE.stopAdvertise();
  BLEDevice central = BLE.central();
  if (central && central.connected()) central.disconnect();
  BLE.end();
  debugLog("Advertising stopped");
  if (_debug) {
    digitalWrite(LEDG, HIGH);
  }
}

void onSleeping() {
  if (_debug) {
    digitalWrite(LEDB, LOW);
  }
  debugLog("Sleeping");
  delay((unsigned long) -1);
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

  analogReadResolution(adcResolution);

  pinMode(vPin, OUTPUT);
  digitalWrite(vPin, LOW);

  advTimeoutTicker.stop();

  fsm.add_transition(&stateReadingTemp, &stateAdvertising, EVENT_TEMP_READY, NULL);
  fsm.add_transition(&stateAdvertising, &stateSleeping, EVENT_ADV_TIMEOUT, NULL);
  fsm.add_transition(&stateAdvertising, &stateSleeping, EVENT_CHARACTERISTIC_READ, NULL);
  fsm.add_transition(&stateSleeping, &stateReadingTemp, EVENT_RESUMED, NULL);
}

void loop() {
  // reset watchdog
  nrf_wdt_reload_request_set(NRF_WDT_RR0);
  BLE.poll();
  advTimeoutTicker.update();
  fsm.run_machine();
}