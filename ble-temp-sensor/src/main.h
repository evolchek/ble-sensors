#include <Arduino.h>
#include <NTC_Thermistor.h>
#include <ArduinoBLE.h>
#include <Fsm.h>
#include <Ticker.h>
#include <nrf_wdt.h>
#include <Types.h>
#include <DebugFlag.h>
#include "config.h"

enum Event {
  EVENT_TEMP_READY,
  EVENT_ADV_TIMEOUT,
  EVENT_RESUMED,
  EVENT_CHARACTERISTIC_READ
  };