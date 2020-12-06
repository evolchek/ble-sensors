#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Fsm.h>
#include <nrf_wdt.h>
#include <Wire.h>
#include <Types.h>
#include <DebugFlag.h>
#include "config.h"

enum Event {
  EVENT_ERROR,
  EVENT_CONNECTED,
  EVENT_READ_FAILURE,
  EVENT_DISCONNECTED
  };