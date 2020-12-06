const _uchar tPin = A0; // thermistor pin
const _uchar vPin = D2; // voltage pin
const _uchar wPin = D3; // watchdog disable pin
const _uint r0 = 10000; // voltage divider resistor

const _uint tRn = 10000; // thermistor nominal resistance
const _uchar tTn = 25; // thermistor nominal temperature
const _ushort tB = 3950; // thermistor B parameter

const _uchar adcResolution = 12; // board ADC resolution
const _ushort avgReadings = 100; // number of temperature readings to average

const struct {
  _ushort category;
  _uchar subCategory;
} bleAppearance = {
  .category = 1344, // Generic sensor
  .subCategory = 3 // Temperature sensor
};
const char bleLocalName[] = "...";
const char bleServiceUuid[] = "...";

const _uint watchdogTimeout = 5000;
const _uint sleepFor = 1;
const _ushort advertiseFor = 10000;

const char centralMac[] = "...";