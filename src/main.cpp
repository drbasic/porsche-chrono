#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <TinyGPSPlus.h>
#include "driver/twai.h"

constexpr gpio_num_t CAN_TX = GPIO_NUM_17;
constexpr gpio_num_t CAN_RX = GPIO_NUM_16;
constexpr uint8_t BUTTON_H_PLUS = 14;
constexpr uint8_t BUTTON_LED_PLUS = 12;
constexpr uint8_t GPS_RX = 4;
constexpr uint8_t GPS_TX = 13;
constexpr uint8_t STATUS_LED = 2;

RTC_DS3231 rtc;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
bool rtcAvailable = false;
int32_t softwareTimeOffsetMinutes = 0;
uint32_t softwareClockLastMillis = 0;
uint64_t softwareClockMillis = 0;
uint8_t brightness = 0;
uint32_t last3C0 = 0;
uint32_t last6B6 = 0;
uint32_t lastButtonScan = 0;
uint32_t lastTimeStatus = 0;
uint32_t lastGpsAdjustment = 0;
uint8_t canLedPulsesRemaining = 0;
uint32_t canLedOffAt = 0;
uint32_t canLedNextPulseAt = 0;
uint32_t lastGpsLedReport = 0;
uint32_t gpsLedOffAt = 0;
uint32_t gpsLedNextPulseAt = 0;
uint8_t gpsLedLongRemaining = 0;
uint8_t gpsLedShortRemaining = 0;
bool previousButtonState[2] = {};
uint32_t buttonPressStarted[2] = {};
bool buttonLongHandled[2] = {};
bool clockStopped = false;

const uint8_t buttonPins[2] = {BUTTON_H_PLUS, BUTTON_LED_PLUS};

void printHexByte(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

void printData(const uint8_t *data, uint8_t length) {
  for (uint8_t i = 0; i < length; ++i) {
    if (i) Serial.print(' ');
    printHexByte(data[i]);
  }
}

bool sendCan(uint16_t id, const uint8_t *data, uint8_t length) {
  twai_message_t message = {};
  message.identifier = id;
  message.data_length_code = length;
  memcpy(message.data, data, length);
  const bool success = twai_transmit(&message, pdMS_TO_TICKS(20)) == ESP_OK;
  canLedPulsesRemaining = success ? 0 : 1;
  canLedNextPulseAt = millis();
  return success;
}

void send3C0() {
  const uint8_t data[4] = {0x00, 0x02, 0x03, 0x00};
  sendCan(0x3C0, data, 4);
}

DateTime currentTime() {
  if (rtcAvailable) return rtc.now();

  const uint32_t now = millis();
  softwareClockMillis += static_cast<uint32_t>(now - softwareClockLastMillis);
  softwareClockLastMillis = now;
  const int64_t totalSeconds = static_cast<int64_t>(softwareClockMillis / 1000) +
                               static_cast<int64_t>(softwareTimeOffsetMinutes) * 60;
  int64_t daySeconds = totalSeconds % 86400;
  if (daySeconds < 0) daySeconds += 86400;
  return DateTime(2000, 1, 1, daySeconds / 3600,
                  (daySeconds / 60) % 60, daySeconds % 60);
}

void send6B6Time() {
  const DateTime now = currentTime();
  const uint8_t data[8] = {
      0x00, 0x00, 0x00,
      now.hour(), now.minute(), now.second(), 0x00, 0x00};
  sendCan(0x6B6, data, 8);
}

void printTimeStatus() {
  const DateTime now = currentTime();
  Serial.printf("TIME=%02d:%02d:%02d SOURCE=%s\n",
                now.hour(), now.minute(), now.second(),
                rtcAvailable ? "DS3231" : "ESP32 SOFTWARE CLOCK");

  Serial.print("GPS DATA: satellites=");
  if (gps.satellites.isValid()) {
    Serial.print(gps.satellites.value());
  } else {
    Serial.print("N/A");
  }

  Serial.print(" coordinates=");
  if (gps.location.isValid()) {
    Serial.printf("%.6f,%.6f", gps.location.lat(), gps.location.lng());
  } else {
    Serial.print("N/A");
  }

  Serial.print(" utc=");
  if (gps.date.isValid() && gps.time.isValid()) {
    Serial.printf("%04d-%02d-%02d %02d:%02d:%02d",
                  gps.date.year(), gps.date.month(), gps.date.day(),
                  gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    Serial.print("N/A");
  }

  Serial.print(" GPS_RTC_DIFF=");
  if (rtcAvailable && gps.time.isValid() &&
      gps.time.minute() <= 59 && gps.time.second() <= 59) {
    const int rtcSeconds = now.minute() * 60 + now.second();
    const int gpsSeconds = gps.time.minute() * 60 + gps.time.second();
    int difference = gpsSeconds - rtcSeconds;
    if (difference > 1800) difference -= 3600;
    if (difference < -1800) difference += 3600;
    Serial.printf("%+d s", difference);
  } else {
    Serial.print("N/A");
  }

  Serial.print(" GPS_SYNC_AGE=");
  if (lastGpsAdjustment != 0) {
    Serial.printf("%lu s", static_cast<unsigned long>(
        (millis() - lastGpsAdjustment) / 1000));
  } else {
    Serial.print("N/A");
  }
  Serial.println();
}

void send6BDLight() {
  const uint8_t data[8] = {
      brightness, brightness, brightness, brightness,
      brightness, brightness, brightness, brightness};
  if (sendCan(0x6BD, data, 8)) {
    Serial.print("ID=0x6BD DATA=");
    printData(data, 8);
    Serial.println();
  }
}

void send6BDOff() {
  const uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  sendCan(0x6BD, data, 8);
}

void stopClock() {
  if (clockStopped) return;
  send6BDOff();
  clockStopped = true;
  Serial.println("CLOCK STOPPED");
}

void resumeClock() {
  if (!clockStopped) return;
  clockStopped = false;
  send6BDLight();
  Serial.println("CLOCK RESUMED");
}

void adjustTime(int8_t hours, int8_t minutes) {
  if (!rtcAvailable) {
    softwareTimeOffsetMinutes =
        (softwareTimeOffsetMinutes + hours * 60 + minutes) % (24 * 60);
    return;
  }
  DateTime now = rtc.now();
  int32_t totalMinutes = static_cast<int32_t>(now.hour()) * 60 + now.minute();
  totalMinutes += hours * 60 + minutes;
  while (totalMinutes < 0) totalMinutes += 24 * 60;
  totalMinutes %= 24 * 60;
  rtc.adjust(DateTime(now.year(), now.month(), now.day(),
                      totalMinutes / 60, totalMinutes % 60, now.second()));
}

void handleButtons() {
  if (millis() - lastButtonScan < 30) return;
  lastButtonScan = millis();

  for (uint8_t i = 0; i < 2; ++i) {
    const bool pressed = digitalRead(buttonPins[i]) == LOW;
    if (pressed && !previousButtonState[i]) {
      buttonPressStarted[i] = millis();
      buttonLongHandled[i] = false;
      const bool wasClockStopped = clockStopped;
      if (wasClockStopped) resumeClock();
      if (i == 0 && !wasClockStopped) adjustTime(1, 0);
      if (i == 0) send6B6Time();
    }

    if (pressed && i == 1 && !buttonLongHandled[i] &&
        millis() - buttonPressStarted[i] > 1000) {
      buttonLongHandled[i] = true;
      stopClock();
    }

    if (!pressed && previousButtonState[i] && i == 1 &&
        !buttonLongHandled[i]) {
      brightness = brightness < 100 ? brightness + 1 : 0;
      send6BDLight();
    }
    previousButtonState[i] = pressed;
  }
}

void handleGps() {
  while (gpsSerial.available() > 0) {
    const char c = static_cast<char>(gpsSerial.read());
    gps.encode(c);
  }

  const bool validGpsDateTime =
      gps.date.isValid() && gps.time.isValid() &&
      gps.date.age() < 1500 && gps.time.age() < 1500 &&
      gps.time.isUpdated() &&
      gps.date.year() >= 2020 && gps.date.month() >= 1 &&
      gps.date.month() <= 12 && gps.date.day() >= 1 &&
      gps.date.day() <= 31 && gps.time.hour() <= 23 &&
      gps.time.minute() <= 59 && gps.time.second() <= 59;

  if (!rtcAvailable || !validGpsDateTime ||
      millis() - lastGpsAdjustment < 1000) {
    return;
  }

  const DateTime now = rtc.now();
  const int rtcSeconds = now.minute() * 60 + now.second();
  const int gpsSeconds = gps.time.minute() * 60 + gps.time.second();
  int difference = gpsSeconds - rtcSeconds;
  if (difference > 1800) difference -= 3600;
  if (difference < -1800) difference += 3600;

  const bool hourElapsed = lastGpsAdjustment == 0 ||
      millis() - lastGpsAdjustment >= 3600000UL;
  if (!hourElapsed && abs(difference) <= 2) {
    return;
  }

  // Keep the manually selected hour offset while carrying across hour/day boundaries.
  rtc.adjust(now + TimeSpan(difference));
  lastGpsAdjustment = millis();
  Serial.printf("RTC SYNCED BY GPS: %02d:%02d:%02d UTC\n",
                gps.time.hour(), gps.time.minute(), gps.time.second());
}

void handleGpsLed() {
  const uint32_t now = millis();

  if (canLedOffAt != 0 && static_cast<int32_t>(now - canLedOffAt) >= 0) {
    digitalWrite(STATUS_LED, LOW);
    canLedOffAt = 0;
    canLedNextPulseAt = now + 120;
  }

  if (canLedOffAt == 0 && canLedPulsesRemaining != 0 &&
      static_cast<int32_t>(now - canLedNextPulseAt) >= 0) {
    digitalWrite(STATUS_LED, HIGH);
    canLedOffAt = now + 50;
    --canLedPulsesRemaining;
  }

  if (canLedOffAt != 0 || canLedPulsesRemaining != 0) return;

  if (gpsLedOffAt != 0 && static_cast<int32_t>(now - gpsLedOffAt) >= 0) {
    digitalWrite(STATUS_LED, LOW);
    gpsLedOffAt = 0;
    gpsLedNextPulseAt = now + 120;
  }

  if (gpsLedOffAt == 0 &&
      (gpsLedLongRemaining != 0 || gpsLedShortRemaining != 0) &&
      static_cast<int32_t>(now - gpsLedNextPulseAt) >= 0) {
    const bool longPulse = gpsLedLongRemaining != 0;
    digitalWrite(STATUS_LED, HIGH);
    gpsLedOffAt = now + (longPulse ? 500 : 120);
    if (longPulse) {
      --gpsLedLongRemaining;
    } else {
      --gpsLedShortRemaining;
    }
  }

  if (now - lastGpsLedReport >= 10000 && gpsLedOffAt == 0 &&
      gpsLedLongRemaining == 0 && gpsLedShortRemaining == 0) {
    lastGpsLedReport = now;
    const uint32_t satellites = gps.satellites.isValid()
        ? gps.satellites.value() : 0;
    if (satellites == 0 && gps.date.isValid() && gps.time.isValid()) {
      gpsLedShortRemaining = 1;
    } else if (satellites != 0) {
      gpsLedLongRemaining = satellites / 5;
      gpsLedShortRemaining = satellites % 5;
    }
    gpsLedNextPulseAt = now;
  }
}

void setupCan() {
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&general, &timing, &filter) != ESP_OK ||
      twai_start() != ESP_OK) {
    Serial.println("TWAI INIT=ERROR");
    return;
  }
  Serial.println("TWAI INIT=OK 500 kbps TX=GPIO17 RX=GPIO16");
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  Wire.begin(21, 22);
  for (uint8_t pin : buttonPins) {
    pinMode(pin, INPUT_PULLUP);
  }

  if (!rtc.begin()) {
    Serial.println("DS3231 INIT=ERROR");
  } else {
    rtcAvailable = true;
    Serial.println("DS3231 INIT=OK");
    if (rtc.lostPower()) {
      Serial.println("DS3231 WARNING: CLOCK NOT SET / LOST POWER");
    }
  }

  setupCan();
  const uint32_t now = millis();
  last3C0 = now - 1000;
  last6B6 = now - 1000;
  lastTimeStatus = now - 5000;
}

void loop() {
  handleGps();
  handleGpsLed();
  handleButtons();
  const uint32_t now = millis();

  if (now - last3C0 >= 1000) {
    last3C0 = now;
    send3C0();
  }
  if (!clockStopped && now - last6B6 >= 1000) {
    last6B6 = now;
    send6B6Time();
  }

  if (now - lastTimeStatus >= 5000) {
    lastTimeStatus = now;
    printTimeStatus();
  }
}
