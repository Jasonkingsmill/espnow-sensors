// This sketch is designed for ESP32

#include <WiFi.h>
#include <esp_now.h>

#include "driver/rtc_io.h"

#include <Arduino.h>

#include <map>
#include <sys/cdefs.h>
#include <string>


#define WIFI_CHANNEL 1
//#define DEBUG_LOG  // Enable (uncomment) to print debug info. Disabling (comment) debug output saves some 4-5 ms ...

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex

#define SLEEP_TIME_US 10000000    // 60 second sleep time
#define WAKE_TIME_MS 5000         // Time to stay awake since last detection
#define CACHE_TIME_MS 1500        // Time to cache sensor data before sending
#define VOLTAGE_CALC 4.21 / 2096  // A measurement of 2096 equals 4.21V
#define WAKEUP_GPIO GPIO_NUM_1
#define LED_PIN 15
#define BATTERY_PIN 0
#define LITRES_PER_PULSE 0.5
#define DEBOUNCE_TIME_MS 100

#define ESPINIT_FAILED 2
#define SEND_FAILED 3
#define SEND_CALLBACK_ERROR 4

RTC_DATA_ATTR unsigned long sleepStartedTime = 0;
RTC_DATA_ATTR unsigned long lastSendTime = 0;
RTC_DATA_ATTR boolean ignoreNextWakeTrigger = false;
volatile unsigned long lastPulseTime = 0;
volatile byte pulseCount = 0;
bool sendFinished = false;
bool sendFailed = false;
int sendError = 0;
bool wifiInitialized = false;

uint8_t GatewayMac[] = { 0x02, 0x10, 0x11, 0x12, 0x13, 0x14 };

// -----------------------------------------------------------------------------------------
// GLOBALS
// -----------------------------------------------------------------------------------------

/// @brief The ISR used to track pulses
void IRAM_ATTR handleInterrupt() {
  unsigned long time_now = millis();
  // If interrupts come faster than DEBOUNCE_TIME_MS, assume it's a bounce and ignore
  if ((time_now - lastPulseTime) > DEBOUNCE_TIME_MS) {
    pulseCount++;
    lastPulseTime = time_now;
  }
}

void flashError(int code, int suberror = 0) {
  if (code <= 0)
    return;
  for (int i = 0; i <= 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }
  delay(1000);
  for (int i = 0; i < code; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
  delay(1000);

  if (suberror > 0) {
    for (int i = 0; i < suberror; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(300);
      digitalWrite(LED_PIN, LOW);
      delay(300);
    }
  }

  for (int i = 0; i <= 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }
  digitalWrite(LED_PIN, HIGH);
}

/// @brief Sets up the wifi/espnow config
void setupWifi() {
  if (wifiInitialized)
    return;
  // Disable WiFi until we shall use it, to save energy
  //WiFi.persistent(false);  // Dont save WiFi info to Flash - to save time
  WiFi.mode(WIFI_STA);  // Station mode for esp-now sensor node
  WiFi.channel(WIFI_CHANNEL);
  WiFi.disconnect();
  delay(10);
  // Initialize ESP-now ----------------------------
  esp_err_t initResult = esp_now_init();
  if (initResult != 0) {
    flashError(ESPINIT_FAILED, initResult);
#ifdef DEBUG_LOG
    Serial.println("*** ESP_Now init failed.");
#endif
    return;
  }
  esp_now_register_send_cb([](const uint8_t* mac, esp_now_send_status_t sendStatus) {
    if (sendStatus == ESP_NOW_SEND_FAIL) {
      sendFailed = true;
      sendError = sendStatus;
    }
    sendFinished = true;
  });

  esp_now_peer_info_t gateway = {};
  memcpy(gateway.peer_addr, GatewayMac, 6);
  gateway.channel = 0;
  gateway.encrypt = false;  // no encryption
  esp_now_add_peer(&gateway);
  wifiInitialized = true;
}


/// @brief Gets the battery data
float getBatteryVoltage() {
  int pinValue = analogRead(BATTERY_PIN);
  return pinValue * VOLTAGE_CALC;
}

void sendData() {
  setupWifi();
  String msg = "device=watermeter";
  msg += "|litres=" + String(LITRES_PER_PULSE * pulseCount);
  msg += "|battery=" + String(getBatteryVoltage());
  unsigned long millisSinceLastSend = millis() - lastSendTime;
  if (lastSendTime == 0 || pulseCount == 0) {
    msg += "|flowrate=" + String(0);
  } else {
    msg += "|flowrate=" + String(LITRES_PER_PULSE * pulseCount * (60.0 / (millisSinceLastSend / 1000.0)));
  }

  lastSendTime = millis();
  pulseCount = 0;
#ifdef DEBUG_LOG
  Serial.println("sending message: " + msg);

#endif
  // Convert String to char array for ESP-NOW
  int msgLen = msg.length();
  char msgData[msgLen + 1];
  msg.toCharArray(msgData, msgLen + 1);

  for (int i = 0; i < 10; i++) {
    uint16_t result = esp_now_send(GatewayMac, (uint8_t*)msgData, msgLen);
    if (result != 0) {
      flashError(SEND_FAILED, result);
    }
    while (sendFinished == false) {
      delay(1);
    }
    if (!sendFailed) {
      break;
    }

    sendFailed = false;
    sendFinished = false;
  }
  if (sendFailed) {
    flashError(SEND_CALLBACK_ERROR, sendError);
  }
  sendFinished = false;
  sendFailed = false;
  sendError = 0;

  // if (result != 0) {
  //   flashError(result);
  // }
}

/// @brief Configures the pins IO config
void setupIO() {
  pinMode(WAKEUP_GPIO, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(WAKEUP_GPIO), handleInterrupt, FALLING);
}


void goToSleep(int currentState) {

  // Current state is low, invert
  if (currentState == 0) {
    esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(WAKEUP_GPIO), ESP_EXT1_WAKEUP_ANY_HIGH);
    ignoreNextWakeTrigger = true;
  } else if (currentState == 1) {
    esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(WAKEUP_GPIO), ESP_EXT1_WAKEUP_ANY_LOW);
  }

  rtc_gpio_pullup_en(WAKEUP_GPIO);
  rtc_gpio_pulldown_dis(WAKEUP_GPIO);
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);
  WiFi.disconnect();
  esp_now_unregister_send_cb();
  esp_now_deinit();
  sleepStartedTime = millis();
  esp_deep_sleep_start();
}

void resetCounters() {
  //sleepStartedTime = millis();
  pulseCount = 0;
  lastPulseTime = 0;
}

void handleReset() {

  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
  switch (wakeupCause) {
    case ESP_SLEEP_WAKEUP_EXT1:
#ifdef DEBUG_LOG
      delay(3000);
      Serial.println("Awake");
#endif
      if (ignoreNextWakeTrigger == true) {
        ignoreNextWakeTrigger = false;
        goToSleep(digitalRead(WAKEUP_GPIO));
      }
      lastPulseTime = millis();
      pulseCount++;
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      ignoreNextWakeTrigger = false;
#ifdef DEBUG_LOG
      delay(5000);
#endif
      sendData();  // send empty payload as a checkin
                   // delay(500);
      goToSleep(digitalRead(WAKEUP_GPIO));
      break;
    default:             // not sure here
      lastSendTime = 0;  // unknown why we started here so don't
#ifdef DEBUG_LOG
      Serial.begin(115200);
      Serial.println("\n\nStart");
      Serial.printf("This device mac: %s", WiFi.macAddress().c_str());
      Serial.println("");
      Serial.printf("Gateway MAC: %02x:%02x:%02x:%02x:%02x:%02x", GatewayMac[0], GatewayMac[1], GatewayMac[2], GatewayMac[3], GatewayMac[4], GatewayMac[5]);
      Serial.printf(", on channel: %i\n", WIFI_CHANNEL);
#endif
      sendData();  // send empty payload as a checkin
      delay(500);
      goToSleep(digitalRead(WAKEUP_GPIO));
      break;
  }
}

void setup() {
  setupIO();
  digitalWrite(LED_PIN, HIGH);  // Alert that "I'm awake" for visual diagnostics
  handleReset();
}




void loop() {
  // Loop while the last pulse is within the "wake" period
  while ((millis() - lastPulseTime) < WAKE_TIME_MS) {
    sendData();            // this will send a packet on the first loop, immediately after boot
    delay(CACHE_TIME_MS);  // Delay going to sleep so we "catch" more pulses while still awake
    //resetCounters();
  }
  digitalWrite(LED_PIN, LOW);

  goToSleep(digitalRead(WAKEUP_GPIO));
}
