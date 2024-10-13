// This sketch is designed for ESP32

#include <WiFi.h>
#include <esp_now.h>

#include "driver/rtc_io.h"

#include <Arduino.h>

#include <map>
#include <sys/cdefs.h>
#include <string>


#define WIFI_CHANNEL 1
#define DEBUG_LOG  // Enable (uncomment) to print debug info. Disabling (comment) debug output saves some 4-5 ms ...

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex

#define SLEEP_TIME_US 10000000    // 60 second sleep time
#define WAKE_TIME_MS 5000         // Time to stay awake since last detection
#define CACHE_TIME_MS 1500        // Time to cache sensor data before sending
#define VOLTAGE_CALC 4.21 / 2096  // A measurement of 2096 equals 4.21V
#define WAKEUP_GPIO GPIO_NUM_1
#define LED_PIN 15
#define BATTERY_PIN 0
#define LITRES_PER_PULSE 0.5
#define DEBOUNCE_TIME_MS 200

RTC_DATA_ATTR unsigned long sleepStartedTime = 0;
volatile unsigned long lastInterruptTime = 0;
volatile byte pulseCount = 0;


uint8_t GatewayMac[] = { 0x02, 0x10, 0x11, 0x12, 0x13, 0x14 };

// -----------------------------------------------------------------------------------------
// GLOBALS
// -----------------------------------------------------------------------------------------

/// @brief The ISR used to track pulses
void IRAM_ATTR handleInterrupt() {
  unsigned long interrupt_time = millis();
  // If interrupts come faster than DEBOUNCE_TIME_MS, assume it's a bounce and ignore
  if ((interrupt_time - lastInterruptTime) > DEBOUNCE_TIME_MS) {
    pulseCount++;
    lastInterruptTime = interrupt_time;
  }
}

/// @brief Sets up the wifi/espnow config
void setupWifi() {
  // Disable WiFi until we shall use it, to save energy
  WiFi.persistent(false);  // Dont save WiFi info to Flash - to save time
  //WiFi.mode(WIFI_OFF);     // Wifi OFF - during sensor reading - to save current/power

  WiFi.mode(WIFI_STA);  // Station mode for esp-now sensor node
  //WiFi.disconnect();

  // Initialize ESP-now ----------------------------
  if (esp_now_init() != 0) {
#ifdef DEBUG_LOG
    Serial.println("*** ESP_Now init failed.");
#endif
    return;
  }
  esp_now_register_send_cb([](const uint8_t* mac, esp_now_send_status_t sendStatus) {
#ifdef DEBUG_LOG
    Serial.printf("Message sent out, sendStatus = %i\n", sendStatus);
#endif
  });
  esp_now_peer_info_t gateway = {};
  memcpy(gateway.peer_addr, GatewayMac, 6);
  gateway.channel = WIFI_CHANNEL;
  gateway.encrypt = false;  // no encryption
  esp_now_add_peer(&gateway);
}


/// @brief Gets the battery data
float getBatteryVoltage() {
  int pinValue = analogRead(BATTERY_PIN);
  return pinValue * VOLTAGE_CALC;
}

void sendData(long pulses) {
  setupWifi();
  String msg = "device=watermeter";
  msg += "|litres=" + String(LITRES_PER_PULSE * pulses);
  msg += "|battery=" + String(getBatteryVoltage());
  unsigned long millisSinceLastSend = millis() - sleepStartedTime;
  if (millisSinceLastSend == 0 || pulses == 0) {
    msg += "|flowrate=" + String(0);
  } else {
    msg += "|flowrate=" + String(LITRES_PER_PULSE * pulses * (60.0 / (millisSinceLastSend / 1000.0)));
  }

  //Serial.println(msg);

  // Convert String to char array for ESP-NOW
  int msgLen = msg.length();
  char msgData[msgLen + 1];
  msg.toCharArray(msgData, msgLen + 1);

  uint16_t result = esp_now_send(NULL, (uint8_t*)msgData, msgLen);

  // if (result == 0) {
  //   Serial.println("Message sent successfully");
  // } else {
  //   Serial.print("Error sending message: ");
  //   Serial.println(result);
  // }
  delay(200);
}

/// @brief Configures the pins IO config
void setupIO() {
  pinMode(WAKEUP_GPIO, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(WAKEUP_GPIO), handleInterrupt, FALLING);
}


void goToSleep() {
  // enable the wakeup source
  esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(WAKEUP_GPIO), ESP_EXT1_WAKEUP_ANY_LOW);
  rtc_gpio_pullup_en(WAKEUP_GPIO);
  rtc_gpio_pulldown_dis(WAKEUP_GPIO);
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);
  sleepStartedTime = millis();
  esp_deep_sleep_start();
}

void resetCounters() {
  sleepStartedTime = millis();
  pulseCount = 0;
  lastInterruptTime = 0;
}

void handleReset() {
  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
  switch (wakeupCause) {
    case ESP_SLEEP_WAKEUP_EXT1:
      lastInterruptTime = millis();
      pulseCount++;
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      sendData(0);  // send empty payload as a checkin
      goToSleep();
      break;
    default:        // not sure here
      sendData(0);  // send empty payload as a checkin
      goToSleep();
      break;
  }
}

void setup() {
  setupIO();
  digitalWrite(LED_PIN, HIGH);  // Alert that "I'm awake" for visual diagnostics
  // #ifdef DEBUG_LOG

  //   Serial.begin(115200);
  //   Serial.println("\n\nStart");
  //   Serial.printf("This device mac: %s", WiFi.macAddress().c_str());
  //   Serial.println("");
  //   Serial.printf("Gateway MAC: %02x:%02x:%02x:%02x:%02x:%02x", GatewayMac[0], GatewayMac[1], GatewayMac[2], GatewayMac[3], GatewayMac[4], GatewayMac[5]);
  //   Serial.printf(", on channel: %i\n", WIFI_CHANNEL);
  // #endif
  handleReset();
}




void loop() {
  while ((millis() - lastInterruptTime) < WAKE_TIME_MS) {
    delay(CACHE_TIME_MS);
    sendData(pulseCount);
    resetCounters();
  }
  digitalWrite(LED_PIN, LOW);
  goToSleep();
}
