// This sketch is designed for ESP32

#include <WiFi.h>
#include <esp_now.h>

#include <Arduino.h>

#include <map>
#include <sys/cdefs.h>
#include <string>


#define WIFI_CHANNEL 1 

#define DEBUG_LOG // Enable (uncomment) to print debug info. Disabling (comment) debug output saves some 4-5 ms ...

#define MAX_WAKETIME_MS 1000  // [ms]  Timeout until forced gotosleep if no sending success

#define SENSOR_CACHE_TIME 3000 // [ms] The time that data is cached before sending

#define VOLTAGE_CALC 4.21 / 2096 // A measurement of 2096 equals 4.21V
#define INTERRUPT_PIN 1
#define BATTERY_PIN 0
#define DEBOUNCE_TIME_MS 300
#define LITRES_PER_PULSE 0.5



uint8_t GatewayMac[] = {0x02, 0x10, 0x11, 0x12, 0x13, 0x14};

// -----------------------------------------------------------------------------------------
// GLOBALS
// -----------------------------------------------------------------------------------------

volatile boolean isMessageSent; 
volatile byte pulseCount = 0;
volatile uint lastSendTime = 0;
static uint lastInterruptTime = 0;


/// @brief Sets up the wifi/espnow config
void setupWifi()
{
    // Disable WiFi until we shall use it, to save energy
  WiFi.persistent(false); // Dont save WiFi info to Flash - to save time
  WiFi.mode(WIFI_OFF);    // Wifi OFF - during sensor reading - to save current/power
  
  WiFi.mode(WIFI_STA); // Station mode for esp-now sensor node
  WiFi.disconnect();

// Initialize ESP-now ----------------------------
  if (esp_now_init() != 0) {
    #ifdef DEBUG_LOG
    //Serial.println("*** ESP_Now init failed. Going to sleep");
    #endif
    delay(100);
    //gotoSleep();
  }

  esp_now_peer_info_t gateway = {};
  memcpy(gateway.peer_addr, GatewayMac, 6);
  gateway.channel = WIFI_CHANNEL;
  gateway.encrypt = false;            // no encryption
  esp_now_add_peer(&gateway);  

  esp_now_register_send_cb([](const uint8_t* mac, esp_now_send_status_t sendStatus)
    {
      // callback for message sent out
      isMessageSent = true; // flag message is sent out - we can now safely go to sleep ...
#ifdef DEBUG_LOG
      Serial.printf("Message sent out, sendStatus = %i\n", sendStatus);
#endif
    });
}


/// @brief Gets the battery data
float getBatteryVoltage()
{
  int pinValue = analogRead(BATTERY_PIN);
  return pinValue * VOLTAGE_CALC;
}

/// @brief THe ISR used to track pulses
void IRAM_ATTR handleInterrupt() {
  unsigned long interrupt_time = millis();
  // If interrupts come faster than DEBOUNCE_TIME_MS, assume it's a bounce and ignore
  if ((interrupt_time - lastInterruptTime) > DEBOUNCE_TIME_MS) {
    pulseCount++;
    lastInterruptTime = interrupt_time;
  }
}

/// @brief Gets the millis count since the last sent time, which considers that 
/// ESP8266 rolls over back to 0 when it reaches UINT_MAX
/// @return millis
uint getMillisSinceLastSendWithRollover(){
  uint currentTime = millis();

  if(currentTime < lastSendTime){ // detects to case of integer rollover
    return UINT_MAX - lastSendTime + currentTime; // calculate the time since last send, where UINT_MAX is the last int before rollover
  }
  else
    return currentTime - lastSendTime;
}


/// @brief Performs calculations on the data before sending them
/// to the gateway device
void calculateAndSendData()
{
  int litres = pulseCount * LITRES_PER_PULSE;

  String msg = "device=watermeter";
  msg += "|pulses=" + String(pulseCount);
  msg += "|litres=" + String(litres);
  msg += "|battery=" + String(getBatteryVoltage());

  uint32_t millisSinceLastSend = getMillisSinceLastSendWithRollover();
  if(litres == 0 || millisSinceLastSend == 0) {
    msg += "|flowrate=" + String(0);
  } else {
    msg += "|flowrate=" + String(litres * (60.0 / (millisSinceLastSend / 1000.0)));
  }

  Serial.println(msg);

  // Convert String to char array for ESP-NOW
  int msgLen = msg.length();
  char msgData[msgLen + 1];
  msg.toCharArray(msgData, msgLen + 1);

  uint16_t result = esp_now_send(NULL, (uint8_t *)msgData, msgLen);

  if (result == 0) {
    Serial.println("Message sent successfully");
  } else {
    Serial.print("Error sending message: ");
    Serial.println(result);
  }

  // reset counts
  pulseCount = 0;
  lastSendTime = millis();
}



/// @brief Configures the pins IO config
void setupIO()
{
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, FALLING);
}




void setup()
{
  //  struct rst_info *rstInfo = ESP.getResetInfoPtr();
  //  if(rstInfo->reason == REASON_DEEP_SLEEP_AWAKE){
  //   ulong currentTime = millis();
  //   if((currentTime - nextWakeTime) > 100){ // we assume this is caused by a trigger
  //     sensorData.litres++;
  //   }
  //  }

  setupWifi();
  setupIO();

#ifdef DEBUG_LOG
  Serial.begin(115200);
  while (!Serial)
  {
  };
  Serial.println("\n\nStart");
  Serial.printf("This device mac: %s", WiFi.macAddress().c_str());
  Serial.println("");
  Serial.printf("Gateway MAC: %02x:%02x:%02x:%02x:%02x:%02x", GatewayMac[0], GatewayMac[1], GatewayMac[2], GatewayMac[3], GatewayMac[4], GatewayMac[5]);
  Serial.printf(", on channel: %i\n", WIFI_CHANNEL);

#endif

  isMessageSent = false;
  lastSendTime = millis();
}




void loop()
{
  // Wait until ESP-Now message is sent, or timeout, then goto sleep
  if (isMessageSent || (millis() > MAX_WAKETIME_MS))
  {
    delay(SENSOR_CACHE_TIME);
    calculateAndSendData();
  }
}


