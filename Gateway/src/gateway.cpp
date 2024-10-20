#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <map>
#include <ArduinoJson.h>

// This is the MAC address to be installed (sensors shall then send to this MAC address)
uint8_t GatewayMac[] =      {0x02, 0x10, 0x11, 0x12, 0x13, 0x14};

#define WIFI_CHANNEL 1

// Function to split a string by a delimiter
void splitString(String data, char delimiter, String* &result, int &size) {
  size = 1;
  for (int i = 0; i < data.length(); i++) {
    if (data.charAt(i) == delimiter) {
      size++;
    }
  }
  
  result = new String[size];
  int index = 0;
  for (int i = 0; i < size; i++) {
    int nextDelimiter = data.indexOf(delimiter, index);
    if (nextDelimiter == -1) {
      result[i] = data.substring(index);
    } else {
      result[i] = data.substring(index, nextDelimiter);
      index = nextDelimiter + 1;
    }
  }
}

// Callback function when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  // Convert incoming data to a string
  String receivedMessage = "";
  for (int i = 0; i < len; i++) {
    receivedMessage += (char)incomingData[i];
  }

  // Print received message
  //Serial.print("Received message: ");
  //Serial.println(receivedMessage);

  // Split the received message into key-value pairs
  String* keyValuePairs;
  int numPairs;
  splitString(receivedMessage, '|', keyValuePairs, numPairs);

  // Create a JSON document
  JsonDocument doc;

  for (int i = 0; i < numPairs; i++) {
    String key, value;
    int separatorIndex = keyValuePairs[i].indexOf('=');
    if (separatorIndex != -1) {
      key = keyValuePairs[i].substring(0, separatorIndex);
      value = keyValuePairs[i].substring(separatorIndex + 1);
      
      // Add key and value to the JSON document
      doc[key] = value;
    }
  }

  // Free the allocated memory for keyValuePairs
  delete[] keyValuePairs;

  // Serialize the JSON document to a string
  String jsonString;
  serializeJson(doc, jsonString);

  // Print the JSON string
  //Serial.print("JSON string: ");
  Serial.println(jsonString);
}




// ------------------------------------------------------------------------------------
void setup()
// ------------------------------------------------------------------------------------
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite (LED_BUILTIN, !HIGH);              // Led ON    (Most often not working on ESP32)

  // Init Serial
  Serial.begin(115200);
  while (!Serial) {};
  delay(100);                                     // Needed for some boards
  Serial.println("\n\n");
  
  
  // Set device in AP mode to begin with
  WiFi.mode(WIFI_AP_STA);                         // AP _and_ STA is required (!IMPORTANT)
  WiFi.channel(WIFI_CHANNEL);
  wifi_set_macaddr(SOFTAP_IF, &GatewayMac[0]);          //8266 code

  // Come here - we are connected
  Serial.println(" Done");
   
  // Print MAC addresses
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());

  // Init ESP-Now 
  #define ESPNOW_SUCCESS 0

  if (esp_now_init() == ESPNOW_SUCCESS) {
    Serial.println("ESP - Now Init Success");
  } else {
    Serial.println("ESP - Now Init Failed");
    ESP.restart();                                // just restart if we cant init ESP-Now
  }
  
  // ESP-Now is now initialized. Register a callback fcn for when data is received
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
  
  digitalWrite (LED_BUILTIN, !LOW);               // Led OFF
}


void loop()
{
  // no work required - all work is done in the OnDataRecv callback from espnow
  // Serial.println("{\"device\":\"watermeter\",\"flowrate\":\"12\"}");
  // delay(5000);
}