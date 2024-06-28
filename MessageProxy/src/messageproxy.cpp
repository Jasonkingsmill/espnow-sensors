#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// MQTT Broker details
const char* mqttServer = MQTT_SERVER;
const int mqttPort = MQTT_PORT;
const char* mqttUser = MQTT_USER;
const char* mqttPassword = MQTT_PASSWORD;

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqttUser, mqttUser, mqttPassword)) {
      Serial.println("connected");
      // // Once connected, publish an announcement...
      // client.publish(mqttTopic, "ESP8266 connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Initialize WiFi connection
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize MQTT connection
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP8266Client", mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed MQTT connection, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void loop() {
  // Ensure the client is connected to the MQTT broker
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Check if data is available to read from Serial
  if (Serial.available() > 0) {
    // Read the incoming JSON string
    String jsonString = Serial.readStringUntil('\n');
    Serial.print("Received JSON: ");
    Serial.println(jsonString);

    // Parse the JSON string
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonString);
    if (error) {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.c_str());
      return;
    }

    // Publish the JSON string to the MQTT topic
    char buffer[256];
    serializeJson(doc, buffer);
    String topicName = "home/" + String(doc["device"]); // Adjust topic structure as needed
    char topic[250];
    topicName.toCharArray(topic, sizeof(topic));
    client.publish(topic, buffer);
    Serial.println("JSON published to MQTT");
  }
}
