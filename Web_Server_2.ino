#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

const char* ssid = "DWIT-Hotspot";
const char* password = "@DWZone-hotspot1";

WebSocketsServer webSocketServer = WebSocketsServer(81); // Initialize WebSocket server on port 81

// Relay pin definitions
const int relay1Pin = 26;
const int relay2Pin = 25;
const int relay3Pin = 21;
const int relay4Pin = 19;

// Voltage and current sensor pins for each pair
const int voltageSensor1Pin = 35;
const int currentSensor1Pin = 34;
const int voltageSensor2Pin = 33;
const int currentSensor2Pin = 32;

// Power measurement variables
float voltage1, voltage2;
float current1, current2;

// Read voltage from sensor 1
float readVoltage1() {
  float sensorValue = analogRead(voltageSensor1Pin);
  return sensorValue * (3.3 / 3830) * 5; // Assuming 5V reference voltage
}

// Read voltage from sensor 2
float readVoltage2() {
  int sensorValue = analogRead(voltageSensor2Pin);
  return sensorValue * (3.3 / 3830) * 5; // Assuming 5V reference voltage
}

// Read current from sensor 1
float readCurrent1() {
  int sensorValue = analogRead(currentSensor1Pin);
  return sensorValue * (3.3 / 4096); // Assuming 0.01V per ampere sensitivity
}

// Read current from sensor 2
float readCurrent2() {
  int sensorValue = analogRead(currentSensor2Pin);
  return sensorValue * (3.3 / 4096); // Assuming 0.01V per ampere sensitivity
}

// Get Power and return JSON object
String getPowerMeasurements() {
  DynamicJsonDocument doc(1024);

  // Read voltage and current for each user and add to userData object
  JsonObject user1Data = doc.createNestedObject("User1");
  user1Data["voltagereading"] = readVoltage1();
  user1Data["currentReading"] = readCurrent1();

  JsonObject user2Data = doc.createNestedObject("User2");
  user2Data["voltagereading"] = readVoltage2();
  user2Data["currentReading"] = readCurrent2();

  String output;
  serializeJson(doc, output);
  return output;
}

void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocketServer.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      break;
    case WStype_TEXT:
      {
        String message = String((char*)payload);
        if (message.startsWith("activateRelays:")) {
          // Parse the message to activate relays
          int relay1State = message.charAt(15) == '1' ? HIGH : LOW;
          int relay2State = message.charAt(16) == '1' ? HIGH : LOW;
          int relay3State = message.charAt(17) == '1' ? HIGH : LOW;
          int relay4State = message.charAt(18) == '1' ? HIGH : LOW;
          // Activate/deactivate relays accordingly
          digitalWrite(relay1Pin, relay1State);
          digitalWrite(relay2Pin, relay2State);
          digitalWrite(relay3Pin,relay3State);
          digitalWrite(relay4Pin,relay4State);
        }
      }
      break;
  }
}

void initWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  digitalWrite(relay1Pin, HIGH);
  digitalWrite(relay2Pin, HIGH);

  pinMode(voltageSensor1Pin, INPUT);
  pinMode(currentSensor1Pin, INPUT);
  pinMode(voltageSensor2Pin, INPUT);
  pinMode(currentSensor2Pin, INPUT);

  Serial.begin(115200);
  initWiFi();

  webSocketServer.begin();
  webSocketServer.onEvent(handleWebSocketEvent);

  Serial.println("Setup Completed");
}

void loop() {
  webSocketServer.loop();

  // Read power measurements and send data periodically
  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastSendTime > 500) {
    String powerData = getPowerMeasurements();
    webSocketServer.broadcastTXT(powerData);
    lastSendTime = currentTime;
  }
}