#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <HTTPClient.h>

#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#include "Config.h"
#include "Secrets.h"

void setClock() {
  configTime(0, 0, "pool.ntp.org");

  Serial.print(F("[setClock] Waiting for NTP time sync: "));
  time_t nowSecs = time(nullptr);
  while (nowSecs < 8 * 3600 * 2) {
    delay(500);
    Serial.print(F("."));
    yield();
    nowSecs = time(nullptr);
  }

  Serial.println();
  struct tm timeinfo;
  gmtime_r(&nowSecs, &timeinfo);
  Serial.print(F("[setClock] Current time: "));
  Serial.print(asctime(&timeinfo));
}

void TaskRecordSpeedUpdate(void *pvParameters) {
  for(;;) {
    WiFiClientSecure *client = new WiFiClientSecure;
    if (!client) {
      Serial.println("[TaskRecordSpeedUpdate] Failed to create WiFiClientSecure.");
      delay(5000);
      continue;
    }
    Serial.printf("Setting CA certificate:\n%s\n", CONFIG_ROOT_CA_CERTIFICATE);
    client->setInsecure();
    //client->setCACert(CONFIG_ROOT_CA_CERTIFICATE);

    {
      HTTPClient http;
      Serial.printf("[TaskRecordSpeedUpdate] begin request to %s\n", CONFIG_API_URL);
      delay(100);

      if (http.begin(*client, String(CONFIG_API_URL))) {
        Serial.println("[TaskRecordSpeedUpdate] Requesting...");

        http.addHeader("Content-Type", "application/json");
        const char* payload = "{\"timestamp\":1,\"velocity\":5}";

        int httpCode = http.POST(String(payload));
        if (httpCode > 0) {
          String response = http.getString();
          switch (httpCode) {
            case HTTP_CODE_OK:
            case HTTP_CODE_ACCEPTED:
            case HTTP_CODE_MOVED_PERMANENTLY:
            Serial.printf("[TaskRecordSpeedUpdate] success response %d: %s\n", httpCode, response.c_str());
            break;
            default:
            Serial.printf("[TaskRecordSpeedUpdate] unexpected response %d: %s\n", httpCode, response.c_str());
            break;
          }
        } else {
          Serial.printf("[TaskRecordSpeedUpdate] Request failed, error: %s\n", http.errorToString(httpCode).c_str());
        }

        http.end();
      } else {
        Serial.println("[TaskRecordSpeedUpdate] Unable to connect");
      }
    }

    delete client;
    Serial.println("[TaskRecordSpeedUpdate] Waiting 5s...");
    delay(5000);
  }
}

void TaskUpdateDisplay(void *pvParameters) {
  Adafruit_7segment matrix = Adafruit_7segment();

  Serial.println("[TaskUpdateDisplay] Beginning...");
  matrix.begin(0x70);
  matrix.clear();

  uint16_t value = 0;
  boolean drawDots = false;
  for(uint16_t counter = 0; counter < 9999; counter++) {
    Serial.printf("[TaskUpdateDisplay] Displaying %d\n", counter);
    matrix.writeDigitNum(0, (counter / 1000), drawDots);
    matrix.writeDigitNum(1, (counter / 100) % 10, drawDots);
    matrix.drawColon(drawDots);
    matrix.writeDigitNum(3, (counter / 10) % 10, drawDots);
    matrix.writeDigitNum(4, counter % 10, drawDots);
    matrix.writeDisplay();
    delay(1000);
  }

  matrix.clear();
  matrix.println(0);
  matrix.writeDisplay();
}

char* formatWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_READY: return "READY";
    case ARDUINO_EVENT_WIFI_SCAN_DONE: return "SCAN_DONE";
    case ARDUINO_EVENT_WIFI_STA_START: return "STA_START";
    case ARDUINO_EVENT_WIFI_STA_STOP: return "STA_STOP";
    case ARDUINO_EVENT_WIFI_STA_CONNECTED: return "STA_CONNECTED";
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: return "STA_DISCONNECTED";
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE: return "STA_AUTHMODE_CHANGE";
    case ARDUINO_EVENT_WIFI_STA_GOT_IP: return "STA_GOT_IP";
    case ARDUINO_EVENT_WIFI_STA_LOST_IP: return "STA_LOST_IP";
    default: return "n/a";
  }
}

void handleWiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %s (%d)\n", formatWiFiEvent(event), event);

  switch (event) {
    case ARDUINO_EVENT_WIFI_READY: 
      Serial.println("[WiFi-event] WiFi interface ready");
      break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
      Serial.println("[WiFi-event] Completed scan for access points");
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("[WiFi-event] WiFi client started");
      break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      Serial.println("[WiFi-event] WiFi clients stopped");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WiFi-event] Connected to access point");
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("[WiFi-event] Disconnected from WiFi access point");
      break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
      Serial.println("[WiFi-event] Authentication mode of access point has changed");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print("[WiFi-event] Obtained IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("[WiFi-event] Lost IP address and IP address is reset to 0");
      break;
    default: break;
  }
}

void setup() {
  Serial.begin(921600);
  while (!Serial) { delay(100); }
  delay(1000);
  Serial.println("[setup] Serial connection ready");

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.printf("[setup] Connecting to WiFi SSID '%s'\n", WIFI_SSID);
  WiFi.disconnect(true);
  WiFi.onEvent(handleWiFiEvent);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(100); }
  
  setClock();

  xTaskCreate(TaskUpdateDisplay, "[setup] Task Update Display", 2049, NULL, 2, NULL);
  xTaskCreate(TaskRecordSpeedUpdate, "[setup] Task Record Speed Update", 16384, NULL, 2, NULL);
}

void loop() {}
