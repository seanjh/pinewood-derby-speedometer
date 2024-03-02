#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration_EEPROM.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>

#include "Config.h"
#include "Secrets.h"

const int hallSensorPin = 13;
const int stateMagnet = LOW;
const int stateNothing = HIGH;

// Speed update (velocity and acceleration)
typedef struct SpeedUpdate {
  unsigned long timestamp;
  int velocity;
  float accelerationX;
  float accelerationY;
  float accelerationZ;
} SpeedUpdate_t;

QueueHandle_t speedUpdateQueue;

// Assign the current time in ISO8601 format to buff and return 0,
// or a -1 to represent an error for cases when buff doesn't fit
// the time string or the time cannot be generated for some other
// reason.
int currentTimeISO8601(char* buff, size_t buffSize) {
  // TODO(sean) rewrite w/ baseline + ticks
  if (!buff || buffSize < 21) {
    Serial.println("[currentTimeISO8601] Invalid buffer");
    return -1;
  }

  time_t currentTime = time(NULL);
  Serial.print("[currentTimeISO8601] Formatting current time as ISO8601: ");
  Serial.println(currentTime);
  struct tm *utcTime = gmtime(&currentTime);

  if (!utcTime) {
    Serial.println("[currentTimeISO8601] Failed to convert current time to UTC time");
    return -1;
  }

  strftime(buff, buffSize, "%Y-%m-%dT%H:%M:%SZ", utcTime);
  Serial.printf("[currentTimeISO8601] Formatted current time: %s\n", buff);
  return 0;
}

// Assign the payload JSON to the buffer, returning the number of
// characters written to the buffer, or a number < 0 when the payload
// has now been completely written.
int serializeSpeedUpdate(char* buff, int buffSize, struct SpeedUpdate *update) {
  if (!buff || buffSize < 150) {
    Serial.println("[serializeSpeedUpdate] Invalid buffer");
    return -1;
  }

  Serial.println("[serializeSpeedUpdate] Serializing speed update");
  return snprintf(
    buff,
    buffSize,
    "{\"timestamp\":\"%s\"," \
    "\"velocity\":%d," \
    "\"acceleration_x\":%d," \
    "\"acceleration_y\":%d," \
    "\"acceleration_z\":%d}",
    update->timestamp,
    update->velocity,
    update->accelerationX,
    update->accelerationY,
    update->accelerationZ
  );
}

// Synchronize the system clock with the NTP (Network Time Protocol)
// server.
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

void initSensors(void) {
}

void setupSensors(void) {
}

// Return the magnitude of the vector
float magnitude(float x, float y, float z) {
  return sqrt(x*x + y*y + z*z);
}

int formatState(int value, char *buff, size_t buff_size) {
  if (buff_size < 4) {
    return 7;
  }

  switch (value) {
  case stateMagnet:
  strcpy(buff, "magnet");
  return 0;
  case stateNothing:
  strcpy(buff, "nothing");
  return 0;
  }
}

// Record the current velocity from the accelerometer.
void TaskMeasureSpeed(void *pvParameters) {
  Serial.println("[TaskMeasureSpeed] Starting...");

  int lastState = stateNothing;
  int lastTime = 0;
  for (;;) {
    unsigned long now = millis();
    int state = digitalRead(hallSensorPin);

    if (state == lastState) {
    char s[7];
    formatState(state, s, sizeof(s));
      Serial.printf("Ignoring repeat state: %s\n", s);
      vTaskDelay(pdMS_TO_TICKS(CONFIG_SENSOR_UPDATE_MILLISECONDS));
      continue;
    }

    int elapsed = now - lastTime;
    lastTime = now;
    lastState = state;

    char s[7];
    formatState(state, s, sizeof(s));
    Serial.printf("Sensor transitioned after %d: %s\n", elapsed, s);

#ifdef CONFIG_ENABLE_DEBUG_OUTPUT
  Serial.printf("\n");
#endif

    SpeedUpdate_t update;
    update.timestamp = 0;
    //strcpy(update.timestamp, "2024-01-01T00:00:00Z");
    update.velocity = 0;
    update.accelerationX = 0;
    update.accelerationY = 0;
    update.accelerationZ = 0;

    // Move on if there is no room in the queue
    if(xQueueSendToBack(speedUpdateQueue, &update, 0) != pdPASS) {
      Serial.println("[TaskMeasureSpeed] Failed to send update");
      continue;
    }
#ifdef CONFIG_ENABLE_DEBUG_OUTPUT
    Serial.println("[TaskMeasureSpeed] Sent speed update");
#endif
    vTaskDelay(pdMS_TO_TICKS(CONFIG_SENSOR_UPDATE_MILLISECONDS));
  }
}


// Forward the speed update to the remote server.
#ifdef CONFIG_ENABLE_SPEED_RECORDING
void TaskRecordSpeed(void *pvParameters) {
  Serial.println("[TaskRecordSpeed] Starting...");

  for(;;) {
    WiFiClientSecure *client = new WiFiClientSecure;
    if (!client) {
      Serial.println("[TaskRecordSpeed] Failed to create WiFiClientSecure.");
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }
    client->setInsecure();
    // TODO(sean) fix TLS with local server + self-signed certificate
    //client->setCACert(CONFIG_ROOT_CA_CERTIFICATE);

    {
      HTTPClient http;
      Serial.printf("[TaskRecordSpeed] begin request to %s\n", CONFIG_API_URL);
      vTaskDelay(pdMS_TO_TICKS(100));

      if (http.begin(*client, String(CONFIG_API_URL))) {
        Serial.println("[TaskRecordSpeed] Requesting...");

        struct SpeedUpdate update;
        char currentTimestamp[] = "2024-01-01T00:00:00.000Z";
        strcpy(update.timestamp, currentTimestamp);
        update.velocity = 0;
        update.accelerationX = 0;
        update.accelerationY = 0;
        update.accelerationZ = 0;

        http.addHeader("Content-Type", "application/json");
        char payload[150];
        if (serializeSpeedUpdate(payload, sizeof(payload), &update) < 0) {
          Serial.println("[TaskRecordSpeed] Failed to serialize payload");
          continue;
        }
        Serial.printf("[TaskRecordSpeed] sending payload: %s\n", payload);

        int httpCode = http.POST(String(payload));
        if (httpCode > 0) {
          String response = http.getString();
          switch (httpCode) {
            case HTTP_CODE_OK:
            case HTTP_CODE_ACCEPTED:
            case HTTP_CODE_MOVED_PERMANENTLY:
            Serial.printf("[TaskRecordSpeed] success response %d: %s\n", httpCode, response.c_str());
            break;
            default:
            Serial.printf("[TaskRecordSpeed] unexpected response %d: %s\n", httpCode, response.c_str());
            break;
          }
        } else {
          Serial.printf("[TaskRecordSpeed] Request failed, error: %s\n", http.errorToString(httpCode).c_str());
        }

        http.end();
      } else {
        Serial.println("[TaskRecordSpeed] Unable to connect");
      }
    }

    delete client;
    Serial.println("[TaskRecordSpeed] Waiting 5s...");
    // TODO(sean) remove when we're processing queue messages
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
#endif

// Output the velocity to the 7-Segment display.
void TaskUpdateDisplay(void *pvParameters) {
  Serial.println("[TaskUpdateDisplay] Starting...");

  Adafruit_7segment matrix = Adafruit_7segment();
  matrix.begin(0x70);
  matrix.clear();

  for(;;) {
    SpeedUpdate_t update;
    if (xQueueReceive(speedUpdateQueue, &update, portMAX_DELAY) != pdPASS) {
      Serial.println("[TaskUpdateDisplay] Failed to receive message from queue");
    }
#ifdef CONFIG_ENABLE_DEBUG_OUTPUT
    Serial.println("[TaskUpdateDisplay] Received message from queue");
#endif
    matrix.writeDigitNum(0, (update.velocity / 1000));
    matrix.writeDigitNum(1, (update.velocity / 100) % 10);
    matrix.writeDigitNum(3, (update.velocity / 10) % 10);
    matrix.writeDigitNum(4, update.velocity % 10);
    matrix.writeDisplay();
  }
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
  pinMode(hallSensorPin, INPUT);

  Serial.printf("[setup] Connecting to WiFi SSID '%s'\n", WIFI_SSID);
  WiFi.disconnect(true);
  WiFi.onEvent(handleWiFiEvent);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(100); }
  
  setClock();

  speedUpdateQueue = xQueueCreate(10, sizeof(SpeedUpdate_t));
  if (speedUpdateQueue == NULL) {
    Serial.println("[setup] Failed to create speed update queue");
    return;
  }

  xTaskCreate(
    TaskUpdateDisplay,
    "TaskUpdateDisplay",
    CONFIG_TASK_UPDATE_DISPLAY_MEMORY_WORDS,
    NULL,
    2,
    NULL);
  xTaskCreate(
    TaskMeasureSpeed,
    "TaskMeasureSpeed",
    CONFIG_TASK_MEASURE_SPEED_MEMORY_WORDS,
    NULL,
    3,
    NULL);
  #ifdef CONFIG_ENABLE_SPEED_RECORDING
  xTaskCreate(
    TaskRecordSpeed,
    "TaskRecordSpeed",
    CONFIG_TASK_RECORD_SPEED_MEMORY_WORDS,
    NULL,
    1,
    NULL);
  #endif
}

void loop() {}
