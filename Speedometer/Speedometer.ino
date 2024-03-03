#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#include "Config.h"
#include "Secrets.h"

static const int hallSensorPin = 12;
static const int stateMagnet = LOW;
static const int stateNothing = HIGH;

static const float millisecondsPerSecond = 1000.0;
static const float millimetersPerMeter = 1000.0;
static const float wheelCircumference = 0.0879646;
static const float kphToMph = 0.621371;
#ifdef CONFIG_CONVERT_TO_MPH
static const char velocityUnits[] = "mi/h";
#else
static const char velocityUnits[] = "km/h";
#endif

// Speed update (velocity and acceleration)
typedef struct SpeedUpdate {
  uint64_t timestamp;
  int velocity;
} SpeedUpdate_t;

QueueHandle_t speedUpdateQueue;

// Assign the current time in ISO8601 format to buff and return 0,
// or a -1 to represent an error for cases when buff doesn't fit
// the time string or the time cannot be generated for some other
// reason.
int currentTimeISO8601(char* buff, size_t buffSize, uint64_t epochTimeMillis) {
  if (!buff || buffSize < 24) {
    Serial.println("[currentTimeISO8601] Invalid buffer");
    return -1;
  }

  time_t currentTimeSeconds = epochTimeMillis / 1000;
  int milliseconds = epochTimeMillis % 1000;
  struct tm *utcTime = gmtime(&currentTimeSeconds);

  if (!utcTime) {
    Serial.println("[currentTimeISO8601] Failed to convert time to UTC time");
    return -1;
  }

  char isoTime[20];
  strftime(isoTime, sizeof(isoTime), "%Y-%m-%dT%H:%M:%S", utcTime);

  if (snprintf(buff, buffSize, "%s.%03dZ", isoTime, milliseconds) >= buffSize) {
    Serial.println("[currentTimeISO8601] Buffer size too small");
    return -1;
  }

  return 0;
}

uint64_t uptimeToEpochMillis(int64_t uptime) {
  struct timeval now;
  gettimeofday(&now, NULL);

  uint64_t nowEpochMillis = (uint64_t)now.tv_sec * 1000LL + (now.tv_usec / 1000LL);

  uint64_t currentUptimeMillis = esp_timer_get_time() / 1000;

  return nowEpochMillis - (currentUptimeMillis - uptime / 1000);
}

// Assign the payload JSON to the buffer, returning the number of
// characters written to the buffer, or a number < 0 when the payload
// has now been completely written.
int serializeSpeedUpdate(char* buff, int buffSize, struct SpeedUpdate *update) {
  if (!buff || buffSize < 60) {
    Serial.println("[serializeSpeedUpdate] Invalid buffer");
    return -1;
  }

  // convert epoch time (in millis) to an ISO8601 string
  char timestamp[25];
  currentTimeISO8601(timestamp, sizeof(timestamp), update->timestamp);

#ifdef CONFIG_ENABLE_DEBUG_OUTPUT
  Serial.println("[serializeSpeedUpdate] Serializing speed update");
#endif
  return snprintf(
    buff, buffSize,
    "{\"timestamp\":\"%s\",\"velocity\":%d}",
    timestamp, update->velocity
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
  Serial.println(asctime(&timeinfo));
}

// Record the current velocity from the accelerometer.
void TaskMeasureSpeed(void *pvParameters) {
  Serial.println("[TaskMeasureSpeed] Starting...");

  int lastState = stateNothing;
  int64_t lastTime = esp_timer_get_time();
  int64_t lastRotationTime = lastTime;
  for (;;) {
    int64_t now = esp_timer_get_time();
    int state = digitalRead(hallSensorPin);

    if (state == lastState) {
      continue;
    }
    lastState = state;

    // don't need to record the time the magnet remains "on" the sensor
    if (state == stateNothing) {
      continue;
    }

    // rotation time is the time between two "magnet on" states
    int durationMilliseconds = ((now - lastRotationTime) % 1000000) / 1000;
    lastRotationTime = now;

    float velocity = (
      wheelCircumference // change in position
      / durationMilliseconds // change in time
      * 3600 // conversion m/ms -> km/h
#ifdef CONFIG_CONVERT_TO_MPH
      * kphToMph // conversion km/h -> mi/h
#endif
    );

    uint64_t nowEpochMillis = uptimeToEpochMillis(now);
#ifdef CONFIG_ENABLE_DEBUG_OUTPUT
    char timestamp[42];
    currentTimeISO8601(timestamp, sizeof(timestamp), nowEpochMillis);
    Serial.printf("Rotation time: %d ms\n", durationMilliseconds);
    Serial.printf("Velocity at %s: %0.9f %s\n", timestamp, velocity, velocityUnits);
#endif

    SpeedUpdate_t update;
    update.timestamp = nowEpochMillis;
    update.velocity = (int)(round(velocity));

    // Move on if there is no room in the queue
    if(xQueueSendToBack(speedUpdateQueue, &update, 0) != pdPASS) {
      Serial.println("[TaskMeasureSpeed] Failed to send update");
      continue;
    }
#ifdef CONFIG_ENABLE_DEBUG_OUTPUT
    Serial.println("[TaskMeasureSpeed] Sent speed update");
#endif
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

        http.addHeader("Content-Type", "application/json");
        char payload[60];
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
