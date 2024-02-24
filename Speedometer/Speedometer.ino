#include <stdio.h>
#include <string.h>
#include <time.h>

#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <Adafruit_LSM6DSOX.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "Config.h"
#include "Secrets.h"

// Assign the current time in ISO8601 format to buff and return 0,
// or a -1 to represent an error for cases when buff doesn't fit
// the time string or the time cannot be generated for some other
// reason.
int currentTimeISO8601(char* buff, size_t buffSize) {
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

  Serial.println("[currentTimeISO8601] Formatting current time");
  strftime(buff, buffSize, "%Y-%m-%dT%H:%M:%SZ", utcTime);
  return 0;
}

// Speed update (velocity and acceleration)
typedef struct SpeedUpdate {
  char timestamp[21];
  int velocity;
  int accelerationX;
  int accelerationY;
  int accelerationZ;
} SpeedUpdate_t;

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

// Record the current velocity from the accelerometer.
void TaskCaptureSpeed(void *pvParameters) {
  Adafruit_LSM6DSOX sensor;
  sensor.setAccelDataRate(CONFIG_ACCEL_DATA_RATE);
  sensor.setAccelRange(CONFIG_ACCEL_RANGE);
  
  if (!sensor.begin_I2C()) {
    Serial.println("Failed to find LSM6DS chip");
    return;
  }
  Serial.println("LSM6DS found.");

  for (;;) {
    sensors_event_t accel, gyro, mag, temp;
    sensor.getEvent(&accel, &gyro, &temp);
    
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("\t\tAccel X: ");
    Serial.print(accel.acceleration.x, 4);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y, 4);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z, 4);
    Serial.println(" \tm/s^2 ");

    /* Display the results (rotation is measured in rad/s) */
    Serial.print("\t\tGyro  X: ");
    Serial.print(gyro.gyro.x, 4);
    Serial.print(" \tY: ");
    Serial.print(gyro.gyro.y, 4);
    Serial.print(" \tZ: ");
    Serial.print(gyro.gyro.z, 4);
    Serial.println(" \tradians/s ");

    Serial.print("\t\tTemp   :\t\t\t\t\t");
    Serial.print(temp.temperature);
    Serial.println(" \tdeg C");
    Serial.println();

    delay(CONFIG_SENSOR_UPDATE_MILLISECONDS);
  }
}

// Forward the speed update to the remote server.
void TaskRecordSpeedUpdate(void *pvParameters) {
  Serial.println("[TaskRecordSpeedUpdate] Starting...");

  for(;;) {
    WiFiClientSecure *client = new WiFiClientSecure;
    if (!client) {
      Serial.println("[TaskRecordSpeedUpdate] Failed to create WiFiClientSecure.");
      delay(5000);
      continue;
    }
    Serial.printf("Setting CA certificate:\n%s\n", CONFIG_ROOT_CA_CERTIFICATE);
    client->setInsecure();
    // TODO(sean) fix TLS with local server + self-signed certificate
    //client->setCACert(CONFIG_ROOT_CA_CERTIFICATE);

    {
      HTTPClient http;
      Serial.printf("[TaskRecordSpeedUpdate] begin request to %s\n", CONFIG_API_URL);
      delay(100);

      if (http.begin(*client, String(CONFIG_API_URL))) {
        Serial.println("[TaskRecordSpeedUpdate] Requesting...");

        // TODO(sean) move timestamp creation to code handling accelerometer updates
        char currentTime[21];
        if (!currentTimeISO8601(currentTime, sizeof(currentTime)) < 0) {
          Serial.println("[TaskRecordSpeedUpdate] Failed to create timestamp");
          continue;
        }

        struct SpeedUpdate update;
        strcpy(update.timestamp, currentTime);
        update.velocity = 0;
        update.accelerationX = 0;
        update.accelerationY = 0;
        update.accelerationZ = 0;

        http.addHeader("Content-Type", "application/json");
        char payload[150];
        if (serializeSpeedUpdate(payload, sizeof(payload), &update) < 0) {
          Serial.println("[TaskRecordSpeedUpdate] Failed to serialize payload");
          continue;
        }
        Serial.printf("[TaskRecordSpeedUpdate] sending payload: %s", payload);

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

// Output the velocity to the 7-Segment display.
void TaskUpdateDisplay(void *pvParameters) {
  Serial.println("[TaskUpdateDisplay] Starting...");

  Adafruit_7segment matrix = Adafruit_7segment();
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

  xTaskCreate(TaskUpdateDisplay, "TaskUpdateDisplay", 2049, NULL, 2, NULL);
  xTaskCreate(TaskRecordSpeedUpdate, "TaskRecordSpeedUpdate", 16384, NULL, 2, NULL);
  xTaskCreate(TaskCaptureSpeed, "TaskCaptureSpeed", 2049, NULL, 2, NULL);
}

void loop() {}
