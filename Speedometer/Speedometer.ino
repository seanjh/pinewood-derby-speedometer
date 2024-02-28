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
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
//#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_Sensor_Calibration_EEPROM.h>

#include "Config.h"
#include "Secrets.h"

QueueHandle_t speedUpdateQueue;

// Speed update (velocity and acceleration)
typedef struct SpeedUpdate {
  unsigned long timestamp;
  int velocity;
  float accelerationX;
  float accelerationY;
  float accelerationZ;
} SpeedUpdate_t;

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

// Record the current velocity from the accelerometer.
void TaskMeasureSpeed(void *pvParameters) {
  Serial.println("[TaskMeasureSpeed] Starting...");
  Adafruit_Sensor_Calibration_EEPROM cal;

  if (!cal.begin()) {
    Serial.println("[TaskMeasureSpeed] Failed to initialize calibration helper");
  } else if (!cal.loadCalibration()) {
    Serial.println("[TaskMeasureSpeed] No calibration loaded/found");
  }
  Serial.println("[TaskMeasureSpeed] Calibration loaded from EEPROM");

  Adafruit_LIS3MDL lis3mdl;
  Adafruit_LSM6DSOX lsm6ds;

  if (!lsm6ds.begin_I2C() || !lis3mdl.begin_I2C()) {
    Serial.println("[TaskMeasureSpeed] Failed to find sensors");
    return;
  }
  Serial.println("[TaskMeasureSpeed] Found LIS3MDL & LSMDSOX");

  Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
  Adafruit_NXPSensorFusion filter; // slowest
  // Adafruit_Madgwick filter;  // faster than NXP
  // Adafruit_Mahony filter;  // fastest/smalleset

  accelerometer = lsm6ds.getAccelerometerSensor();
  gyroscope = lsm6ds.getGyroSensor();
  magnetometer = &lis3mdl;
  Serial.println("[TaskMeasureSpeed] Initialized sensors");

  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  // set lowest range
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  // set slightly above refresh rate
  lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  Serial.println("[TaskMeasureSpeed] Setup sensors");

  filter.begin(CONFIG_FILTER_UPDATE_RATE_HZ);
  Serial.println("[TaskMeasureSpeed] Setup NXP sensor fusion");

  float roll, pitch, heading;
  float gx, gy, gz;

  // float accelX, accelY, accelZ; // Accelerometer readings
  // float gyroX, gyroY, gyroZ; // Gyroscope readings
  // float roll, pitch, heading;
  // float accAngleX, accAngleY; // Accelerometer-derived angle estimates
  // float gyroAngleX, gyroAngleY; // Gyroscope-derived angle estimates
  // float finalAngleX, finalAngleY; // Fused angle estimates
  float linearAccelX, linearAccelY, linearAccelZ; // Linear acceleration components
  float velocityX = 0.0, velocityY = 0.0, velocityZ = 0.0; // Velocity components
  // const float alpha = 0.98;  // Weighting factor for accelerometer data
  const float accelGravity = 9.81;

  unsigned long lastStart = 0;
  unsigned long start = 0;
  for (;;) {
    lastStart = start;
    start = millis();
    int elapsed = start - lastStart;

    sensors_event_t accel, gyro, mag;
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);
    
    unsigned long elapsedI2c = millis() - start;

    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);

    // Gyroscope needs to be converted from Rad/s to Degree/s
    // the rest are not unit-important
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

#ifdef CONFIG_ENABLE_DEBUG_OUTPUT
    Serial.printf(
      "[TaskMeasureSpeed] I2C duration: %d ms\n",
      elapsedI2c);
    Serial.printf(
      "[TaskMeasureSpeed] Accel X=%0.3f \tY=%0.3f, \tZ=%0.3f \tm/s^2\n",
      accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    Serial.printf(
      "[TaskMeasureSpeed] Gyro: X=%0.3f \tY=%0.3f, \tZ=%0.3f \tdegrees/s\n",
      gx, gy, gz);
    Serial.printf(
      "[TaskMeasureSpeed] Mag: X=%0.3f \tY: %0.3f, \tZ: %0.3f \ttesla/s\n",
      mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
#endif

    filter.update(
      gx, gy, gz,
      accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
      mag.magnetic.x, mag.magnetic.y, mag.magnetic.z
    );

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    unsigned long elapsedFusion = millis() - start - elapsedI2c;

#ifdef CONFIG_ENABLE_DEBUG_OUTPUT
    Serial.printf(
      "[TaskMeasureSpeed] Sensor fusion duration: %d ms\n",
      elapsedFusion);
    Serial.printf(
      "[TaskMeasureSpeed] Orientation: Roll(X)=%0.3f, Pitch(Y)=%0.3f, Heading/Yaw(Z)=%0.3f\n",
      roll, pitch, heading);
#endif

    /*
    accelX = measure.acceleration.x;
    accelY = measure.acceleration.y;
    accelZ = measure.acceleration.z;
    gyroX = measure.gyro.x;
    gyroY = measure.gyro.y;
    gyroZ = measure.gyro.z;

    // Calculate roll and pitch angles using accelerometer data
    roll = atan2(accelY, accelZ) * 180.0 / PI;
    pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

    // Calculate accelerometer-derived angle estimates
    accAngleX = roll;
    accAngleY = pitch;
    // Integrate gyroscope data to obtain gyroAngleX and gyroAngleY
    gyroAngleX += gyroX * elapsedMs;
    gyroAngleY += gyroY * elapsedMs;

    // Combine accelerometer and gyroscope data using complementary filter
    finalAngleX = alpha * gyroAngleX + (1 - alpha) * accAngleX;
    finalAngleY = alpha * gyroAngleY + (1 - alpha) * accAngleY;

    // Calculate linear acceleration components by subtracting gravitational component
    linearAccelX = accelX * cos(finalAngleY * PI / 180.0) + accelZ * sin(finalAngleY * PI / 180.0);
    linearAccelY = accelY * cos(finalAngleX * PI / 180.0) - accelZ * sin(finalAngleX * PI / 180.0);
    linearAccelZ = accelZ - 9.81; // Subtract gravity (approx. 9.81 m/s^2)
    */

    // Calculate linear acceleration components by subtracting gravitational component
    //linearAccelX = accel.acceleration.x * cos(pitch * PI / 180.0) + accel.acceleration.z * sin(pitch * PI / 180.0);
    linearAccelX = 0;
    linearAccelY = accel.acceleration.y * cos(roll * PI / 180.0) - accel.acceleration.z * sin(roll * PI / 180.0);
    linearAccelZ = accel.acceleration.z - accelGravity; // Subtract gravity (approx. 9.81 m/s^2)
    Serial.printf(
      "[TaskMeasureSpeed] Linear accel X=%0.3f \tY=%0.3f, \tZ=%0.3f \tm/s^2\n",
      linearAccelX, linearAccelY, linearAccelZ);

    // Integrate linear acceleration to obtain velocity
    //velocityX += linearAccelX * elapsedMs;
    velocityX = 0; // we don't measure lateral velocity
    velocityY += (abs(linearAccelY) < CONFIG_ACCEL_THRESHOLD ? 0 : (linearAccelY * elapsed));
    velocityZ += (abs(linearAccelZ) < CONFIG_ACCEL_THRESHOLD ? 0 : (linearAccelZ * elapsed));

    // Calculate speed (magnitude of velocity)
    float speed = magnitude(velocityX, velocityY, velocityZ);

    unsigned long elapsedIntegration = millis() - start - elapsedI2c - elapsedFusion;

#ifdef CONFIG_ENABLE_DEBUG_OUTPUT
    Serial.printf("[TaskMeasureSpeed] Integration duration: %d ms\n", elapsedIntegration);
    Serial.printf("[TaskMeasureSpeed] Velocity: X=%0.3f, Y=%0.3f, Z=%0.3f \tm/s\n", velocityX, velocityY, velocityZ);

    Serial.printf("[TaskMeasureSpeed] Speed: %d\n", speed);
#endif

    SpeedUpdate_t update;
    update.timestamp = start;
    //strcpy(update.timestamp, "2024-01-01T00:00:00Z");
    update.velocity = speed;
    update.accelerationX = linearAccelX;
    update.accelerationY = linearAccelY;
    update.accelerationZ = linearAccelZ;

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
