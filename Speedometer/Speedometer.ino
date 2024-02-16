#include <Arduino.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <HTTPClient.h>

#include "Secrets.h"

const char* rootCACertificate = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIF0DCCBLigAwIBAgIQCDTMrVyVtdfKfdMVRCEeAzANBgkqhkiG9w0BAQsFADA8\n" \
"MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRwwGgYDVQQDExNBbWF6b24g\n" \
"UlNBIDIwNDggTTAyMB4XDTIzMDkyMTAwMDAwMFoXDTI0MTAxODIzNTk1OVowFjEU\n" \
"MBIGA1UEAxMLaHR0cGJpbi5vcmcwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEK\n" \
"AoIBAQDBVjuEo18opPDv4KlJRXURhusTLkzCOKnAXKtBspjyQKke3CszXvstyfed\n" \
"eI0D1bF0u3vKh3lxWMz1LaDzzyTDVgBB/IELF1avvjKmJblc1AVHw7AueZNnPNmI\n" \
"DfdA6CYoO9DlSTRmAn4mqx2FMVGeIOAlg32zZcRVTkNx8GpvWYYHk/InAy0ki7Sj\n" \
"DLHdt16Q/dBqHTjbgEggqco8b24ivY3Lr3+z26tPnMFE/GS4KuynIR3DKDcm1i2s\n" \
"qi8geci6tsoHNNwHhypIEOI1rHWEYAuNeRWxmBmHLyBisMXhJKLiaKSkt14h8Z25\n" \
"fN/qPAVqWPjS8111zrmHdGaXx7ojAgMBAAGjggLyMIIC7jAfBgNVHSMEGDAWgBTA\n" \
"MVLNWlDDgnx0cc7L6Zz5euuC4jAdBgNVHQ4EFgQUJ8G4VRQ8UXJ6gQ7dy2fEg9gL\n" \
"hVEwJQYDVR0RBB4wHIILaHR0cGJpbi5vcmeCDSouaHR0cGJpbi5vcmcwEwYDVR0g\n" \
"BAwwCjAIBgZngQwBAgEwDgYDVR0PAQH/BAQDAgWgMB0GA1UdJQQWMBQGCCsGAQUF\n" \
"BwMBBggrBgEFBQcDAjA7BgNVHR8ENDAyMDCgLqAshipodHRwOi8vY3JsLnIybTAy\n" \
"LmFtYXpvbnRydXN0LmNvbS9yMm0wMi5jcmwwdQYIKwYBBQUHAQEEaTBnMC0GCCsG\n" \
"AQUFBzABhiFodHRwOi8vb2NzcC5yMm0wMi5hbWF6b250cnVzdC5jb20wNgYIKwYB\n" \
"BQUHMAKGKmh0dHA6Ly9jcnQucjJtMDIuYW1hem9udHJ1c3QuY29tL3IybTAyLmNl\n" \
"cjAMBgNVHRMBAf8EAjAAMIIBfQYKKwYBBAHWeQIEAgSCAW0EggFpAWcAdQDuzdBk\n" \
"1dsazsVct520zROiModGfLzs3sNRSFlGcR+1mwAAAYq2hcA4AAAEAwBGMEQCIEft\n" \
"1ktm3y2WoRfy9noFdifQsLp9XC+2EmCX6pE4WEA0AiB7YQwx/F6uPpsALsR25Tys\n" \
"m+w02/kmVAhFX3oCiUAaWwB1AEiw42vapkc0D+VqAvqdMOscUgHLVt0sgdm7v6s5\n" \
"2IRzAAABiraFwCsAAAQDAEYwRAIgTUeg1ktTGNuy3aL+FDTbRsQh137T91B9lEUy\n" \
"tT9ASd0CICrBl4VdnGb7mb/A8BtNz2GogxooecRcxh9WrhjqeyEXAHcA2ra/az+1\n" \
"tiKfm8K7XGvocJFxbLtRhIU0vaQ9MEjX+6sAAAGKtoW/+gAABAMASDBGAiEApKXo\n" \
"axgLIL/YnlKIefDOK1hqwg7KJssKa3GNUQgKKqsCIQD4W1OveQX/Rkf/wdq1Gp8r\n" \
"aB3NK7jJuwfybuMvNNjxJzANBgkqhkiG9w0BAQsFAAOCAQEAp6dI1Ao/G7TkLeRs\n" \
"dHZU0HFexfghJfKtkep/+ZDhs9Ivy3KIddeAoujPVHpnq0SnYA11LmpEQZUe27uX\n" \
"fB4nQjB8/cWAcYl50rkaww6hEOFJNml5pglEkLCjBfnZNUXkvHHGxdWePRlpwIvp\n" \
"QShb1cE3HMkbel21wJvWkgzyrkhQPOf/EZLb4nkBdR1MUY0F4c2ztZc7/CO+Qg25\n" \
"qaGQ6QDDJCrm6DjxHvWyH1YQFyOsJbFQkqFWIBjhmHr3h+jSO9+Bn5ikDDvKp5sX\n" \
"IW+QMa4xEMvvyrpJHUrRH0xEkw6EoB0nYWRArEMaz/JdqOcz0KFTZSpMNYsS2c/d\n" \
"blkutw==\n" \
"-----END CERTIFICATE-----\n" \
"-----BEGIN CERTIFICATE-----\n" \
"MIIEXjCCA0agAwIBAgITB3MSSkvL1E7HtTvq8ZSELToPoTANBgkqhkiG9w0BAQsF\n" \
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n" \
"b24gUm9vdCBDQSAxMB4XDTIyMDgyMzIyMjUzMFoXDTMwMDgyMzIyMjUzMFowPDEL\n" \
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEcMBoGA1UEAxMTQW1hem9uIFJT\n" \
"QSAyMDQ4IE0wMjCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALtDGMZa\n" \
"qHneKei1by6+pUPPLljTB143Si6VpEWPc6mSkFhZb/6qrkZyoHlQLbDYnI2D7hD0\n" \
"sdzEqfnuAjIsuXQLG3A8TvX6V3oFNBFVe8NlLJHvBseKY88saLwufxkZVwk74g4n\n" \
"WlNMXzla9Y5F3wwRHwMVH443xGz6UtGSZSqQ94eFx5X7Tlqt8whi8qCaKdZ5rNak\n" \
"+r9nUThOeClqFd4oXych//Rc7Y0eX1KNWHYSI1Nk31mYgiK3JvH063g+K9tHA63Z\n" \
"eTgKgndlh+WI+zv7i44HepRZjA1FYwYZ9Vv/9UkC5Yz8/yU65fgjaE+wVHM4e/Yy\n" \
"C2osrPWE7gJ+dXMCAwEAAaOCAVowggFWMBIGA1UdEwEB/wQIMAYBAf8CAQAwDgYD\n" \
"VR0PAQH/BAQDAgGGMB0GA1UdJQQWMBQGCCsGAQUFBwMBBggrBgEFBQcDAjAdBgNV\n" \
"HQ4EFgQUwDFSzVpQw4J8dHHOy+mc+XrrguIwHwYDVR0jBBgwFoAUhBjMhTTsvAyU\n" \
"lC4IWZzHshBOCggwewYIKwYBBQUHAQEEbzBtMC8GCCsGAQUFBzABhiNodHRwOi8v\n" \
"b2NzcC5yb290Y2ExLmFtYXpvbnRydXN0LmNvbTA6BggrBgEFBQcwAoYuaHR0cDov\n" \
"L2NydC5yb290Y2ExLmFtYXpvbnRydXN0LmNvbS9yb290Y2ExLmNlcjA/BgNVHR8E\n" \
"ODA2MDSgMqAwhi5odHRwOi8vY3JsLnJvb3RjYTEuYW1hem9udHJ1c3QuY29tL3Jv\n" \
"b3RjYTEuY3JsMBMGA1UdIAQMMAowCAYGZ4EMAQIBMA0GCSqGSIb3DQEBCwUAA4IB\n" \
"AQAtTi6Fs0Azfi+iwm7jrz+CSxHH+uHl7Law3MQSXVtR8RV53PtR6r/6gNpqlzdo\n" \
"Zq4FKbADi1v9Bun8RY8D51uedRfjsbeodizeBB8nXmeyD33Ep7VATj4ozcd31YFV\n" \
"fgRhvTSxNrrTlNpWkUk0m3BMPv8sg381HhA6uEYokE5q9uws/3YkKqRiEz3TsaWm\n" \
"JqIRZhMbgAfp7O7FUwFIb7UIspogZSKxPIWJpxiPo3TcBambbVtQOcNRWz5qCQdD\n" \
"slI2yayq0n2TXoHyNCLEH8rpsJRVILFsg0jc7BaFrMnF462+ajSehgj12IidNeRN\n" \
"4zl+EoNaWdpnWndvSpAEkq2P\n" \
"-----END CERTIFICATE-----\n" \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n" \
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n" \
"b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n" \
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n" \
"b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n" \
"ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n" \
"9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n" \
"IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n" \
"VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n" \
"93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n" \
"jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n" \
"AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n" \
"A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n" \
"U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n" \
"N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n" \
"o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n" \
"5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n" \
"rqXRfboQnoZsG4q5WTP468SQvvG5\n" \
"-----END CERTIFICATE-----\n";

typedef struct {
  const char* rootCACertificate;
  const char* url;
} TaskPollParameters_t;

void setup() {
  Serial.begin(921600);
  while (!Serial) { delay(100); }
  delay(1000);
  Serial.println("[setup] Serial connection ready");

  Serial.printf("[setup] Connecting to WiFi SSID '%s'\n", WIFI_SSID);
  WiFi.disconnect(true);
  WiFi.onEvent(handleWiFiEvent);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(100); }
  
  setClock();

  uint32_t blink_delay = 1000;
  xTaskCreate(TaskStrobe, "Task Strobe", 2048, (void*) &blink_delay, 2, NULL);

  TaskPollParameters_t *taskPollParams = ((TaskPollParameters_t *) pvPortMalloc(sizeof(TaskPollParameters_t)));
  taskPollParams->url = "http://192.168.1.111:8000/health";
  taskPollParams->rootCACertificate = rootCACertificate;
  xTaskCreate(TaskPoll, "Task Poll", 16384, (void*) taskPollParams, 2, NULL);
}

void loop() {}

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

void TaskPoll(void *pvParameters) {
  const TaskPollParameters_t* params = (TaskPollParameters_t *) pvParameters;

  for(;;) {
    //WiFiClient *client = new WiFiClient;
    WiFiClientSecure *client = new WiFiClientSecure;
    if (!client) {
      Serial.println("[TaskPoll] Failed to create WiFiClientSecure.");
      delay(5000);
      continue;
    }
    client->setInsecure();
    //client->setCACert(rootCACertificate);

    {
      //HTTPClient https;
      HTTPClient http;
      Serial.printf("[TaskPoll] begin request: %s\n", params->url);
      if (http.begin(*client, String(params->url))) {
      //if (https.begin(*client, String(params->url))) {
        Serial.println("[TaskPoll] GET...");
        int httpCode = http.GET();
        if (httpCode > 0) {
          switch (httpCode) {
            case HTTP_CODE_OK:
            case HTTP_CODE_MOVED_PERMANENTLY:
            String payload = http.getString();
            Serial.printf("[TaskPoll] response: %s\n", payload.c_str());
          }
        } else {
          Serial.printf("[TaskPoll] GET failed, error: %d %s\n", httpCode, http.errorToString(httpCode).c_str());
        }

        http.end();
      } else {
        Serial.println("[TaskPoll] Unable to connect");
      }
    }

    //delete client;

    Serial.println("[TaskPoll] Waiting 5s...");
    delay(5000);
  }
}

void TaskStrobe(void *pvParameters) {
  uint32_t blink_delay = *((uint32_t*)pvParameters);

  pinMode(LED_BUILTIN, OUTPUT);

  for(;;) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(blink_delay);
    digitalWrite(LED_BUILTIN, LOW);
    delay(blink_delay);
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
      Serial.println("[WiFi-event] Disconnected from WiFi access point");
      break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
      Serial.println("[WiFi-event] Authentication mode of access point has changed");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("[WiFi-event] Obtained IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.println("[WiFi-event] Lost IP address and IP address is reset to 0");
      break;
    default: break;
  }
}
