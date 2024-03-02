#include <Adafruit_LSM6DSOX.h>

#ifndef CONFIG_H
#define CONFIG_H

//#define CONFIG_ENABLE_SPEED_RECORDING
#define CONFIG_ENABLE_DEBUG_OUTPUT

#define CONFIG_TASK_UPDATE_DISPLAY_MEMORY_WORDS 4096
#define CONFIG_TASK_RECORD_SPEED_MEMORY_WORDS 16384
#define CONFIG_TASK_MEASURE_SPEED_MEMORY_WORDS 16384

#define CONFIG_FILTER_UPDATE_RATE_HZ 100
#define CONFIG_ACCEL_THRESHOLD 1.0

#define CONFIG_ACCEL_RANGE LSM6DS_ACCEL_RANGE_4_G
#define CONFIG_ACCEL_DATA_RATE LSM6DS_RATE_104_HZ
#define CONFIG_SENSOR_UPDATE_MILLISECONDS 10

#define CONFIG_API_URL "https://192.168.1.111:8443/speed-update"

#define CONFIG_ROOT_CA_CERTIFICATE \
	"-----BEGIN CERTIFICATE-----\n" \
	"MIICJTCCAaqgAwIBAgIUAlVv1JHc16bsJc9NieJO9fnimfAwCgYIKoZIzj0EAwIw\n" \
	"STELMAkGA1UEBhMCVVMxEDAOBgNVBAgMB0Zsb3JpZGExDjAMBgNVBAcMBU1pYW1p\n" \
	"MRgwFgYDVQQKDA9TZWFuIEhlcm1hbiBMdGQwHhcNMjQwMjIzMDQwODI2WhcNMzQw\n" \
	"MjIwMDQwODI2WjBJMQswCQYDVQQGEwJVUzEQMA4GA1UECAwHRmxvcmlkYTEOMAwG\n" \
	"A1UEBwwFTWlhbWkxGDAWBgNVBAoMD1NlYW4gSGVybWFuIEx0ZDB2MBAGByqGSM49\n" \
	"AgEGBSuBBAAiA2IABHJojDht7EDK5146hskLKbYA2MYrt+3TWL/aHZ5JwEG+Dx3Z\n" \
	"77/R01ZdDgvYBCJ03nkFVwODfgZIaQPjzr9fTchomQX/vMvnyNMzQ4h3sc3OsI6E\n" \
	"7ErbpT7q25TmfReJEqNTMFEwHQYDVR0OBBYEFIaHiYJmoBzmxrmqnxU9T6e5hGGN\n" \
	"MB8GA1UdIwQYMBaAFIaHiYJmoBzmxrmqnxU9T6e5hGGNMA8GA1UdEwEB/wQFMAMB\n" \
	"Af8wCgYIKoZIzj0EAwIDaQAwZgIxAJRebPwfj8/2FlErYi1DfT05KXb0SaHLJR3U\n" \
	"HQUTZeAH1Ao2PoUuZCyMcn3ESPV/tgIxAIJc9RzLUC8eLSr1n6ssyTQUiGqqUSpi\n" \
	"d5p7uhGg8eeGSV52b22berxVDBt7qrr9Qw==\n" \
	"-----END CERTIFICATE-----"

#endif // !CONFIG_H
