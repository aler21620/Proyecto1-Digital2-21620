/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account, or if you need your Adafruit IO key.
#define IO_USERNAME  "aler21620"
#define IO_KEY       "aio_ZnAT551Ex3hUpRMrU2UoAmDNw2Z7"

/******************************* WIFI **************************************/
#define WIFI_SSID "iPhone de Alejandra"
#define WIFI_PASS "digital2"

// comment out the following lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
