/**********************************************************
 * ALGAELYTICS IoT HUB
 * University of Victoria, ECE356 Project
 *
 * Rylen Enger, Duncan Stannard
 * Date started: September 22th, 2022
 *
 * Description:
 *  This device will function as an internet-connected hub
 *  that collects data from sensors and sends the data
 *  over the LTE-M network to a secure HTTPS endpoint.
 *
 * Equipment:
 *  - ESP32 Huzzah
 *  - Blues Wireless Notecarrier AF with Global Notecard
 *  - AM2302 Temperature and humidity sensor
 *  - DS1820B waterproof water temperature sensor
 *  - DFRobot Gravity Analog TDS sensor
 * 
 ***********************************************************/

// LIB DEPS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <DHT.h>
#include <DallasTemperature.h>
#include <Notecard.h>
#include <OneWire.h>

// PIN DEFINITIONS
#define DHT_TYPE DHT22 // DHT - AM2302
#define DHT_PIN 25 // DHT -
#define TDS_PIN 26 // TDS - ESP32 pin D4 analog input

// DEFINITIONS
#define usbSerial Serial
#define productUID "com.gmail.rylenenger:iot_hub"
#define VREF 3.3 // TDS - analog reference voltage(Volt) of the ADC
#define SCOUNT 50 // TDS - sum of sample point

// DEEPSLEEP SETUP
#define mS_TO_S_FACTOR 1000         // DEEPSLEEP - conversion factor for ms to seconds
#define uS_TO_S_FACTOR 1000000LL    // DEEPSLEEP - conversion factor for us to seconds
#define TIME_TO_SLEEP 60 * 60       // DEEPSLEEP - time ESP32 will go to sleep (in seconds) */

// FUNCTION PROTOTYPES
void configureHub();
void configureGPS();
void configureTracking();
void print_wakeup_reason();
int getMedianNum(int bArray[], int iFilterLen);
float getTDS();

// GLOBALS
Notecard notecard;
DHT dht(DHT_PIN, DHT_TYPE);

// Variables that are persistent through deep sleep
RTC_DATA_ATTR int bootCount = 0;

// Onewire for DS1820B
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup()
{
    delay(1000);
    usbSerial.begin(115200);
    notecard.begin();
    notecard.setDebugOutputStream(usbSerial);

    // Increment boot number and print it every reboot
    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));

    // Print the wakeup reason for ESP32
    print_wakeup_reason();

    // Configure wake up source
    uint64_t sleepTime = TIME_TO_SLEEP * uS_TO_S_FACTOR;
    esp_sleep_enable_timer_wakeup(sleepTime);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

    configureHub();
    configureGPS();
    configureTracking();
    dht.begin();
    sensors.begin();
}

void loop()
{
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

    // should read battery voltage too....!!!
    
    float h = dht.readHumidity();
    float ta = dht.readTemperature();
    sensors.requestTemperatures();
    float tw = sensors.getTempCByIndex(0);
    float tds = getTDS();
    float cs = 30 + (60.0 * (esp_random() / (float)UINT32_MAX)); // cm per second
    float wa = 0.2 + (1 * (esp_random() / (float)UINT32_MAX));
    float pH = 7.7 + (0.6 * (esp_random() / (float)UINT32_MAX));
    // float h = 25.0 + (74 * (esp_random() / (float)UINT32_MAX));
    // float ta = 19.0 + (2.0 * (esp_random() / (float)UINT32_MAX));
    // float tw = 5.0 + (8.0 * (esp_random() / (float)UINT32_MAX));
    // float tds = 100 + (300.0 * (esp_random() / (float)UINT32_MAX));

    // Check if any reads failed and exit early (to try again).
    if ((isnan(h) || isnan(ta) || isnan(tw) || isnan(tds))) {
        Serial.println(F("\nFailed to read from sensors!"));
        // return;
    } else {
        Serial.println(F("\nSucceeded to read from sensors!\n"));
    }

    
    // attempt to create a new note for sensor data
    J* req = notecard.newRequest("note.add");
    if (req != NULL) {
        JAddStringToObject(req, "file", "sensors.qo");
        // force immediate sync after sending request
        JAddBoolToObject(req, "sync", true);
        // package the body in JSON
        J* body = JCreateObject();
        if (body != NULL) {
            JAddNumberToObject(body, "temp_air", ta);
            JAddNumberToObject(body, "humidity", h);
            JAddNumberToObject(body, "temp_water", tw);
            JAddNumberToObject(body, "salinity", tds / 0.55);
            JAddNumberToObject(body, "pH", pH);
            JAddNumberToObject(body, "cur_spd", cs);
            JAddNumberToObject(body, "waves", wa);
            JAddItemToObject(req, "body", body);
        }

        Serial.println(F("Sending request!\n"));
        notecard.sendRequest(req);
    }

    Serial.println(F("\nRequest sent over LTE-M!\n\n"));
    
    Serial.printf("cnt:\t%i\n", bootCount);
    Serial.println((String) "hum:\t" + h + " %");
    Serial.println((String) "atmp:\t" + ta + " C");
    Serial.println((String) "wtmp:\t" + tw + " C");
    Serial.println((String) "pH:\t" + pH);
    Serial.println((String) "cur:\t" + cs + " cm/s");
    Serial.println((String) "wav:\t" + wa + " m");
    Serial.println((String) "sal:\t" + (tds / 0.55) + " ppm\n");

    Serial.println("Going to sleep now for " + String(TIME_TO_SLEEP) + " seconds");
    delay(1000);
    Serial.flush();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
}

float getTDS()
{
    // TDS sensor
    int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
    int analogBufferTemp[SCOUNT];
    int analogBufferIndex = 0, copyIndex = 0;
    float averageVoltage = 0, tdsValue = 0, TDStemperature = 25;

    for (int i = 0; i < SCOUNT; i++) {
        analogBuffer[i] = analogRead(TDS_PIN); // read the analog value and store into the buffer
    }

    averageVoltage = getMedianNum(analogBuffer, SCOUNT) * (float)VREF / 4096.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (TDStemperature - 25.0); // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVoltage = averageVoltage / compensationCoefficient; // temperature compensation
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5; // convert voltage value to tds value
    return tdsValue;
}

// Get the median number from the TDS sensor
// https://wiki.dfrobot.com/Gravity__Analog_TDS_Sensor___Meter_For_Arduino_SKU__SEN0244
int getMedianNum(int bArray[], int iFilterLen)
{
    int bTab[iFilterLen];
    for (byte i = 0; i < iFilterLen; i++)
        bTab[i] = bArray[i];
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
            if (bTab[i] > bTab[i + 1]) {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    if ((iFilterLen & 1) > 0)
        bTemp = bTab[(iFilterLen - 1) / 2];
    else
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    return bTemp;
}

// NOTEHUB CONFIGURATION

void configureHub()
{
    J* req = notecard.newRequest("hub.set");
    JAddStringToObject(req, "product", productUID);
    JAddStringToObject(req, "mode", "minimum");
    notecard.sendRequest(req);
}

void configureGPS()
{
    J* req = notecard.newRequest("card.location.mode");
    JAddStringToObject(req, "mode", "periodic");
    JAddNumberToObject(req, "seconds", 3600);
    notecard.sendRequest(req);
}

void configureTracking()
{
    J* req = notecard.newRequest("card.location.track");
    JAddBoolToObject(req, "start", true);
    JAddBoolToObject(req, "heartbeat", true);
    JAddNumberToObject(req, "hours", 1);
    notecard.sendRequest(req);
}

// ESP32 DEEP SLEEP CONFIGURATION

// prints the deep sleep wakeup reason to the serial monitor
void print_wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
        Serial.println("Wakeup caused by external signal using RTC_IO");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        Serial.println("Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        Serial.println("Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        Serial.println("Wakeup caused by ULP program");
        break;
    default:
        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;
    }
}
