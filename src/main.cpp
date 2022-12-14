/**********************************************************
 * ALGAELYTICS IoT HUB
 * University of Victoria, ECE356 Project
 *
 * Rylen Enger, Duncan Stannard
 * Date started: September 22th, 2022
 *
 * Description:
 *    This device will function as
 *
 * Equipment:
 *   -
 ***********************************************************/

// LIB DEPS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <DHT.h>
#include <Notecard.h>

// DEFINITIONS
#define usbSerial Serial
#define productUID "com.gmail.rylenenger:iot_hub"
#define DHT_PIN 5 // DHT -
#define DHT_TYPE DHT22 // DHT - AM2302
#define TDS_PIN 4 // TDS - ESP32 pin D4 analog input
#define VREF 3.3 // TDS - analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // TDS - sum of sample point
#define mS_TO_S_FACTOR 1000 // DEEPSLEEP -
#define uS_TO_S_FACTOR 1000000 // DEEPSLEEP - conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP 10 // DEEPSLEEP - time ESP32 will go to sleep (in seconds) */

// FUNCTION PROTOTYPES
void configureHub();
void configureGPS();
void configureTracking();
void Task1(void* pvParameters);
void Task2(void* pvParameters);
void Task3(void* pvParameters);
void Task4(void* pvParameters);

// GLOBALS
Notecard notecard;
DHT dht(DHT_PIN, DHT_TYPE);
int count = 0;
bool dataReadyFlag = false;
// Sensor values
float hum_val = 0;
float tempAir_val = 0;
float tempWater_val = 0;
float tds_val = 0;
float salinity_val = 0;
// TDS sensor
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, TDStemperature = 25;
// Variables that are persistent through deep sleep
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int sleepCount = 0;

void setup()
{
    delay(2500);
    usbSerial.begin(115200);
    notecard.begin();
    notecard.setDebugOutputStream(usbSerial);

    configureHub();
    configureGPS();
    configureTracking();

    xTaskCreatePinnedToCore(Task1, "Task1", 2048, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(Task2, "Task2", 2048, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(Task3, "Sensor Task", 2048, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(Task4, "Transmit Task", 2048, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
}

void loop()
{
    // nothingness
}

// Task1 - Prints global sensor variables to console at 0.1Hz
void Task1(void* pvParameters)
{
    (void)pvParameters;

    delay(1000);

    while (1) // A Task shall never return or exit.
    {
        Serial.printf("cnt:\t%i\n", count);
        Serial.println((String) "hum:\t" + hum_val + " %");
        Serial.println((String) "atmp:\t" + tempAir_val + " C");
        Serial.println((String) "wtmp:\t" + tempWater_val + " C");
        Serial.println((String) "tds:\t" + tds_val + " ppm\n");
        vTaskDelay(10000);
    }
}

// Task2 - Placeholder. Increases count variable at 1Hz while operating
void Task2(void* pvParameters)
{
    (void)pvParameters;

    while (1) {
        count++;
        vTaskDelay(1000);
    }
}

// Task3 - Poll sensors for data and store in global sensor variables
void Task3(void* pvParameters)
{
    (void)pvParameters;

    while (1) {
        vTaskDelay(1000);
    }
}

// Task4 - Transmit data to notehub and put device back to sleep
void Task4(void* pvParameters)
{
    (void)pvParameters;

    //dht.begin();

    while (1) // A Task shall never return or exit.
    {
        // Reading temperature or humidity takes about 250 milliseconds!
        // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
        // float h = dht.readHumidity();
        // float t = dht.readTemperature();
        float h = 25.0 + (74 * (esp_random() / (float)UINT32_MAX));
        float ta = 19.0 + (2.0 * (esp_random() / (float)UINT32_MAX));
        float tw = 5.0 + (8.0 * (esp_random() / (float)UINT32_MAX));
        float tds = 100 + (300.0 * (esp_random() / (float)UINT32_MAX));

        // Check if any reads failed and exit early (to try again).
        if ((isnan(h) || isnan(ta) || isnan(tw) || isnan(tds)) && (dataReadyFlag == false)) {
            Serial.println(F("Failed to read from sensors!"));
        } else {
            Serial.println(F("Succeeded to read from sensors!\n"));
            hum_val = h;
            tempAir_val = ta;
            tempWater_val = tw;
            tds_val = tds;
            salinity_val = tds / 0.55;
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
                JAddNumberToObject(body, "temp_air", tempAir_val);
                JAddNumberToObject(body, "humidity", hum_val);
                JAddNumberToObject(body, "temp_water", tempWater_val);
                JAddNumberToObject(body, "salinity", salinity_val);
                JAddItemToObject(req, "body", body);
            }
            Serial.println(F("Sending request!"));
            notecard.sendRequest(req);
            dataReadyFlag = false;

            vTaskDelay(10 * 60 * mS_TO_S_FACTOR); // wait for 60 minutes
        }
        //vTaskDelay(1000); // check every second
    }
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
void wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    uint32_t previous = 0;

    switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
        Serial.println("Wakeup caused by external signal using RTC_IO");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        Serial.println("Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        Serial.println("Wakeup caused by ULP program");
        break;
    default:
        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;
    }
}

// if TRUE is passed, device will enter timer wakeup deepsleep
void deepsleep_setup(bool start)
{
    // setup wakeup timer
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, LOW);

    // increment boot number and print it every reboot
    Serial.println("Sleepcount: " + String(++sleepCount));
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
    Serial.flush();

    // go to sleep
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
}