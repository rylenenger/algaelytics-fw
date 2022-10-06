#include <Arduino.h>
#include <Notecard.h>

#define usbSerial Serial
#define productUID "com.gmail.rylenenger:iot_hub"

Notecard notecard;

void setup()
{
  delay(2500);
  usbSerial.begin(115200);
  notecard.begin();
  notecard.setDebugOutputStream(usbSerial);
  J *req = notecard.newRequest("hub.set");
  JAddStringToObject(req, "product", productUID);
  JAddStringToObject(req, "mode", "continuous");
  notecard.sendRequest(req);
}

void loop()
{
  // put your main code here, to run repeatedly:
}