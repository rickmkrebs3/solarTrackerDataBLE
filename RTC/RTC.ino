#include <Wire.h>
#include <RTClib.h>

RTC_PCF8523 rtc;

void setup() {
  Serial.begin(9600);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (!rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    // Comment out the next line once the time is set.
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  rtc.start();
}

void loop() {
  // Get the current time
  DateTime now = rtc.now();

  // Print the current time
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  // Wait for 1 second
  delay(1000);
}
