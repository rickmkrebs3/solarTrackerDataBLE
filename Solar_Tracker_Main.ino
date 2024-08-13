//  ECE211/212 Project Design
//  Summer 2024
//  Portland State University 
//  Group 6 (Mohammed, Eisa, Fernando, Nate, Rick)
//
//
//---------------------------------------------------------------------------------------------
//                          Library Calls, Definitions, and Header Files
//---------------------------------------------------------------------------------------------
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include "DHT.h"
#define DHTPIN 2
#define DHTTYPE DHT11
#include <RTClib.h>
#include <Adafruit_INA260.h>

Adafruit_INA260 ina260;
RTC_PCF8523 rtc;
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x3F, 16 column and 2 rows
DHT dht(DHTPIN, DHTTYPE);

//Servo variables
Servo servo1;
Servo servo2;
int lightValue1;
int lightValue2;
int lightValue3;
int lightValue4;
int difference1;
int difference2;
int angle1 = 90;
int angle2 = 90;

void setup()
{
  Serial.begin(9600);
  //---------------------------------------------------------------------------------------------
  //                Temperature & Humidity Sensor + Liquid Crystal I2C LCD Display
  //---------------------------------------------------------------------------------------------
  dht.begin();     // initialize the sensor
  lcd.init();      // initialize the lcd
  lcd.backlight(); // open the backlight 
  
  //---------------------------------------------------------------------------------------------
  //                Real Time Clock (RTC)
  //---------------------------------------------------------------------------------------------
  /*
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
  */

  //---------------------------------------------------------------------------------------------
  //                Servo Motors(2) & Photoresistors(4)
  //---------------------------------------------------------------------------------------------
  servo1.attach(8);
  servo2.attach(7);
  servo1.write(angle1);
  servo2.write(angle2);

  //---------------------------------------------------------------------------------------------
  //                Voltage, Current, Power Module
  //---------------------------------------------------------------------------------------------
  while (!Serial) {
    delay(10); // Wait for the Serial Monitor to open
  }
  Serial.println("Adafruit INA260 Test");
  // Initialize the INA260
  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");
}


void loop()
{
  //---------------------------------------------------------------------------------------------
  //                Servo Motors(2) & Photoresistors(4)
  //---------------------------------------------------------------------------------------------
  lightValue1 = analogRead(A0);  // Read the value from the first photoresistor 
  lightValue2 = analogRead(A1);  // Read the value from the second photoresistor 
  lightValue3 = analogRead(A2);  // Read the value from the third photoresistor 
  lightValue4 = analogRead(A3);  // Read the value from the fourth photoresistor   
  difference1 = lightValue1 - lightValue2;
  difference2 = lightValue3 - lightValue4;

  if (difference1 > 50) {
    angle1 -= 3;
  } else if (difference1 < -50) {
    angle1 += 3;
  }
  if (difference2 > 50) {
    angle2 -= 3;
  } else if (difference2 < -50) {
    angle2 += 3;
  }
  angle1 = constrain(angle1, 0, 180);
  angle2 = constrain(angle2, 0, 180);
  servo1.write(angle2);
  servo2.write(angle1);
  delay(10);

  //---------------------------------------------------------------------------------------------
  //                Real Time Clock (RTC)
  //---------------------------------------------------------------------------------------------
  // Get the current time
  //DateTime now = rtc.now();

  // Print the current time
  //Serial.print(now.year(), DEC);
  //Serial.print('/');
  //Serial.print(now.month(), DEC);
  //Serial.print('/');
  //Serial.print(now.day(), DEC);
  //Serial.print(" ");
  //Serial.print(now.hour(), DEC);
  //Serial.print(':');
  //Serial.print(now.minute(), DEC);
  //Serial.print(':');
  //Serial.print(now.second(), DEC);
  //Serial.println();



  //---------------------------------------------------------------------------------------------
  //                Temperature & Humidity Sensor + Liquid Crystal I2C LCD Display
  //---------------------------------------------------------------------------------------------
  //delay(2000);                                             // 2 second delay between measurements
  float humi  = dht.readHumidity();                        // read humidity
  float tempC = dht.readTemperature();                     // read temperature
  float tempF = tempC * 1.8 + 32;                          //Convert to fahrenheit
  lcd.clear();                                    
  if (isnan(humi) || isnan(tempC)) {                       // check if any reads failed
    lcd.setCursor(0, 0);
    lcd.print("Failed");
  } 
  else {
    lcd.setCursor(0, 0);                                   // start to print at the first row
    //lcd.print("Time: ");                                   // 'Temp:' text displayed
    //lcd.print("now")
    lcd.setCursor(0, 1);                                   // start to print at the second row
    lcd.print("T/H:");                               //'Humidity:' displayed
    lcd.print(tempF);                                      // print the temperature
    lcd.print((char)223);                                  // print ° character
    //lcd.print("F");                                        // 'F' displayed
    lcd.print(humi);                                       // print the humidity
    lcd.print("%");                                        //'%' displayed
  }
  delay(2000);
  lcd.clear();

  //---------------------------------------------------------------------------------------------
  //                Voltage, Current, Power Module
  //---------------------------------------------------------------------------------------------
  // Read and print voltage
  float bus_voltage = ina260.readBusVoltage();
  float current = ina260.readCurrent();
  float power = ina260.readPower();
  
  Serial.print("Bus Voltage:   "); Serial.print(bus_voltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(-current); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(bus_voltage*(-current)); Serial.println(" mW");
  
  power = bus_voltage * -current;

  if (isnan(bus_voltage) || isnan(current) || isnan(power)) {                       // check if any reads failed
    lcd.setCursor(0, 1);
    lcd.print("Failed");
  } 
  else {
    lcd.setCursor(0, 0);                                   // start to print at the first row
    //lcd.print("Time: ");                                   // 'Temp:' text displayed
    //lcd.print("now");
    lcd.setCursor(0, 1);                                   // start to print at the second row
    lcd.print("Power: ");                               //'Humidity:' displayed
    lcd.print(power);                                      // print the temperature
    lcd.print("mW");
  }
  delay(2000); // Wait for 2 seconds before the next reading
  lcd.clear();
}
