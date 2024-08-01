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


LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x3F, 16 column and 2 rows
DHT dht(DHTPIN, DHTTYPE);

//Servo variables
Servo servo1;
Servo servo2;
int pos = 0;    // variable to store the servo angle position
int lightValue1;
int lightValue2;

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

  //---------------------------------------------------------------------------------------------
  //                Servo Motors(2) & Photoresistors(4)
  //---------------------------------------------------------------------------------------------
  servo1.attach(8);
  servo2.attach(9);
  
}





void loop()
{
  //---------------------------------------------------------------------------------------------
  //                Temperature & Humidity Sensor + Liquid Crystal I2C LCD Display
  //---------------------------------------------------------------------------------------------
  delay(2000);                                             // 2 second delay between measurements
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
    lcd.print("Temp: ");                                   // 'Temp:' text displayed
    lcd.print(tempF);                                      // print the temperature
    lcd.print((char)223);                                  // print ° character
    lcd.print("F");                                        // 'F' displayed
    lcd.setCursor(0, 1);                                   // start to print at the second row
    lcd.print("Humidity: ");                               //'Humidity:' displayed
    lcd.print(humi);                                       // print the humidity
    lcd.print("%");                                        //'%' displayed
  }

  //---------------------------------------------------------------------------------------------
  //                Real Time Clock (RTC)
  //---------------------------------------------------------------------------------------------
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

  //---------------------------------------------------------------------------------------------
  //                Servo Motors(2) & Photoresistors(4)
  //---------------------------------------------------------------------------------------------
  lightValue1 = analogRead(A1);
  lightValue2 = analogRead(A2);
  lightValue1 = map (lightValue1, 0, 1023, 0, 180);
  lightValue2 = map (lightValue2, 0, 1023, 0, 180);
  
  Serial.println("Analog  Value: ");
  
  servo1.write(lightValue1*5);
  servo2.write(lightValue2*5);

}
