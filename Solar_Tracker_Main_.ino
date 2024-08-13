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
#include <DHT.h>
#define DHTPIN 2
#define DHTTYPE DHT11
// BT SoftwareSerial SW pins to emulate pins 0 and 1 (HW USB/UART ports)
#define rxPin 10
#define txPin 11
#include <RTClib.h>
#include <Adafruit_INA260.h>
// enables BT functionality through HW ports emulated in SW
#include "SoftwareSerial.h" 

// enable I2C for generic communication
#include <Wire.h>

Adafruit_INA260 ina260;
RTC_PCF8523 rtc;
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x3F, 16 column and 2 rows
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial BTSerial(rxPin, txPin); // construct BT class object "BT" that will create virtual
                                       //  UART to Uno pins 10 (RXD) and 11 (TXD)

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

// pin number for the CO2 gas sensor, digital I/O pin 5 (also PWM pin)
const int MQ_PIN = 3;  // for analog pin variant: const int MQ_PIN = A0; 
const int MQ_DELAY = 2000;

// variables for ST0252 gas sensor
const int RL_VALUE = 5;    // RL resistance of the module in Kilo ohms
const int R0 = 10;          // Sensor R0 resistance in Kilo ohms

// Data for multiple readings
const int READ_SAMPLE_INTERVAL = 100;    // Time between samples
const int READ_SAMPLE_TIMES = 5;     // Number of samples

// concentration curve point variables
const float X0 = 200;
const float Y0 = 1.7;
const float X1 = 10000;
const float Y1 = 0.28;

// Concentration curve points {X, Y}
const float point0[] = { log10(X0), log10(Y0) };
const float point1[] = { log10(X1), log10(Y1) };

// Calculate slope and abscissa coordinate
const float scope = (point1[1] - point0[1]) / (point1[0] - point0[0]);
const float coord = point0[1] - point0[0] * scope;

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

  //---------------------------------------------------------------------------------------------
  //                Bluetooth Classic Module
  //---------------------------------------------------------------------------------------------
  /********** uncomment this block after setting HC-05 enable (EN/Key) pin to Uno pin (or Vcc)
  in order to enter AT mode and change settings such as device name, master/slave/loop mode, etc.
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  Serial.println("Enter AT command: ");
  ********** after AT mode, remember to set pin 9 to ground (GND) again */
  
  /*** setup BT functionality
    * set digital pin to control as an output
    * creates a "virtual" serial port/UART
    * connect BT module TXD to D10
    * connect BT module RXD to D11
    * connect BT Vcc to 5V, GND to GND
  ***/

  // set receive (RXD) and transmit (TXD) pins
  pinMode(rxPin, INPUT);  
  pinMode(txPin, OUTPUT);

  BTSerial.begin(9600);

  // check if remote device (running CoolTerm) is listening to BT-USB port for "solarTracker"
  if (BTSerial.isListening())
  {
    Serial.println("CoolTerm client is listening...");
  }
  // make sure there is no BT write buffer overflow
  if (BTSerial.overflow())
  {
    Serial.println("BTSerial overflow!");
  } else {
    Serial.println("No BTSerial overflow.");
  }
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

  // RTC Output to CoolTerm
  BTSerial.print(now.year());
  BTSerial.print('/');
  BTSerial.print(now.month());
  BTSerial.print('/');
  BTSerial.print(now.day()); 
  BTSerial.print(' ');
  BTSerial.print(now.hour());
  BTSerial.print(':');
  BTSerial.print(now.minute());
  BTSerial.print(':');
  BTSerial.println(now.second()); 

  // Wait for 1 second
  delay(1000);

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
    BTSerial.println(tempF);
    lcd.print((char)223);                                  // print ° character
    //lcd.print("F");                                        // 'F' displayed
    lcd.print(humi);                                       // print the humidity
    lcd.print("%");
    BTSerial.println(humi);                                        //'%' displayed
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
    BTSerial.println(power);
    lcd.print("mW");
  }
  delay(2000); // Wait for 2 seconds before the next reading
  lcd.clear();

  //---------------------------------------------------------------------------------------------
  //                CO2 Sensor Module
  //---------------------------------------------------------------------------------------------
  bool state = digitalRead(MQ_PIN);

  if (!state)
  {
    Serial.println("MQ sensor module detected.");
  } else {
    Serial.println("MQ Sensor module not detected!");
  }
  
  delay(MQ_DELAY);

  // begin to read values 
  float rs_med = readMQ(MQ_PIN);    // Get the average Rs
  float concentration = getConcentration(rs_med/R0);  // Get the concentration
  
  // Display the concentration value via serial
  Serial.println("Gas concentration: ");
  lcd.print("CO2 Conc: ");  
  lcd.print(concentration);   // gas concentration displayed to LCD screen
  Serial.println(concentration); 
  BTSerial.println(concentration); // gas concentration sent to over BT to Coolterm
}

// Get the average resistance in N samples
float readMQ(int mq_pin)
{
  float rs = 0;
  for (int i = 0;i<READ_SAMPLE_TIMES;i++) {
    rs += getMQResistance(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  return rs / READ_SAMPLE_TIMES;
}

// Get resistance from analog reading
float getMQResistance(int raw_adc)
{
  return (((float)RL_VALUE / 1000.0*(1023 - raw_adc) / raw_adc));
}

// Get concentration 10^(coord + scope * log (rs/r0)
float getConcentration(float rs_ro_ratio)
{
  return pow(10, coord + scope * log(rs_ro_ratio));
}