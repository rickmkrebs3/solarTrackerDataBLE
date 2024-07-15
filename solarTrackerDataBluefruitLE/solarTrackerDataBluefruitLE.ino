/**************************
  ECE211/212 Project Design
  Summer 2024
  Portland State University 
  Group 6 (Mohammed, Eisa, Fernando, Nate, Rick)

  Code segments herein:
  A. Solar 
  B. Data recording
  C. Bluetooth (BLE) 
    1. BLE master/slave code
    2. I2C or other serial port code to connect to Uno (through Bluefruit shield)

  TODO:
  1. (once can test) make sure BLE is starting and communicating with laptop
  2. make sure Uno/shield can be inferfaced using Coolterm (or other terminal)
  3. make sure data can be streamed and stored in some way on laptop (in some type of DB)
  4. connect shield to Uno through SPI (or other option like UART)
  5. make sure communication protocols are defined (in setup) and enacted in loop() from INA260 
     (for solar), DHT11 (through One Wire) such that dummy data from these function (use random())
     to generate are sent back to Uno
  6. then once received by Uno, can be sent (through shield ) to data collection
  7. read documentation on Hayes AT command set
  8. read documentation on BluefruitLE libraries
  9. change/define appropriate characteristic properties
  10. think about adding debug feature
  11. think about interrupts
  12. think about power needs/limitations
  13. if BLE does not work out for some reason think about BT using HC-05 (?configure to master) or 06
  14. strongly consider setting up real-time clock/SD card/PHY data logging function (see references)
  15. think about Matlab for data analytics!
**************************/

// Libraries
#include <Arduino.h>
#include <SPI.h>
#include "SoftwareSerial.h" // to emulate hardware port with SW
#include "Wire.h"
#include <Adafruit_INA260.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <OneWire.h>
#include <dht.h>
#include <Adafruit_Sensor.h>

// BluefruitLE libraries
#include <Adafruit_ATParser.h> // can comment out?
#include <Adafruit_BLE.h>
#include <Adafruit_BLEBattery.h> // can comment out?
#include <Adafruit_BLEEddystone.h> // can comment out?
#include <Adafruit_BLEGatt.h> // can comment out?
#include <Adafruit_BLEMIDI.h> // can comment out?
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>

#include "BluefruitConfig.h"

// Defines
#define I2C_ADDRESS 0x60

// miscellaneous pin
#define xPIN 0

// enable DHT11 sensor/define DHT pin
#define DHT11_PIN 7 // not best way to define this b/c precompiler could change other text inadvertently
                    // better to declare as const int DHT11_PIN 7

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

//Global variables and class declarations
const int xPin = xPIN; // pin to use for generic input/output

// Adafruit I2C class object
Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(I2C_ADDRESS);

// DS18S20 temperature chip IO
OneWire ds(10);

// define DHT temp/humidity class object
dht DHT;

// create class object for INA Bluetooth-to-serial
Adafruit_INA260 ina260 = Adafruit_INA260();

// Solar Tracker Service and Characteristic information
int32_t solarCharacteristicID;
int32_t solarTrackerServiceID;
int32_t tempCharacteristicID;
int32_t humidCharacteristicID;
int32_t CO2CharacteristicID;

// A small helper; what does this class actually do?  Looks like function print errors
void error(const __FlashStringHelper*err) 
{
  Serial.println(err);
  while (1);
}

void setup() 
{
  Serial.begin(115200); // ok with this bD rate?  does it need to be 9600? other? place after while loop
                        // and boolean success ?
 
  // Wait until serial port is opened
  while (!Serial) 
  {
    delay(500); // suggested in command mode documentation for Flora and Micro; may be able
                // to change back to 10
  }
  // boolean success; // ?place before Serial.begin(115200) statement?

  // ***************************** BLE code setup

  Serial.println(F(" *** Bluetooth Low Energy Solar Tracker System *** "));
  Serial.println(F("---------------------------------------"));

  randomSeed(micros()); // TODO: learn what this does, esp with micros

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    // should be in data mode??
    error(F("Couldn't find Bluetooth device, make sure it's in CoMmanD mode & check wiring..."));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));

    if ( ! ble.factoryReset() )
    {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting BLE device info:");
  /* Print Bluefruit information */
  ble.info();

  /*
  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then enter characters to send to BLE device"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!
  */

  // Set device name
  Serial.println(F("Setting device name to 'SolarTracker': "));
  
  if (!ble.sendCommandCheckOK(F("AT+GAPDEVNAME=SolarTracker")))
  {
    error(F("Could not set devivce name!"));
  }

  // Add the solarTracker service, service ID should be 1 (?)
  Serial.println(F("Adding the solarTrackingService definition (UUID = be1b57df-de01-4137-807b-374c9e936fc5): "));
  bool success = ble.sendCommandWithIntReply(F("AT+GATTADDSERVICE=UUID=be1b57df-de01-4137-807b-374c9e936fc5"), &solarTrackerServiceID);

  if (!success)
  {
    error(F("Could not add solarTracker service..."));
  }

  // next add the characteristics for solar, temperature, humidity, and CO2
  // TODO for all next 4 blocks: figure out correct properties (and their hex), min and max length, and value (what does this mean: 00-40 ... a range?)
  // characteristic ID for solar should be 1
  Serial.println(F("Next adding the solar data collection characteristic (UUID = ba8cc98c-801d-427d-89e9-a4dfede8cfd2): "));
  success = ble.sendCommandWithIntReply(F("AT+GATTADDCHAR=UUID=ba8cc98c-801d-427d-89e9-a4dfede8cfd2, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &solarCharacteristicID);

  if (!success)
  {
    error(F("Could not add solar data collection characteristic!"));
  }

  // characteristic ID for temperature should be 2
  Serial.println(F("Next adding the temperature data collection characteristic (UUID = 1b28c2fd-9a69-4d8a-8c7c-c11e0e36653c): "));
  success = ble.sendCommandWithIntReply(F("AT+GATTADDCHAR=UUID=1b28c2fd-9a69-4d8a-8c7c-c11e0e36653c, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &tempCharacteristicID);

  if (!success)
  {
    error(F("Could not add temperature data collection characteristic!"));
  }

  // characteristic ID for temperature should be 3
  Serial.println(F("Next adding the humidity data collection characteristic (UUID = d745033c-f71c-4497-bccf-47347d172dc8): "));
  success = ble.sendCommandWithIntReply(F("AT+GATTADDCHAR=UUID=d745033c-f71c-4497-bccf-47347d172dc8, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &humidCharacteristicID);

  if (!success)
  {
    error(F("Could not add humidity data collection characteristic!"));
  }

    // characteristic ID for temperature should be 4
  Serial.println(F("Next adding the carbon dioxide (CO2) data collection characteristic (UUID = 8ffc8356-5f2d-4e20-b827-a57365184212): "));
  success = ble.sendCommandWithIntReply(F("AT+GATTADDCHAR=UUID=8ffc8356-5f2d-4e20-b827-a57365184212, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &CO2CharacteristicID);

  if (!success)
  {
    error(F("Could not add CO2 data collection characteristic!"));
  }

  // add the solarTracker service to the advertising data (required by Nordic apps to detect the service)
  Serial.print(F("Adding the solarTracker service UUID to the advertising payload: "));
  ble.sendCommandCheckOK(F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18"));

  // reset the device for the new service setting changes to take effect
  Serial.print(F("Performing a software reset in order for service changes to take effect: "));
  ble.reset();

  Serial.println();

  /******************** below not used
  // Wait for connection (data command function, not used here)
  while (!ble.isConnected()) 
  {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
  */
   // ***************************** end BLE code setup

  // INA260 setup
  Serial.println("Adafruit INA260 Test");

  if (!ina260.begin()) 
  {
    Serial.println("Could not find INA260 chip!");
    while (1);
  }

  Serial.println("Found INA260 chip");
}

void loop() 
{
  // call function that accepts voltage input from solar panel
  solarDataAnalyze();

  // function that reads from the HT11, sending signals back at intervals to Uno
  tempHumSensorInput();

  // Code that sends data from Uno INA260
  // will likely need to call this function when triggered by new data (see BLE interrupt protocol)
  sendSensorData();

  // call Bluetooth® Low Energy function to read from sensor and transmit to characteristic
  bleDataCheck();
  /****** not needed given Uno functionality built-in
  // serial passthrough function between Serial (USB) and Serial1
  if (Serial.available()) {        // If anything comes in Serial (USB),
    Serial1.write(Serial.read());  // read it and send it out Serial1 (pins 0 & 1)
  }

  if (Serial1.available()) {       // If anything comes in Serial1 (pins 0 & 1)
    Serial.write(Serial1.read());  // read it and send it out Serial (USB)
  }
  ********/
}

// INA260 Code to convert solar voltage 
// bus voltage (Vbus) is tied to Vin+ by default (for high side measurements), but 
// for low side measurements we will cut the VB jumper on the right side of the breakout
// and connect Vbus to the power bus (? logic/power pin Vcc 3.3-5V In)
void solarDataAnalyze()
{
  Serial.print("Current: ");
  Serial.print(ina260.readCurrent());
  Serial.println(" mA");

  Serial.print("Bus Voltage: ");
  Serial.print(ina260.readBusVoltage());
  Serial.println(" mV");

  Serial.print("Power: ");
  Serial.print(ina260.readPower());
  Serial.println(" mW");

  Serial.println();
  delay(1000);

  // recall there is trigger pin "Alert" to send interrupt should certain condition arise
  // such as going under/over specified voltage, etc.; also can use this pin to signal
  // a one shot conversion being ready; voltage level is the same as Vcc
}

void tempHumSensorInput()
{
  int chk = DHT.read11(DHT11_PIN);

// check the data coming from the DHT pin
  Serial.print("Temperature = ");
  
  // print temperature on the serial monitor
  Serial.println(DHT.temperature);

  Serial.print("Humidity = ");// Print humidity on the serial monitor
  
  Serial.println(DHT.humidity);

  delay(1000); // delay of 1 second
}

void sendSensorData()
{
    Serial.println("I2C device read and write test");

  if (!i2c_dev.begin()) 
  {
    Serial.print("Did not find device at 0x");
    Serial.println(i2c_dev.address(), HEX);

    while (1);
  }
  Serial.print("Device found on address 0x");
  Serial.println(i2c_dev.address(), HEX);

  uint8_t buffer[32];
  
  // Try to read 32 bytes
  i2c_dev.read(buffer, 32);
  Serial.print("Read: ");
  for (uint8_t i=0; i<32; i++) 
  {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  
  Serial.println();

  // read a register by writing first, then reading
  buffer[0] = 0x0C;  // we'll reuse the same buffer
  i2c_dev.write_then_read(buffer, 1, buffer, 2, false);
  Serial.print("Write then Read: ");
  
  for (uint8_t i=0; i<2; i++) 
  {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  
  Serial.println();
}

void bleDataCheck()
{
  float solarVoltage = random(0.0001, 10);

  Serial.print(F("Updating solar voltage to: "));
  Serial.print(solarVoltage);

  // command is sent when \n (\r) or println is called
  // AT+GATTCHAR=CharacteristicID, value
  ble.print(F("AT+GATTCHAR="));
  ble.print(solarCharacteristicID);
  ble.print(F(",00-"));
  ble.println(solarVoltage, HEX);

  // check that command executed OK
  if (!ble.waitForOK())
  {
    Serial.println(F("Failed to get a response!"));
  }

  // delay before next measurement update
  delay(1000);

  /************** not used at this point
  // Check for user input
  char n, inputs[BUFSIZE+1];

  if (Serial.available())
  {
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    Serial.print("Sending: ");
    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);
  }

  // Echo received data
  while ( ble.available() )
  {
    int c = ble.read();

    Serial.print((char)c);

    // Hex output too, helps w/debugging!
    Serial.print(" [0x");
    if (c <= 0xF) Serial.print(F("0"));
    Serial.print(c, HEX);
    Serial.print("] ");
  }
  */
}
