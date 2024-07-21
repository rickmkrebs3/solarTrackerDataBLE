/********************************************************** 
  ECE211/212 Project Design
  Summer 2024
  Portland State University 
  Group 6 (Mohammed, Eisa, Fernando, Nate, Rick)

  Code segments herein by general flow:
  A. Solar 
    - outputs voltage, and this is connected to INA260 physically by *** connection; codewise the INA then reads
    by the .read() method the bus voltage, current, and power and then displays on LCD (see this function elsewhere)
    through the Uno.  Will need to add statements that send serial data from INA260 (?writeCurrent, etc.) over *** to Uno
    From there the data will be sent to HT-06 (or BLE shield) to remote device
    TODO:
    * fit all program compoenents together, deciding on which analog pins should be used for all devices
    * recommend setting const int for each device pin
    * where does real-time clock fit in?

  B. Data recording
    - temperature and humidity data will be sent by 1-wire protocol to Uno, where it will then be sent to remote device
    via HT-06 (BT) or BLE shield (see solarTrackerSolarBLE.ino)
    TODO:
    * connect HT11 to LCD as well for real-time display

  C. Bluetooth
    - construct BT object, which can send (and receive) data from the remote device 
    TODO:
    - attach INA260 and other devices to final working (complete) devices

  D. Remote data-management
    - all BT or BLE data are sent serially to database manager for storage and analysis, ideally
    through Linux
    TODO:
    * opt: uncomment SD/local data storage function storeSDData() in loop() if team decides to add this function (not needed)
**********************************************************/

// Libraries
// enables BT functionality through HW ports emulated in SW
#include "SoftwareSerial.h" 

// enable I2C for generic communication
#include "Wire.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>

// library for INA260 device
#include <Adafruit_INA260.h>

// enables HT11 sensor communication
#include <OneWire.h> 
#include <SPI.h>

// library for SD optional storage function (also needs SPI.h just above)
#include <SD.h>

// #include <solarBTAdapter.h> // add later if helpful for more BT functionality

// Defines
#define I2C_ADDRESS 0x60

// BT SoftwareSerial SW pins to emulate pins 0 and 1 (HW USB/UART ports)
// consider changing these to const int rxPin 10 etc... for better data stability
// but unlikely need to do this, but can reassign these to other analog pins
#define rxPin 10
#define txPin 11

//Global variables and class declarations
// construct BT class object "BT" that will create virtual UART to Uno pins 10 (RXD) and 11 (TXD)
SoftwareSerial BTSerial(rxPin, txPin); 

// DS18S20 temperature chip IO
OneWire ds(10);

// create class object for INA Bluetooth-to-serial
Adafruit_INA260 ina260 = Adafruit_INA260();

// Generic I2C object creation in order to transmit data by I2C if necessary
// Currently devices have internal transmission mechanisms
Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(I2C_ADDRESS);

// for BT testing, generates pseudorandom number from analog pin A0
long randNum; // for testing only; delete out/uncomment when devices are attached and pins determined
double solarBTdata;
double gasData;

// SD card pin chip select
const int chipSelect = 4;

// pin using for random number seed for testing *** uncomment/delete out once pins are 
// set for each device such as below
const int sensorPin = A0;

// pin number for the CO2 gas sensor, but can change to any analog pin
const int sensorPin1 = A1;

void setup() 
{
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
   
  Serial.begin(38400);
  BTSerial.begin(38400);  // HC-05 default speed in AT command, but can be 

  // wait until serial port is opened
  while (!Serial) 
  {
    delay(10); 
  }

  // create pseudorandom number from seed from floating A0 pin
  randomSeed(analogRead(0));

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
  
  Serial.println("Adafruit INA260 Test");

  if (!ina260.begin()) 
  {
    Serial.println("Could not find INA260 chip!");
  }
  
  Serial.println("Found INA260 chip");

  /************* SD card initialization: uncomment if using this as well to store data
    Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  *************/

  // Send test message to other device
  Serial.println("Hello from solarTracker!");
}

void loop() 
{
  // call function that accepts voltage input from solar panel after analysis and 
  // sent to internal board pins on INA260
  solarDataAnalyze();

  // function that reads from the HT11, sending signals back at intervals to Uno
  tempHumSensorInput();

  // Code that sends data via I2C, function simply ignored if no I2C device connected
  // will likely need to call this function when triggered by new data (?BT interrupt protocol)
  sendSensorData();

  // next loop to BT function/sending BT data remote device
  sendBTData();

  // function to read CO2 level on A0
  gasSensor();

  // loop through function to store sensor data as .csv file
  // storeSDData();
}

// function that reads data from INA260 voltage converter
void solarDataAnalyze()
{
  // INA260 Code to convert solar voltage 
  // bus voltage (Vbus) is tied to Vin+ by default (for high side measurements), but 
  // for low side measurements we will cut the VB jumper on the right side of the breakout
  // and connect Vbus to the power bus (? logic/power pin Vcc 3.3-5V In)
  Serial.print("Current: ");
  int16_t A = Serial.print(ina260.readCurrent());
  Serial.println(" mA");
  BTSerial.println(A);

  Serial.print("Bus Voltage: ");
  int16_t V = Serial.print(ina260.readBusVoltage());
  Serial.println(" mV");
  BTSerial.println(V);

  Serial.print("Power: ");
  int16_t W = Serial.print(ina260.readPower());
  Serial.println(" mW");
  BTSerial.println(W);

  Serial.println();
  delay(1000);

  // recall there is trigger pin "Alert" to send interrupt should certain condition arise
  // such as going under/over specified voltage, etc.; also can use this pin to signal
  // a one shot conversion being ready; voltage level is the same as Vcc
}
// function to obtain and send HT11 temperature/humidity sensor through I2C to Uno
void tempHumSensorInput()
{
  // 1-wire input 
  byte b;
  byte present = 0;
  byte data[12];
  byte addr[8];

  ds.reset_search();

  if (!ds.search(addr)) 
  {
    Serial.print("No more SPI addresses.\n");

    ds.reset_search();
    return;
  }

  Serial.print("R = ");

  for (b = 0; b < 8; b++) 
  {
    Serial.print(addr[b], HEX);
    Serial.print(" ");
  }
  
  if (OneWire::crc8(addr, 7) != addr[7]) 
  {
    Serial.print("CRC is not valid!\n");
  
    return;
  }
  
  if (addr[0] == 0x10) 
  {
    Serial.print("Device is a DS18S20 family device.\n");
  } else if (addr[0] == 0x28) {
    Serial.print("Device is a DS18B20 family device.\n");
  } else {

  Serial.print("Device family is not recognized: 0x");
  Serial.println(addr[0],HEX);

  return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  delay(1000);  // consider down to 750 ms

  // may do a ds.depower() here but the reset will take care of it
  present = ds.reset();

  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  Serial.print("P = ");

  Serial.print(present,HEX);

  Serial.print(" ");

  for (b = 0; b < 9; b++) 
  {
  // we need 9 bytes
  data[b] = ds.read();
  Serial.print(data[b], HEX);

  Serial.print(" ");
  }

  Serial.print(" CRC = ");
  Serial.print(OneWire::crc8(data, 8), HEX);

  Serial.println();
}

// function to transmit data back to Uno
void sendSensorData()
{
    Serial.println("I2C device read and write test");

  if (!i2c_dev.begin()) 
  {
    Serial.print("Did not find device at 0x");
    Serial.println(i2c_dev.address(), HEX);
    return;
    // while (1);
  }
  Serial.print("Device found on address 0x");
  Serial.println(i2c_dev.address(), HEX);

  uint8_t buffer[32];
  
  // Try to read 32 bytes
  i2c_dev.read(buffer, 32);
  Serial.print("Read: ");
  for (uint8_t i = 0; i < 32; i++) 
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

  Serial.print("Generic sensor device data (0-100): ");
  randNum = random(100);
  Serial.println(randNum);
  BTSerial.println(randNum);
  delay(50);

}
// obtain data from analog Uno pins and send through HC-05 Bluetooth to client
// make sure to change stop bits to 2 on CoolTerm (8 data, no parity, 38400 bD rate)
void sendBTData()
{
  Serial.print("solarBTdata output from INA260 device: ");
  solarBTdata = random(600);
  solarBTdata /= 100;
  Serial.println(solarBTdata);
  BTSerial.println(solarBTdata);
  delay(2000);

  /*********** uncomment for AT programming (see setup()) if desired
  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (BTSerial.available())
    Serial.write(BTSerial.read());

  Keep reading from Arduino Serial Monitor and send to HC-05
  if (Serial.available())
    BT.write(Serial.read());
  *****************************************************************/   
}

// function to store data in batch form to files to SD card (optional)
void storeSDData()
{
  // make a string for assembling the data to log:
  String dataString = "";

  // read from sensors A2-A7 (6 total) and append to the string
  for (int analogPin = 2; analogPin < 8; analogPin++)
  {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 7) 
    {
      dataString += ",";
    }
  }

  // open the file (though only one file can be open at a time)
  File dataFile = SD.open("solarTrackerDataLog.csv", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) 
  {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  } else { // if the file isn't open, pop up an error:
    Serial.println("error opening solarTrackerDataLog.csv");
  }
}

// for MQ-2 heating-driven gas sensor, which reads 10-bit analog output from sensor and prints
// to serial monitor; sensor must preheat for 3+ minutes before stable readings
// connect to 5V, GND, and A0; ** of note detects multiple gas, cannot differentiate between them
// but gases include CO2, ETOH, CO, H2, isobutene, liquefied petroleum gas, CH4 (methane), propane,
// and smoke
void gasSensor()
{
  Serial.print("MQ-2 CO2 level output: ");
  gasData = random(1000);
  Serial.println(gasData);
  BTSerial.println(gasData);

  delay(2000);
  Serial.print("Analog output: ");
  BTSerial.println(analogRead(sensorPin1));  // Read the analog value of the gas sensor
                                          // and print it to the serial monitor
}