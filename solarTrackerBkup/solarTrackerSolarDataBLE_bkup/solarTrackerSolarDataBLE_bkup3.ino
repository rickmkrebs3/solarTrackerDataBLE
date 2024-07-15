#include <aci.h>
#include <aci_cmds.h>
#include <aci_evts.h>
#include <aci_protocol_defines.h>
#include <aci_queue.h>
#include <aci_setup.h>
#include <acilib.h>
#include <acilib_defs.h>
#include <acilib_if.h>
#include <acilib_types.h>
#include <ble_assert.h>
#include <boards.h>
#include <bootloader_setup.h>
#include <dtm.h>
#include <hal_aci_tl.h>
#include <hal_platform.h>
#include <lib_aci.h>

/* ECE211/212 Project Design
  Summer 2024
  Portland State University 
  Group 6 (Mohammed, Eisa, Fernando, Nate, Rick)

  Code segments herein:
  A. Solar 
  B. Data recording
  C. Bluetooth (BLE) 
    1. BLE master/slave code
    2. I2C code to connect to Uno
*/

// Libraries
#include "SoftwareSerial.h"

#include "Wire.h"
#include <Adafruit_INA260.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <OneWire.h>

// Defines
#define I2C_ADDRESS 0x60
Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(I2C_ADDRESS);
#define xPIN 0

// BLE advertising parameter
const uint8_t completeRawAdvertisingData[] = {0x02,0x01,0x06,0x09,0xff,0x01,0x01,0x00,0x01,0x02,0x03,0x04,0x05};   


//Global variables and class declarations
// DS18S20 temperature chip IO
OneWire ds(10);

// create class object for INA Bluetooth-to-serial
Adafruit_INA260 ina260 = Adafruit_INA260();

// Declare BLE Services
BLEService solarTrackerService("be1b57df-de01-4137-807b-374c9e936fc5"); // create service

// create characteristics for each sensor and allow remote device to read and write
BLEIntCharacteristic tempCharacteristic("1b28c2fd-9a69-4d8a-8c7c-c11e0e36653c", BLERead | BLEWrite | BLEBroadcast);
BLEIntCharacteristic humidCharacteristic("d745033c-f71c-4497-bccf-47347d172dc8", BLERead | BLEWrite);
BLEIntCharacteristic CO2Characteristic("8ffc8356-5f2d-4e20-b827-a57365184212", BLERead | BLEWrite);
BLEIntCharacteristic solarCharacteristic("ba8cc98c-801d-427d-89e9-a4dfede8cfd2", BLERead | BLEWrite);

// BLE device designations
#define BLE_DEVICE_NAME                           "solarTracker"
#define BLE_LOCAL_NAME                            "solarTracker"

const int xPin = xPIN; // pin to use for generic input/output

void setup() 
{
  Serial.begin(115200); // ok with this bD rate?  does it need to be 9600? other?
 
  // Wait until serial port is opened
  while (!Serial) 
  {
    delay(10); 
  }

  // BLE setup protocol in INA260 (as peripheral) to laptop with BT enabled (as client)
  setupBLE();

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

  // poll for Bluetooth® Low Energy events
  BLE.poll();

  // if central found and there are data to transmit then establish connection and send data to central
}

void solarDataAnalyze()
{
  // INA260 Code to convert solar voltage 
  // bus voltage (Vbus) is tied to Vin+ by default (for high side measurements), but 
  // for low side measurements we will cut the VB jumper on the right side of the breakout
  // and connect Vbus to the power bus (? logic/power pin Vcc 3.3-5V In)
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
  // 1-wire input 
  byte b;
  byte present = 0;
  byte data[12];
  byte addr[8];

  ds.reset_search();

  if ( !ds.search(addr)) 
  {
    Serial.print("No more addresses.\n");

    ds.reset_search();
    return;
  }

  Serial.print("R = ");

  for(b = 0; b < 8; b++) 
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

  delay(1000);  // maybe 750ms is enough, maybe not

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

  Serial.print(" CRC=");
  Serial.print( OneWire::crc8( data, 8), HEX);

  Serial.println();
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

void setupBLE()
  {
    // start the BLE module
    if (!BLE.begin()) 
    {
      Serial.println("failed to initialize BLE!");
      while (1);
    }
  // add overarching solar Tracker device service
  BLE.addService(solarTrackerService);

  // add functions subsumed under above service to collect and send data to central
  solarTrackerService.addCharacteristic(tempCharacteristic);
  solarTrackerService.addCharacteristic(humidCharacteristic);
  solarTrackerService.addCharacteristic(CO2Characteristic);
  solarTrackerService.addCharacteristic(solarCharacteristic);

  // build advertising data packet
  BLEAdvertisingData advData;

  // If a packet has a raw data parameter, then all the other parameters of the packet will be ignored
  advData.setRawData(completeRawAdvertisingData, sizeof(completeRawAdvertisingData));  
  // Copy set parameters in the actual advertising packet
  BLE.setAdvertisingData(advData);

  // Build scan response data packet
  BLEAdvertisingData scanData;
  scanData.setLocalName("Test advertising raw data");

  // Copy set parameters in the actual scan response packet
  BLE.setScanResponseData(scanData);
  
  BLE.advertise();

  Serial.println("Advertising ...");
  }