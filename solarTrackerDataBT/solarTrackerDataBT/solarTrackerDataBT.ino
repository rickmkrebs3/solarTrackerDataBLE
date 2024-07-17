/* ECE211/212 Project Design
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
    * confirm exact serial/other connection type and all pins involved
    * confirm how to get data from Uno to 
    * should these data be written to csv file as well as sent over serial line
    * if serial (yes), then at what frequency?
    * what determines when the INA is queried for data coming from the solar panel (is this the trigger)?
    * incorporate CoolTerm to log these data?
    * what kind of feedback?
    * confirm power need for INA260
    * where does real-time clock fit in?

  B. Data recording
    - temperature and humidity data will be sent by 1-wire protocol to Uno, where it will then be sent to remote device
    via HT-06 (BT) or BLE shield (see solarTrackerSolarBLE.ino)
    TODO:
    * confirm how data are queried
    * record to csv and transmit to BT device
    * connect HT11 to LCD as well for real-time display

  C. Bluetooth
    - construct BT object, which can send (and receive) data from the remote device 
    TODO:
    * attach HT-06, ensure connections
    * configure receive data from Uno and write to remote device at intervals

  D. Remote data-management
    - all BT or BLE data will be sent by csv file or serially to database manager for storage and analysis, ideally
    through Linux
    TODO:
    * figure out how data will arrive (method, interval, amount per interval)
    * use DS3231 & AT24C32 RTC module as real-time clock
    * establish remote database system (ideally on laptop that hosts the DB program)
*/

// Libraries
#include "SoftwareSerial.h" // enables BT functionality through HW ports emulated in SW
 // enables I2C
#include "Wire.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>

#include <Adafruit_SPIDevice.h>
#include <Adafruit_INA260.h>
#include <OneWire.h> // enables HT11 sensor communication

#include <solarBTAdapter.h>

// Defines
#define I2C_ADDRESS 0x60
Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(I2C_ADDRESS);

#define xPIN 0

// construct BT class object "BT" that will create virtual UART to Uno pins 10 (RXD) and 11 (TXD)
SoftwareSerial BT(10, 11); 

//Global variables and class declarations
// DS18S20 temperature chip IO
OneWire ds(10);

// create class object for INA Bluetooth-to-serial
Adafruit_INA260 ina260 = Adafruit_INA260();

// pin to use for generic input/output
const int xPin = xPIN; 

// for BT: stores incoming character from other device
char a; 

void setup() 
{
  pinMode(9, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode
  digitalWrite(9, HIGH);
  
  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  BT.begin(38400);  // HC-05 default speed in AT command more

  // Serial.begin(115200); // ok with this bD rate?  does it need to be 9600? other?
 
  // Wait until serial port is opened
  while (!Serial) 
  {
    delay(10); 
  }

  Serial.println("Adafruit INA260 Test");

  if (!ina260.begin()) 
  {
    Serial.println("Could not find INA260 chip!");
    while (1);
  }

  Serial.println("Found INA260 chip");

  /* setup BT functionality
    - set digital pin to control as an output
    - creates a "virtual" serial port/UART
    - connect BT module TX to D10
    - connect BT module RX to D11
    - connect BT Vcc to 5V, GND to GND
  */
  pinMode(13, OUTPUT);

  // set the data rate for the SoftwareSerial port
  BT.begin(9600); // change to bD rate 115200?

  // Send test message to other device
  BT.println("Hello from solarTracker!");
}

void loop() 
{
  // call function that accepts voltage input from solar panel
  solarDataAnalyze();

  // function that reads from the HT11, sending signals back at intervals to Uno
  tempHumSensorInput();

  // Code that sends data from Uno INA260
  // will likely need to call this function when triggered by new data (?BT interrupt protocol)
  sendSensorData();

  // next loop to BT function/sending BT data remote device
  sendBTData();
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

// sendSenorData() function: transmits data back to Uno
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

void sendBTData()
{
  // Keep reading from Arduino Serial Monitor and send to HC-05
  if (Serial.available())
    BT.write(Serial.read());

  /******* code below is for taking commands from HC-05 (i.e., remote client) to Arduino ****
  if (BT.available())
  // if text arrived in from BT serial...
  {
    a=(BT.read());
    if (a=='1')
    {
      digitalWrite(13, HIGH);
      BT.println("LED on");
    }
    if (a=='2')
    {
      digitalWrite(13, LOW);
      BT.println("LED off");
    }
    if (a=='?')
    {
      BT.println("Send '1' to turn LED on");
      BT.println("Send '2' to turn LED on");
    }
  }
  */   
}