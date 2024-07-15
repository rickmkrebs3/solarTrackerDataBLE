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

 * from BLE SDK for Arduino library/broadcast example
 * IMPORTANT: This example still is not compatible with CHIPKIT
 *
 * Click on the "Serial Monitor" button on the Arduino IDE to get reset the Arduino and start the application.
 * The setup() function is called first and is called only one for each reset of the Arduino.
 * The loop() function as the name implies is called in a loop.
 * The setup() and loop() function are called in this way.
 * main()
 *  {
 *   setup();
 *   while(1)
 *   {
 *     loop();
 *   }
 * }
 *
 * Use the PERSONAL HEALTH DEVICES TRANSCODING WHITE PAPER in bluetooth.org to understand the
 * format of the temperature measurement characteristic.
 * The format used is IEEE 11073-20601 FLOAT

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

  /* from BLE NDK for Arduino library, this code from ble_temperature_broadcast example

#include <SPI.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "services.h"
#include <lib_aci.h>

#include <aci_setup.h>
#include "timer1.h"

#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
  static services_pipe_type_mapping_t
      services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
  #define NUMBER_OF_PIPES 0
  static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

#define TEMPERATURE_NUM_SAMPLES  5

static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;
// aci_struct that will contain
// total initial credits
// current credit
// current state of the aci (setup/standby/active/sleep)
// open remote pipe pending
// close remote pipe pending
// Current pipe available bitmap
// Current pipe closed bitmap
// Current connection interval, slave latency and link supervision timeout
// Current State of the the GATT client (Service Discovery)
static struct aci_state_t aci_state;
static hal_aci_evt_t aci_data;

// Counter in seconds.
// When this counter counts down to zero -> wakeup the nRF8001
static int16_t sleep_to_wakeup_timeout;

// Variables used for the temperature measurement and transmission

//static h_thermo_temp_measure_t h_temperature;
//static h_temp_type_t current_type;

static int32_t temperature = 0; // Needs to be an int32 to measure negative values
static float temperature_f;
static uint8_t temperature_count = TEMPERATURE_NUM_SAMPLES;
static int32_t temperature_total = 0;
static bool first_temp_measure_pending = true;

// Variables used for the timer on the AVR

volatile uint8_t timer1_f = 1;

// Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line)
{
  Serial.print("ERROR ");
  Serial.print(file);
  Serial.print(": ");
  Serial.print(line);
  Serial.print("\n");
  while(1);
}

void aci_loop()
{
  static bool setup_required = false;

  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;
    aci_evt = &aci_data.evt;

    switch(aci_evt->evt_opcode)
    {
      case ACI_EVT_DEVICE_STARTED:
      {
        aci_state.data_credit_available = aci_evt->params.device_started.credit_available;
        switch(aci_evt->params.device_started.device_mode)
        {
          case ACI_DEVICE_SETUP:
          
           // When the device is in the setup mode

            aci_state.device_state = ACI_DEVICE_SETUP;
            Serial.println(F("Evt Device Started: Setup"));
            setup_required = true;
            break;

          case ACI_DEVICE_STANDBY:
            aci_state.device_state = ACI_DEVICE_STANDBY;
            sleep_to_wakeup_timeout = 30;
            Serial.println(F("Evt Device Started: Standby"));
            if (aci_evt->params.device_started.hw_error)
            {
              delay(20); //Magic number used to make sure the HW error event is handled correctly.
            }
            else
            {
              // prepare first temperature measurement
              Timer1start();
            }
            break;
        }
      }
        break; //ACI Device Started Event

      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status )
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command

          Serial.print(F("ACI Status of ACI Evt Cmd Rsp 0x"));
          Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
          Serial.print(F("ACI Command 0x"));
          Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          Serial.println(F("Evt Cmd respone: Error. Arduino is in an while(1); loop"));
          while (1);
        }
        else
        {
          switch (aci_evt->params.cmd_rsp.cmd_opcode)
          {
            case ACI_CMD_GET_TEMPERATURE:
              if (0 != temperature_count)
              {
                temperature_total  = temperature_total +
                                    (aci_evt->params.cmd_rsp.params.get_temperature.temperature_value);
                Serial.print(F("Sampling Temperature "));
                Serial.print(temperature_count);
                Serial.print(F(" "));
                Serial.println(aci_evt->params.cmd_rsp.params.get_temperature.temperature_value);
                temperature_count--;
              }
              if (0 == temperature_count)
              {
                Serial.println(temperature_total);
                temperature_count = TEMPERATURE_NUM_SAMPLES;
                /**
                Multiply by 100 for exp = -2 : divide by 4 -> See ACI GetTemperature in datasheet
                */
                temperature = temperature_total * (float)(25/(float)TEMPERATURE_NUM_SAMPLES);
                temperature_total = 0;
                temperature_f = (float)temperature/100.00;
                Serial.print(F("Temperature :"));
                Serial.print(temperature_f, 2);
                Serial.println(" C");
                if (true)//lib_aci_is_pipe_available(&aci_state, PIPE_HEALTH_THERMOMETER_TEMPERATURE_MEASUREMENT_SET))
                {
                  Serial.println(F("Setting the temperature"));
                  temperature &= 0x00FFFFFF; //Mask the exponent part
                  temperature |= 0xFE000000; //Exponent is -2 since we multipled by 100
                  lib_aci_set_local_data(&aci_state, PIPE_HEALTH_THERMOMETER_TEMPERATURE_MEASUREMENT_SET, (uint8_t*) &temperature, 4);

                  //Start broadcasting our temperature value, now that we have one
                  if (first_temp_measure_pending)
                  {
                    lib_aci_open_adv_pipe(PIPE_HEALTH_THERMOMETER_TEMPERATURE_MEASUREMENT_BROADCAST);
                    lib_aci_broadcast(0/* indefinitely */, 0x0100 /* advertising interval 100ms*/);
                    Serial.println(F("Broadcasting started"));

                    first_temp_measure_pending = false;
                  }
                }
              }
              break;
          }
        }
        break;

      case ACI_EVT_HW_ERROR:
        Serial.println(F("HW error: "));
        Serial.println(aci_evt->params.hw_error.line_num, DEC);

        for(uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
        {
          Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
        }
        break;

      default:
        Serial.print(F("Evt Opcode 0x"));
        Serial.print(aci_evt->evt_opcode, HEX);
        Serial.println(F(" unhandled"));
        break;
    }
  }
  else
  {
  
    // No event in the ACI Event queue and if there are no commands in the command queue
    // Arduino can go to sleep
    
  }
  
  // setup_required is set to true when the device starts up and enters setup mode.
   // It indicates that do_aci_setup() should be called. The flag should be cleared if
   // do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
   
  if(setup_required)
  {
    if (SETUP_SUCCESS == do_aci_setup(&aci_state))
    {
      setup_required = false;
    }
  }
}


void setup(void)
{
  Serial.begin(115200);
  //Wait until the serial port is available (useful only for the Leonardo)
  //As the Leonardo board is not reseted every time you open the Serial Monitor
  #if defined (__AVR_ATmega32U4__)
    while(!Serial)
    {}
    delay(5000);  //5 seconds delay for enabling to see the start up comments on the serial board
  #elif defined(__PIC32MX__)
    delay(1000);
  #endif

  Serial.println(F("Arduino setup"));

  
  // Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001

  if (NULL != services_pipe_type_mapping)
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  }
  else
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs         = (hal_aci_data_t*) setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;


  // Tell the ACI library, the MCU to nRF8001 pin connections.
  // The Active pin is optional and can be marked UNUSED
  
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details
  aci_state.aci_pins.reqn_pin   = 9;
  aci_state.aci_pins.rdyn_pin   = 8;
  aci_state.aci_pins.mosi_pin   = MOSI;
  aci_state.aci_pins.miso_pin   = MISO;
  aci_state.aci_pins.sck_pin    = SCK;

  aci_state.aci_pins.spi_clock_divider      = SPI_CLOCK_DIV8;//SPI_CLOCK_DIV8  = 2MHz SPI speed
                                                             //SPI_CLOCK_DIV16 = 1MHz SPI speed
  
  aci_state.aci_pins.reset_pin              = 4; //4 for Nordic board, UNUSED for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.active_pin             = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = false; //Interrupts still not available in Chipkit
  aci_state.aci_pins.interrupt_number       = 1;

  // We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
  // and initialize the data structures required to setup the nRF8001

  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
  lib_aci_init(&aci_state, false);


   // Initalize the data structures required for the GATT Health Thermometer Service
  
}

void loop()
{
  aci_loop();


 // Temperature sampling application that samples the temperature every 4 seconds
  //OR
  //Wakes up the sleeping nRF8001 every 4 seconds

  if (1 == timer1_f)
  {
    uint8_t i = 0;
    timer1_f  = 0;

    for(i=0; i<TEMPERATURE_NUM_SAMPLES; i++)
    {
      Serial.println(F("Get temperature from nRF8001 sensor"));
      lib_aci_get_temperature();
    }
  }
}



  */