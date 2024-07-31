#include <Wire.h>
#include <Adafruit_INA260.h>

Adafruit_INA260 ina260;

void setup() {
  Serial.begin(115200);
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

void loop() {
  // Read and print voltage
  float bus_voltage = ina260.readBusVoltage();
  Serial.print("Bus Voltage:   "); Serial.print(bus_voltage); Serial.println(" V");
  
  // Read and print current
  float current = ina260.readCurrent();
  Serial.print("Current:       "); Serial.print(-current); Serial.println(" mA");
  
  // Read and print power
  float power = ina260.readPower();
  Serial.print("Power:         "); Serial.print(bus_voltage*(-current)); Serial.println(" mW");
  
  Serial.println();
  delay(2000); // Wait for 2 seconds before the next reading
}
