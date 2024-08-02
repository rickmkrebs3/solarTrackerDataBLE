#include <Servo.h>  
#include "DHT.h"  

#define DHTPIN 7  // Define the pin where the DHT11 sensor is connected
#define DHTTYPE DHT11  // Define the type of DHT sensor (DHT11)

DHT dht(DHTPIN, DHTTYPE);  // Create a DHT object

Servo servo1;  // Create servo object to control the first servo motor
Servo servo2;  // Create servo object to control the second servo motor

int pos = 0;  // Variable to store the servo angle position
int lightValue1;  // Variable to store the light intensity value from the first photoresistor
int lightValue2;  // Variable to store the light intensity value from the second photoresistor

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 bits per second
  servo1.attach(8);  // First servo connected to pin 8
  servo2.attach(9);  // Second servo connected to pin 9
  dht.begin();  // Initialize the DHT sensor
}

void loop() {
  lightValue1 = analogRead(A1);  // Read the value from the first photoresistor 
  lightValue2 = analogRead(A2);  // Read the value from the second photoresistor 
  
  lightValue1 = map(lightValue1, 0, 1023, 0, 180);  // Map the first light value to a range of 0-180 degrees
  lightValue2 = map(lightValue2, 0, 1023, 0, 180);  // Map the second light value to a range of 0-180 degrees
  
  servo1.write(lightValue1);  // Set the position of the first servo based on the light value
  servo2.write(lightValue2);  // Set the position of the second servo based on the light value

  float h = dht.readHumidity();  // Read humidity from the DHT sensor
  float t = dht.readTemperature();  // Read temperature from the DHT sensor
  
  Serial.print("Humidity: ");  // Print humidity value to the serial monitor
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");  // Print temperature value to the serial monitor
  Serial.print(t);
  Serial.println(" *C");
  
  delay(2000);  // Wait for 2 seconds before repeating the loop
}
