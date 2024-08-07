
#include <Servo.h>

Servo servo1;  // Create servo object to control the first servo motor
Servo servo2;  // Create servo object to control the second servo motor

int lightValue1;  // Variable to store the light intensity value from the first photoresistor
int lightValue2;  // Variable to store the light intensity value from the second photoresistor
int lightValue3;  // Variable to store the light intensity value from the third photoresistor
int lightValue4;  // Variable to store the light intensity value from the fourth photoresistor

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 bits per second
  servo1.attach(8);  // First servo connected to pin 8
  servo2.attach(9);  // Second servo connected to pin 9
}

void loop() {
  lightValue1 = analogRead(A0);  // Read the value from the first photoresistor 
  lightValue2 = analogRead(A1);  // Read the value from the second photoresistor 
  lightValue3 = analogRead(A2);  // Read the value from the third photoresistor 
  lightValue4 = analogRead(A3);  // Read the value from the fourth photoresistor 
  
  // Map each light value to a range of 0-180 degrees
  int mappedValue1 = map(lightValue1, 0, 1023, 0, 180);
  int mappedValue2 = map(lightValue2, 0, 1023, 0, 180);
  int mappedValue3 = map(lightValue3, 0, 1023, 0, 180);
  int mappedValue4 = map(lightValue4, 0, 1023, 0, 180);
  
   int maxMappedValue1 = (mappedValue1 + mappedValue2) / 2; 
   int maxMappedValue2 = (mappedValue3 + mappedValue4) / 2; 
   servo1.write(maxMappedValue1); 
   servo2.write(maxMappedValue2);

  // Print the light values and mapped values for debugging
  Serial.print("LightValue1: ");
  Serial.print(lightValue1);
  Serial.print("\tMappedValue1: ");
  Serial.print(mappedValue1);
  Serial.print("\tLightValue2: ");
  Serial.print(lightValue2);
  Serial.print("\tMappedValue2: ");
  Serial.print(mappedValue2);
  Serial.print("\tLightValue3: ");
  Serial.print(lightValue3);
  Serial.print("\tMappedValue3: ");
  Serial.print(mappedValue3);
  Serial.print("\tLightValue4: ");
  Serial.print(lightValue4);
  Serial.print("\tMappedValue4: ");
  Serial.print(mappedValue4);
  Serial.println();
  
  delay(2000);  // Wait for 2 seconds before repeating the loop
}
