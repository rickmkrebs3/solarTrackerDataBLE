/* BTSerial Test Program for transmitting Uno data over HC-05 to CoolTerm running on Mac OSX*/

#include "SoftwareSerial.h"

#define rxPin 10
#define txPin 11

SoftwareSerial BTSerial(rxPin, txPin);

long randNum;

void setup() 
{
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  Serial.begin(38400);
  BTSerial.begin(38400);
  randomSeed(analogRead(0));

  if (BTSerial.isListening())
  {
    Serial.println("is listening...");
  }

  if (BTSerial.overflow())
  {
    Serial.println("BTSerial overflow!");
  } else {
    Serial.println("No BTSerial overflow.");
  }
}

void loop()
{  
  /********* optional
  if (BTSerial.available() > 0)
  {
    BTSerial.read();
  }
  *********/

  Serial.println("Voltage in: ");
  randNum = random(100);
  BTSerial.println(randNum);

  delay(2000);
}