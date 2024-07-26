//www.elegoo.com [2016.12.08]
// Modified for Adafruit HUZZAH32 ESP32 Feather (5/2/2019)
// Requires the following library to be installed:
//   ESP32Servo

#include <Servo.h>

const int servoPin = 9; // Servo data pulse pin

Servo myservo;  // create servo object to control a servo

int pos = 0;    // variable to store the servo angle position

void setup()
{
  myservo.attach(servoPin);  // assigns servo pin to the servo object
}

void loop()
{
  for (pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees in 1 deg steps
  {
    myservo.write(pos);               // tell servo to go to position in variable 'pos'
    delay(15);                        // waits 15ms for the servo to reach the position
  }

  for (pos = 180; pos >= 0; pos -= 1) // goes from 180 degrees to 0 degrees
  {
    myservo.write(pos);               // tell servo to go to position in variable 'pos'
    delay(15);                        // waits 15ms for the servo to reach the position
  }
}