#include <Arduino.h>
#include <PID_v1.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Motor.h>
#include <Encoder.h>

Motor motorLeft(6, 30, 28);
Motor motorRight(7, 26, 24);

Encoder * Encoder::instances [2] = { NULL, NULL };

Encoder encoderLeft(3, 5, PINE, 5, 3, PINE);
Encoder encoderRight(2, 4, PINE, 4, 5, PING);


void resetData();

void setup()
{
  Serial.begin(9600);

  encoderLeft.begin();
  encoderRight.begin();
  

  motorLeft.begin();
  motorRight.begin();
}

void loop()
{
  encoderLeft.getData();
  encoderRight.getData();
  Serial.print(encoderLeft.protected_dir);
  Serial.print("\t");
  Serial.println(encoderRight.protected_dir);

}