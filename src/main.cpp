#include <Arduino.h>
#include <PID_v1.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Motor.h>
#include <Encoder.h>
#include <MotorController.h>

RF24 radio(49, 48); // nRF24L01 (CE, CSN)
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;
// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package
{
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte Switch;
  float rec_lKp;
  float rec_lKi;
  float rec_lKd;
  float rec_rKp;
  float rec_rKi;
  float rec_rKd;
};
Data_Package data; //Create a variable with the above structure
void resetData();

Motor motorLeft(6, 30, 28, 0);
Motor motorRight(7, 26, 24, 0);

Encoder *Encoder::instances[2] = {NULL, NULL};
Encoder encoderLeft(3, 5, PINE, 5, 3, PINE);
Encoder encoderRight(2, 4, PINE, 4, 5, PING);

MotorController controllerLeft(motorLeft, encoderLeft);
MotorController controllerRight(motorRight, encoderRight);

void setup()
{
  Serial.begin(9600);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //  Set the module as receiver

  controllerLeft.setup(1);
  controllerRight.setup(0);

  controllerLeft.rpm_PID_setup(6, 1, 0, 255);
  controllerRight.rpm_PID_setup(6, 1, 0, 255);
}

void recieve_data()
{
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if (currentTime - lastReceiveTime > 1000)
  {              // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone jas a throttle up, if we lose connection it can keep flying away if we dont reset the function
  }
  // Check whether there is data to be received
  if (radio.available())
  {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis();              // At this moment we have received the data
  }

  if (135.75 > data.j1PotY && data.j1PotY > 115.75)
  {
    controllerLeft.moveRPM(0);
    controllerRight.moveRPM(0);
  }
  else
  {
    int desired_RPM_L = map(data.j1PotY, 0, 255, -80, 80);
    int desired_RPM_R = map(data.j1PotY, 0, 255, -80, 80);
    Serial.print(data.j2PotX);

    if (135.75 < data.j2PotX || data.j2PotX < 115.75)
    {
      desired_RPM_L = desired_RPM_L * ((255 / (float(data.j2PotX))) - 0.5);
      desired_RPM_R = desired_RPM_R * -((255 / (float(data.j2PotX))) - 0.5);
      Serial.print("\t");
      Serial.print(((255 / data.j2PotX)) - 0.5);
    }
    Serial.print("\t");
    Serial.println(desired_RPM_L);
    controllerLeft.moveRPM(desired_RPM_L);
    controllerRight.moveRPM(desired_RPM_R);
  }
}

void resetData()
{
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.Switch = 0;
}

void loop()
{
  recieve_data();
  controllerLeft.update();
  controllerRight.update();
}
