#include <Arduino.h>
#include <PID_v1.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(49, 48);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;
// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
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

#define motorA 6
#define motorA1 30
#define motorA2 28

#define motorB 7
#define motorB1 26
#define motorB2 24

#define encL_pinA 3 //outputA digital pin2
#define encL_pinB 5 //outoutB digital pin3
#define encR_pinA 2 //outputA digital pin2
#define encR_pinB 4 //outoutB digital pin3

void isrA(), isrB();
#define read_encL_A bitRead(PINE, 5) //faster than digitalRead()
#define read_encL_B bitRead(PINE, 3) //faster than digitalRead()
#define read_encR_A bitRead(PINE, 4) //faster than digitalRead()
#define read_encR_B bitRead(PING , 5) //faster than digitalRead()

volatile int step_countL = 0;
int prot_step_countL = 0;


volatile int step_countR = 0;
int prot_step_countR = 0;


volatile long step_timeL;
volatile unsigned long prev_timeL;
long prot_step_timeL;

volatile long step_timeR;
volatile unsigned long prev_timeR;
long prot_step_timeR;

int emergency_stop = -10;
int emergency_stop_cooldown = 0;

double RPM_L, desired_RPM_L, RPM_L_PID_output;
double RPMLKp = 2, RPMLKi = 2, RPMLKd = 0;
PID RPM_L_PID(&RPM_L, &RPM_L_PID_output, &desired_RPM_L, RPMLKp, RPMLKi, RPMLKd, DIRECT);

double RPM_R, desired_RPM_R, RPM_R_PID_output;
double RPMRKp = 2, RPMRKi = 2, RPMRKd = 0;
PID RPM_R_PID(&RPM_R, &RPM_R_PID_output, &desired_RPM_R, RPMRKp, RPMRKi, RPMRKd, DIRECT);

double stepsL, desired_stepsL, stepL_PID_output;
double stepLKp = 3, stepLKi = 1, stepLKd = 0;
PID stepL_PID(&stepsL, &stepL_PID_output, &desired_stepsL, stepLKp, stepLKi, stepLKd, DIRECT);

double stepsR, desired_stepsR, stepR_PID_output;
double stepRKp = 3, stepRKi = 1, stepRKd = 0;
PID stepR_PID(&stepsR, &stepR_PID_output, &desired_stepsR, stepRKp, stepRKi, stepRKd, DIRECT);

int motorL_pwm;
int motorR_pwm;

void resetData();

void setup()
{
  Serial.begin(9600);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorA, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(encL_pinA, INPUT_PULLUP);
  pinMode(encL_pinB, INPUT_PULLUP);
  pinMode(encR_pinA, INPUT_PULLUP);
  pinMode(encR_pinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encL_pinA), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(encR_pinA), isrB, RISING);

  motorL_pwm = 0;
  motorR_pwm = 0;

  desired_RPM_L = 0;
  RPM_L = 0;
  RPM_L_PID.SetOutputLimits(-255, 255);
  RPM_L_PID.SetSampleTime(50);
  RPM_L_PID.SetMode(AUTOMATIC);

  desired_RPM_R = 0;
  RPM_R = 0;
  RPM_R_PID.SetOutputLimits(-255, 255);
  RPM_R_PID.SetSampleTime(50);
  RPM_R_PID.SetMode(AUTOMATIC);

  desired_stepsL = 75;
  stepsL = 0;
  stepL_PID.SetOutputLimits(-150, 150);
  stepL_PID.SetSampleTime(50);
  stepL_PID.SetMode(AUTOMATIC);

  desired_stepsR = 75;
  stepsR = 0;
  stepR_PID.SetOutputLimits(-150, 150);
  stepR_PID.SetSampleTime(50);
  stepR_PID.SetMode(AUTOMATIC);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //  Set the module as receiver
  resetData();
}

void motorL(int velocity)
{
  velocity = constrain(velocity, -255, 255);
  if (velocity > 5)
  {
    velocity = map(velocity, 5, 255, 40, 255);
    analogWrite(motorA, velocity);
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
  }
  else if (velocity < -5)
  {
    velocity = map(velocity, -5, -255, 40, 255);
    analogWrite(motorA, velocity);
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
  }
  else
  {
    analogWrite(motorA, 0);
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, LOW);
  }
}

void motorR(int velocity)
{
  velocity = constrain(velocity, -255, 255);

  if (velocity > 5)
  {
    velocity = map(velocity, 5, 255, 40, 255);
    analogWrite(motorB, velocity);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  }
  else if (velocity < -5)
  {
    velocity = map(velocity, -5, -255, 40, 255);
    analogWrite(motorB, velocity);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  }
  else
  {
    analogWrite(motorB, 0);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, LOW);
  }
}

//INTERRUPT SERVICE ROUTINES
void isrA()
{
  if (read_encL_B == HIGH)
  {
    step_countL++;
    step_timeL = +(micros() - prev_timeL);
    prev_timeL = micros();
  }
  else
  {
    step_countL--;
    step_timeL = -(micros() - prev_timeL);
    prev_timeL = micros();
  }
}

void isrB()
{
  if (read_encR_B == HIGH)
  {
    step_countR--;
    step_timeR = -(micros() - prev_timeR);
    prev_timeR = micros();
  }
  else
  {
    step_countR++;
    step_timeR = (micros() - prev_timeR);
    prev_timeR = micros();
  }
}

void load_ISR_values()
{
  noInterrupts();

  if ((micros() - prev_timeL) >= 100000)
  {
    step_timeL = 100000;
    prev_timeL = micros();
  }

  if ((micros() - prev_timeR) >= 100000)
  {
    step_timeR = 100000;
    prev_timeR = micros();
  }

  prot_step_countL = step_countL;
  prot_step_countR = step_countR;
  prot_step_timeL = constrain(step_timeL, -100000, 100000);
  prot_step_timeR = constrain(step_timeR, -100000, 100000);
  interrupts();

  if (prot_step_timeL < 100000)
  {
    RPM_L = 60000000 / (75 * prot_step_timeL);
  }
  else
  {
    RPM_L = 0;
  }

  if (prot_step_timeR < 100000)
  {
    RPM_R = 60000000 / (75 * prot_step_timeR);
  }
  else
  {
    RPM_R = 0;
  }
}


void radio_debugg(){  
    Serial.print("LX ");
    Serial.print(data.j1PotX);
    Serial.print(", LY ");
    Serial.print(data.j1PotY);
    Serial.print(", LBut ");
    Serial.print(data.j1Button);
    Serial.print(", RX ");
    Serial.print(data.j2PotX);
    Serial.print(", RY ");
    Serial.print(data.j2PotY);
    Serial.print(", RBut ");
    Serial.print(data.j2Button);
    Serial.print(", SW ");
    Serial.print(data.Switch);
    Serial.print(", lKp ");
    Serial.print(data.rec_lKp);
    Serial.print(", lKi ");
    Serial.print(data.rec_lKi);
    Serial.print(", lKd ");
    Serial.println(data.rec_lKd);
}

void recieve_data(){
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone jas a throttle up, if we lose connection it can keep flying away if we dont reset the function
  }
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data
    // radio_debugg();
  }

  RPM_L_PID.SetTunings(data.rec_lKp, data.rec_lKi, data.rec_rKd);
  RPM_R_PID.SetTunings(data.rec_lKp, data.rec_lKi, data.rec_rKd);

  if (132 > data.j1PotY && data.j1PotY > 122)
  {
    desired_RPM_L = 0;
    desired_RPM_R = 0;
  } else
  {
    desired_RPM_L = map(data.j1PotY, 0, 255, -150, 150);
    desired_RPM_R = map(data.j1PotY, 0, 255, -150, 150);
  }

  
  float balancer = map(data.j2PotX, 0, 255, -2, 2);

  desired_RPM_L += desired_RPM_L * balancer;
  desired_RPM_R += desired_RPM_R * (-balancer);
 
}

void PID_debugg()
{
  Serial.print(desired_stepsL - stepsL); //STEP LEFT COUNT ERROR
  Serial.print(",");
  Serial.print(desired_stepsR - stepsR); //STEP RIGHT COUNT ERROR
  Serial.print(",");

  Serial.print(desired_RPM_L); //
  Serial.print(",");
  Serial.print(desired_RPM_R); //
  Serial.print(",");

  Serial.print(desired_RPM_L - RPM_L); //STEP LEFT TIME ERROR
  Serial.print(",");
  Serial.print(desired_RPM_R - RPM_R); //STEP RIGT TIME ERROR
  Serial.print(",");
  
  Serial.print(RPM_L_PID_output); //
  Serial.print(",");
  Serial.print(RPM_R_PID_output); //
  Serial.print(",");
  Serial.print(prot_step_timeL);
  Serial.print(",");
  Serial.print(prot_step_timeR);
  Serial.print(",");
  Serial.print(RPM_L);
  Serial.print(",");
  Serial.println(RPM_R);
}

void loop()
{
  load_ISR_values();

  recieve_data();

  stepsL = prot_step_countL;
  stepsR = prot_step_countR;

  if ((stepsL > desired_stepsL + 1) || (stepsL < desired_stepsR - 1))
    stepL_PID.Compute();
  else
  {
    stepL_PID_output = 0;
  }

  if ((stepsR > desired_stepsR + 1) || (stepsR < desired_stepsR - 1))
    stepR_PID.Compute();
  else
  {
    stepR_PID_output = 0;
  }

  // desired_RPM_L = 60;
  // desired_RPM_R = 60;


  if ((desired_RPM_L > 1) || (desired_RPM_L < -1))
    RPM_L_PID.Compute();
  else
  {
    RPM_L_PID_output = 0;
  }

  if ((desired_RPM_R > 1) || (desired_RPM_R < -1))
    RPM_R_PID.Compute();
  else
  {
    RPM_R_PID_output = 0;
  }

  RPM_L_PID.Compute();
  RPM_R_PID.Compute();

  motorL(RPM_L_PID_output);
  motorR(RPM_R_PID_output);

  PID_debugg();
}

void resetData() {
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.Switch = 0;
}