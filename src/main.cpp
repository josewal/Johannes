#include <Arduino.h>
#include <PID_v1.h>

#define motorA 6
#define motorA1 7
#define motorA2 8

#define motorB 11
#define motorB1 10
#define motorB2 9

#define encL_pinA 3 //outputA digital pin2
#define encL_pinB 5 //outoutB digital pin3
#define encR_pinA 2 //outputA digital pin2
#define encR_pinB 4 //outoutB digital pin3

void isrA(), isrB();
#define read_encL_A bitRead(PIND, encL_pinA)         //faster than digitalRead()
#define read_encL_B bitRead(PIND, encL_pinB)         //faster than digitalRead()
#define read_encR_A bitRead(PIND, encR_pinA) //faster than digitalRead()
#define read_encR_B bitRead(PIND, encR_pinB) //faster than digitalRead()

volatile int step_countL = 0;
int prot_step_countL = 0;
int prev_step_countL = 0;

volatile int step_countR = 0;
int prot_step_countR = 0;
int prev_step_countR = 0;

volatile long step_timeL;
volatile unsigned long prev_timeL;
long prot_step_timeL;

volatile long step_timeR;
volatile unsigned long prev_timeR;
long prot_step_timeR;

int emergency_stop = -10;
int emergency_stop_cooldown = 0;

double stepL_time, desired_stepL_time, stepL_time_PID_output;
double step_timeLKp = 0.0001, step_timeLKi = 0, step_timeLKd = 0;
PID stepL_timePID(&stepL_time, &stepL_time_PID_output, &desired_stepL_time, step_timeLKp, step_timeLKi, step_timeLKd, DIRECT);

double stepR_time, desired_stepR_time, stepR_time_PID_output;
double step_timeRKp = 0.0001, step_timeRKi = 0, step_timeRKd = 0;
PID stepR_timePID(&stepR_time, &stepR_time_PID_output, &desired_stepR_time, step_timeRKp, step_timeRKi, step_timeRKd, DIRECT);

double stepsL, desired_stepsL, stepL_PID_output;
double stepLKp = 5, stepLKi = 0, stepLKd = 0;
PID stepL_PID(&stepsL, &stepL_PID_output, &desired_stepsL, stepLKp, stepLKi, stepLKd, DIRECT);

double stepsR, desired_stepsR, stepR_PID_output;
double stepRKp = 5, stepRKi = 0, stepRKd = 0;
PID stepR_PID(&stepsR, &stepR_PID_output, &desired_stepsR, stepRKp, stepRKi, stepRKd, DIRECT);

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

  desired_stepL_time = 0;
  stepL_time = 0;
  stepL_timePID.SetOutputLimits(-255, 255);
  stepL_timePID.SetSampleTime(50);
  stepL_timePID.SetMode(AUTOMATIC);

  desired_stepR_time = 0;
  stepR_time = 0;
  stepR_timePID.SetOutputLimits(-255, 255);
  stepR_timePID.SetSampleTime(50);
  stepR_timePID.SetMode(AUTOMATIC);

  desired_stepsL = 200;
  stepsL = 0;
  stepL_PID.SetOutputLimits(-200, 200);
  stepL_PID.SetSampleTime(50);
  stepL_PID.SetMode(AUTOMATIC);

  desired_stepsR = 200;
  stepsR = 0;
  stepR_PID.SetOutputLimits(-200, 200);
  stepR_PID.SetSampleTime(50);
  stepR_PID.SetMode(AUTOMATIC);
}

void motorL(int velocity)
{
  velocity = constrain(velocity, -255, 255);
  if (velocity > 20)
  {
    analogWrite(motorA, velocity);
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
  }
  else if (velocity < 20)
  {
    analogWrite(motorA, -velocity);
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

  if (velocity > 20)
  {
    analogWrite(motorB, velocity);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  }
  else if (velocity < 20)
  {
    analogWrite(motorB, -velocity);
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
    step_countL--;
    step_timeL = (micros() - prev_timeL);
    prev_timeL = micros();
  }
  else
  {
    step_countL++;
    step_timeL = -(micros() - prev_timeL);
    prev_timeL = micros();
  }
}

void isrB()
{
  if (read_encR_B == HIGH)
  {
    step_countR++;
    step_timeR = -(micros() - prev_timeR);
    prev_timeR = micros();
  }
  else
  {
    step_countR--;
    step_timeR = (micros() - prev_timeR);
    prev_timeR = micros();
  }
}

void load_ISR_values()
{
  noInterrupts();
  prot_step_countL = constrain(step_countL, -200000, 200000);
  prot_step_countR = constrain(step_countR, -200000, 200000);
  prot_step_timeL = step_timeL;
  prot_step_timeR = step_timeR;
  interrupts();
}




void SendSerial(){
    Serial.print(desired_stepL_time - prot_step_timeL);  //STEP LEFT TIME ERROR 
    Serial.print(",");
    Serial.print(desired_stepR_time - prot_step_timeR);  //STEP RIGT TIME ERROR 
    Serial.print(",");
    Serial.print(desired_stepsL - stepsL);              //STEP LEFT COUNT ERROR
    Serial.print(",");
    Serial.print(desired_stepsR - stepsR);              //STEP RIGHT COUNT ERROR
    Serial.print(",");
    Serial.print(stepL_PID_output);                     //
    Serial.print(",");
    Serial.print(stepR_PID_output);                     //
    Serial.print(",");    
    Serial.print(stepL_time_PID_output);          //
    Serial.print(",");  
    Serial.print(stepR_time_PID_output);          //
    Serial.print(",");
    Serial.print(prot_step_timeL); 
    Serial.print(",");
    Serial.println(prot_step_timeR);                           //                        //

    prev_step_countL = prot_step_countL;
    prev_step_countR = prot_step_countR;
}


void loop()
{
  load_ISR_values();

  // stepsL = prot_step_countL;
  // stepL_PID.Compute();
  // stepsR = prot_step_countR;
  // stepR_PID.Compute();

  // if (stepsL > desired_stepsL){
  //   stepL_PID_output = 200 + stepL_PID_output;
  // } else if (stepsL < desired_stepsL)
  // {
  //   stepL_PID_output = 200 - stepL_PID_output;
  // } else 
  // {
  //   stepL_PID_output = 200;
  // }

  // if (stepsR > desired_stepsR){
  //   stepR_PID_output = 200 + stepR_PID_output;
  // } else if (stepsR < desired_stepsR)
  // {
  //   stepR_PID_output = 200 - stepR_PID_output;
  // } else 
  // {
  //   stepR_PID_output = 200;
  // }


  // desired_stepL_time = stepL_PID_output * 1000;
  // desired_stepR_time = stepR_PID_output * 1000;

  desired_stepL_time = 100000;
  desired_stepR_time = 100000;

  stepL_time = prot_step_timeL;
  stepL_timePID.Compute();
  stepR_time = prot_step_timeR;
  stepR_timePID.Compute();

// if (step_timeL > desired_stepL_time){
//     stepL_time_PID_output = 255 - stepL_time_PID_output;
//   } else if (stepL_time < desired_stepL_time)
//   {
//     stepL_time_PID_output = 255 + stepL_time_PID_output;
//   } else 
//   {
//     stepL_time_PID_output = 0;
//   }

//   if (stepR_time > desired_stepR_time){
//     stepR_time_PID_output = 255 + stepR_time_PID_output;
//   } else if (stepR_time < desired_stepR_time)
//   {
//     stepR_time_PID_output = 255 - stepR_time_PID_output;
//   } else 
//   {
//     stepR_time_PID_output = 0;
//   }

  motorL(stepL_time_PID_output);
  motorR(stepR_time_PID_output);

  SendSerial();
}
