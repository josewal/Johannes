#include <Arduino.h>
#include <PID_v1.h>

int emergency_stop = -10;
int emergency_stop_cooldown = 0;

const byte encoder1PinA = 2;//outputA digital pin2
const byte encoder1PinB = 4;//outoutB digital pin3

volatile int count1 = 0;
int protectedCount1 = 0;
int previousCount1 = 0;

#define readA1 bitRead(PIND,2)//faster than digitalRead()
#define readB1 bitRead(PIND,4)//faster than digitalRead()

void isrA(), isrB();

const byte encoder2PinA = 3;//outputA digital pin2
const byte encoder2PinB = 5;//outoutB digital pin3

volatile int count2 = 0;
int protectedCount2 = 0;
int previousCount2 = 0;

#define readA2 bitRead(PIND,encoder2PinA)//faster than digitalRead()
#define readB2 bitRead(PIND,encoder2PinB )//faster than digitalRead()

volatile long step_time_L;
volatile unsigned long prev_time_L;
long protected_step_time_L;
volatile float RPM_L;
float RPM_L_protected;

volatile long step_time_R;
volatile unsigned long prev_time_R;
long protected_step_time_R;
volatile float RPM_R;
float RPM_R_protected;

void isrA(), isrB();

#define motorA 6
#define motorA1 7
#define motorA2 8

#define motorB 11
#define motorB1 10
#define motorB2 9


double balance;
double desired_balance;
double balance_factor;

double Kp = 0.001, Ki = 0.01, Kd = 0 ;
PID spBalancePID(&balance, &balance_factor, &desired_balance, Kp, Ki, Kd, DIRECT);


double steps;
double desired_steps;
double sp;
double Kp2 = 5, Ki2 = 0, Kd2 = 0 ;
PID stepPID(&steps, &sp, &desired_steps, Kp2, Ki2, Kd2, DIRECT);

int dir = 1;

void setup() {
  Serial.begin(9600);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorA, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorB, OUTPUT);

  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);

  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), isrB, RISING);

  desired_balance = 0;
  balance = 0;
  spBalancePID.SetOutputLimits(-255, 255);
  spBalancePID.SetSampleTime(50);
  spBalancePID.SetMode(AUTOMATIC);

  desired_steps = 200;
  steps = 0;
  stepPID.SetOutputLimits(-255, 255);
  stepPID.SetSampleTime(50);
  stepPID.SetMode(AUTOMATIC);

}


void motorL(int velocity) {
  velocity = constrain(velocity, -255, 255);
  if (velocity > 0)
  {
    analogWrite(motorA, velocity);
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
  } else if ( velocity < 0)
  {
    analogWrite(motorA, -velocity);
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
  } else
  {
    analogWrite(motorA, 0);
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, LOW);
  }

}


void motorR(int velocity) {
  velocity = constrain(velocity, -255, 255);

  if (velocity > 0)
  {
    analogWrite(motorB, velocity);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  } else if ( velocity < 0)
  {
    analogWrite(motorB, -velocity);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  } else
  {
    analogWrite(motorB, 0);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, LOW);
  }
}

void loop() {
  noInterrupts();
  protectedCount1 = count1;
  protectedCount2 = count2;
  protected_step_time_L = step_time_L;
  protected_step_time_R = step_time_R;
  interrupts();

  balance = protected_step_time_L;
  balance = protected_step_time_R - balance;
  spBalancePID.Compute();

  steps = (protectedCount1 + protectedCount2) / 2;
  stepPID.Compute();

  
    if (sp > 20 || sp < -20) {
    motorL(sp - balance_factor);
    motorR(sp + balance_factor);
  } else {
    motorL(0);
    motorR(0);
  }

  if ((previousCount1 != protectedCount1) or (previousCount2 != protectedCount2)) {
    Serial.print("BALANCE \t");
    Serial.print(balance);
    Serial.print("\t BALANCE FACTOR R\t");
    Serial.print( balance_factor);
    Serial.print("\t RPM L \t");
    Serial.print(protected_step_time_L);
    Serial.print("\t RPM R \t");
    Serial.println(protected_step_time_R);
  }

  previousCount1 = protectedCount1;
  previousCount2 = protectedCount2;
}


void isrA() {

  if (readB1 == HIGH) {
    count1 --;
    step_time_L = (micros() - prev_time_L);
    prev_time_L = micros();
  } else {
    count1 ++;
    step_time_L = -(micros() - prev_time_L);
    prev_time_L = micros();
  }
}

void isrB() {
  if (readB2 == HIGH) {
    count2 ++;
    step_time_R = -(micros() - prev_time_R);
    prev_time_R = micros();
  } else {
    count2 --;
    step_time_R = micros() - prev_time_R;
    prev_time_R = micros();
  }
}