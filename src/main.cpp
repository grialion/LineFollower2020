/**
 * https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill
 * PinOut: https://solovjov.net/reblag.dk/The-Generic-STM32F103-Pinout-Diagram.jpg
 * PinOut2: https://opencircuit.shop/resources/content/ad8542259ac19/crop/900-600/STM32-ARM-development-board-STM32F103C8T6.jpg
 * Pins:
 */

#include <Arduino.h>
#include <FTDebouncer.h>
#include <SoftTimer.h>
#include <BlinkTask.h>
#include <Ramp.h>

#define PIN_LED PC13

#define PIN_ULTRASONIC_TRIG PA7
#define PIN_ULTRASONIC_ECHO PA6
#define ULTRASONIC_TIMEOUT_MICROS 2500

#define PIN_L2 PA0
#define PIN_L1 PA2
#define PIN_MD PA1
#define PIN_R1 PA3
#define PIN_R2 PA4

#define PIN_MOTOR_LEFT_FWD PB7
#define PIN_MOTOR_LEFT_BWD PB6
#define PIN_MOTOR_RIGHT_FWD PB8
#define PIN_MOTOR_RIGHT_BWD PB9

#define MOTOR_MIN_SPEED 30
#define MOTOR_SPEED_SLOW 90
#define MOTOR_RAMP_TIME_MS 100

#define PIN_BUTTON PB12

// States
#define LIFECYCLE_STATE_READY 0
#define LIFECYCLE_STATE_RUNNING 1

float getUltrasonicDistance();
void sendTrigger();
void doRamp();
void goRight(int val);
void goLeft(int val);
void doAlways(Task* me);

FTDebouncer pinDebouncer;
BlinkTask hartbeat(PIN_LED, 100, 300);
Task alwaysTask(0, doAlways);

unsigned long pingStarted = 0;
volatile unsigned long echoReceived = 0;
rampInt rightMotorRamp;
rampInt leftMotorRamp;
int rightMotorTargetValue = 0;
int leftMotorTargetValue = 0;
int rightMotorActualValue = 0;
int leftMotorActualValue = 0;

byte liefcycleState = LIFECYCLE_STATE_READY;

void setup()
{
  Serial.begin(115200);
  while(!Serial.availableForWrite())
  {
    delay(10);
  }
  Serial.println();
  Serial.println(F("Starting up..."));

//  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinDebouncer.addPin(PIN_BUTTON, HIGH, INPUT_PULLUP);
  pinMode(PIN_L2, INPUT);
  pinMode(PIN_L1, INPUT);
  pinMode(PIN_MD, INPUT);
  pinMode(PIN_R1, INPUT);
  pinMode(PIN_R2, INPUT);

//  pinMode(PIN_LED, OUTPUT);
//  digitalWrite(PIN_LED, HIGH);

  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);

  pinMode(PIN_MOTOR_LEFT_FWD, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_BWD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_BWD, OUTPUT);

  analogWriteFrequency(500);

  hartbeat.onLevel = LOW;
  hartbeat.start();
  pinDebouncer.init();
  SoftTimer.add(&alwaysTask);
  Serial.println("Ready.");
}

void doAlways(Task* me)
{
//  unsigned long start = millis();
  pinDebouncer.run();
  doRamp();

  if (liefcycleState != LIFECYCLE_STATE_RUNNING)
  {
    return;
  }

  // Read line sensors
  bool isToRight = digitalRead(PIN_L1);
  bool isToLeft = digitalRead(PIN_R1);
  if (isToLeft)
  {
    // Right side has priority.
    goLeft(MOTOR_SPEED_SLOW);
    goRight(MOTOR_MIN_SPEED);
//Serial.print("R");
  }
  else if (isToRight)
  {
    goLeft(MOTOR_MIN_SPEED);
    goRight(MOTOR_SPEED_SLOW);
//Serial.print("L");
  }
  else
  {
    goLeft(MOTOR_SPEED_SLOW);
    goRight(MOTOR_SPEED_SLOW);
//Serial.print("=");
  }
//  unsigned long end = millis();
//  Serial.print("Loop time: ");
//  Serial.println(end - start);
}

void doRamp()
{
  int rightVal = rightMotorRamp.update();
  if (rightVal != rightMotorActualValue)
  {
    analogWrite(PIN_MOTOR_RIGHT_FWD, rightVal);
    rightMotorActualValue = rightVal;
  }
  int leftVal = leftMotorRamp.update();
  if (leftVal != leftMotorActualValue)
  {
    analogWrite(PIN_MOTOR_LEFT_FWD, leftVal);
    leftMotorActualValue = leftVal;
  }
}

void goRight(int val)
{
  if (val != rightMotorTargetValue)
  {
    if (val < rightMotorTargetValue)
    {
      // De-accelerating
      rightMotorRamp.go(val, MOTOR_RAMP_TIME_MS, CUBIC_OUT);
    }
    else
    {
      // Accelerating
      rightMotorRamp.go(val, MOTOR_RAMP_TIME_MS, CUBIC_IN);
    }
    
    rightMotorTargetValue = val;
  }
}
void goLeft(int val)
{
  if (val != leftMotorTargetValue)
  {
    if (val < leftMotorTargetValue)
    {
      // De-accelerating
      leftMotorRamp.go(val, MOTOR_RAMP_TIME_MS, CUBIC_OUT);
    }
    else
    {
      // Accelerating
      leftMotorRamp.go(val, MOTOR_RAMP_TIME_MS, CUBIC_IN);
    }

    leftMotorTargetValue = val;
  }
}

void testOpto()
{
  unsigned long start = micros();
  uint32_t a0 = digitalRead(PA0);
  uint32_t a1 = digitalRead(PA1);
  uint32_t a2 = digitalRead(PA2);
  uint32_t a3 = digitalRead(PA3);
  uint32_t a4 = digitalRead(PA4);
  unsigned long end = micros();
  Serial.print(end - start);
  Serial.print(" ");
  Serial.print(a0);
  Serial.print(" ");
  Serial.print(a1);
  Serial.print(" ");
  Serial.print(a2);
  Serial.print(" ");
  Serial.print(a3);
  Serial.print(" ");
  Serial.print(a4);
}

// test ultrasonic
void testUltrasonic()
{
  Serial.print(" ");

  float cm = getUltrasonicDistance();
  Serial.print(cm);

  Serial.println();
/*
  digitalWrite(PIN_LED, LOW);
  delay(200);
  digitalWrite(PIN_LED, HIGH);
  delay(200);
  */
}

float getUltrasonicDistance()
{
  sendTrigger();
  uint32_t duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_MICROS);

  float cm = ((float)duration/2) / (float)29.1;
  return cm;
}

void sendTrigger()
{
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  echoReceived = 0;
  pingStarted = micros();
}

// -- When Start/Stop button pressed
void onButtonPressed(uint8_t pinNr)
{
	if (liefcycleState == LIFECYCLE_STATE_READY)
  {
    Serial.println(F("Start"));
    liefcycleState = LIFECYCLE_STATE_RUNNING;
    hartbeat.onMs = 100;
    hartbeat.offMs = 100;
  }
	else if (liefcycleState == LIFECYCLE_STATE_RUNNING)
  {
    goRight(0);
    goLeft(0);
    Serial.println(F("Stop"));
    liefcycleState = LIFECYCLE_STATE_READY;
    hartbeat.onMs = 100;
    hartbeat.offMs = 300;
  }
}
// -- When Start/Stop button released
void onButtonReleased(uint8_t pinNr)
{
  // Not used
}