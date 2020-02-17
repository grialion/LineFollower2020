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

#define PIN_LED PC13

#define PIN_ULTRASONIC_TRIG PB11
#define PIN_ULTRASONIC_ECHO PB10 // Must be 5V tolerant pin!
//#define ULTRASONIC_TIMEOUT_MICROS 2500
#define ULTRASONIC_TIMEOUT_MICROS 4000

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
#define MOTOR_SPEED_FAST 255

#define PIN_BUTTON PB12

// States
#define LIFECYCLE_STATE_READY 0
#define LIFECYCLE_STATE_RUNNING 1

int32_t getUltrasonicDistance();
void echoChanged(void);
void sendTrigger();
void goRight(int val);
void goLeft(int val);
void doAlways(Task* me);

FTDebouncer pinDebouncer(100);
BlinkTask hartbeat(PIN_LED, 100, 300);
Task alwaysTask(0, doAlways);

int rightMotorValue = 0;
int leftMotorValue = 0;
bool rightFromTrack;
bool leftFromTrack;
bool objectInFront = false;
volatile unsigned long echoChangedTime;

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
  attachInterrupt(PIN_ULTRASONIC_ECHO, echoChanged, CHANGE);
  SoftTimer.add(&alwaysTask);
  Serial.println("Ready.");
}

void doAlways(Task* me)
{
  int32_t distanceFromObject = getUltrasonicDistance();
//  Serial.println(distanceFromObject);

  pinDebouncer.run();
  if (liefcycleState != LIFECYCLE_STATE_RUNNING)
  {
    return;
  }


  if (distanceFromObject > 0)
  {
    objectInFront = true;
    hartbeat.onMs = 30;
    hartbeat.offMs = 30;
    if (distanceFromObject < 10)
    {
      goLeft(0);
      goRight(0);
      return;
    }
  }
  else
  {
    objectInFront = false;
    hartbeat.onMs = 100;
    hartbeat.offMs = 100;
  }

  // Read line sensors
  bool onTrack = digitalRead(PIN_MD);
  bool trackOnLeft = digitalRead(PIN_L1);
  bool trackOnRight = digitalRead(PIN_R1);
  if (onTrack)
  {
    if (trackOnLeft)
    {
      leftFromTrack = true;
    }
    else if (trackOnRight)
    {
      rightFromTrack = true;
    }
    else
    {
      rightFromTrack = false;
      leftFromTrack = false;
    }
  }
  if (rightFromTrack)
  {
    // Right side has priority.
    goLeft(MOTOR_SPEED_FAST);
    goRight(onTrack ? MOTOR_SPEED_SLOW : MOTOR_MIN_SPEED);
//Serial.print("R");
  }
  else if (leftFromTrack)
  {
    goLeft(onTrack ? MOTOR_SPEED_SLOW : MOTOR_MIN_SPEED);
    goRight(MOTOR_SPEED_FAST);
//Serial.print("L");
  }
  else
  {
    goLeft(MOTOR_SPEED_FAST);
    goRight(MOTOR_SPEED_FAST);
//Serial.print("=");
  }
}

void goRight(int val)
{
  if (val != rightMotorValue)
  {
    analogWrite(PIN_MOTOR_RIGHT_FWD, val);
    rightMotorValue = val;
  }
}
void goLeft(int val)
{
  if (val != leftMotorValue)
  {
    analogWrite(PIN_MOTOR_LEFT_FWD, val);
    leftMotorValue = val;
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

int32_t getUltrasonicDistance()
{
//  attachInterrupt(PIN_ULTRASONIC_ECHO, echoChanged, CHANGE);
  sendTrigger();

  unsigned long startTime = micros();
  unsigned long prevEchoChangedTime = echoChangedTime;

  // TODO: might want to asynchronous working instead of using wait loops.
  while(1)
  {
    if (prevEchoChangedTime != echoChangedTime)
    {
      break;
    }
    if ((micros()-startTime) > ULTRASONIC_TIMEOUT_MICROS)
    {
      return -1;
    }
  }

  unsigned long frameStartBegins = prevEchoChangedTime = echoChangedTime;

  while(1)
  {
    if (prevEchoChangedTime != echoChangedTime)
    {
      break;
    }
    if ((micros()-startTime) > ULTRASONIC_TIMEOUT_MICROS)
    {
      return -1;
    }
  }

//  detachInterrupt(PIN_ULTRASONIC_ECHO);

  long time = echoChangedTime - frameStartBegins;

  return time / 59;
}
void echoChanged(void)
{
  echoChangedTime = micros();
}

void sendTrigger()
{
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
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