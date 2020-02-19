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

#define PIN_BUTTON PB12

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
#define SMALL_TURN_TIME_MS 100

// States over lifetime
#define LIFECYCLE_STATE_READY 0
#define LIFECYCLE_STATE_RUNNING 1

// States over track
#define TRACK_STATE_NORMAL           0
#define TRACK_STATE_OBJECT_IN_FRONT 10
#define TRACK_STATE_BYPASS_START    20
#define TRACK_STATE_BYPASS_STAGE1   21
#define TRACK_STATE_BYPASS_STAGE2   22
#define TRACK_STATE_BYPASS_STAGE3   23
#define TRACK_STATE_CHANGE_L        31
#define TRACK_STATE_CHANGE_R        32

int32_t getUltrasonicDistance();
void echoChanged(void);
void sendTrigger();
void goRight(int val);
void goLeft(int val);
void stop();
void doAlways(Task* me);

FTDebouncer pinDebouncer(60);
BlinkTask hartbeat(PIN_LED, 100, 300);
Task alwaysTask(0, doAlways);

int rightMotorValue = 0;
int leftMotorValue = 0;
bool rightFromTrack;
bool leftFromTrack;
bool leftChangeLineMark = false;
bool rightChangeLineMark = false;
volatile unsigned long echoChangedTime;
unsigned long startTime;

byte lifecycleState = LIFECYCLE_STATE_READY;
byte trackState = TRACK_STATE_NORMAL;

void setup()
{
  Serial.begin(115200);
  while(!Serial.availableForWrite())
  {
    delay(10);
  }
  Serial.println();
  Serial.println(F("Starting up..."));

  pinDebouncer.addPin(PIN_BUTTON, HIGH, INPUT_PULLUP);

  pinMode(PIN_L2, INPUT);
  pinMode(PIN_L1, INPUT);
  pinMode(PIN_MD, INPUT);
  pinMode(PIN_R1, INPUT);
  pinMode(PIN_R2, INPUT);
  pinDebouncer.addPin(PIN_L2, HIGH, INPUT);
  pinDebouncer.addPin(PIN_R2, HIGH, INPUT);

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
  if (lifecycleState != LIFECYCLE_STATE_RUNNING)
  {
    return;
  }
  unsigned long now = millis();

  if (distanceFromObject > 0)
  {
    trackState = TRACK_STATE_OBJECT_IN_FRONT;
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
    trackState = TRACK_STATE_NORMAL;
    hartbeat.onMs = 100;
    hartbeat.offMs = 100;
  }

  // Check finnish line
  if (rightChangeLineMark && leftChangeLineMark)
  {
    if ((now - startTime) > 3000)
    {
      Serial.println(F("Finnish line mark reached"));
      stop();
      return;
    }
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
  else
  {
    if (leftChangeLineMark)
    {
      goLeft(MOTOR_SPEED_SLOW);
      goRight(MOTOR_SPEED_FAST);
      delay(SMALL_TURN_TIME_MS);
      goLeft(MOTOR_SPEED_FAST);
      goRight(MOTOR_SPEED_FAST);
      trackState = TRACK_STATE_CHANGE_L;
    }
    else if (rightChangeLineMark)
    {
      goLeft(MOTOR_SPEED_FAST);
      goRight(MOTOR_SPEED_SLOW);
      delay(SMALL_TURN_TIME_MS);
      goLeft(MOTOR_SPEED_FAST);
      goRight(MOTOR_SPEED_FAST);
      trackState = TRACK_STATE_CHANGE_R;
    }
  }
  
  if ((trackState == TRACK_STATE_CHANGE_L) || (trackState == TRACK_STATE_CHANGE_R))
  {
    if (onTrack)
    {
      // Back on track :)
      trackState = TRACK_STATE_NORMAL;
    }
  }

  if (trackState == TRACK_STATE_NORMAL)
  {
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

void onButtonPressed(uint8_t pinNr)
{
  if (pinNr == PIN_L2)
  {
    leftChangeLineMark = true;
  }
  else if (pinNr == PIN_R2)
  {
    rightChangeLineMark = true;
  }
}

void onButtonReleased(uint8_t pinNr)
{
  if (pinNr == PIN_BUTTON)
  {
    // -- When Start/Stop button pressed
    if (lifecycleState == LIFECYCLE_STATE_READY)
    {
      Serial.println(F("Start"));
      lifecycleState = LIFECYCLE_STATE_RUNNING;
      hartbeat.onMs = 100;
      hartbeat.offMs = 100;
      startTime = millis();
    }
    // else if (lifecycleState == LIFECYCLE_STATE_RUNNING)
    // {
    //   stop();
    // }
  }
  else if (pinNr == PIN_L2)
  {
    leftChangeLineMark = false;
  }
  else if (pinNr == PIN_R2)
  {
    rightChangeLineMark = false;
  }
}

void stop()
{
  goRight(0);
  goLeft(0);
  Serial.println(F("Stop"));
  lifecycleState = LIFECYCLE_STATE_READY;
  hartbeat.onMs = 700;
  hartbeat.offMs = 700;
}