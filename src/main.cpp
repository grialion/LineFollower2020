/**
 * Arduino UNO
 */

#include <Arduino.h>
#include <SoftTimer.h>
#include <BlinkTask.h>
#include <PciManager.h>
#include <Debouncer.h>

#define PIN_LED 13

#define PIN_ULTRASONIC_TRIG 7
#define PIN_ULTRASONIC_ECHO 2
#define ULTRASONIC_TIMEOUT_MICROS 2500

#define PIN_L2 8
#define PIN_L1 10
#define PIN_MD 9
#define PIN_R1 11
#define PIN_R2 12

#define PIN_MOTOR_LEFT_FWD  5
#define PIN_MOTOR_LEFT_BWD  6
#define PIN_MOTOR_RIGHT_FWD 3
#define PIN_MOTOR_RIGHT_BWD 4 // Non PWM pin!

#define MOTOR_SPEED_MIN 30
#define MOTOR_SPEED_SLOW 70
#define MOTOR_SPEED_FAST 130
#define SMALL_TURN_TIME_MS 900

#define PIN_BUTTON A0

// States
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
#define TRACK_STATE_RETURNING_FROM_L 33
#define TRACK_STATE_RETURNING_FROM_R 34

int32_t getUltrasonicDistance();
void sendTrigger();
void goRight(int val);
void goLeft(int val);
void doAlways(Task* me);
void debounce();
void onButtonPressed();
void onButtonReleased(unsigned long pressTimespan);
void onL2Entered();
void onL2Left(unsigned long pressTimespan);
bool l2LeftDelayed(Task* task);
void onR2Entered();
void onR2Left(unsigned long pressTimespan);
bool r2LeftDelayed(Task* task);
void changeTrackState(byte newState);
//void testSensors();
//void testMotors();

void stop();

int buttonState = HIGH;             // the current reading from the input pin
int lastReading = HIGH;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

BlinkTask hartbeat(PIN_LED, 100, 300);
Task alwaysTask(0, doAlways);

int rightMotorValue = 0;
int leftMotorValue = 0;
bool rightFromTrack;
bool leftFromTrack;
bool leftChangeLineMark = false;
bool rightChangeLineMark = false;
bool objectInFront = false;
volatile unsigned long echoChangedTime;
unsigned long lastOnTrack = 0;
unsigned long startTime;

byte liefcycleState = LIFECYCLE_STATE_READY;
byte trackState = TRACK_STATE_NORMAL;
Debouncer buttonDebouncer(PIN_BUTTON, MODE_CLOSE_ON_PUSH, onButtonPressed, onButtonReleased, true);
Debouncer l2Debouncer(PIN_L2, MODE_OPEN_ON_PUSH, onL2Entered, onL2Left);
Debouncer r2Debouncer(PIN_R2, MODE_OPEN_ON_PUSH, onR2Entered, onR2Left);
DelayRun l2LeftDelayedTask(300, l2LeftDelayed);
DelayRun r2LeftDelayedTask(300, r2LeftDelayed);

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Starting up..."));

  buttonDebouncer.init();
  PciManager.registerListener(PIN_BUTTON, &buttonDebouncer);
  pinMode(PIN_L1, INPUT);
  pinMode(PIN_MD, INPUT);
  pinMode(PIN_R1, INPUT);
  l2Debouncer.init();
  PciManager.registerListener(PIN_L2, &l2Debouncer);
  r2Debouncer.init();
  PciManager.registerListener(PIN_R2, &r2Debouncer);

//  pinMode(PIN_LED, OUTPUT);
//  digitalWrite(PIN_LED, HIGH);

  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);

  pinMode(PIN_MOTOR_LEFT_FWD, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_BWD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_BWD, OUTPUT);

  hartbeat.onLevel = LOW;
  hartbeat.start();
  SoftTimer.add(&alwaysTask);
  Serial.println("Ready.");
}

void doAlways(Task* me)
{
  unsigned long now = millis();
  int32_t distanceFromObject = getUltrasonicDistance();
//  Serial.println(distanceFromObject);

  if (liefcycleState != LIFECYCLE_STATE_RUNNING)
  {
    return;
  }


  if (distanceFromObject > 0)
  {
    changeTrackState(TRACK_STATE_OBJECT_IN_FRONT);
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
    if (trackState == TRACK_STATE_OBJECT_IN_FRONT)
    {
      lastOnTrack = now;
      changeTrackState(TRACK_STATE_NORMAL);
    }
    objectInFront = false;
    hartbeat.onMs = 100;
    hartbeat.offMs = 100;
  }

  // Check finnish line
  if ((trackState == TRACK_STATE_NORMAL) && rightChangeLineMark && leftChangeLineMark)
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
    lastOnTrack = now;

    if (trackOnRight && (trackState == TRACK_STATE_CHANGE_R))
    {
      changeTrackState(TRACK_STATE_RETURNING_FROM_L);
      goRight(MOTOR_SPEED_FAST);
      goLeft(MOTOR_SPEED_MIN);
    }
    else if (trackOnRight && (trackState == TRACK_STATE_CHANGE_L))
    {
      changeTrackState(TRACK_STATE_RETURNING_FROM_R);
      goRight(MOTOR_SPEED_MIN);
      goLeft(MOTOR_SPEED_FAST);
    }

    if (!trackOnLeft && (trackState == TRACK_STATE_RETURNING_FROM_L))
    {
      changeTrackState(TRACK_STATE_NORMAL);
    }
    if (!trackOnRight && (trackState == TRACK_STATE_RETURNING_FROM_R))
    {
      changeTrackState(TRACK_STATE_NORMAL);
    }
  }

  if ((now - lastOnTrack) < 3000)
  {
    if ((trackOnLeft) && (trackState != TRACK_STATE_CHANGE_L))
    {
      leftFromTrack = true;
    }
    else if ((trackOnRight) && (trackState != TRACK_STATE_CHANGE_R))
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
    stop();
    return;
  }

  if ((trackState == TRACK_STATE_NORMAL) && (!onTrack))
  {
    if (leftChangeLineMark)
    {
      changeTrackState(TRACK_STATE_CHANGE_L);
      goLeft(MOTOR_SPEED_SLOW);
      goRight(MOTOR_SPEED_FAST);
      delay(SMALL_TURN_TIME_MS);
      goLeft(MOTOR_SPEED_FAST);
      goRight(MOTOR_SPEED_FAST);
      return;
    }
    else if (rightChangeLineMark)
    {
      changeTrackState(TRACK_STATE_CHANGE_R);
      goLeft(MOTOR_SPEED_FAST);
      goRight(MOTOR_SPEED_SLOW);
      delay(SMALL_TURN_TIME_MS);
      goLeft(MOTOR_SPEED_FAST);
      goRight(MOTOR_SPEED_FAST);
      return;
    }
  }

  if (trackState == TRACK_STATE_NORMAL)
  {
    if (rightFromTrack)
    {
      // Right side has priority.
      goLeft(MOTOR_SPEED_FAST);
      goRight(onTrack ? MOTOR_SPEED_SLOW : MOTOR_SPEED_MIN);
  //Serial.print("R");
    }
    else if (leftFromTrack)
    {
      goLeft(onTrack ? MOTOR_SPEED_SLOW : MOTOR_SPEED_MIN);
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
Serial.print(millis());
Serial.print("GO R");
Serial.print(leftMotorValue);
Serial.print(" L");
Serial.println(rightMotorValue);
  }
}
void goLeft(int val)
{
  if (val != leftMotorValue)
  {
    analogWrite(PIN_MOTOR_LEFT_FWD, val);
    leftMotorValue = val;
Serial.print(millis());
Serial.print("GO R");
Serial.print(leftMotorValue);
Serial.print(" L");
Serial.println(rightMotorValue);
  }
}

int32_t getUltrasonicDistance()
{
  sendTrigger();

  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  unsigned long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_MICROS);
  return duration / 58;
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
void onButtonPressed()
{
}
 
// -- When Start/Stop button released
void onButtonReleased(unsigned long pressTimespan)
{
	if (liefcycleState == LIFECYCLE_STATE_RUNNING)
  {
    stop();
  }
	else if (liefcycleState == LIFECYCLE_STATE_READY)
  {
    Serial.println(F("Start"));
    liefcycleState = LIFECYCLE_STATE_RUNNING;
    changeTrackState(TRACK_STATE_NORMAL);
    hartbeat.onMs = 100;
    hartbeat.offMs = 100;
    lastOnTrack = millis();
  }
}

void onL2Entered()
{
  leftChangeLineMark = true;
//Serial.print("L2+");
}
void onL2Left(unsigned long pressTimespan)
{
//Serial.print("L2-");
  l2LeftDelayedTask.startDelayed();
}
bool l2LeftDelayed(Task* task)
{
  leftChangeLineMark = false;
//Serial.print("L2--");
  return false;
}

void onR2Entered()
{
  rightChangeLineMark = true;
//Serial.print("R2+");
}
void onR2Left(unsigned long pressTimespan)
{
  r2LeftDelayedTask.startDelayed();
//Serial.print("R2-");
}
bool r2LeftDelayed(Task* task)
{
  rightChangeLineMark = false;
//Serial.print("R2--");
  return false;
}

void stop()
{
    goRight(0);
    goLeft(0);
    Serial.println(F("Stop"));
    liefcycleState = LIFECYCLE_STATE_READY;
    hartbeat.onMs = 700;
    hartbeat.offMs = 700;
}

void changeTrackState(byte newState)
{
  if (trackState != newState)
  {
    Serial.print(millis());
    Serial.print("Track state: ");
    Serial.println(newState);
    trackState = newState;
  }
}

void testSensors()
{
  unsigned long start = micros();
  uint32_t l2 = digitalRead(PIN_L2);
  uint32_t l1 = digitalRead(PIN_L1);
  uint32_t md = digitalRead(PIN_MD);
  uint32_t r1 = digitalRead(PIN_R1);
  uint32_t r2 = digitalRead(PIN_R2);
  unsigned long end = micros();
  Serial.print(end - start);
  Serial.print(" ");
  Serial.print(l2);
  Serial.print(" ");
  Serial.print(l1);
  Serial.print(" ");
  Serial.print(md);
  Serial.print(" ");
  Serial.print(r1);
  Serial.print(" ");
  Serial.println(r2);
}

void testMotors()
{
  Serial.println("Motor Left FWD");
  analogWrite(PIN_MOTOR_LEFT_FWD, MOTOR_SPEED_FAST);
  delay(2000);
  analogWrite(PIN_MOTOR_LEFT_FWD, 0);
  Serial.println("Motor RIGHT FWD");
  analogWrite(PIN_MOTOR_RIGHT_FWD, MOTOR_SPEED_FAST);
  delay(1000);
  analogWrite(PIN_MOTOR_RIGHT_FWD, 0);
}