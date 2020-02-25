/**
 * Arduino UNO
 */

#include <Arduino.h>
#include <SoftTimer.h>
#include <BlinkTask.h>

#define PIN_LED 13

#define PIN_ULTRASONIC_TRIG 7
#define PIN_ULTRASONIC_ECHO 2
#define ULTRASONIC_TIMEOUT_MICROS 2500

#define PIN_L2 12
#define PIN_L1 11
#define PIN_MD 9
#define PIN_R1 10
#define PIN_R2 8

#define PIN_MOTOR_LEFT_FWD  3
#define PIN_MOTOR_LEFT_BWD  4 // Non PWM pin!
#define PIN_MOTOR_RIGHT_FWD 5
#define PIN_MOTOR_RIGHT_BWD 6

#define MOTOR_MIN_SPEED 30
#define MOTOR_SPEED_SLOW 70
#define MOTOR_SPEED_FAST 130

#define PIN_BUTTON A0

// States
#define LIFECYCLE_STATE_READY 0
#define LIFECYCLE_STATE_RUNNING 1

int32_t getUltrasonicDistance();
void sendTrigger();
void goRight(int val);
void goLeft(int val);
void doAlways(Task* me);
void debounce();
void onButtonPressed(uint8_t pinNr);
void onButtonReleased(uint8_t pinNr);
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
bool objectInFront = false;
volatile unsigned long echoChangedTime;
unsigned long lastOnTrack = 0;

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

  pinMode(PIN_BUTTON, INPUT_PULLUP);
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

  debounce();
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
    lastOnTrack = now;
  }

  if ((now - lastOnTrack) < 3000)
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
    stop();
    return;
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

void debounce()
{
  int reading = digitalRead(PIN_BUTTON);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastReading) {
    // reset the debouncing timer
    lastDebounceTime = millis();
    // save the reading. Next time through the loop, it'll be the lastButtonState:
    lastReading = reading;
  }
  else if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        onButtonPressed(PIN_BUTTON);
      }
      else {
        onButtonReleased(PIN_BUTTON);
      }
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
void onButtonPressed(uint8_t pinNr)
{
}
// -- When Start/Stop button released
void onButtonReleased(uint8_t pinNr)
{
	if (liefcycleState == LIFECYCLE_STATE_RUNNING)
  {
    stop();
  }
	else if (liefcycleState == LIFECYCLE_STATE_READY)
  {
    Serial.println(F("Start"));
    liefcycleState = LIFECYCLE_STATE_RUNNING;
    hartbeat.onMs = 100;
    hartbeat.offMs = 100;
  }
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