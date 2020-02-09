/**
 * https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill
 * PinOut: https://solovjov.net/reblag.dk/The-Generic-STM32F103-Pinout-Diagram.jpg
 * PinOut2: https://opencircuit.shop/resources/content/ad8542259ac19/crop/900-600/STM32-ARM-development-board-STM32F103C8T6.jpg
 * Pins:
 */

#include <Arduino.h>

#define PIN_LED PC13

#define PIN_ULTRASONIC_TRIG PB4
#define PIN_ULTRASONIC_ECHO PB3
#define ULTRASONIC_TIMEOUT_MICROS 2500

#define PIN_L2 PA0
#define PIN_L1 PA2
#define PIN_MD PA1
#define PIN_R1 PA3
#define PIN_R2 PA4

#define PIN_MOTOR_LEFT_FWD PB1
#define PIN_MOTOR_RIGHT_FWD PB0
//#define PIN_MOTOR_B1 PA7
//#define PIN_MOTOR_B2 PA6
#define MOTOR_MIN_SPEED 60
#define MOTOR_SPEED_SLOW 90

#define PIN_BUTTON PB12

float getUltrasonicDistance();
void sendTrigger();
void goRight(int val);
void goLeft(int val);
unsigned long pingStarted = 0;
volatile unsigned long echoReceived = 0;

void setup()
{
  // put your setup code here, to run once:
//  Serial.begin(19200);
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("Startup.");

  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_L2, INPUT);
  pinMode(PIN_L1, INPUT);
  pinMode(PIN_MD, INPUT);
  pinMode(PIN_R1, INPUT);
  pinMode(PIN_R2, INPUT);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);

  pinMode(PIN_MOTOR_LEFT_FWD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_FWD, OUTPUT);

  analogWriteFrequency(500);

  Serial.println("Ready.");
}

void loop()
{
  bool isToRight = digitalRead(PIN_L1);
  bool isToLeft = digitalRead(PIN_R1);
  if (isToRight)
  {
    goLeft(MOTOR_MIN_SPEED);
    goRight(MOTOR_SPEED_SLOW);
Serial.print("L");
  }
  else if (isToLeft)
  {
    goLeft(MOTOR_SPEED_SLOW);
    goRight(MOTOR_MIN_SPEED);
Serial.print("R");
  }
  else
  {
    goLeft(MOTOR_SPEED_SLOW);
    goRight(MOTOR_SPEED_SLOW);
Serial.print("=");
  }
}

int valRight = 999;
void goRight(int val)
{
  if (val != valRight)
  {
    analogWrite(PIN_MOTOR_RIGHT_FWD, val);
    valRight = val;
  }
}
int valLeft = 999;
void goLeft(int val)
{
  if (val != valLeft)
  {
    analogWrite(PIN_MOTOR_LEFT_FWD, val);
    valLeft = val;
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

  digitalWrite(PIN_LED, LOW);
  delay(200);
  digitalWrite(PIN_LED, HIGH);
  delay(200);
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
