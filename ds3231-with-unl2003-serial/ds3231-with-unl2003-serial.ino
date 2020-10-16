/* Setting system time using RTC, display date/time and clock temp on console.
    Breadboard tested on Nano and Uno R3 with DS3231/AT24C32 Clock Module
      DS3231/AT24C32 Clock Module
    Author: David X Johnson - 2020
    ---------------------------------------*/

/* ---
   Arduino DS3232RTC Library
   Device specs: https://datasheets.maximintegrated.com/en/ds/DS3231.pdf
   Library source: https://github.com/JChristensen/DS3232RTC
   Pinout (I2C): SDA/A4, SCL/A5, 5V, GRD */
#include <DS3232RTC.h>

/* ---
   RockScream Low Power library
   Library source: https://github.com/rocketscream/Low-Power */
#include "LowPower.h"

/* ---
   Adafruit Stepper Library for Arduino
   Device specs:
   Library source: https://github.com/arduino-libraries/Stepper
   Pinout (DIO): any 4 pins of D4 thru D12 */
#include <Stepper.h>
/* ---
   Create Instance of Stepper Class
   Specify Pins used for motor coils
   Connected to ULN2003 Motor Driver In1, In2, In3, In4
   Pins entered in sequence 1-3-2-4 for proper step sequencing */

const int STEPPER_PIN1 = 8;
const int STEPPER_PIN2 = 9;
const int STEPPER_PIN3 = 10;
const int STEPPER_PIN4 = 11;
const int STEPPER_SPEED = 800; // highest reliable setting for the 28BYJ-48 is 800 (5v 4.3 sec/rev), 1,400 (9V 2.8 sec/rev), 2,000 (12V 2.0 rev/sec)
const float STEPS_PER_REV = 32; // Number of steps per internal motor revolution
const float GEAR_RED = 64; //  Amount of Gear Reduction
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED; // Number of steps per geared output rotation
const int STEPPER_STEPS_REQ = STEPS_PER_OUT_REV * 1; // the number of steps needed to turn a ball valve off or on

/* ---
   Arduino Wire Library allows you to communicate with I2C / TWI devices.
   Source: https://www.arduino.cc/en/Reference/Wire */
#include <Wire.h>

#define RTC_SQW_PIN 2   // RTC provides a 1Hz interrupt signal - also known as D2 on the Nano
#define SLEEP_BUTTON_PIN 4 // button push interrupt pin - also known as D4 on Nano
#define SLEEP_BUTTON_PAUSE 1000 // 1 second pause before another button push is accepted */

/* global vars for pushbutton logic and clock interrupt */
uint8_t currentSqwState = HIGH, lastSqwState = HIGH; /* values set by interrupt handler clockTick() */
volatile bool displayRefreshNeeded = false;
unsigned long lastSleepButtonPush = millis();


void setup() {
  Serial.begin(115200);
  setFromRTC();
  /* setup interups */
  pinMode(SLEEP_BUTTON_PIN, INPUT);
  pinMode(RTC_SQW_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RTC_SQW_PIN), clockTick, HIGH);
  /* enable clock square wave interupts */
  setSqw(true);
}

/* main loop time-share across scensors and interrupts */
void loop() {
  showTimeTemp();
  sleepButtonPush();
}

/* set system time from RTC */
void setFromRTC() {
  setSyncProvider(RTC.get);   // set system time (get from the RTC)
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC.");
  else
    Serial.println("RTC has set the system time.");
}

/* 1Hz interrupt handler */
void clockTick() {
  currentSqwState = digitalRead(RTC_SQW_PIN);
  if (currentSqwState != lastSqwState) {
    lastSqwState = currentSqwState;
    displayRefreshNeeded = (currentSqwState == HIGH);
  }
}

/* sleep button-push handler */
void sleepButtonPush() {
  /* only accept first button push and filter out button bounce for a second (or more) */
  if ((digitalRead(SLEEP_BUTTON_PIN) == HIGH) && (millis() - lastSleepButtonPush > SLEEP_BUTTON_PAUSE)) {
    lastSleepButtonPush = millis();
    Serial.println("Button push detected.");
    setSqw(false); /* disable clock square wave interrupts */
    clearAlarms(); /* prep before setting new alarm */

    /* calculate both current and wakeup time */
    time_t sleepTime = RTC.get();
    time_t wakeTime = addTime(sleepTime, 120); /* add 60 seconds to sleepTime */

    /* display alarm page, set the alarm and put micro-controller to sleep */
    motorTurn (STEPPER_STEPS_REQ);
    showAlarmNotice(sleepTime, wakeTime);
    RTC.setAlarm(ALM1_MATCH_HOURS, minute(wakeTime), hour(wakeTime), 1);
    RTC.alarmInterrupt(ALARM_1, true);
    sleepForever();

    /* interrupt takes micro-controller out of sleep */
    clearAlarms();
    setSqw(true); /* enable clock square wave interrupts */
    motorTurn (-STEPPER_STEPS_REQ); // turn valve off
  }
}

void sleepForever() {
  Serial.println("System entering deep sleep.");
  delay(50);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  Serial.println("System has been woken by interrupt.");
  setFromRTC();
  delay(50);
}

/* add number of seconds to given time to get wakeup time*/
time_t addTime( time_t sleepTime, int seconds) {
  tmElements_t scratch;
  breakTime(sleepTime + seconds, scratch);
  scratch.Second = 0; /* clock alarm ignores seconds */
  return makeTime(scratch);
}

/* reset alarms, disable alarm interrupts */
void clearAlarms() {
  RTC.alarm(ALARM_1); /* Returns true/false for given alarm if it has been triggered; resets the alarm flag bit. */
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, false); /* Enable or disable an alarm "interrupt" which asserts the INT pin on the RTC. */
  RTC.alarmInterrupt(ALARM_2, false);
}

/* enable/disable 1Hz square wave signal from RTC */
void setSqw(bool enable) {
  if (enable) RTC.squareWave(SQWAVE_1_HZ);
  else RTC.squareWave(SQWAVE_NONE);
}

/* prints alarm notice message */
void showAlarmNotice( time_t sleepTime, time_t wakeTime ) {
  char buff[100];
  snprintf(buff, sizeof buff, "Sleep requested at: %02d:%02d:%02d.\nScheduled wake-up time: %02d:%02d:%02d.", hour(sleepTime), minute(sleepTime), second(sleepTime), hour(wakeTime), minute(wakeTime), second(wakeTime));
  Serial.println(buff);
}

/* show the current date/time & clock temp */
void showTimeTemp() {
  if (displayRefreshNeeded) { // only print time once a second
    char buff[100], tempString[10]; /* used to convert float to string */
    /* convert clock temp from float to 3.1 format in degrees fahrenheit */
    dtostrf((RTC.temperature() / 4.0 * 9.0 / 5.0 + 32.0), 3, 1, tempString);
    /* Get date/time/temp into a readable format and output to the serial monitor */
    snprintf(buff, sizeof buff, "Time: %02d:%02d:%02d %3s %02d/%02d/%4d - Temp: %s (F)", hour(), minute(), second(), dayShortStr(weekday()), month(), day(), year(), tempString);
    Serial.println(buff);
    displayRefreshNeeded = false;
  }
}

void motorTurn (int motorSteps) {
  char buff[100];
  snprintf(buff, sizeof buff, "Turning stepper hold-power on.\nStepper executing %04d steps.", motorSteps);
  Serial.println(buff);
  Stepper steppermotor(STEPS_PER_REV, STEPPER_PIN1, STEPPER_PIN3, STEPPER_PIN2, STEPPER_PIN4);
  steppermotor.setSpeed(STEPPER_SPEED);
  steppermotor.step(motorSteps);
  motorPowerOff();
}

void motorPowerOff () {
  Serial.println("Turning stepper hold-power off.");
  digitalWrite(STEPPER_PIN1, LOW);
  digitalWrite(STEPPER_PIN2, LOW);
  digitalWrite(STEPPER_PIN3, LOW);
  digitalWrite(STEPPER_PIN4, LOW);
}
