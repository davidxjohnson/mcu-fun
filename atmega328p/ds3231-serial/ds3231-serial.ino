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
  setSyncProvider(RTC.get);   // set system time (get from the RTC)
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC.");
  else
    Serial.println("RTC has set the system time.");
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
    setSqw(false); /* disable clock square wave interrupts */
    clearAlarms(); /* prep before setting new alarm */

    /* calculate both current and wakeup time */
    time_t sleepTime = RTC.get();
    time_t wakeTime = addTime(sleepTime, 60); /* add 60 seconds to sleepTime */

    /* display alarm page, set the alarm and put micro-controller to sleep */
    showAlarmNotice(sleepTime, wakeTime);
    RTC.setAlarm(ALM1_MATCH_HOURS, minute(wakeTime), hour(wakeTime), 1);
    RTC.alarmInterrupt(ALARM_1, true);
    sleepForever();

    /* interrupt takes micro-controller out of sleep */
    clearAlarms();
    setSqw(true); /* enable clock square wave interrupts */
  }
}

void sleepForever() {
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  setSyncProvider(RTC.get);   /* reset system time to match RTC */
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
  char buff[50];
  snprintf(buff, sizeof buff, "Sleep: %02d:%02d:%02d - Wake: %02d:%02d:%02d", hour(sleepTime), minute(sleepTime), second(sleepTime), hour(wakeTime), minute(wakeTime), second(wakeTime));
  Serial.println(buff);
  delay(50);
}

/* show the current date/time & clock temp */
void showTimeTemp() {
  if (displayRefreshNeeded) { // only print time once a second
    char buff[50], tempString[10]; /* used to convert float to string */
    /* convert clock temp from float to 3.1 format in degrees fahrenheit */
    dtostrf((RTC.temperature() / 4.0 * 9.0 / 5.0 + 32.0), 3, 1, tempString);
    /* Get date/time/temp into a readable format and output to the serial monitor */
    snprintf(buff, sizeof buff, "Time: %02d:%02d:%02d %3s %02d/%02d/%4d - Temp: %s (F)", hour(), minute(), second(), dayShortStr(weekday()), month(), day(), year(), tempString);
    Serial.println(buff);
    displayRefreshNeeded = false;
  }
}

