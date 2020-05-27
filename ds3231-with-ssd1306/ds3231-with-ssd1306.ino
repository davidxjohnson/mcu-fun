/* Setting system time using RTC, display date/time and temp on OLED display.
    Breadboard tested on Nano and Uno R3 with:
      DS3231/AT24C32 Clock Module
      SSD1302 OLED 128X64 LCD Screen Module
    Author: David X Johnson - 2020
    ---------------------------------------*/

/* ---
   Arduino DS3232RTC Library
   Device specs: https://datasheets.maximintegrated.com/en/ds/DS3231.pdf
   Library source: https://github.com/JChristensen/DS3232RTC
   Pinout (I2C): SDA/A4, SCL/A5, 5V, GRD */
#include <DS3232RTC.h>

/* ---
   Adafruit OLED Library for Monochrome OLEDs based on SSD1306 drivers
   Device specs: https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
   Library source: https://github.com/adafruit/Adafruit_SSD1306
   Pinout (I2C): SDA/A4, SCL/A5, 5V, GRD */
#include <Adafruit_SSD1306.h>
#define SSD1306_WIDTH 128 // OLED display width, in pixels
#define SSD1306_HEIGHT 64 // OLED display height, in pixels
#define SSD1306_RESET   4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SSD1306_FONT_SCALE_1 1 // small font scale
#define SSD1306_FONT_SCALE_2 2 // medium font scale
#define SSD1306_ADDR 0x3C
Adafruit_SSD1306 ssd1306(SSD1306_WIDTH, SSD1306_HEIGHT, &Wire, SSD1306_RESET); /* initialize OLED display driver */

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

  /* SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
     Address for 128x64 OLED on the I2C bus */
  if (!ssd1306.begin(SSD1306_SWITCHCAPVCC, SSD1306_ADDR)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)delay(1000); // Don't proceed, loop forever
  }
  /* clear Adafruit splash screen. */
  paintBlankPage();
  /* setup interups */
  pinMode(SLEEP_BUTTON_PIN, INPUT);
  pinMode(RTC_SQW_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RTC_SQW_PIN), clockTick, HIGH);
  /* enable clock square wave interupts */
  clearAlarms();
  setSqw(true);
}

/* main loop time-share across scensors and interrupts */
void loop() {
  paintClockPage();
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
    time_t wakeTime = addTime(sleepTime, 120); /* add x seconds to sleepTime */

    /* display alarm page, set the alarm and put micro-controller to sleep */
    paintAlarmPage(sleepTime, wakeTime);
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

/* clear the oled display */
void paintBlankPage() {
  ssd1306.clearDisplay();
  ssd1306.display();
}

/* paints alarm test page */
void paintAlarmPage( time_t sleepTime, time_t wakeTime ) {
  char msgString[30], timeStamp[30];

  /* update display with sleep start time */
  ssd1306.clearDisplay();
  ssd1306.drawRoundRect(0, 0, ssd1306.width(), ssd1306.height(), ssd1306.height() / 8, SSD1306_WHITE);
  sprintf(msgString, "RTC Alarm Test");
  printAtPoint(24, 8, msgString, SSD1306_FONT_SCALE_1, SSD1306_WHITE);
  formatTime(timeStamp, sleepTime, 8);
  sprintf(msgString, "S:%s", timeStamp);
  printAtPoint(5, 22, msgString, SSD1306_FONT_SCALE_2, SSD1306_WHITE);

  /* update display with wakeup time */
  formatTime(timeStamp, wakeTime, 8);
  sprintf(msgString, "W:%s", timeStamp);
  printAtPoint(5, 40, msgString, SSD1306_FONT_SCALE_2, SSD1306_WHITE);
  ssd1306.display();
  delay(5000); /* wait a little to clear the display before sleeping (to save energy) */
  paintBlankPage();
}

/* update the display with the current date/time & clock temp */
void paintClockPage() {
  if (displayRefreshNeeded) { // only print time once a second
    char dateString[12];
    char timeString[10];
    char tempString[10];
    /* Get date/time into a readable format */
    sprintf(dateString, "%4d/%02d/%02d", year(), month(), day());
    sprintf(timeString, "%02d:%02d:%02d", hour(), minute(), second());
    /* get temp into readable format */
    dtostrf((RTC.temperature() / 4.0 * 9.0 / 5.0 + 32.0), 3, 1, tempString);
    sprintf(tempString, "%s F", tempString);
    /* paint the display */
    ssd1306.clearDisplay();
    ssd1306.drawRoundRect(0, 0, ssd1306.width(), ssd1306.height(), ssd1306.height() / 8, SSD1306_WHITE);
    printAtPoint(6, 8, dateString, SSD1306_FONT_SCALE_1, SSD1306_WHITE);
    printAtPoint(17, 28, timeString, SSD1306_FONT_SCALE_2, SSD1306_WHITE);
    printAtPoint(87, 8, tempString, SSD1306_FONT_SCALE_1, SSD1306_WHITE);
    ssd1306.display();
    displayRefreshNeeded = false;
  }
}

/* helper function for placing text at a specific location with scale and color. */
void printAtPoint(int x, int y, char* string, int scale, int color  ) {
  ssd1306.setTextColor(color);
  ssd1306.setTextSize(scale);
  ssd1306.setCursor(x, y);
  ssd1306.println(string);
}

// format a time_t value, return the formatted string in buf (must be at least 25 bytes)
void formatTime(char *buf, time_t t, int length)
{
  sprintf(buf, "%.2d:%.2d:%.2d %s %.2d %s %d", hour(t), minute(t), second(t), dayShortStr(weekday(t)), day(t), monthShortStr(month(t)), year(t));
  buf[length] = 0;
}
