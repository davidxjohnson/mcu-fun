/* Setting system time using RTC, display date/time and clock temp on console.
    Breadboard tested on Nano and Uno R3 with DS3231/AT24C32 Clock Module
      DS3231/AT24C32 Clock Module
    Author: David X Johnson - 2020
    ---------------------------------------*/

/* ---
   Arduino DS3232RTC Library
   Version 2.0
   Device specs: https://datasheets.maximintegrated.com/en/ds/DS3231.pdf
   Library source: https://github.com/JChristensen/DS3232RTC
   Pinout (I2C): SDA/A4, SCL/A5, 5V, GRD */
#include <DS3232RTC.h>
DS3232RTC RTC;

/* ---
   Arduino Wire Library allows you to communicate with I2C / TWI devices.
   Source: https://www.arduino.cc/en/Reference/Wire */
#include <Wire.h>

void setup() {
  int year, month, day, hour, min, second;
  tmElements_t tm;
  String userdata;
  char tmdata[30];
  
  RTC.begin();
  Serial.begin(115200);

  // prompt for date/time and set RTC accordingly
  Serial.println("Enter time in format \"YYYY MM DD HH MM SS\" and press enter.");
  Serial.println("Example: 2023 03 25 12 54 00");
  
  while (Serial.available() == 0) {};
  userdata = Serial.readString();
  userdata.toCharArray(tmdata,30);

  sscanf(tmdata, "%d %d %d %d %d %d", &year, &month, &day, &hour, &min, &second ); // no error checking on user input
  sprintf(tmdata,"User input interpreted: %d/%d/%d %d:%d:%d\n",year, month, day, hour, min, second);
  Serial.print(tmdata);
  tm.Year = year-1970;
  tm.Month = month;
  tm.Day = day;
  tm.Hour = hour;
  tm.Minute = min;
  tm.Second = second;
  RTC.write(tm);
  
  setSyncProvider(RTC.get);   // set system time (get from the RTC)
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC.");
  else
    Serial.println("RTC has set the system time.");
  /* setup interups */
}

/* main loop time-share across scensors and interrupts */
void loop() {
  
}
