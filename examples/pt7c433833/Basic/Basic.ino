#include <RTClib.h>

RTC_PT7C433833 rtc;

void setup() {
  Serial.begin(115200);

  initRTC();

  rtc.adjust(DateTime(2023, 3, 17, 11, 56, 00));

}

void loop() {
  DateTime old = rtc.now();
  Serial.print("Year: "); 
  Serial.println(old.year()); 
  Serial.print("Month: "); 
  Serial.println(old.month()); 
  Serial.print("Date: "); 
  Serial.println(old.day()); 
  Serial.print("Hour: "); 
  Serial.println(old.hour()); 
  Serial.print("Minute: "); 
  Serial.println(old.minute()); 
  Serial.print("Second: "); 
  Serial.println(old.second()); 

  delay(1500);

}



void initRTC()
{
  while (!rtc.begin())  // Initialize I2C communications with RTC
  {
    Serial.println(F("Unable to find GAMA_RTC_PT7C433833. Checking again in 3s."));
    delay(3000);
  }  // of loop until device is located
  Serial.println(F("GAMA_RTC_PT7C433833 initialized."));
  while (!rtc.isrunning())  // Turn oscillator on if necessary
  {
    Serial.println(F("Oscillator is off, turning it on."));
    bool deviceStatus = rtc.deviceStart();  // Start oscillator and return new state
    if (!deviceStatus) {
      Serial.println(F("Oscillator did not start, trying again."));
      delay(1000);
    }                // of if-then oscillator didn't start
  }                  // of while the oscillator is off
  Serial.println("RTC Oscillator on");
}
