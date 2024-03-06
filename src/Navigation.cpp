#include <Navigation.h>

bool Navigation::beginGps(const Gps_Wrapper::Gps_Config_I2C &gps_config)
{
  _gps = Gps_Wrapper(nullptr, "GPS");
  unsigned long start = millis();
  while (!_gps.begin(gps_config))
  {
    if (millis() - start > 10000)
    {
      Serial.println("Auto GPS begin failed");
      Serial.println("Reseting GPS module");
      // Reset the gps module
      pinMode(6, OUTPUT_12MA);
      digitalWrite(6, LOW);
      delay(1000);
      digitalWrite(6, HIGH);
      delay(1000);
      digitalWrite(6, LOW);
      if (!_gps.begin(gps_config))
      {
        Serial.println("GPS begin failed after reset");
        return false;
      }
      return true;
    }
    // Serial.println("GPS begin failed, retrying");
  }

  return true;
}

bool Navigation::readGps(NAVIGATION_DATA &navigation_data)
{
  bool position_valid = false;
  bool time_valid = false;
  if (!_gps.read(navigation_data.gps, position_valid, time_valid))
  {
    return false;
  }
  return true;
}