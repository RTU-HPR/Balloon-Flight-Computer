#pragma once
#include <Config.h>

class Navigation
{
public:
  /**
   * @brief Structure to store all sensor data
   */
  struct NAVIGATION_DATA
  {
    Gps_Wrapper::Gps_Data gps;
  };

private:
  /**
   * @brief GPS object
   */
  Gps_Wrapper _gps;

public:
  /**
   * @brief Structure to store all sensor data
   */
  NAVIGATION_DATA navigation_data;

  /**
   * @brief Initialise the GPS
   * @param gps_config GPS config object
   * @return Whether the GPS was initialised successfully
   */
  bool beginGps(const Gps_Wrapper::Gps_Config_I2C &gps_config);

  /**
   * @brief Read the GPS data
   * @param data GPS data object
   * @return Whether the GPS data was read successfully
   * @bug To keep consistent with readRanging, I'm passing the entire navigation data structure
   * here as well, even though I can only pass the GPS data object
   */
  bool readGps(NAVIGATION_DATA &navigation_data);
};