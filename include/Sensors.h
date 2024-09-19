#pragma once
#include <Config.h>
#include <Logging.h>

extern Config config;
extern Logging logging;

// Get performance monitoring global variables
extern int onboard_baro_read_time;
extern int imu_read_time;
extern int battery_voltage_read_time;
extern int outside_thermistor_read_time;
extern int gps_read_time;
extern int ranging_read_time;

class Sensors
{
private:
  static void component_info_function(String msg) { logging.component_info_function(msg); }
  static void component_error_function(String msg) { logging.component_error_function(msg); }

public:
  MS56XX onboard_baro = MS56XX("MS5611", component_info_function, component_error_function);
  LSM6DSL imu = LSM6DSL("IMU", component_info_function, component_error_function);
  Thermistor outside_thermistor = Thermistor("Outside Thermistor", component_info_function, component_error_function);
  ADC_Voltage battery_voltage_reader = ADC_Voltage("Battery Voltage", component_info_function, component_error_function);
  GPS gps = GPS("GPS", component_info_function, component_error_function);

#if RANGING == 1
  Ranging ranging = Ranging("Ranging", component_info_function, component_error_function);
#endif

  /**
   * @brief Initializes all sensor objects and gets them ready to be used.
   *
   * @param config The configuration object containing the sensor settings.
   * @return True if the initialization is successful, false otherwise.
   */
  bool begin(Config &config);

  void read_sensors();
};