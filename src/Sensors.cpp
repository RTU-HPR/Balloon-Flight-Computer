#include "Sensors.h"

// Performance monitoring
unsigned int last_on_board_baro_read_millis = 0;
unsigned int last_imu_read_millis = 0;
unsigned int last_battery_voltage_read_millis = 0;
unsigned int last_outside_thermistor_read_millis = 0;
unsigned int last_gps_read_millis = 0;
unsigned int last_ranging_read_millis = 0;

bool Sensors::begin(Config &config)
{
  bool success = true;
  // Change analogRead resolution
  // This is needed for higher accuracy battery voltage and thermistor readings
  analogReadResolution(12);

  // MAIN BOARD
  if (!onboard_baro.begin(config.ms56xx_config))
  {
    success = false;
  }

  if (!imu.begin(config.imu_config))
  {
    success = false;
  }

  if (!outside_thermistor.begin(config.thermistor_config))
  {
    success = false;
  }

  if (!battery_voltage_reader.begin(config.battery_voltage_config))
  {
    success = false;
  }

  if (!gps.begin(config.gps_config))
  {
    success = false;
  }

  if (!ranging.begin(config.master_config))
  {
    success = false;
  }

  return success;
}

void Sensors::read_sensors()
{
  last_outside_thermistor_read_millis = millis();
  outside_thermistor.read();
  outside_thermistor_read_time = millis() - last_outside_thermistor_read_millis;

  last_on_board_baro_read_millis = millis();
  onboard_baro.read(outside_thermistor.data.temperature);
  onboard_baro_read_time = millis() - last_on_board_baro_read_millis;

  last_imu_read_millis = millis();
  imu.read();
  imu_read_time = millis() - last_imu_read_millis;

  last_battery_voltage_read_millis = millis();
  battery_voltage_reader.read();
  battery_voltage_read_time = millis() - last_battery_voltage_read_millis;

  last_gps_read_millis = millis();
  gps.read();
  gps_read_time = millis() - last_gps_read_millis;

  last_ranging_read_millis = millis();
  ranging.run_master();
  ranging_read_time = millis() - last_ranging_read_millis;
}