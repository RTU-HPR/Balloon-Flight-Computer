#pragma once
#include <Config.h>
#include <Logging.h>
#include <Communication.h>
#include <Sensors.h>

extern Config config;
extern Logging logging;

// Get performance monitoring global variables
extern int total_loop_time;
extern int continuous_actions_time;
extern int timed_actions_time;
extern int requested_actions_time;
extern int gps_read_time;
extern int ranging_read_time;
extern int logging_time;
extern int sensor_read_time;

class Actions
{
private:
  void receive_commands_action(Communication &communication);
  bool command_receive_action_enabled = true;

  void read_sensors_action(Sensors &sensors);
  bool sensor_action_enabled = true;

  void logging_action();
  bool logging_action_enabled = true;

  void check_battery_voltage_action(Sensors &sensors);
  bool battery_voltage_check_enabled = true;
  bool battery_voltage_low_last_beep_state = false;
  bool battery_voltage_low_last_beep_time = 0;

  // Continuous actions
  void run_continous_actions(Sensors &sensors, Communication &communication);

  // Timed actions
  void run_timed_actions(Sensors &sensors, Communication &communication);

  // Requested actions
  void run_requested_actions(Sensors &sensors, Communication &communication);

public:
  void run_all_actions(Sensors &sensors, Communication &communication);
};