#pragma once
#include <Vehicle_Config.h>
#include <Sensor_Manager.h>
#include <Radio_Manager.h>
#include <Power_Manager.h>
#include <Event_Manager.h>
#include <Led_Manager.h>
#include <Recovery_Manager.h>
#include <Logging_Manager.h>
#include <Loop_Time_Manager.h>

class Vehicle_Manager
{
private:
  Sensor_Manager sensor_manager;
  Radio_Manager radio_manager;
  Power_Manager power_manager;
  Event_Manager event_manager;
  Led_Manager led_manager;
  Recovery_Manager recovery_manager;
  Logging_Manager logging_manager;
  Loop_Time_Manager loop_time_manager;

public:
  /**
   * @brief Initialise the BFC
   */
  void initializeFC();

  /**
   * @brief Initialise the communication busses
   */
  void initializeComms();

  /**
   * @brief Read last state from flash
   */
  void configureVehicleState();

  /**
   * @brief Read sensors
   */
  void runSensorManager();

  /**
   * @brief Handle radio
   */
  void runRadioManager();

  /**
   * @brief Handle power
   */
  void runPowerManager();

  /**
   * @brief Handle events
   */
  void runEventManager();

  /**
   * @brief Handle LEDs
   */
  void runLedManager();

  /**
   * @brief Handle pyro and/or detachement
   */
  void runRecoveryManager();

  /**
   * @brief Log data
   */
  void runLoggingManager();

  /**
   * @brief Manage constant loop time
   */
  void runLoopTimeManager();

  /**
   * @brief Update watchdog
   */
  void runWatchdogUpdate();
};