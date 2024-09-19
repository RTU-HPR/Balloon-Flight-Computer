#include <Actions.h>
#include <Config.h>
#include <Logging.h>

// Performance monitoring
unsigned long last_sensor_read_millis = 0;
unsigned long last_logging_millis = 0;

extern Config config;
extern Logging logging;

void Actions::run_continous_actions(Sensors &sensors, Communication &communication)
{
  // Receive any commands
  if (commandReceiveActionEnabled)
  {
    command_receive_action(communication);
  }

  // Run the sensor action
  if (sensorActionEnabled)
  {
    last_sensor_read_millis = millis();
    read_sensors_action(sensors);
    sensor_read_time = millis() - last_sensor_read_millis;
  }

  // Check the battery voltage
  if (batteryVoltageCheckEnabled)
  {
    read_battery_voltage_action(sensors);
  }
}

void Actions::command_receive_action(Communication &communication)
{
  // ...
}
void Actions::read_sensors_action(Sensors &sensors)
{
  // Read all sensors
  sensors.read_sensors();
}

void Actions::logging_action()
{
  // ...
}

void Actions::read_battery_voltage_action(Sensors &sensors)
{
  // Check if the battery voltage is below the threshold
  if (sensors.battery_voltage_reader.data.voltage <= config.BATTERY_LOW_VOLTAGE)
  {
    if (millis() - batteryVoltageLowLastBeepTime >= config.BATTERY_LOW_BEEP_INTERVAL)
    {
      batteryVoltageLowLastBeepState = !batteryVoltageLowLastBeepState;
      digitalWrite(config.BUZZER_PIN, batteryVoltageLowLastBeepState);
      batteryVoltageLowLastBeepTime = millis();
    }
  }
  else
  {
    digitalWrite(config.BUZZER_PIN, LOW);
    batteryVoltageLowLastBeepState = false;
  }
}