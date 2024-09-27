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
  if (command_receive_action_enabled)
  {
    receive_commands_action(communication);
  }

  if (sensor_action_enabled)
  {
    last_sensor_read_millis = millis();
    read_sensors_action(sensors);
    sensor_read_time = millis() - last_sensor_read_millis;
  }

  if (battery_voltage_check_enabled)
  {
    check_battery_voltage_action(sensors);
  }

  if (switch_state_check_enabled)
  {
    check_switch_state_action();
  }

  if (telemetry_action_enabled)
  {
    telemetry_action(sensors);
  }

  if (logging_action_enabled)
  {
    last_logging_millis = millis();
    logging_action();
    logging_time = millis() - last_logging_millis;
  }
}

void Actions::receive_commands_action(Communication &communication)
{
  if (communication.radio.receive_bytes())
  {
    // Parse the received CCSDS packet
    uint16_t apid;
    uint16_t sequence_count;
    uint32_t epoch_time;
    uint16_t subseconds;
    byte *data[244];
    uint16_t data_length;

    parse_ccsds_telemetry(
        communication.radio.received_data.bytes,
        apid,
        sequence_count,
        epoch_time,
        subseconds,
        *data,
        data_length);

    // Extract packetID, from the first 2 bytes of the data
    Converter value;
    String data_format = "uint16";
    extract_ccsds_data_values(*data, 1, &data_format, &value);

    uint16_t packet_id = value.i16;

    logging.log_info("Received telecommand - Packet ID: " + String(packet_id) + " Sequence Count: " + String(sequence_count) + " Epoch Time: " + String(epoch_time) + " Subseconds: " + String(subseconds));

    // Perform the action based on the packet ID
#if VEHICLE_TYPE == 1
    // Balloon telecommand APID is 0
    if (apid != 0)
    {
      logging.log_info("APID does not match Balloon vehicle telecommand definition - APID: " + String(apid));
      return;
    }

    if (packet_id == 0)
    {
      communication.set_configuration_request(*data, data_length);
    }
    else if (packet_id == 1)
    {
      communication.configuration_request();
    }
    else if (packet_id == 2)
    {
      communication.rwc_status_reques();
    }
    else
    {
      logging.log_error("Unknown packet ID: " + String(packet_id));
    }
#elif VEHICLE_TYPE == 2
    // Payload telecommand APID is 100
    if (apid != 100)
    {
      logging.log_info("APID does not match the Payload vehicle telecommand definition - APID: " + String(apid));
      return;
    }
    if (packet_id == 0)
    {
      communication.set_configuration_request(*data, data_length);
    }
    if (packet_id == 1)
    {
      communication.configuration_request();
    }
    else if (packet_id == 2)
    {
      communication.heated_container_status_request();
    }
    else
    {
      logging.log_error("Unknown packet ID: " + String(packet_id));
    }
#endif
  }
}

void Actions::read_sensors_action(Sensors &sensors)
{
  sensors.read_sensors();
}

void Actions::logging_action()
{
  logging.write_to_file();
}

void Actions::check_battery_voltage_action(Sensors &sensors)
{
  if (sensors.battery_voltage_reader.data.voltage <= config.BATTERY_LOW_VOLTAGE)
  {
    if (millis() - battery_voltage_low_last_beep_time >= config.BATTERY_LOW_BEEP_INTERVAL)
    {
      battery_voltage_low_last_beep_state = !battery_voltage_low_last_beep_state;
      digitalWrite(config.BUZZER_PIN, battery_voltage_low_last_beep_state);
      battery_voltage_low_last_beep_time = millis();
    }
  }
  else
  {
    digitalWrite(config.BUZZER_PIN, LOW);
    battery_voltage_low_last_beep_state = false;
  }
}

void Actions::check_switch_state_action()
{
  bool current_state = !digitalRead(config.SWITCH_PIN);

  if (current_state != switch_current_state)
  {
    logging.log_info("Switch state changed - New state: " + String(current_state ? "ON" : "OFF"));
    switch_current_state = current_state;
  }
}

void Actions::telemetry_action(Sensors &sensors)
{
  // CSV header values are defined in Config.h for each vehicle type
  String telemetry_data = "";

#if VEHICLE_TYPE == 1
  // INIT_HEADER_VALUES
  telemetry_data += String(telemetry_index) + ",";
  telemetry_data += String(millis()) + ",";
  // GPS_HEADER_VALUES
  telemetry_data += String(sensors.gps.data.epoch_microseconds) + ",";
  telemetry_data += String(sensors.gps.data.lat, 6) + ",";
  telemetry_data += String(sensors.gps.data.lng, 6) + ",";
  telemetry_data += String(sensors.gps.data.altitude) + ",";
  telemetry_data += String(sensors.gps.data.ground_speed) + ",";
  telemetry_data += String(sensors.gps.data.satellites) + ",";
  telemetry_data += String(sensors.gps.data.pdop) + ",";
  // ONBOARD_BARO_HEADER_VALUES
  telemetry_data += String(sensors.onboard_baro.data.temperature, 2) + ",";
  telemetry_data += String(sensors.onboard_baro.data.pressure, 2) + ",";
  telemetry_data += String(sensors.onboard_baro.data.altitude, 2) + ",";
  // OUTSIDE_THERMISTOR_HEADER_VALUES
  telemetry_data += String(sensors.outside_thermistor.data.temperature, 2) + ",";
  // IMU_HEADER_VALUES
  telemetry_data += String(sensors.imu.data.accel_x, 2) + ",";
  telemetry_data += String(sensors.imu.data.accel_y, 2) + ",";
  telemetry_data += String(sensors.imu.data.accel_z, 2) + ",";
  telemetry_data += String(sensors.imu.data.gyro_x, 2) + ",";
  telemetry_data += String(sensors.imu.data.gyro_y, 2) + ",";
  telemetry_data += String(sensors.imu.data.gyro_z, 2) + ",";
  telemetry_data += String(sensors.imu.data.temperature, 2) + ",";
  // BATTERY_VOLTAGE_HEADER_VALUES
  telemetry_data += String(sensors.battery_voltage_reader.data.voltage, 2) + ",";
  // SWITCH_STATE_HEADER_VALUES
  telemetry_data += String(switch_current_state) + ",";
  // PERFORMANCE_HEADER_VALUES
  telemetry_data += String(rp2040.getFreeHeap()) + ",";
  telemetry_data += String(total_loop_time) + ",";
  telemetry_data += String(continuous_actions_time) + ",";
  telemetry_data += String(timed_actions_time) + ",";
  telemetry_data += String(requested_actions_time) + ",";
  telemetry_data += String(gps_read_time) + ",";
  telemetry_data += String(logging_time) + ",";
  telemetry_data += String(sensor_read_time) + ",";
  telemetry_data += String(onboard_baro_read_time) + ",";
  telemetry_data += String(imu_read_time);
#elif VEHICLE_TYPE == 2
  // INIT_HEADER_VALUES
  telemetry_data += String(telemetry_index) + ",";
  telemetry_data += String(millis()) + ",";
  // GPS_HEADER_VALUES
  telemetry_data += String(sensors.gps.data.epoch_microseconds) + ",";
  telemetry_data += String(sensors.gps.data.lat, 6) + ",";
  telemetry_data += String(sensors.gps.data.lng, 6) + ",";
  telemetry_data += String(sensors.gps.data.altitude) + ",";
  telemetry_data += String(sensors.gps.data.ground_speed) + ",";
  telemetry_data += String(sensors.gps.data.satellites) + ",";
  telemetry_data += String(sensors.gps.data.pdop) + ",";
  // ONBOARD_BARO_HEADER_VALUES
  telemetry_data += String(sensors.onboard_baro.data.temperature, 2) + ",";
  telemetry_data += String(sensors.onboard_baro.data.pressure, 2) + ",";
  telemetry_data += String(sensors.onboard_baro.data.altitude, 2) + ",";
  // OUTSIDE_THERMISTOR_HEADER_VALUES
  telemetry_data += String(sensors.outside_thermistor.data.temperature, 2) + ",";
  // IMU_HEADER_VALUES
  telemetry_data += String(sensors.imu.data.accel_x, 2) + ",";
  telemetry_data += String(sensors.imu.data.accel_y, 2) + ",";
  telemetry_data += String(sensors.imu.data.accel_z, 2) + ",";
  telemetry_data += String(sensors.imu.data.gyro_x, 2) + ",";
  telemetry_data += String(sensors.imu.data.gyro_y, 2) + ",";
  telemetry_data += String(sensors.imu.data.gyro_z, 2) + ",";
  telemetry_data += String(sensors.imu.data.temperature, 2) + ",";
  // BATTERY_VOLTAGE_HEADER_VALUES
  telemetry_data += String(sensors.battery_voltage_reader.data.voltage, 2) + ",";
  // SWITCH_STATE_HEADER_VALUES
  telemetry_data += String(switch_current_state) + ",";
  // PERFORMANCE_HEADER_VALUES
  telemetry_data += String(rp2040.getFreeHeap()) + ",";
  telemetry_data += String(total_loop_time) + ",";
  telemetry_data += String(continuous_actions_time) + ",";
  telemetry_data += String(timed_actions_time) + ",";
  telemetry_data += String(requested_actions_time) + ",";
  telemetry_data += String(gps_read_time) + ",";
  telemetry_data += String(logging_time) + ",";
  telemetry_data += String(sensor_read_time) + ",";
  telemetry_data += String(onboard_baro_read_time) + ",";
  telemetry_data += String(imu_read_time);
#endif

  logging.log_telemetry(telemetry_data);
  telemetry_index++;
}