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

  if (logging_action_enabled)
  {
    last_logging_millis = millis();
    logging_action();
    logging_time = millis() - last_logging_millis;
  }

  if (battery_voltage_check_enabled)
  {
    check_battery_voltage_action(sensors);
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