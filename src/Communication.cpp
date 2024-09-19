#include "Communication.h"

#if VEHICLE_TYPE == 1
void Communication::set_configuration_request(byte *data, uint16_t data_length)
{
  // The first 2 bytes of the data contain the packet ID
  uint16_t packetID;
  Converter value;
  String data_format = "uint16";
  extract_ccsds_data_values(data, 1, &data_format, &value);
  packetID = value.i16;

  // Extract the configuration values from the data
  data_format = "float";
  extract_ccsds_data_values(data + 2, 1, &data_format, &value);
  received_configuration.frequency = value.f;

  data_format = "uint8";
  extract_ccsds_data_values(data + 6, 1, &data_format, &value);
  received_configuration.tx_power = value.i8;

  extract_ccsds_data_values(data + 7, 1, &data_format, &value);
  received_configuration.spreading_factor = value.i8;

  data_format = "float";
  extract_ccsds_data_values(data + 8, 1, &data_format, &value);
  received_configuration.signal_bandwidth = value.f;

  data_format = "uint8";
  extract_ccsds_data_values(data + 12, 1, &data_format, &value);
  received_configuration.coding_rate = value.i8;

  data_format = "float";
  extract_ccsds_data_values(data + 13, 1, &data_format, &value);
  received_configuration.barometer_reference = value.f;

  logging.log_info("Received configuration - Frequency: " + String(received_configuration.frequency) + " Tx Power: " + String(received_configuration.tx_power) + " Spreading Factor: " + String(received_configuration.spreading_factor) + " Signal Bandwidth: " + String(received_configuration.signal_bandwidth) + " Coding Rate: " + String(received_configuration.coding_rate) + " Barometer Reference: " + String(received_configuration.barometer_reference));
}

void Communication::configuration_request()
{
  logging.log_info("Configuration request received");
  configuration_request_flag = true;
}

void Communication::rwc_status_reques()
{
  logging.log_info("RWC status request received");
  rwc_status_request_flag = true;
}

#elif VEHICLE_TYPE == 2
void Communication::set_configuration_request(byte *data, uint16_t data_length)
{
  // The first 2 bytes of the data contain the packet ID
  uint16_t packetID;
  Converter value;
  String data_format = "uint16";
  extract_ccsds_data_values(data, 1, &data_format, &value);
  packetID = value.i16;

  // Extract the configuration values from the data
  data_format = "float";
  extract_ccsds_data_values(data + 2, 1, &data_format, &value);
  received_configuration.frequency = value.f;

  data_format = "uint8";
  extract_ccsds_data_values(data + 6, 1, &data_format, &value);
  received_configuration.tx_power = value.i8;
  
  extract_ccsds_data_values(data + 7, 1, &data_format, &value);
  received_configuration.spreading_factor = value.i8;

  data_format = "float";
  extract_ccsds_data_values(data + 8, 1, &data_format, &value);
  received_configuration.signal_bandwidth = value.f;

  data_format = "uint8";
  extract_ccsds_data_values(data + 12, 1, &data_format, &value);
  received_configuration.coding_rate = value.i8;

  data_format = "float";
  extract_ccsds_data_values(data + 13, 1, &data_format, &value);
  received_configuration.barometer_reference = value.f;

  logging.log_info("Received configuration - Frequency: " + String(received_configuration.frequency) + " Tx Power: " + String(received_configuration.tx_power) + " Spreading Factor: " + String(received_configuration.spreading_factor) + " Signal Bandwidth: " + String(received_configuration.signal_bandwidth) + " Coding Rate: " + String(received_configuration.coding_rate) + " Barometer Reference: " + String(received_configuration.barometer_reference));
}

void Communication::configuration_request()
{
  logging.log_info("Configuration request received");
  configuration_request_flag = true;
}

void Communication::heated_container_status_request()
{
  logging.log_info("Heated container status request received");
  heated_container_status_request_flag = true;
}
#endif
