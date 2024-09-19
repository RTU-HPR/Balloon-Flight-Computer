#pragma once
#include <Config.h>
#include <Logging.h>

extern Config config;
extern Logging logging;

class Communication
{
private:
  // ...
public:
  Radio radio;

#if VEHICLE_TYPE == 1
  struct received_configuration
  {
    float frequency;
    uint8_t tx_power;
    uint8_t spreading_factor;
    float signal_bandwidth;
    uint8_t coding_rate;
    float barometer_reference;
  } received_configuration;

  bool set_configuration_request_flag = false;
  bool configuration_request_flag = false;
  bool rwc_status_request_flag = false;

  void set_configuration_request(byte *data, uint16_t data_length);
  void configuration_request();
  void rwc_status_reques();

#elif VEHICLE_TYPE == 2
  struct received_configuration
  {
    float frequency;
    uint8_t tx_power;
    uint8_t spreading_factor;
    float signal_bandwidth;
    uint8_t coding_rate;
    float barometer_reference;
  } received_configuration;

  bool set_configuration_request_flag = false;
  bool configuration_request_flag = false;
  bool heated_container_status_request_flag = false;

  void set_configuration_request(byte *data, uint16_t data_length);
  void configuration_request();
  void heated_container_status_request();
#endif
};
