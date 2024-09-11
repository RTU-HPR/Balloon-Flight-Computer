#pragma once
#include <Config.h>
#include <Logging.h>

extern Config config;
extern Logging logging;

class Communication
{
private:
  Radio _radio;

public:
  bool begin_radio(Config &config);
  bool send_radio(byte *ccsds_packet, uint16_t ccsds_packet_length);
};
