#include "Communication.h"

bool Communication::begin_radio(Config &config)
{
  // Initialize the radio
  if (!_radio.begin(config.radio_config))
  {
    return false;
  }

  return true;
}

bool Communication::send_radio(byte *ccsds_packet, uint16_t ccsds_packet_length)
{
  if (!_radio.initialized())
  {
    return false;
  }

  // Send the message
  return _radio.transmit_bytes(ccsds_packet, ccsds_packet_length);
}