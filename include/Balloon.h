#pragma once
#include <Sensors.h>
#include <Actions.h>
#include <Communication.h>

class Balloon
{
private:
  /**
   * @brief Initialise the HardwareSerial, SPI, I2C communication busses
   */
  bool init_communication_busses();

public:
  Sensors sensors;
  Actions actions;
  Communication communication;

  /**
   * @brief Initialise the BFC
   */
  void begin();
};
