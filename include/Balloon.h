#pragma once
#include <Config.h>
#include <Sensors.h>
#include <Navigation.h>
#include <Actions.h>
#include <Communication.h>
#include <Logging.h>

// Get servo objects
extern Servo servo_1;
extern Servo servo_2;

class Balloon
{
private:
  /**
   * @brief Initialise the HardwareSerial, SPI, I2C communication busses
   */
  bool initCommunicationBusses();

public:
  Config config;
  Sensors sensors;
  Navigation navigation;
  Actions actions;
  Communication communication;
  Logging logging;

  /**
   * @brief Initialise the BFC
   */
  void begin();
};
