#include "Vehicle.h"

extern Config config;
extern Logging logging;

bool Vehicle::init_communication_busses()
{
  bool success = true;

  // Wire0
  if (Wire.setSCL(config.WIRE0_SCL) && Wire.setSDA(config.WIRE0_SDA))
  {
    Wire.begin();
    logging.log_info("Wire0 communication bus initialized");
  }
  else
  {
    logging.log_error("Wire0 begin fail");
    success = false;
  }

  // SPI
  if (SPI.setRX(config.SPI0_RX) && SPI.setTX(config.SPI0_TX) && SPI.setSCK(config.SPI0_SCK))
  {
    SPI.begin();
    logging.log_info("SPI0 communication bus initialized");
  }
  else
  {
    logging.log_error("SPI begin fail");
    success = false;
  }

  return success;
}

void Vehicle::begin()
{
  // Initialize the logger
  if (!logging.begin_logging())
  {
    Serial.println("Error in initializing logger");
  }
  // From now on, use logging.log_info() and logging.log_error() instead of Serial.println()

  // Enable watchdog
  rp2040.wdt_begin(config.WATCHDOG_TIMER);
  logging.log_info("Watchdog enabled");

  // Initialize PC serial
  Serial.begin(config.PC_BAUDRATE);
  while (!Serial && config.WAIT_PC)
  {
    delay(100);
  }
  if (Serial || config.WAIT_PC)
  {
    logging.log_info("PC serial initialized");
  }

  // Initialize the communication busses
  if (!init_communication_busses())
  {
    logging.log_error("Error in initializing communication busses");
  }
  else
  {
    logging.log_info("All communication busses initialized successfully");
  }

  // Enable sensor power
  pinMode(config.SENSOR_POWER_ENABLE_PIN, OUTPUT_12MA);
  digitalWrite(config.SENSOR_POWER_ENABLE_PIN, HIGH);

  logging.log_info("Sensor power enabled");

  // Set the switch pin to input
  pinMode(config.SWITCH_PIN, INPUT_PULLUP);

  // Set the recovery channels to output and pull them low
  pinMode(config.RECOVERY_CHANNEL_MOSFET_1, OUTPUT_12MA);
  pinMode(config.RECOVERY_CHANNEL_MOSFET_2, OUTPUT_12MA);
  pinMode(config.RECOVERY_CHANNEL_SIGNAL_1, OUTPUT_12MA);
  pinMode(config.RECOVERY_CHANNEL_SIGNAL_2, OUTPUT_12MA);
  digitalWrite(config.RECOVERY_CHANNEL_MOSFET_1, LOW);
  digitalWrite(config.RECOVERY_CHANNEL_MOSFET_2, LOW);
  digitalWrite(config.RECOVERY_CHANNEL_SIGNAL_1, LOW);
  digitalWrite(config.RECOVERY_CHANNEL_SIGNAL_2, LOW);

  logging.log_info("Recovery channels set to output and pulled low");

  // Set the buzzer to output and pull it low
  pinMode(config.BUZZER_PIN, OUTPUT_12MA);
  digitalWrite(config.BUZZER_PIN, LOW);

  // Initialize the SD card for logging
  // If successful, all previous and future logs will be written to the SD card
  if (!logging.begin_sd_card(config))
  {
    logging.log_error("SD begin fail");
  }
  else
  {
    logging.log_info("SD card initialized successfully");
  }

  // Initialise the radio
  if (!communication.radio.begin(config.radio_config))
  {
    logging.log_error("Radio begin fail");
  }
  else
  {
    logging.log_info("Radio initialized successfully");
  }

  // Initialise all sensors
  if (!sensors.begin(config))
  {
    logging.log_error("Error initializing sensors");
  }
  else
  {
    logging.log_info("Sensors initialized successfully");
  }
}