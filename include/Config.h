#pragma once
// Main libraries
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LittleFS.h>
#include <SDFS.h>
#include <cppQueue.h>
#include <Servo.h>

// Public libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSL.h>
#include <NTC_Thermistor.h>
#include <SimpleKalmanFilter.h>

// Our wrappers
#include <RadioLib_wrapper.h>
#include <Gps_Wrapper.h>
#include <Sd_card_wrapper.h>

// Our functions
#include <Ccsds_packets.h>

// Our sensor libaries
#include <MS56XX.h>
#include <Adc_Voltage.h>

// Used radio module
#define radio_module SX1268

class Config
{
public:
  bool WAIT_PC = false;
  const bool LOG_TO_STORAGE = true;

  // 433 MHz LoRa
  RadioLib_Wrapper<radio_module>::Radio_Config radio_config{
      .frequency = 434.0,
      .cs = 5,
      .dio0 = 8,
      .dio1 = 9,
      .family = RadioLib_Wrapper<radio_module>::Radio_Config::Chip_Family::Sx126x,
      .rf_switching = RadioLib_Wrapper<radio_module>::Radio_Config::Rf_Switching::Dio2,
      // .rx_enable = 0, // only needed if rf_switching = gpio
      // .tx_enable = 0, // only needed if rf_switching = gpio
      .reset = 7,
      .sync_word = 0xF4,
      .tx_power = 22,
      .spreading = 11,
      .coding_rate = 8,
      .signal_bw = 62.5,
      .frequency_correction = false,
      .spi_bus = &SPI,
  };

  // GPS
  Gps_Wrapper::Gps_Config_I2C gps_config{
      .config = {
          .timeout = 5000,                       // Time it takes for anything to timeout
          .measurement_rate = 500,               // how often measurement will be taken in ms
          .navigation_frequency = 2,             // how often tu updated navigation in s
          .dynamic_model = DYN_MODEL_AIRBORNE2g, // DYN_MODEL_AIRBORNE2g
          .com_settings = COM_TYPE_UBX,          // COM_TYPE_UBX
          .auto_pvt = true                       // for neo6m dont use this
      },
      .wire = &Wire,
      .i2c_address = 0x42, // Default
  };

  // SD card
  String TELMETRY_FILE_HEADER = "index,time_on_ms,gps_epoch_time,gps_hour:gps_minute:gps_second,gps_lat,gps_lng,gps_altitude,gps_speed,gps_satellites,gps_heading,gps_pdop,onboard_baro_temp,onboard_baro_pressure,onboard_baro_altitude,outside_thermistor_temp,imu_accel_x,imu_accel_y,imu_accel_z,imu_heading,imu_pitch,imu_roll,imu_gyro_x,imu_gyro_y,imu_gyro_z,imu_temp,battery_voltage,used_heap,loop_time,continuous_actions_time,timed_actions_time,requested_actions_time,gps_read_time,logging_time,sensor_read_time,on_board_baro_read_time,imu_read_time,battery_voltage_read_time,outside_thermistor_read_time";
  String INFO_FILE_HEADER = "time,info";
  String ERROR_FILE_HEADER = "time,error";
  String CONFIG_FILE_HEADER = "descent_flag,remaining_descent_time,parachutes_deployed_flag";

  struct Config_File_Values
  {
    int descent_flag;
    long remaining_descent_time;
    int parachutes_deployed_flag;
  };

  Config_File_Values config_file_values = {
      .descent_flag = 0,
      .remaining_descent_time = DESCENT_TIME_BEFORE_PARACHUTE_DEPLOYMENT,
      .parachutes_deployed_flag = 0,
  };

  SD_Card_Wrapper::Config sd_card_config = {
      // spi bus
      .spi_bus = &SPI,
      .cs_pin = 14,
      .data_file_path_base = "/BFC_TELEMETRY_",
      .info_file_path_base = "/BFC_INFO_",
      .error_file_path_base = "/BFC_ERROR_",
      .config_file_path = "/BFC_CONFIG",
      .data_file_header = TELMETRY_FILE_HEADER,
      .info_file_header = INFO_FILE_HEADER,
      .error_file_header = ERROR_FILE_HEADER,
      .config_file_header = CONFIG_FILE_HEADER,
  };

  // MS56XX
  MS56XX::MS56XX_Config ms56xx_config = {
      .wire = &Wire,
      .i2c_address = MS56XX::MS56XX_I2C_ADDRESS::I2C_0x76, // or 0x76
      .ms56xx_type = MS56XX::MS56XX_TYPE::MS5611,          // or MS5607
      .oversampling = MS56XX::MS56XX_OVERSAMPLING::OSR_STANDARD,
  };

  // IMU
  struct IMU_Config
  {
    TwoWire *wire;
    int i2c_address;
  };

  IMU_Config imu_config = {
      .wire = &Wire,
      .i2c_address = 0x6B, // or 0x6A
  };

  // Thermistor
  struct Thermistor_Config
  {
    int pin;
    float reference_resistor;
    float nominal_resistor;
    float nominal_temperature;
    float b_coefficient;
    int adc_resolution;
  };

  Thermistor_Config outside_thermistor_config = {
      .pin = 27,
      .reference_resistor = 10000,
      .nominal_resistor = 10000,
      .nominal_temperature = 25,
      .b_coefficient = -4050,
      .adc_resolution = 4095, // 12 bit
  };

  // Battery voltage reader
  AdcVoltage::AdcVoltage_Config battery_voltage_reader_config = {
      .pin = 26,                // Taken from the schematic
      .adc_resolution = 4095,   // 12 bit
      .reference_voltage = 3.3, // MCU voltage
      .R1_value = 51000,        // Taken from the schematic
      .R2_value = 24000,        // Taken from the schematic
  };

// Info/error queue
#define QUEUE_IMPLEMENTATION FIFO
#define INFO_ERROR_QUEUE_SIZE 20
#define INFO_ERROR_MAX_LENGTH 100

  // Watchdog
  const int WATCHDOG_TIMER = 8000; // Max is 8400

  // Sensor power
  const int SENSOR_POWER_ENABLE_PIN = 17;

  // Wire0
  const int WIRE0_SCL = 1;
  const int WIRE0_SDA = 0;

  // SPI0
  const int SPI0_RX = 4;
  const int SPI0_TX = 3;
  const int SPI0_SCK = 2;

  // logging
  const int PC_BAUDRATE = 115200;

  // Battery
  const float BATTERY_LOW_VOLTAGE = 5.5;
  const int BATTERY_LOW_BEEP_INTERVAL = 200;

  // Servos
  const int RECOVERY_CHANNEL_1 = 19;
  const int RECOVERY_CHANNEL_2 = 18;
  const int SERVO_INITIAL_POSITION = 75;
  const int SERVO_FINAL_POSITION = 0;

  const int LAUNCH_RAIL_SWITCH_PIN = 10;
  const int LAUNCH_RAIL_SWITCH_OFF_THRESHOLD = 5000;
  const int DESCENT_TIME_BEFORE_PARACHUTE_DEPLOYMENT = 30000;
  const int LAUNCH_RAIL_SWITCH_ALTITUDE_THRESHOLD = 300;

  // Buzzer
  const int BUZZER_PIN = 16;
  const int OUTSIDE_BUZZER_PIN = 20;
  const int BUZZER_BEEP_TIME = 2000;
  const int BUZZER_ACTION_START_TIME = 3600 * 1000; // 3600 seconds after turning on == 1 hour

  // Actions
  const int TIMED_ACTION_INITIAL_DELAY = 10000;
  // Data send action interval
  // 15 second cycle
  // 0 to 4 seconds - listen for command
  // 4 to 9 seconds - send command response if required
  // 9 to 12 seconds - send essential data
  // 12 to 15 seconds - payload sends essential data
  const int COMMUNICATION_CYCLE_INTERVAL = 15;
  const int COMMUNICATION_RESPONSE_SEND_TIME_START = 4000;
  const int COMMUNICATION_RESPONSE_SEND_TIME_END = 9000;
  const int COMMUNICATION_ESSENTIAL_DATA_SEND_TIME_START = 9000;
  const int COMMUNICATION_ESSENTIAL_DATA_SEND_TIME_END = 12000;

  // Sendable commands
  const int BFC_ESSENTIAL_DATA_RESPONSE = 200;
  const int BFC_COMPLETE_DATA_RESPONSE = 201;
  const int BFC_INFO_ERROR_RESPONSE = 202;
  const int BFC_FORMAT_RESPONSE = 203;
  const int BFC_RECOVERY_RESPONSE = 205;

  // Receiveable commands
  const int BFC_COMPLETE_DATA_REQUEST = 2000;
  const int BFC_INFO_ERROR_REQUEST = 2001;
  const int BFC_FORMAT_REQUEST = 2002;
  const int BFC_RECOVERY_REQUEST = 2004;
  const int RESET_SERVO_POSITION_REQUEST = 2005;
};