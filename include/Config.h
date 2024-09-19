#pragma once

// Define which vehicle type is being used
#define VEHICLE_TYPE 1          // 1 - Balloon | 2 - Payload
#define RANGING 0               // 0 - No ranging | 1 - Ranging Master mode | 2 - Ranging Slave mode
#define RECOVERY_CHANNEL_TYPE 0 // 0 - Disabled | 1 - Servo | 2 - Pyro

// Main libraries
// Public libraries
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LittleFS.h>
#include <SDFS.h>
#include <cppQueue.h>
#include <Servo.h>

// Custom libraries
#include <MUFFINS_Component_Base.h>
#include <MUFFINS_SD_Card.h>
#include <MUFFINS_Radio.h>
#include <MUFFINS_GPS.h>
#include <MUFFINS_LSM6DSL.h>
#include <MUFFINS_MS56XX.h>
#include <MUFFINS_ADC_Voltage.h>
#include <MUFFINS_CCSDS_Packets.h>
#include <MUFFINS_Checksums.h>
#include <MUFFINS_Thermistor.h>

// Specific libraries
#if RANGING != 0
#include <MUFFINS_Ranging.h>
#endif

class Config
{
public:
  const bool WAIT_PC = true;
  const bool LOG_TO_STORAGE = true;

  // 433 MHz LoRa
  const Radio::Config radio_config = {
      .frequency = 434.0,
      .cs = 5,
      .dio0 = 8,
      .dio1 = 9,
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
  const GPS::Config gps_config = {
      .read_interval = 25,
      .measurement_rate = GPS::MEASUREMENT_RATE_25ms,
      .navigation_rate = GPS::NAVIGATION_RATE_8Hz,
      .dynamic_model = DYN_MODEL_AIRBORNE2g,
      .wire = &Wire,
      .i2c_address = 0x42,
  };

  // MS56XX
  const MS56XX::Config ms56xx_config = {
      .wire = &Wire,
      .i2c_address = MS56XX::I2C_0x76,
      .type = MS56XX::MS5611,
      .oversampling = MS56XX::OSR_ULTRA_HIGH,
      .reference_pressure = 101325,
  };

  // IMU
  const LSM6DSL::Config imu_config = {
      .wire = &Wire,
      .i2c_address = 0x6B,
      .accel_range = LSM6DS_ACCEL_RANGE_2_G,
      .gyro_range = LSM6DS_GYRO_RANGE_250_DPS,
      .accel_data_rate = LSM6DS_RATE_1_66K_HZ,
      .gyro_data_rate = LSM6DS_RATE_1_66K_HZ,
  };

  // Thermistor
  const Thermistor::Config thermistor_config = {
      .pin = 27,
      .adc_resolution = 4095,
      .reference_resistance = 10000,
      .nominal_resistance = 10000,
      .nominal_temperature_kelvin = 298.15,
      .b_value = -4050,
  };

  // Battery voltage reader
  const ADC_Voltage::Config battery_voltage_config = {
      .pin = 26,
      .adc_resolution = 4095,
      .reference_voltage = 3.3,
      .R1_value = 51000,
      .R2_value = 24000,
  };

// Ranging
#if RANGING == 1
  static const int slave_count = 1;
  Ranging master;
  Ranging::Slave slaves[slave_count] = {{.address = 0x12345678}};

  Ranging::Config master_config{
      .frequency = 2405.6,
      .cs = 15,
      .dio0 = 11,
      .dio1 = 12,
      .reset = 13,
      .sync_word = 0xF5,
      .tx_power = 10,
      .spreading = 10,
      .coding_rate = 7,
      .signal_bw = 406.25,
      .spi_bus = &SPI,
      .mode = Ranging::Mode::MASTER,
      .timeout = 200,
      .slave_count = slave_count,
      .slaves = &*slaves};
#elif RANGING == 2
  Ranging slave;
  Ranging::Config slave_config{
      .frequency = 2405.6,
      .cs = 15,
      .dio0 = 11,
      .dio1 = 12,
      .reset = 13,
      .sync_word = 0xF5,
      .tx_power = 10,
      .spreading = 10,
      .coding_rate = 7,
      .signal_bw = 406.25,
      .spi_bus = &SPI,
      .mode = Ranging::Mode::SLAVE,
      .timeout = 200,
      .address = 0x12345678};
#endif

  const String INFO_FILE_HEADER = "time,info";
  const String ERROR_FILE_HEADER = "time,error";

  // Define the telemetry file header based on the vehicle type
#if VEHICLE_TYPE == 1
  const String TELMETRY_FILE_HEADER = "index,time_on_ms,gps_epoch_time,gps_hour:gps_minute:gps_second,gps_lat,gps_lng,gps_altitude,gps_speed,gps_satellites,gps_heading,gps_pdop,onboard_baro_temp,onboard_baro_pressure,onboard_baro_altitude,outside_thermistor_temp,imu_accel_x,imu_accel_y,imu_accel_z,imu_heading,imu_pitch,imu_roll,imu_gyro_x,imu_gyro_y,imu_gyro_z,imu_temp,battery_voltage,used_heap,loop_time,continuous_actions_time,timed_actions_time,requested_actions_time,gps_read_time,logging_time,sensor_read_time,onboard_baro_read_time,imu_read_time,battery_voltage_read_time,outside_thermistor_read_time";
#elif VEHICLE_TYPE == 2
  const String TELMETRY_FILE_HEADER = "index,time_on_ms,gps_epoch_time,gps_hour:gps_minute:gps_second,gps_lat,gps_lng,gps_altitude,gps_speed,gps_satellites,gps_heading,gps_pdop,onboard_baro_temp,onboard_baro_pressure,onboard_baro_altitude,outside_thermistor_temp,imu_accel_x,imu_accel_y,imu_accel_z,imu_heading,imu_pitch,imu_roll,imu_gyro_x,imu_gyro_y,imu_gyro_z,imu_temp,battery_voltage,used_heap,loop_time,continuous_actions_time,timed_actions_time,requested_actions_time,gps_read_time,logging_time,sensor_read_time,onboard_baro_read_time,imu_read_time,battery_voltage_read_time,outside_thermistor_read_time";
#endif

  // SD card
  const SD_Card::Config sd_card_config = {
      .spi_bus = &SPI,
      .cs_pin = 14,
      .telemetry_file_path_base = "/BFC_TELEMETRY_",
      .info_file_path_base = "/BFC_INFO_",
      .error_file_path_base = "/BFC_ERROR_",
      .telemetry_file_header = TELMETRY_FILE_HEADER,
      .info_file_header = INFO_FILE_HEADER,
      .error_file_header = ERROR_FILE_HEADER,
  };

// Info/error queue
#define QUEUE_IMPLEMENTATION FIFO

#define TELEMETRY_MAX_LENGTH 255
#define TELEMETRY_QUEUE_SIZE 100

#define INFO_ERROR_MAX_LENGTH 100
#define INFO_ERROR_QUEUE_SIZE 20

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
  const float BATTERY_LOW_VOLTAGE = 0;
  const int BATTERY_LOW_BEEP_INTERVAL = 200;

  // Recovery channels
  // In Payload V2 schematic, mosfet pins are called RECOV_1 and RECOV_2,
  // but signal pins are called PYRO_1 and PYRO_2
  const int RECOVERY_CHANNEL_MOSFET_1 = 21;
  const int RECOVERY_CHANNEL_MOSFET_2 = 20;
  const int RECOVERY_CHANNEL_SIGNAL_1 = 19;
  const int RECOVERY_CHANNEL_SIGNAL_2 = 18;

// Servo
#if RECOVERY_CHANNEL_TYPE == 1
  const int SERVO_INITIAL_POSITION = 75;
  const int SERVO_FINAL_POSITION = 0;
  const int TIME_BETWEEN_SERVO_POSITIONS = 1000; // ms
#endif
  // Pyro
  // ...

  // Switch
  const int SWITCH_PIN = 10;
// For now switch is used only on payload
#if VEHICLE_TYPE == 2
  const int LAUNCH_RAIL_SWITCH_OFF_THRESHOLD = 5000;
  const int DESCENT_TIME_BEFORE_PARACHUTE_DEPLOYMENT = 30000;
  const int LAUNCH_RAIL_SWITCH_ALTITUDE_THRESHOLD = 300;
#endif

  // Buzzer
  const int BUZZER_PIN = 16;
  const int BUZZER_BEEP_TIME = 2000;
  const int BUZZER_ACTION_START_TIME = 3600 * 1000; // 3600 seconds after turning on == 1 hour

  // Actions
  const int TIMED_ACTION_INITIAL_DELAY = 10000;

  // Sendable commands
  // ...

  // Receiveable commands
  // ...
};