/*
Main configuration file for the vehicle. 
This file is used to define the used libraries and hardware configuration.

ALL VARIABLES MUST BE CONST AND DEFINED HERE.
*/

#pragma once

// Core libraries
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// Public libraries
#include <LittleFS.h>
#include <SDFS.h>
#include <cppQueue.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSL.h>
#include <NTC_Thermistor.h>
#include <SimpleKalmanFilter.h>
#include <Servo.h>

// Our libraries
#include <RadioLib_wrapper.h>
#include <Gps_Wrapper.h>
#include <Sd_card_wrapper.h>
#include <Ccsds_packets.h>
#include <MS56XX.h>
#include <Adc_Voltage.h>

// Used radio module
#define radio_module SX1268

// Feature flags
const bool WAIT_FOR_PC = false;
const bool ENABLE_WATCHDOG = true;
const bool ENABLE_LOGGING = true;
const bool ENABLE_LOW_BATTERY_BUZZER = true;
const bool ENABLE_RADIO = true;
const bool ENABLE_SENSORS = true;
const bool ENABLE_GPS = true;
const bool ENABLE_RANGING = true;
const bool ENABLE_LED = true;
const bool ENABLE_RECOVERY = true;

// Constants
const int LOOP_TIME = 10;                    // ms
const int PC_BAUDRATE = 115200;              // bps
const int WATCHDOG_TIMEOUT = 8000;           // ms
const int RECOVERY_BUZZER_INTERVAL = 2000;   // ms
const int LOW_BATTERY_BUZZER_INTERVAL = 500; // ms
const int LOW_BATTERY_VOLTAGE = 6.0;         // V

// Pin definitions
const int SENSOR_POWER_ENABLE_PIN = 17;

const int WIRE0_SCL = 1;
const int WIRE0_SDA = 0;

const int SPI0_RX = 4;
const int SPI0_TX = 3;
const int SPI0_SCK = 2;

// Communication LoRa
const RadioLib_Wrapper<radio_module>::Radio_Config radio_config{
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
const Gps_Wrapper::Gps_Config_I2C gps_config{
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

// MS56XX
const MS56XX::MS56XX_Config ms56xx_config = {
    .wire = &Wire,
    .i2c_address = MS56XX::MS56XX_I2C_ADDRESS::I2C_0x76, // or 0x76
    .ms56xx_type = MS56XX::MS56XX_TYPE::MS5611,          // or MS5607
    .oversampling = MS56XX::MS56XX_OVERSAMPLING::OSR_STANDARD,
};

// Battery voltage reader
const AdcVoltage::AdcVoltage_Config battery_voltage_reader_config = {
    .pin = 26,                // Taken from the schematic
    .adc_resolution = 4095,   // 12 bit
    .reference_voltage = 3.3, // MCU voltage
    .R1_value = 51000,        // Taken from the schematic
    .R2_value = 24000,        // Taken from the schematic
};
