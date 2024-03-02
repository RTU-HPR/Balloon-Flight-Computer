#include <Actions.h>

// Performance monitoring
unsigned long last_sensor_read_millis = 0;
unsigned long last_gps_read_millis = 0;
unsigned long last_logging_millis = 0;

// Get Servo objects
extern Servo servo_1;
extern Servo servo_2;

void Actions::runContinousActions(Sensors &sensors, Navigation &navigation, Communication &communication, Logging &logging, Config &config)
{
  // Receive any commands
  if (commandReceiveActionEnabled)
  {
    runCommandReceiveAction(communication, logging, config);
  }
  // Serial.println("Finished command receive action");
  // Check if the communication cycle should be started
  if (getCommunicationCycleStartActionEnabled)
  {
    runGetCommunicationCycleStartAction(navigation, config);
  }

  // Run the sensor action
  if (sensorActionEnabled)
  {
    last_sensor_read_millis = millis();
    runSensorAction(sensors);
    sensor_read_time = millis() - last_sensor_read_millis;
  }

  // Run the GPS action
  if (gpsActionEnabled)
  {
    last_gps_read_millis = millis();
    runGpsAction(navigation);
    gps_read_time = millis() - last_gps_read_millis;
  }

  // Run the logging action
  if (loggingActionEnabled)
  {
    // Serial.println("Logging: + " + String(millis()) + " ms | ID: " + String(loggable_packed_id));
    last_logging_millis = millis();
    runLoggingAction(logging, navigation, sensors, config);
    logging_time = millis() - last_logging_millis;
    if (loggable_packed_id % 50 == 0)
    {
      Serial.println("Logged packet count: " + String(loggable_packed_id) + " | Turned on time: " + String(millis() / 1000) + " s");
    }
  }

  if (descentActionEnabled)
  {
    runDescentAction(logging, config, sensors, navigation);
  }

  // Run the recovery channel manager action
  if (recoveryChannelManagerActionEnabled)
  {
    runRecoveryChannelManagerAction(config);
  }
}

void Actions::runCommandReceiveAction(Communication &communication, Logging &logging, Config &config)
{
  byte *msg = new byte[256];
  uint16_t msg_length = 0;
  float rssi = 1;
  float snr = 0;
  double frequency = 0;
  bool checksum_good = false;

  // Check for any messages from Radio
  if (communication._radio->receive_bytes(msg, msg_length, rssi, snr, frequency))
  {
    // Check if checksum matches
    if (check_crc_16_cciit_of_ccsds_packet(msg, msg_length))
    {
      checksum_good = true;
    }
  }
  else
  {
    // Free memory after no message has been received
    delete[] msg; // VERY IMPORTANT, otherwise a significant memory leak will occur
    return;
  }

  // Check if the checksum is good
  if (checksum_good)
  {
    // Print the received message
    Serial.print("RADIO COMMAND | RSSI: " + String(rssi) + " | SNR: " + String(snr) + " FREQUENCY: " + String(frequency, 8) + " | MSG: ");

    for (int i = 0; i < msg_length; i++)
    {
      Serial.print(msg[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // A CCSDS Telecommand packet was received
    uint16_t apid = 0;
    uint16_t sequence_count = 0;
    byte *packet_data = new byte[msg_length];
    uint16_t packet_id = 0;
    uint16_t packet_data_length = 0;
    parse_ccsds_telecommand(msg, apid, sequence_count, packet_id, packet_data, packet_data_length);

    Serial.println("APID: " + String(apid));
    Serial.println("Sequence count: " + String(sequence_count));
    Serial.println("Packet data length: " + String(packet_data_length));
    Serial.println("Packet ID: " + String(packet_id));

    // Set the action flag according to the received command
    if (packet_id == config.BFC_COMPLETE_DATA_REQUEST)
    {
      // NOT CORRECTLY IMPLEMENTED, SO IT IS DISABLED
      completeDataRequestActionEnabled = false;
    }
    else if (packet_id == config.BFC_INFO_ERROR_REQUEST)
    {
      infoErrorRequestActionEnabled = true;
    }
    else if (packet_id == config.BFC_FORMAT_REQUEST)
    {
      formatStorageActionEnabled = true;
    }
    else if (packet_id == config.BFC_RECOVERY_REQUEST)
    {
      recoveryFireActionEnabled = true;

      // Get the recovery channel
      Converter recoveryChannel[1];

      // The first value is the packet id, and the second is the recovery channel
      extract_ccsds_data_values(packet_data, recoveryChannel, "uint8");

      // Set the appropriate recovery channel flag
      if (recoveryChannel[0].i8 == 1)
      {
        recoveryChannelShouldBeFired[0] = true;
      }
      else if (recoveryChannel[0].i8 == 2)
      {
        recoveryChannelShouldBeFired[1] = true;
      }
      else
      {
        Serial.println("Invalid recovery channel: " + String(recoveryChannel[0].i8));
        recoveryFireActionEnabled = false;
      }
    }
    else
    {
      Serial.println("No mathcing command found");
    }

    // Free memory after the packet data has been parsed
    delete[] packet_data; // VERY IMPORTANT, otherwise a significant memory leak will occur
  }
  else if (!checksum_good)
  {
    Serial.println("Command with invalid checksum received: ");
    for (int i = 0; i < msg_length; i++)
    {
      Serial.print(msg[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // Free memory after the message has been parsed
  delete[] msg; // VERY IMPORTANT, otherwise a significant memory leak will occur
}

void Actions::runSensorAction(Sensors &sensors)
{
  // Read all sensors
  sensors.readSensors();
}

void Actions::runGpsAction(Navigation &navigation)
{
  navigation.readGps(navigation.navigation_data);
}

void Actions::runLoggingAction(Logging &logging, Navigation &navigation, Sensors &sensors, Config &config)
{
  // Log the data to the sd card
  String packet = createLoggablePacket(sensors, navigation, config);
  logging.writeTelemetry(packet);
}

void Actions::runGetCommunicationCycleStartAction(Navigation &navigation, Config &config)
{
  if (millis() - lastCommunicationCycle <= 3000)
  {
    return;
  }
  if (navigation.navigation_data.gps.epoch_time == 0)
  {
    return;
  }

  if (navigation.navigation_data.gps.second % config.COMMUNICATION_CYCLE_INTERVAL == 0)
  {
    lastCommunicationCycle = millis();
    dataEssentialSendActionEnabled = true;
    Serial.println("New communication cycle started: " + String(lastCommunicationCycle) + " " + String(navigation.navigation_data.gps.hour) + ":" + String(navigation.navigation_data.gps.minute) + ":" + String(navigation.navigation_data.gps.second));
  }
}

void Actions::runRecoveryChannelManagerAction(Config &config)
{
  // Check if the recovery channel should be fired
  for (int i = 0; i < 2; i++)
  {
    if (recoveryChannelShouldBeFired[i])
    {
      // If the recovery channel has not been fired yet, enable it
      if (recoveryChannelFireTimes[i] == 0)
      {
        recoveryChannelFireTimes[i] = millis();
        if (i == 0)
        {
          servo_1.write(config.SERVO_FINAL_POSITION);
          Serial.println("Servo 1 engaged");
        }
        else if (i == 1)
        {
          servo_2.write(config.SERVO_FINAL_POSITION);
          Serial.println("Servo 2 engaged");
        }
      }
      // Reset the recovery channel flag, but keep the fire time as it will not be fired again
      recoveryChannelShouldBeFired[i] = false;
    }
  }
}

void Actions::runDescentAction(Logging &logging, Config &config, Sensors &sensors, Navigation &navigation)
{
  // No decent action is currently required for the balloon platform
}

String Actions::createLoggablePacket(Sensors &sensors, Navigation &navigation, Config &config)
{
  String packet = "";
  packet += String(loggable_packed_id);
  packet += ",";
  packet += String(millis());
  packet += ",";
  // GPS
  packet += String(navigation.navigation_data.gps.epoch_time);
  packet += ",";
  packet += String(navigation.navigation_data.gps.hour);
  packet += ":";
  packet += String(navigation.navigation_data.gps.minute);
  packet += ":";
  packet += String(navigation.navigation_data.gps.second);
  packet += ",";
  packet += String(navigation.navigation_data.gps.lat, 7);
  packet += ",";
  packet += String(navigation.navigation_data.gps.lng, 7);
  packet += ",";
  packet += String(navigation.navigation_data.gps.altitude, 2);
  packet += ",";
  packet += String(navigation.navigation_data.gps.speed, 2);
  packet += ",";
  packet += String(navigation.navigation_data.gps.satellites);
  packet += ",";
  packet += String(navigation.navigation_data.gps.heading);
  packet += ",";
  packet += String(navigation.navigation_data.gps.pdop);
  packet += ",";
  // Onboard temperature/pressure
  packet += String(sensors.data.onBoardBaro.temperature, 2);
  packet += ",";
  packet += String(sensors.data.onBoardBaro.pressure);
  packet += ",";
  packet += String(sensors.data.onBoardBaro.altitude, 2);
  packet += ",";
  packet += String(sensors.data.outsideThermistor.temperature, 2);
  packet += ",";
  // IMU
  packet += String(sensors.data.imu.accel.acceleration.x, 4);
  packet += ",";
  packet += String(sensors.data.imu.accel.acceleration.y, 4);
  packet += ",";
  packet += String(sensors.data.imu.accel.acceleration.z, 4);
  packet += ",";
  packet += String(sensors.data.imu.accel.acceleration.heading, 3);
  packet += ",";
  packet += String(sensors.data.imu.accel.acceleration.pitch, 3);
  packet += ",";
  packet += String(sensors.data.imu.accel.acceleration.roll, 3);
  packet += ",";
  packet += String(sensors.data.imu.gyro.gyro.x, 4);
  packet += ",";
  packet += String(sensors.data.imu.gyro.gyro.y, 4);
  packet += ",";
  packet += String(sensors.data.imu.gyro.gyro.z, 4);
  packet += ",";
  packet += String(sensors.data.imu.temp.temperature, 2);
  packet += ",";
  // Battery/Heater current
  packet += String(sensors.data.battery.voltage, 2);
  packet += ",";
  // Performance/debugging
  packet += String(rp2040.getUsedHeap());
  packet += ",";
  packet += String(total_loop_time);
  packet += ",";
  packet += String(continuous_actions_time);
  packet += ",";
  packet += String(timed_actions_time);
  packet += ",";
  packet += String(requested_actions_time);
  packet += ",";
  packet += String(gps_read_time);
  packet += ",";
  packet += String(logging_time);
  packet += ",";
  packet += String(sensor_read_time);
  packet += ",";
  packet += String(on_board_baro_read_time);
  packet += ",";
  packet += String(imu_read_time);
  packet += ",";
  packet += String(battery_voltage_read_time);
  packet += ",";
  packet += String(outside_thermistor_read_time);

  loggable_packed_id++;

  return packet;
}