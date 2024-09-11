#pragma once
#include <Config.h>

class Logging
{
private:
  SD_Card _sd_card;
  bool _sd_card_initialized = false;

  /**
   * @brief Structure to store telemetry messages. Required for the queue
   */
  typedef struct telemetry
  {
    char data[TELEMETRY_MAX_LENGTH];
  } Telemetry_Message;

  /**
   * @brief Structure to store info and error messages. Required for the queue
   */
  typedef struct info_error
  {
    bool error;
    char message[INFO_ERROR_MAX_LENGTH];
  } Info_Error_Message;

  /**
   * @brief Queue to store telemetry messages
   */
  cppQueue *_telemetry_queue;

  /**
   * @brief Queue to store info and error messages
   */
  cppQueue *_infoError_queue;

  void _print_to_serial(String message);

public:
  /**
   * @brief Initialise the logging
   * @param config Config object
   * @return Whether the logging was initialised successfully
   */
  bool begin_logging();

  bool begin_sd_card(Config &config);

  /**
   * @brief Write the telemetry, info, and error messages to the SD card
   */
  bool _write_to_file();

  /**
   * @brief Record an info message
   * @param info Info message
   * @return Whether the info message was recorded successfully
   */
  bool log_info(String info, bool write_to_file=true, bool add_trace=true);

  /**
   * @brief Record an error message
   * @param error Error message
   * @return Whether the error message was recorded successfully
   */
  bool log_error(String error, bool write_to_file=true, bool add_trace=true);

  /**
   * @brief Record a telemetry message
   * @param data Telemetry data
   * @return Whether the telemetry message was recorded successfully
   */
  bool log_telemetry(String &data, bool print=false);

  void component_info_function(String msg);
  void component_error_function(String msg);
};