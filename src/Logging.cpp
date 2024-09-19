#include <Logging.h>

bool Logging::begin_logging()
{
  // Initialize the queues
  _telemetry_queue = new cppQueue(sizeof(Telemetry_Message), TELEMETRY_QUEUE_SIZE, QUEUE_IMPLEMENTATION, true);
  _infoError_queue = new cppQueue(sizeof(Info_Error_Message), INFO_ERROR_QUEUE_SIZE, QUEUE_IMPLEMENTATION, true);

  return true;
}

bool Logging::begin_sd_card(Config &config)
{
  if (!_sd_card.begin(config.sd_card_config))
  {
    return false;
  }
  _sd_card_initialized = true;

  return true;
}

bool Logging::log_info(String info, bool write_to_file, bool add_trace)
{
  String message = "";

  if (add_trace)
  {
    message = "Time ON: " + String(millis()) + "| Balloon Info: " + info;
  }
  else
  {
    message = info;
  }

  _print_to_serial(message);
  if (write_to_file)
  {
    Info_Error_Message info_message;
    info_message.error = false;
    info.toCharArray(info_message.message, INFO_ERROR_MAX_LENGTH);

    if (!_infoError_queue->push(&info_message))
    {
      return false;
    }
  }
  return true;
}

bool Logging::log_error(String error, bool write_to_file, bool add_trace)
{
  String message;

  if (add_trace)
  {
    message = "Time ON: " + String(millis()) + "| Balloon Error: " + error;
  }
  else
  {
    message = error;
  }

  _print_to_serial(message);
  if (write_to_file)
  {
    Info_Error_Message error_message;
    error_message.error = true;
    error.toCharArray(error_message.message, INFO_ERROR_MAX_LENGTH);

    if (!_infoError_queue->push(&error_message))
    {
      return false;
    }
  }

  return true;
}

bool Logging::log_telemetry(String &data, bool print)
{
  if (print)
  {
    _print_to_serial(*&data);
  }

  Telemetry_Message telemetry_message;
  data.toCharArray(telemetry_message.data, TELEMETRY_MAX_LENGTH);

  if (!_telemetry_queue->push(&telemetry_message))
  {
    return false;
  }

  return true;
}

void Logging::component_info_function(String message)
{
  log_info(message, true, false);
}

void Logging::component_error_function(String message)
{
  log_error(message, true, false);
}

bool Logging::_write_to_file()
{
  if (!_sd_card_initialized)
  {
    return false;
  }

  // Write telemetry messages
  String write_data_telemetry = "";
  while (!_telemetry_queue->isEmpty())
  {
    Telemetry_Message telemetry_message;
    _telemetry_queue->pop(&telemetry_message);
    write_data_telemetry += telemetry_message.data;
    write_data_telemetry += "\n";
  }
  if (!_sd_card.write_telemetry(write_data_telemetry))
  {
    return false;
  }

  // Write info and error messages
  String write_data_info = "";
  String write_data_error = "";
  while (!_infoError_queue->isEmpty())
  {
    Info_Error_Message info_error_message;
    _infoError_queue->pop(&info_error_message);
    if (info_error_message.error)
    {
      write_data_error += info_error_message.message;
      write_data_error += "\n";
    }
    else
    {
      write_data_info += info_error_message.message;
      write_data_info += "\n";
    }
  }

  if (write_data_info.length() > 0)
  {
    if (!_sd_card.write_info(write_data_info))
    {
      return false;
    }
  }

  if (write_data_error.length() > 0)
  {
    if (!_sd_card.write_error(write_data_error))
    {
      return false;
    }
  }

  return true;
}

void Logging::_print_to_serial(String message)
{
  Serial.println(message);
}