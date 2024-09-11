#pragma once
#include <Config.h>
#include <Logging.h>
extern Config config;
extern Logging logging;

class Navigation
{
private:
  GPS _gps;

public:
  bool begin_gps(Config &config);
  bool read_gps();
};