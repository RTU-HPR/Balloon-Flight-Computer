#include <Config.h>
#include <Navigation.h>

bool Navigation::begin_gps(Config &config)
{
  return _gps.begin(config.gps_config);
}

bool Navigation::read_gps()
{
  return _gps.read();
}