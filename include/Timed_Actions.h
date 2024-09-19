#include <Actions.h>

void Actions::run_timed_actions(Sensors &sensors, Communication &communication)
{
  // Only start timed actions 10 seconds after turning on to make sure everything is initialised
  if (millis() < config.TIMED_ACTION_INITIAL_DELAY)
  {
    return;
  }

  // ...
}