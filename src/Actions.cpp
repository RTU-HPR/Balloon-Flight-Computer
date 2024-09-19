#include <Actions.h>
#include <Continous_Actions.h>
#include <Timed_Actions.h>
#include <Requested_Actions.h>

// Performance monitoring
extern int continuous_actions_time;
extern int timed_actions_time;
extern int requested_actions_time;
unsigned long last_continous_actions_millis = 0;
unsigned long last_timed_actions_millis = 0;
unsigned long last_requested_actions_millis = 0;

void Actions::run_all_actions(Sensors &sensors, Communication &communication)
{
  last_continous_actions_millis = millis();
  run_continous_actions(sensors, communication);
  continuous_actions_time = millis() - last_continous_actions_millis;

  last_timed_actions_millis = millis();
  run_timed_actions(sensors, communication);
  timed_actions_time = millis() - last_timed_actions_millis;

  last_requested_actions_millis = millis();
  run_requested_actions(sensors, communication);
  requested_actions_time = millis() - last_requested_actions_millis;
}
