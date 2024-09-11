#include <Config.h>
#include <Logging.h>
#include <Balloon.h>

// Config and Logging objects are declared as global variables
extern Config config;
extern Logging logging;
Config config = Config();
Logging logging = Logging();

// Performance monitoring
unsigned long last_total_loop_millis = 0;
extern int total_loop_time;
extern int continuous_actions_time;
extern int timed_actions_time;
extern int requested_actions_time;
extern int gps_read_time;
extern int logging_time;
extern int sensor_read_time;
extern int onboard_baro_read_time;
extern int imu_read_time;
extern int battery_voltage_read_time;
extern int outside_thermistor_read_time;
int total_loop_time = 0;
int continuous_actions_time = 0;
int timed_actions_time = 0;
int requested_actions_time = 0;
int gps_read_time = 0;
int logging_time = 0;
int sensor_read_time = 0;
int onboard_baro_read_time = 0;
int imu_read_time = 0;
int battery_voltage_read_time = 0;
int outside_thermistor_read_time = 0;

Balloon balloon;

void setup()
{
  // INCREASES SPI CLOCK SPEED TO BE THE SAME AS CPU SPEED
  // MUST BE AT THE START OF SETUP
  // Get the processor sys_clk frequency in Hz
  uint32_t freq = clock_get_hz(clk_sys);

  // clk_peri does not have a divider, so input and output frequencies will be the same
  clock_configure(clk_peri,
                  0,
                  CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                  freq,
                  freq);

  balloon.begin();
  logging.log_info("Balloon setup complete");
  logging.log_info("CPU Speed: " + String(rp2040.f_cpu() / 1000000) + " MHz");
}

void loop()
{
  last_total_loop_millis = millis();
  balloon.actions.run_all_actions(balloon.sensors, balloon.navigation, balloon.communication);
  total_loop_time = millis() - last_total_loop_millis;

  // Reset the watchdog every loop
  rp2040.wdt_reset();
}
