#include <Balloon.h>

Balloon balloon;

// Performance monitoring
unsigned long last_total_loop_millis = 0;

// Declare as global variables
extern int total_loop_time;
extern int continuous_actions_time;
extern int timed_actions_time;
extern int requested_actions_time;
extern int gps_read_time;
extern int logging_time;
extern int sensor_read_time;
extern int on_board_baro_read_time;
extern int imu_read_time;
extern int battery_voltage_read_time;
extern int outside_thermistor_read_time;

// Set initial values
int total_loop_time = 0;
int continuous_actions_time = 0;
int timed_actions_time = 0;
int requested_actions_time = 0;
int gps_read_time = 0;
int logging_time = 0;
int sensor_read_time = 0;
int on_board_baro_read_time = 0;
int imu_read_time = 0;
int battery_voltage_read_time = 0;
int outside_thermistor_read_time = 0;

// Declare Servo objects as global variables
// Really bad way to do this, but it should be fine for now
extern Servo servo_1;
extern Servo servo_2;
Servo servo_1 = Servo();
Servo servo_2 = Servo();

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

  // Enable watchdog
  rp2040.wdt_begin(4000);

  balloon.begin();
  Serial.println("Balloon setup complete");
  Serial.println("CPU Speed: " + String(rp2040.f_cpu() / 1000000) + " MHz");
  Serial.println();

  // Reset the watchdog
  rp2040.wdt_reset();
}

void loop()
{
  last_total_loop_millis = millis();
  balloon.actions.runAllActions(balloon.sensors, balloon.navigation, balloon.communication, balloon.logging, balloon.config);
  total_loop_time = millis() - last_total_loop_millis;

  // Reset the watchdog every loop
  rp2040.wdt_reset();
}
