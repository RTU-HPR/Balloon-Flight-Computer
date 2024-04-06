#include <Vehicle_Manager.h>

Vehicle_Manager vehicle_manager;

void setup()
{
  vehicle_manager.initializeFC();    // Read config, initialize comm busses, sd card, sensors, GPS, ranging LoRa 
  vehicle_manager.initializeComms(); // Initialize communication LoRa
  vehicle_manager.configureVehicleState(); // Read last state from flash  
}

void loop()
{
  vehicle_manager.runSensorManager();   // Read sensors
  vehicle_manager.runRadioManager();    // Handle radio
  vehicle_manager.runPowerManager();    // Handle power
  vehicle_manager.runEventManager();    // Handle events
  vehicle_manager.runLedManager();      // Handle LEDs
  vehicle_manager.runRecoveryManager(); // Handle pyro and/or detachement
  vehicle_manager.runLoggingManager();  // Log data
  vehicle_manager.runLoopTimeManager(); // Manage constant loop time
  vehicle_manager.runWatchdogUpdate();  // Update watchdog
}
