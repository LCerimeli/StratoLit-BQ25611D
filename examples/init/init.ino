#include <StratoLit-BQ25611D.h>

// Instantiate the charger object (default I2C address is 0x6B)
BQ25611D charger;

void setup() {
  charger.begin();                 // Starts Serial and Wire, checks ID
  charger.initAllChargerRegisters(); // Writes default values to all charger config registers and reads status registers
}

void loop() {
  // No loop logic needed for this example
}