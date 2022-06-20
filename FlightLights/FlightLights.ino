#include "FlightInterfaceModule.h"

FlightInterfaceModule flight(PWR_CONTROL, BAT_LVL);


void setup() {
 // on nRF52 this will block until serial is connected, comment out when serial is not used!
  while (!Serial) yield();
  Serial.begin(115200);
  flight.begin();
  delay(1000);
  Serial.println("\n\nFlight Interface Module Light Test Sketch");
//  Serial.println("===========================");
//  Serial.println("f(fast) n(normal)  s(slow)     Modes");
//  Serial.println("Send me a command letter.");

  // need to flush inside setup, otherwise will not print
  Serial.flush(); 

   flight.testMotors();

}

void loop() {
  
  flight.checkAmbient();      // triggers Anti-Collision Lights when lighting is below twilight threshold
  flight.checkLandingAlt();   // triggers white Navigation LED for landing

}
