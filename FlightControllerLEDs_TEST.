////////////////////////////////  LIBRARIES  ////////////////////////////////

#include "FlightControllerLEDs.h"
#include <Adafruit_VL53L0X.h>
#include <Adafruit_APDS9960.h>
//#include <Adafruit_NeoPixel.h>


Adafruit_VL53L0X  lox = Adafruit_VL53L0X();  // Create instance of TOF Sensor
Adafruit_APDS9960 apds9960; // proximity, light, color, gesture

//Adafruit_NeoPixel antiCollPixels(NUM_ANTI_COLL_PIXELS, ANTI_COLL_LEDS, NEO_GRB + NEO_KHZ800);
//Adafruit_NeoPixel navPixel(NUM_NAV_PIXEL, NAV_LED, NEO_GRB + NEO_KHZ800);

antiCollLEDs  antiCollLEDs(ANTI_COLL_LEDS);
navLED  navLED(NAV_LED);

void setup() {
  Serial.begin(115200);
  
  // initialize NeoFlightControllerLEDs
  navLED.begin();
  antiCollLEDs.begin();
  
  // initialize the sensors
  apds9960.begin();
  apds9960.enableProximity(true);
  apds9960.enableColor(true);
  lox.begin();

  delay(1000);
  Serial.println("\n\nFlight Controller LED Test Sketch");
  Serial.println("===========================");  
  Serial.println("f(fly) l(land) h(hover) o(off)    Nav Modes");
  Serial.println("Send me a command letter.");  
}

void loop() {
  if (Serial.available())
  {
    char ch = Serial.read();
    if (ch == 'f') 
    {
      antiCollLEDs.activateAntiCollHigh();
      navLED.navLightsFly();
      Serial.println("flight mode and anti-collision strobe fast");
    } 
    else if (ch == 'l') 
    {
      antiCollLEDs.activateAntiCollNormal();
      navLED.navLightsLand();
      Serial.println("landing moden and anti-collision strobe normal");
    }
    else if (ch == 'h') 
    {
      antiCollLEDs.activateAntiCollHigh();
      navLED.navLightsHover();
      Serial.println("hover mode and anti-collision strobe fast");
    }
    else if (ch == 'o') 
    {
      antiCollLEDs.activateAntiCollNormal();
      navLED.navLightsOff();
      Serial.println("nav lights off and anti-collision strobe normal");
    }
  } 
}
