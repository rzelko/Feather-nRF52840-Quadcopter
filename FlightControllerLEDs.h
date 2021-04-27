/*
  FlightControllerLEDs.h - Library for controlling LEDs on Flight Control PCB.
  Created by Rick Zelko, April 24, 2021.
  To be used with custom Flight Control PCB Ver5 Rev0 
  Released into the public domain.
*/
#ifndef FlightControllerLEDs_h
#define FlightControllerLEDs_h

#include "Arduino.h"


class antiCollLEDs
{
  public:
    antiCollLEDs(int pin);
    void begin();
    void activateAntiCollNormal();
    void activateAntiCollHigh();    
  private:
    #define DELAYVAL 100 // Time (in milliseconds) to pause between pixels
    #define ANTI_COLL_LEDS        5 // MCU pin 5 on Flight Control PCB Ver5 Rev0
    #define NUM_ANTI_COLL_PIXELS 4 // Navigation NeoPixels (LED1 and LED3 on top and LED2 and LED4 on bottom)};
    int _pin;
};

class navLED
{
  public:
    navLED(int pin2);
    void begin();
    void navLightsLand();
    void navLightsFly();
    void navLightsHover();
    void navLightsOff();
  private:
    #define NAV_LED   2 // MCU pin 2 on Flight Control PCB Ver5 Rev0
    #define NUM_NAV_PIXEL 1 // Single NeoPixel for downward navigation lighting 
    int _pin2; 
};

#endif
