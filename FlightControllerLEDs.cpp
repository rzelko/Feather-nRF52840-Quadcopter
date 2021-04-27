/*
  FlightControllerLEDs.h - Library for controlling LEDs on Flight Control PCB.
  Created by Rick Zelko, April 24, 2021.
  To be used with custom Flight Control PCB Ver5 Rev0 
  Released into the public domain.
*/
#include "Arduino.h"
#include "Adafruit_NeoPixel.h"
#include "FlightControllerLEDs.h"


int RED[3] = {150,0,0};
int WHITE[3] = {150,150,150};
int GREEN[3] = {0,150,0};
int BLUE[3] = {0,0,150};


Adafruit_NeoPixel antiCollPixels(NUM_ANTI_COLL_PIXELS, ANTI_COLL_LEDS, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel navPixel(NUM_NAV_PIXEL, NAV_LED, NEO_GRB + NEO_KHZ800);

// constructors
antiCollLEDs::antiCollLEDs(int pin)
{
  pinMode(ANTI_COLL_LEDS, OUTPUT);
  antiCollPixels.begin();
  antiCollPixels.clear();
  antiCollPixels.show();
  _pin = ANTI_COLL_LEDS;
}

navLED::navLED(int pin2)
{
  pinMode(NAV_LED, OUTPUT);
  _pin2 = NAV_LED;
}

void antiCollLEDs::begin()
{
  antiCollPixels.begin();
  antiCollPixels.clear();
  antiCollPixels.show();
}

void antiCollLEDs::activateAntiCollNormal()
{
  antiCollPixels.clear(); // Set all pixel colors to 'off'
  for(int i=0; i<NUM_ANTI_COLL_PIXELS; i++) { // For each pixel...
    antiCollPixels.setPixelColor(i, antiCollPixels.Color(150,0,0));
    antiCollPixels.show();   // Send the updated pixel colors to the hardware.
    delay(DELAYVAL); // Pause before next pass through loop
    antiCollPixels.clear(); // Set all pixel colors to 'off'
    antiCollPixels.setPixelColor(i, antiCollPixels.Color(150,150,150));
    antiCollPixels.show();   // Send the updated pixel colors to the hardware.
    delay(DELAYVAL); 
  } 
  antiCollPixels.clear();
  antiCollPixels.show(); 
}

void antiCollLEDs::activateAntiCollHigh()
{
  antiCollPixels.clear(); // Set all pixel colors to 'off'
  for(int i=0; i<NUM_ANTI_COLL_PIXELS; i++) { // For each pixel...
    antiCollPixels.setPixelColor(i, antiCollPixels.Color(150,0,0));
    antiCollPixels.show();   // Send the updated pixel colors to the hardware.
    delay(DELAYVAL/2); // Pause before next pass through loop
    antiCollPixels.clear(); // Set all pixel colors to 'off'
    antiCollPixels.setPixelColor(i, antiCollPixels.Color(150,150,150));
    antiCollPixels.show();   // Send the updated pixel colors to the hardware.
    delay(DELAYVAL/2); 
  } 
  antiCollPixels.clear();
  antiCollPixels.show(); 
}

void navLED::begin()
{
  navPixel.begin();
  navPixel.clear();
  navPixel.show();
}
void navLED::navLightsLand()
{
  navPixel.clear(); // Set all pixel colors to 'off'
  for(int i=0; i<NUM_NAV_PIXEL; i++) { // For each pixel...
    navPixel.setPixelColor(i, navPixel.Color(150,0,0));
    navPixel.show();   // Send the updated pixel colors to the hardware.
    delay(DELAYVAL*4); // Pause before next pass through loop
    navPixel.clear(); // Set all pixel colors to 'off'
    navPixel.setPixelColor(i, navPixel.Color(0,0,150));
    navPixel.show();   // Send the updated pixel colors to the hardware.
    delay(DELAYVAL*2);  
  }
  navPixel.clear();
  navPixel.show();
}

void navLED::navLightsFly()
{
  navPixel.clear(); // Set all pixel colors to 'off'
  for(int i=0; i<NUM_NAV_PIXEL; i++) { // For each pixel...
      navPixel.setPixelColor(i, navPixel.Color(0,150,0));
      navPixel.show();   // Send the updated pixel colors to the hardware.
      delay(DELAYVAL); // Pause before next pass through loop
      navPixel.clear(); // Set all pixel colors to 'off'
      navPixel.setPixelColor(i, navPixel.Color(0,150,0));
      navPixel.show();   // Send the updated pixel colors to the hardware.
      delay(DELAYVAL);
  }
  navPixel.clear();
  navPixel.show();
}

void navLED::navLightsHover()
{
  navPixel.clear(); // Set all pixel colors to 'off'
  for(int i=0; i<NUM_NAV_PIXEL; i++) { // For each pixel...
      navPixel.setPixelColor(i, navPixel.Color(150,150,150));
      navPixel.show();   // Send the updated pixel colors to the hardware.
  }
}

void navLED::navLightsOff()
{
  navPixel.clear(); // Set all pixel colors to 'off'
  navPixel.show();
}
