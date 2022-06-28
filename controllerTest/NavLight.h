/*
  NavLight.h - Library for controlling Navigation LED on Flight Control PCB.
  Created by Rick Zelko, April 29, 2021 (credit to Montvydas Klumbys for his expert guidance).
  To be used with custom Flight Control PCB Ver5 Rev0 
  Released into the public domain.
*/

#ifndef _NAV_LIGHT_H_
#define _NAV_LIGHT_H_

#include "Arduino.h"
#include "Adafruit_NeoPixel.h"


#define DEFAULT_NAV_LED_PIN   6 // MCU pin 6 on Flight Control PCB Ver7 Rev1
#define DEFAULT_NAV_LED_COUNT 4 // Nav NeoPixel (terminal on bottom of PCB)
#define NAV_FADE_DELAY 100      // Nav NeoPixel fade delay
#define NAV_WARNING_DELAY 50    // Nav NeoPixel warning delay

class NavLight
{
public:
    // Constructor: pin number, number of LEDs 
    NavLight(uint8_t pin = DEFAULT_NAV_LED_PIN, uint16_t length = DEFAULT_NAV_LED_COUNT);
    ~NavLight();

    void begin();
    void land();
    void hover();
    void fly();
    void dark();
    void warning();
    void warningUrgent();

private:
    void fade(uint16_t wait);
    void colorFade(uint8_t r, uint8_t g, uint8_t b, uint8_t wait);
    void repeatFade(uint8_t times, uint16_t wait);
    void repeatColorFade(uint8_t times, uint8_t r, uint8_t g, uint8_t b, uint16_t wait);
    void fill(uint32_t color);
    void blink(uint32_t color1, uint32_t color2, uint16_t wait);
    void repeatBlink(uint8_t times, uint32_t color1, uint32_t color2, uint16_t wait);
    void off();

    Adafruit_NeoPixel* pixels;
};

#endif
