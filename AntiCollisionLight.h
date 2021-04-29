/*
  AntiCollisionLight.h - Library for controlling Anti-Collision LEDs on Flight Control PCB.
  Created by Rick Zelko, April 28, 2021.
  To be used with custom Flight Control PCB Ver5 Rev0 
  Released into the public domain.
*/

#ifndef _ANTI_COLLISION_LIGHT_H_
#define _ANTI_COLLISION_LIGHT_H_

#include "Arduino.h"
#include "Adafruit_NeoPixel.h"


#define DEFAULT_ANTI_COLL_LED_PIN   5 // MCU pin 5 on Flight Control PCB Ver5 Rev0
#define DEFAULT_ANTI_COLL_LED_COUNT 4 // Anti-Collision NeoPixels (LED1 and LED3 on top and LED2 and LED4 on bottom)};
#define ANTI_COLL_BLINK_DELAY 100


class AntiCollisionLight
{
public:
    // Constructor: pin number, number of LEDs 
    AntiCollisionLight(uint8_t pin = DEFAULT_ANTI_COLL_LED_PIN, uint16_t length = DEFAULT_ANTI_COLL_LED_COUNT);
//    AntiCollisionLight(void);
    ~AntiCollisionLight();

    void begin(void);
    void normal(void);
    void high(void);

private:
    void blink(uint32_t color1, uint32_t color2, uint16_t wait);
    void repeatBlink(uint8_t times, uint32_t color1, uint32_t color2, uint16_t wait);
    void fill(uint32_t color);
    void off(void);

    Adafruit_NeoPixel* pixels;
};

#endif
