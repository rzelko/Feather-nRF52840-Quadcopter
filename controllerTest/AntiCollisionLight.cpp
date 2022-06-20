/*
  AntiCollisionLight.cpp - Source code for controlling Anti-Collision LEDs on Flight Control PCB.
  Created by Rick Zelko, April 28, 2021 (credit to Montvydas Klumbys for his expert guidance).
  To be used with custom Flight Control PCB Ver5 Rev0 
  Released into the public domain.
*/
#include "AntiCollisionLight.h"

//#define DEBUG

AntiCollisionLight::AntiCollisionLight(uint8_t pin, uint16_t length)
{
    // user can set the pin and length if wanted
    pixels = new Adafruit_NeoPixel(length, pin, NEO_GRB + NEO_KHZ800);// have to use normal order of variables (length, pin, NEO_GRB + NEO_KHZ800) and add type??
}

AntiCollisionLight::~AntiCollisionLight()
{
    free(pixels);  // Clears memory previously allocated to the NeoPixel??
}

void AntiCollisionLight::begin()
{
    pixels->begin();
    off();
}

void AntiCollisionLight::normalStrobe()
{
    repeatBlink(2,
        pixels->Color(150, 0, 0),
        pixels->Color(150, 150, 150),
        ANTI_COLL_BLINK_DELAY);
#ifdef DEBUG
    Serial.println("normalStrobe active");
#endif    
}

void AntiCollisionLight::fastStrobe()
{
    repeatBlink(2,
        pixels->Color(150, 0, 0),
        pixels->Color(150, 150, 150),
        ANTI_COLL_BLINK_DELAY / 2);
}

void AntiCollisionLight::slowStrobe()
{
    repeatBlink(2,
        pixels->Color(150, 0, 0),
        pixels->Color(150, 150, 150),
        ANTI_COLL_BLINK_DELAY * 2);
}

void AntiCollisionLight::dark()
{
    off();

#ifdef DEBUG
    Serial.println("lights are off");
#endif
}

// how many times to blink, what color and what is the delay
void AntiCollisionLight::repeatBlink(uint8_t times, uint32_t color1, uint32_t color2, uint16_t wait)
{
    for (uint8_t t = 0; t < times; t++)
    {
        blink(color1, color2, wait);
    }
    off();
}

void AntiCollisionLight::blink(uint32_t color1, uint32_t color2, uint16_t wait)
{
    pixels->fill(color1);
    pixels->show(); 
    delay(wait);
    pixels->fill(color2);
    pixels->show();
    delay(wait);
}

// All LEDs off
void AntiCollisionLight::off()
{
    pixels->clear();
    pixels->show();
}

// Set the same color to ALL pixels
void AntiCollisionLight::fill(uint32_t color)
{
    pixels->fill(color, 0, pixels->numPixels());
    pixels->show();
}
