/*
  NavLight.cpp - Source code for controlling Navigation LED on Flight Control PCB.
  Created by Rick Zelko, April 29, 2021 (credit to Montvydas Klumbys for his expert guidance)
  To be used with custom Flight Control PCB Ver5 Rev0 
  Released into the public domain.
*/

#include "NavLight.h"

NavLight::NavLight(uint8_t pin, uint16_t length)
{
    // user can set the pin and length if wanted
    pixels = new Adafruit_NeoPixel(length, pin, NEO_GRB + NEO_KHZ800);// have to use normal order of variables (length, pin, NEO_GRB + NEO_KHZ800) and add type??
}

NavLight::~NavLight()
{
    free(pixels);  // Clears memory previously allocated to the NeoPixel??
}

void NavLight::begin()
{
    pixels->begin();
    off();
}
void NavLight::fly()
{
    repeatFade(2,
        NAV_FADE_DELAY);
    
}

void NavLight::hover()
{
    repeatColorFade(5, 255, 0, 127,
          NAV_FADE_DELAY);
//    repeatFade(2,
//        NAV_FADE_DELAY / 4);
}

void NavLight::land()
{
    fill(pixels->Color(255, 255, 255));
}

void NavLight::dark()
{
    off();
}

// how many times to blink, what color and what is the delay
void NavLight::repeatFade(uint8_t times, uint16_t wait)
{
    for (uint8_t t = 0; t < times; t++)
    {
        fade(wait);
    }
    off();
}

void NavLight::repeatColorFade(uint8_t times, uint8_t r, uint8_t g, uint8_t b, uint16_t wait)
{
    for (uint8_t t = 0; t < times; t++)
    {
        colorFade(r, g, b, wait);
    }
    off();
}

void NavLight::colorFade(uint8_t r, uint8_t g, uint8_t b, uint8_t wait){
  for(uint16_t i = 0; i < pixels->numPixels(); i++) {
      uint8_t startR, startG, startB;
      uint32_t startColor = pixels->getPixelColor(i); // get the current colour
      startB = startColor & 0xFF;
      startG = (startColor >> 8) & 0xFF;
      startR = (startColor >> 16) & 0xFF;  // separate into RGB components

      if ((startR != r) || (startG != g) || (startB != b)){  // while the curr color is not yet the target color
        if (startR < r) startR++; else if (startR > r) startR--;  // increment or decrement the old color values
        if (startG < g) startG++; else if (startG > g) startG--;
        if (startB < b) startB++; else if (startB > b) startB--;
        pixels->setPixelColor(i, startR, startG, startB);  // set the color
        pixels->show();
        // delay(1);  // add a delay if its too fast
      }
      delay(wait);
  }
}

void NavLight::fade(uint16_t wait)
{
    for(long firstPixelHue = 2500; firstPixelHue < 5*65536; firstPixelHue += 512){
        for(int i=0; i<pixels->numPixels(); i++){
            int pixelHue = firstPixelHue + (i * 65536L /pixels->numPixels());
            pixels->setPixelColor(i, pixelHue);
        }
        pixels->show();
        delay(wait);
    }
}

// All LEDs off
void NavLight::off()
{
    pixels->clear();
    pixels->show();
}

// Set the same color to ALL pixels
void NavLight::fill(uint32_t color)
{
    pixels->fill(color, 0, pixels->numPixels());
    pixels->show();
}
