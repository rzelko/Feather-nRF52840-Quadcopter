#include "AntiCollisionLight.h"

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
    Serial.print("Number of Pixels: ");
    Serial.println(pixels->numPixels());
    Serial.print("NeoPixel Pin: ");
    Serial.println(pixels->getPin());
    off();
}

void AntiCollisionLight::normal()
{
    repeatBlink(5,
        pixels->Color(150, 0, 0),
        pixels->Color(150, 150, 150),
        ANTI_COLL_BLINK_DELAY);
}

void AntiCollisionLight::high()
{
    repeatBlink(5,
        pixels->Color(150, 0, 0),
        pixels->Color(150, 150, 150),
        ANTI_COLL_BLINK_DELAY / 2);
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

//    pixels->clear();
//    for(int i=0; i<DEFAULT_ANTI_COLL_LED_COUNT; i++){
//      pixels->setPixelColor(i,color1);
//      pixels->show();
//      delay(wait);
//      pixels->clear();
//      pixels->setPixelColor(i,color2);
//      pixels->show();
//      delay(wait);
//    }
//    pixels->clear();
//    pixels->show();
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
