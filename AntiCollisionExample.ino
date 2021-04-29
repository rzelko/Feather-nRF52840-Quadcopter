#include "AntiCollisionLight.h"

//AntiCollisionLight antiCollLight; // use default values
AntiCollisionLight antiCollLight(DEFAULT_ANTI_COLL_LED_PIN, DEFAULT_ANTI_COLL_LED_COUNT); // Can do this instead too!
//AntiCollisionLight antiCollLight(5,4); // use default values

void setup()
{
  // on nRF52 this will block until serial is connected, comment out when serial is not used!
  while (!Serial) yield();
  Serial.begin(115200);

  
  antiCollLight.begin();

  delay(1000);
  Serial.println("\n\nAntiCollisionLight Test Sketch");
  Serial.println("===========================");
  Serial.println("h(high) l(low)     Modes");
  Serial.println("Send me a command letter.");

  // need to flush inside setup, otherwise will not print
  Serial.flush();
}

void loop()
{
  if (Serial.available())
  {
    char ch = Serial.read();
    if (ch == 'h')
    {
      antiCollLight.high();
      Serial.println("anti-collision strobe fast");
    }
    else if (ch == 'l')
    {
      antiCollLight.normal();
      Serial.println("anti-collision strobe normal");
    }
  }
}
