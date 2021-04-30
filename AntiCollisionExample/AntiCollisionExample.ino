#include "AntiCollisionLight.h"

//AntiCollisionLight antiCollLight; // use default values
AntiCollisionLight antiCollLight(DEFAULT_ANTI_COLL_LED_PIN, DEFAULT_ANTI_COLL_LED_COUNT); // Can do this instead too!
//AntiCollisionLight antiCollLight(5,4); // use default values

#define PWR_CONTROL A5
#define BAT_LVL     A6
float Battery_Level;

void setup()
{
  // on nRF52 this will block until serial is connected, comment out when serial is not used!
  while (!Serial) yield();
  Serial.begin(115200);
  pinMode(PWR_CONTROL,OUTPUT);
  digitalWrite(PWR_CONTROL, LOW);
  
  antiCollLight.begin();
  
  delay(1000);
  Serial.println("\n\nAntiCollisionLight Test Sketch");
  Serial.println("===========================");
  Serial.println("f(fast) n(normal)  s(slow)     Modes");
  Serial.println("Send me a command letter.");

  // need to flush inside setup, otherwise will not print
  Serial.flush();
}

void loop()
{
  Battery_Level = analogRead(BAT_LVL);
  Battery_Level *= 2;    // we divided by 2, so multiply back
  Battery_Level *= 3.3;  // Multiply by 3.3V, our reference voltage
  Battery_Level /= 1024; // convert to voltage

  if (Battery_Level <=2.25){
    digitalWrite(PWR_CONTROL,HIGH);
    } else digitalWrite(PWR_CONTROL,LOW);

  
  if (Serial.available())
  {
    char ch = Serial.read();
    if (ch == 'f')
    {
      antiCollLight.fastStrobe();
      Serial.println("anti-collision strobe fast");
    }
    else if (ch == 'n')
    {
      antiCollLight.normalStrobe();
      Serial.println("anti-collision strobe normal");
    }
    else if (ch == 's')
    {
      antiCollLight.slowStrobe();
      Serial.println("anti-collision strobe slow");
    }
  }
}
