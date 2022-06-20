#include "NavLight.h"

//NavLight navLight; // use default values
NavLight navLight(DEFAULT_NAV_LED_PIN, DEFAULT_NAV_LED_COUNT); 
//NavLight navLight(2,1); 

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
  
  navLight.begin();
  
  delay(1000);
  Serial.println("\n\nNavLight Test Sketch");
  Serial.println("===========================");
  Serial.println("f(fly) h(hover)  l(land)     Modes");
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

  if (Battery_Level <=3.25){
    digitalWrite(PWR_CONTROL,HIGH);
    } else digitalWrite(PWR_CONTROL,LOW);

  
  if (Serial.available())
  {
    char ch = Serial.read();
    if (ch == 'f')
    {
      navLight.fly();
      Serial.println("nav fly mode");
    }
    else if (ch == 'h')
    {
      navLight.hover();
      Serial.println("nav hover mode");
    }
    else if (ch == 'l')
    {
      navLight.land();
      Serial.println("nav landing mode");
    }
    else if (ch == 'o')
    {
      navLight.dark();
      Serial.println("nav off mode");
    }
    else if (ch == 'w')
    {
      navLight.warning();
      Serial.println("warning mode");
    }
    else if (ch == 'u')
    {
      navLight.warningUrgent();
      Serial.println("urgent warning mode");
    }
  }
}
