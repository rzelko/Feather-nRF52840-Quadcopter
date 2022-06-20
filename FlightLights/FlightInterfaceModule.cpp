/*
  FlightInterfaceModule.h - Library for controlling Flight Interface PCB.
  Created by Rick Zelko, April 28, 2021
  To be used with custom Flight Interface PCB Ver5 Rev0
  Paired with the Adafruit Feather nRF52840 Sense MCU and VL53L0X Breakout
  Released into the public domain.
*/

#include "FlightInterfaceModule.h"

//#define DEBUG

NavLight navLight(DEFAULT_NAV_LED_PIN, DEFAULT_NAV_LED_COUNT); 
AntiCollisionLight antiCollLight(DEFAULT_ANTI_COLL_LED_PIN, DEFAULT_ANTI_COLL_LED_COUNT); 
Adafruit_APDS9960 apds9960;         // proximity, light, color, gesture
Adafruit_VL53L0X lox = Adafruit_VL53L0X();  // Create instance of TOF Sensor

Quadcopter quad = Quadcopter(M0_PIN, M1_PIN, M2_PIN, M3_PIN);

int* currSpeed;
int* mySpeed;

//Constructor
FlightInterfaceModule::FlightInterfaceModule(uint8_t pin, uint8_t pinBat)
{

}

FlightInterfaceModule::~FlightInterfaceModule()
{

}

void FlightInterfaceModule::begin()
{
    navLight.begin();
    antiCollLight.begin();
    apds9960.begin();
    apds9960.enableProximity(true);
    apds9960.enableColor(true);
    lox.begin();

    pinMode (PWR_CONTROL, OUTPUT);
    digitalWrite(PWR_CONTROL, LOW);
    
#ifdef DEBUG
    if (digitalRead(PWR_CONTROL) == LOW){
      Serial.println("Flight Interface Module is ready");
      } else Serial.println("Flight Interface Module not ready.  Please check battery!");
#endif
    
    checkBat();

#ifdef DEBUG
    Serial.print("Battery Level: ");
    Serial.println(Battery_Level);
#endif
    
    if (Battery_Level <=3.25)
    {
        warning();
    } else digitalWrite(PWR_CONTROL, LOW);
}

float FlightInterfaceModule::checkBat()
{
    Battery_Level = analogRead(BAT_LVL);
    Battery_Level *= 2;    // we divided by 2, so multiply back
    Battery_Level *= 3.3;  // Multiply by 3.3V, our reference voltage
    Battery_Level /= 1024; // convert to voltage

//    Serial.print("Battery Level: ");
//    Serial.println(Battery_Level);

    return(Battery_Level);
}
void FlightInterfaceModule::warning()
{
    
    if (Battery_Level <= 3.2){
        navLight.warningUrgent();

        Serial.println("Urgent warning triggered");
    } else {
            navLight.warning();
            Serial.println("Warning triggered");
           }
}

void FlightInterfaceModule::checkAmbient()
{
    proximity = apds9960.readProximity();
    while (!apds9960.colorDataReady()) {
      delay(5);
    }
    apds9960.getColorData(&r, &g, &b, &c);

#ifdef DEBUG
    Serial.print("Proximity: ");
    Serial.println(apds9960.readProximity());
    Serial.print("Red: ");
    Serial.print(r);
    Serial.print(" Green: ");
    Serial.print(g);
    Serial.print(" Blue :");
    Serial.print(b);
    Serial.print(" Clear: ");
    Serial.println(c);
#endif

    if (r <= twilight || g <= twilight || b <= twilight || c <= twilight){
      antiCollLight.slowStrobe();    
    } else antiCollLight.dark();
}

void FlightInterfaceModule::checkLandingAlt()
{
    VL53L0X_RangingMeasurementData_t measure;
    Serial.print("Reading a measurement... ");
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

      if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (in): "); Serial.println(measure.RangeMilliMeter/25.4);
        if(measure.RangeMilliMeter/25.4 <= 12){
          navLight.land();
        } else navLight.dark();
    
    } else {
      Serial.println(" out of range ");
      }
}
//void FlightInterfaceModule::checkCalibration()
//{
//  delay(100);
//  Serial.println("Calibration filesys test");
//  if (!cal.begin()) {
//    Serial.println("Failed to initialize calibration helper");
//    while (1) yield();
//  }
//  Serial.print("Has EEPROM: "); Serial.println(cal.hasEEPROM());
//  Serial.print("Has FLASH: "); Serial.println(cal.hasFLASH());
//
//  if (! cal.loadCalibration()) {
//    Serial.println("**WARNING** No calibration loaded/found");
//  }
//  cal.printSavedCalibration();
//
//  Serial.println("Calibrations found: ");
//  Serial.print("\tMagnetic Hard Offset: ");
//  for (int i=0; i<3; i++) {
//    Serial.print(cal.mag_hardiron[i]); 
//    if (i != 2) Serial.print(", ");
//  }
//  Serial.println();
//  
//  Serial.print("\tMagnetic Soft Offset: ");
//  for (int i=0; i<9; i++) {
//    Serial.print(cal.mag_softiron[i]); 
//    if (i != 8) Serial.print(", ");
//  }
//  Serial.println();
//
//  Serial.print("\tGyro Zero Rate Offset: ");
//  for (int i=0; i<3; i++) {
//    Serial.print(cal.gyro_zerorate[i]); 
//    if (i != 2) Serial.print(", ");
//  }
//  Serial.println();
//
//  Serial.print("\tAccel Zero G Offset: ");
//  for (int i=0; i<3; i++) {
//    Serial.print(cal.accel_zerog[i]); 
//    if (i != 2) Serial.print(", ");
//  }
//  Serial.println();
//}

void FlightInterfaceModule::testMotors()
{
  Serial.println("setConstantSpeed");
  quad.setConstantSpeed(20);
  printCurrentSpeed();
  delay(2000);

  Serial.println("arm");
  quad.arm();
  printCurrentSpeed();
  delay(2000);

  Serial.println("disarm");
  quad.disarm();
  printCurrentSpeed();
  delay(2000);

  Serial.println("setSpeed");
  quad.setSpeed(mySpeed);
  printCurrentSpeed();
  delay(2000);

  Serial.println("setSingleMotor");
  for (int i = 0; i < 4; i++){
    quad.setSingleMotor(i,mySpeed[i]);
    printCurrentSpeed();
    delay(2000);
  }
  
  Serial.println("getStabilisedSpeed");
  quad.getStabilisedSpeed(mySpeed, currSpeed, 15.2, -10.6);
  quad.setSpeed(quad.getSpeed());
  printCurrentSpeed();
  delay(2000);

  Serial.println("disarm");
  quad.disarm();
  printCurrentSpeed();
  delay(2000);
}

void FlightInterfaceModule::printCurrentSpeed()
{
    int* currSpeed = quad.getSpeed();
    for (int i = 0; i < 4; i++){
        Serial.print(currSpeed[i]);
        Serial.print(" ");
    }
    Serial.println();    
}
