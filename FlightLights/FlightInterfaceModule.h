/*
  FlightInterfaceModule.h - Library for controlling Flight Interface PCB.
  Created by Rick Zelko, April 28, 2021
  To be used with custom Flight Interface PCB Ver5 Rev0 
  Released into the public domain.
*/

#ifndef _FLIGHT_INTERFACE_MODULE_H_
#define _FLIGHT_INTERFACE_MODULE_H_

#include "Arduino.h"
#include "FlightInterfaceModule.h"
#include "AntiCollisionLight.h"           // Anti-Collision Light Control
#include "NavLight.h"                     // Navigation Light Control
#include <Adafruit_VL53L0X.h>             // ToF Distance Sensor
#include <Adafruit_APDS9960.h>            // Proximity, Light, Color, Gesture Sensor
//#include "Adafruit_Sensor_Calibration.h"  // Calibration library 
#include <Quadcopter.h>                   // 

#define PWR_CONTROL A5
#define BAT_LVL     A6
#define M0_PIN      10 
#define M1_PIN      11
#define M2_PIN      13
#define M3_PIN      12

//// select either EEPROM or SPI FLASH storage:
//#ifdef ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM
//  Adafruit_Sensor_Calibration_EEPROM cal;
//#else
//  Adafruit_Sensor_Calibration_SDFat cal;
//#endif


class FlightInterfaceModule
{
public:
    //Constructor:
    FlightInterfaceModule(uint8_t pin = PWR_CONTROL, uint8_t pinBat = BAT_LVL);
    ~FlightInterfaceModule();

    void begin();
    float checkBat();
    void checkAmbient();
    void checkLandingAlt();
    void warning();
    void shutdown();
//    void checkCalibration();
    void testMotors();
    void printCurrentSpeed();
  
private:
    float Battery_Level;
    int twilight = 20; // threshold light level from APDS9960 to trigger Nav and Anti-Collision lights
    uint8_t proximity;
    uint16_t r, g, b, c;
    int pin, pinBat;    
};
#endif
