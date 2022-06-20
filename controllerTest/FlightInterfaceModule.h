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
#include <Adafruit_BMP280.h>                // Barometric Pressure, Altitude, Temperature
#include <Adafruit_SHT31.h>                 // Humidity
#include "Adafruit_Sensor_Calibration.h"  // Calibration library 
#include <Quadcopter.h>                   // 
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor.h>
//#include "LSM6DS_LIS3MDL.h"  // combination library for LSM6DS33 and LIS3MDL, initialize and setup sensors

#define PWR_CONTROL A5
#define BAT_LVL     A6

#define ADDR_PITCH_OFFSET 0
#define ADDR_ROLL_OFFSET 1
#define BL 10   // MOTOR 1
#define FL 11   // MOTOR 2
#define BR 12   // MOTOR 4
#define FR 13   // MOTOR 3


#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
//#define AHRS_DEBUG_OUTPUT

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
    void checkCalibration();
    void testMotors();
    void printCurrentSpeed();
    float getOrientation(float *data);
//    float returnYPR(*data);
    void getAltitude();
    void getConditions();
    void adjThrottle(int x);
    void lightsLoop();
    
private:
    float Battery_Level;
    int twilight = 20; // threshold light level from APDS9960 to trigger Nav and Anti-Collision lights
    uint8_t proximity;
    uint16_t r, g, b, c;
    int pin, pinBat;
    uint32_t timestamp;
    float temperature, pressure, humidity, altitude;
};
#endif
