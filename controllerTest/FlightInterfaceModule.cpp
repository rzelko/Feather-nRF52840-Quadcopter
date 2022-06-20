/*
  FlightInterfaceModule.h - Library for controlling Flight Interface PCB.
  Created by Rick Zelko, April 28, 2021
  To be used with custom Flight Interface PCB Ver5 Rev0
  Paired with the Adafruit Feather nRF52840 Sense MCU and VL53L0X Breakout
  Released into the public domain.
*/

#include "FlightInterfaceModule.h"

// #define DEBUG

////////////////////////////////  Feather nRF52840 Sense DECLARATIONS/INSTANCES ////////////////////////////////

NavLight navLight(DEFAULT_NAV_LED_PIN, DEFAULT_NAV_LED_COUNT); 
AntiCollisionLight antiCollLight(DEFAULT_ANTI_COLL_LED_PIN, DEFAULT_ANTI_COLL_LED_COUNT); 
Adafruit_APDS9960 apds9960;         // proximity, light, color, gesture
Adafruit_BMP280 bmp280;             // temperautre, barometric pressure
Adafruit_SHT31 sht30;               // humidity
Adafruit_VL53L0X lox = Adafruit_VL53L0X();  // Create instance of TOF Sensor
Quadcopter quad(FL, FR, BL, BR);
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#include "LSM6DS_LIS3MDL.h"         // combination library for LSM6DS33 and LIS3MDL, initialize and setup sensors

Adafruit_Madgwick filter;           // filter for IMU and AHRS algorithms


// select either EEPROM or SPI FLASH storage:
#ifdef ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

    
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
// initialize the sensors
    apds9960.begin();
    apds9960.enableProximity(true);
    apds9960.enableColor(true);
    bmp280.begin();
    sht30.begin();
    lox.begin();

    pinMode (PWR_CONTROL, OUTPUT);
    digitalWrite(PWR_CONTROL, LOW);

    quad.arm();

    
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
    return;  
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
    return;
}
void FlightInterfaceModule::checkCalibration()
{
  
  delay(100);
  Serial.println("Calibration filesys test");
  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
    while (1) yield();
  }
  Serial.print("Has EEPROM: "); Serial.println(cal.hasEEPROM());
  Serial.print("Has FLASH: "); Serial.println(cal.hasFLASH());

    if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz
  cal.printSavedCalibration();

  Serial.println("Calibrations found: ");
  Serial.print("\tMagnetic Hard Offset: ");
  for (int i=0; i<3; i++) {
    Serial.print(cal.mag_hardiron[i]); 
    if (i != 2) Serial.print(", ");
  }
  Serial.println();
  
  Serial.print("\tMagnetic Soft Offset: ");
  for (int i=0; i<9; i++) {
    Serial.print(cal.mag_softiron[i]); 
    if (i != 8) Serial.print(", ");
  }
  Serial.println();

  Serial.print("\tGyro Zero Rate Offset: ");
  for (int i=0; i<3; i++) {
    Serial.print(cal.gyro_zerorate[i]); 
    if (i != 2) Serial.print(", ");
  }
  Serial.println();

  Serial.print("\tAccel Zero G Offset: ");
  for (int i=0; i<3; i++) {
    Serial.print(cal.accel_zerog[i]); 
    if (i != 2) Serial.print(", ");
  }
  Serial.println();

}

void FlightInterfaceModule::testMotors()
{
    int mySpeed[] = {75, 75, 75, 75};
    int* currSpeed = quad.getSpeed(); 
  
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
//  quad.getStabilisedSpeed(mySpeed, currSpeed, 15.2, -10.6);
  quad.getStabilisedSpeed(mySpeed, currSpeed, 15.0, -15.0);
//  quad.getStabilisedSpeed(mySpeed, currSpeed, 0, 0);
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

float FlightInterfaceModule::getOrientation(float *data)
{
    float roll, pitch, heading;
    float gx, gy, gz;
    static uint8_t counter = 0;
    
    if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
      return 0;
    }
    timestamp = millis();
    // Read the motion sensors
    sensors_event_t accel, gyro, mag;
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag); 
  
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("I2C took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif

    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);
    // Gyroscope needs to be converted from Rad/s to Degree/s
    // the rest are not unit-important
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
    filter.update(gx, gy, gz, 
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                  mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif
  
    // only print the calculated output once in a while
    if (counter++ <= PRINT_EVERY_N_UPDATES) {
      return 0;
    }
    // reset the counter
    counter = 0;

#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Raw: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.print(gz, 4); Serial.print(", ");
  Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  Serial.print(mag.magnetic.z, 4); Serial.println("");
#endif

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(", ");
    Serial.print(pitch);
    Serial.print(", ");
    Serial.println(roll);
  
    float qw, qx, qy, qz;
    filter.getQuaternion(&qw, &qx, &qy, &qz);
    Serial.print("Quaternion: ");
    Serial.print(qw, 4);
    Serial.print(", ");
    Serial.print(qx, 4);
    Serial.print(", ");
    Serial.print(qy, 4);
    Serial.print(", ");
    Serial.println(qz, 4);  
  
#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Took "); Serial.print(millis()-timestamp); Serial.println(" ms");
#endif 

    data[0] = heading;
    data[1] = pitch;
    data[2] = roll;
    
    return 0;
}

//float FlightInterfaceModule::returnYPR(float *data)
//{
//  
//    return 0;
//}

void FlightInterfaceModule::getAltitude()
{
    altitude = bmp280.readAltitude(1013.25);
    Serial.print("Altitude: ");
    Serial.print(altitude);
    Serial.println(" m");
}

void FlightInterfaceModule::getConditions()
{

    temperature = bmp280.readTemperature();
    pressure = bmp280.readPressure();
    humidity = sht30.readHumidity();
  
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    Serial.print("Barometric pressure: ");
    Serial.println(pressure);
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
}

void FlightInterfaceModule::adjThrottle(int x)
{
    int mySpeed[] = {x, x, x, x};
    int* currSpeed = quad.getSpeed(); 
  
    Serial.println("setConstantSpeed");
    quad.setConstantSpeed(x);
    printCurrentSpeed();
      
     Serial.println("getStabilisedSpeed");
    quad.getStabilisedSpeed(mySpeed, currSpeed,0, 0); //15.2, -10.6);
    quad.setSpeed(quad.getSpeed());
    printCurrentSpeed();

}

void FlightInterfaceModule::lightsLoop()
{ 
  this->checkLandingAlt();
  this->checkAmbient();
}
