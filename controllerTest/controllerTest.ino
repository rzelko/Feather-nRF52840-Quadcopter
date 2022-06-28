/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <bluefruit.h>
#include "FlightInterfaceModule.h"
#include <PID_v1.h>


// OTA DFU service
BLEDfu bledfu;

// Uart over BLE service
BLEUart bleuart;

// Function prototypes for packetparser.cpp
uint8_t readPacket (BLEUart *ble_uart, uint16_t timeout);
float   parsefloat (uint8_t *buffer);
void    printHex   (const uint8_t * data, const uint32_t numBytes);

// Packet buffer
extern uint8_t packetbuffer[];

FlightInterfaceModule flight(BAT_LVL); //removed "PWR_CONTROL," from declaration 

int mySpeed = 0, previousSpeed = 0;
int targetSpeed[4];
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container 

float ypr_cal[3];

////////////////////////////////  PID CONTROLLER /////////////////////////////////////

//Define Variables we'll be connecting to
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;
double yawSetpoint, yawInput, yawOutput;
double throttleSetpoint, throttleInput, throttleOutput;

//Define the aggressive and conservative Tuning Parameters (proportional–integral–derivative)
double consKp = 0.5, consKi = 0.05, consKd = 0.05;

PID pitchPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, DIRECT);
PID rollPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, DIRECT);
//PID yawPID(&yawInput, &yawOutput, &yawSetpoint, consKp, consKi, consKd, DIRECT);
//PID throttlePID(&throttleInput, &throttleOutput, &throttleSetpoint, consKp, consKi, consKd, DIRECT);



void setup(void)
{
  flight.begin();
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println(F("Adafruit Bluefruit52 Controller App Example"));
  Serial.println(F("-------------------------------------------"));

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and start the BLE Uart service
  bleuart.begin();

  // Set up and start advertising
  startAdv();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();  


  delay(1000);
  Serial.println("\n\nFlight Interface Module Test");

  flight.checkCalibration();

// need to flush inside setup, otherwise will not print
  Serial.flush();

  for (int i = 0; i < 3; i++) {
      ypr_cal[i] = 0.0;
    } 

//   flight.testMotors();

  //------------------------------PID----------------------------------
  //initialize the variables we're linked to
 
  pitchInput = 0.0;
  rollInput = 0.0;

  pitchSetpoint = 0.0;
  rollSetpoint = 0.0;

  //turn the PID on
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);

  pitchPID.SetOutputLimits(-20, 20);
  rollPID.SetOutputLimits(-20, 20);
  //-------------------------------------------------------------------

  for (int i = 0; i < 4; i++) {
    targetSpeed[i] = 0;
  }

  Serial.println(F("CALIBRATING"));
  ypr_cal[0] = ypr[0] * 180 / M_PI;
  ypr_cal[1] = ypr[1] * 180 / M_PI;
  ypr_cal[2] = ypr[2] * 180 / M_PI; 
 
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
 
  // Wait for new data to arrive
  uint8_t len = readPacket(&bleuart, 500);
  if (len == 0) return;

  // Got a packet!
//   printHex(packetbuffer, len);  

  pitchInput = ypr[1] * 180 / M_PI - ypr_cal[1];
  rollInput = ypr[2] * 180 / M_PI - ypr_cal[2];
  
  pitchPID.Compute();
  rollPID.Compute();
    
  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed ) {
      Serial.println(" pressed");
      if (buttnum == 1){
//            int currentSpeed;
            mySpeed = previousSpeed + 10;
            previousSpeed = mySpeed;
            if (mySpeed > 255){
              mySpeed = 255;
            }
            flight.adjThrottle(mySpeed);
            flight.getOrientation(ypr);            
            Serial.println("Speed increased");
            bleuart.write("Speed: ");

      }
      if (buttnum == 2){
            mySpeed = previousSpeed - 10;
            previousSpeed = mySpeed;
            if (mySpeed < 0){
              mySpeed = 0;
            }
            flight.adjThrottle(mySpeed);
            flight.getOrientation(ypr);    
            Serial.println("Speed decreased");
            bleuart.write("Speed: ");

      }  
      if (buttnum == 3){
            mySpeed = previousSpeed + 25;
            previousSpeed = mySpeed;
            if (mySpeed < 0){
              mySpeed = 0;
            }
            flight.adjThrottle(mySpeed);
            flight.getOrientation(ypr);
            Serial.println("Speed increased");
            bleuart.write("Speed: ");
      }  
      if (buttnum == 4){
            mySpeed = 0;
            previousSpeed = mySpeed;
            if (mySpeed < 0){
              mySpeed = 0;
            }
            flight.adjThrottle(mySpeed);
            flight.getOrientation(ypr);
            Serial.println("All Stop!");
            bleuart.write("Speed: ");
      }  
    } else {
      Serial.println(" released");
    }
 }
  flight.lightsLoop();
//  flight.getOrientation(ypr);
  Serial.print("YPR: ");
  Serial.print(ypr[0]);
  Serial.print(", ");
  Serial.print(ypr[1]);
  Serial.print(", ");
  Serial.println(ypr[2]);
  delay(10); 
}









//  // GPS Location
//  if (packetbuffer[1] == 'L') {
//    float lat, lon, alt;
//    lat = parsefloat(packetbuffer+2);
//    lon = parsefloat(packetbuffer+6);
//    alt = parsefloat(packetbuffer+10);
//    Serial.print("GPS Location\t");
//    Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
//    Serial.print('\t');
//    Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
//    Serial.print('\t');
//    Serial.print(alt, 4); Serial.println(" meters");
//  }
//
//  // Accelerometer
//  if (packetbuffer[1] == 'A') {
//    float x, y, z;
//    x = parsefloat(packetbuffer+2);
//    y = parsefloat(packetbuffer+6);
//    z = parsefloat(packetbuffer+10);
//    Serial.print("Accel\t");
//    Serial.print(x); Serial.print('\t');
//    Serial.print(y); Serial.print('\t');
//    Serial.print(z); Serial.println();
//  }
//
//  // Magnetometer
//  if (packetbuffer[1] == 'M') {
//    float x, y, z;
//    x = parsefloat(packetbuffer+2);
//    y = parsefloat(packetbuffer+6);
//    z = parsefloat(packetbuffer+10);
//    Serial.print("Mag\t");
//    Serial.print(x); Serial.print('\t');
//    Serial.print(y); Serial.print('\t');
//    Serial.print(z); Serial.println();
//  }
//
//  // Gyroscope
//  if (packetbuffer[1] == 'G') {
//    float x, y, z;
//    x = parsefloat(packetbuffer+2);
//    y = parsefloat(packetbuffer+6);
//    z = parsefloat(packetbuffer+10);
//    Serial.print("Gyro\t");
//    Serial.print(x); Serial.print('\t');
//    Serial.print(y); Serial.print('\t');
//    Serial.print(z); Serial.println();
//  }
//
//  // Quaternions
//  if (packetbuffer[1] == 'Q') {
//    float x, y, z, w;
//    x = parsefloat(packetbuffer+2);
//    y = parsefloat(packetbuffer+6);
//    z = parsefloat(packetbuffer+10);
//    w = parsefloat(packetbuffer+14);
//    Serial.print("Quat\t");
//    Serial.print(x); Serial.print('\t');
//    Serial.print(y); Serial.print('\t');
//    Serial.print(z); Serial.print('\t');
//    Serial.print(w); Serial.println();
//  }
