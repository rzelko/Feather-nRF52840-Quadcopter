# Feather-nRF52840-Quadcopter

Newly designed Flight Control PCB V7 Rev 1 is complete and underrgoing testing currently.  The major change from V5 was the separation of power supplied to the motors from that of the other functions of the PCB.  Motors are powered via the BAT pin of the nRF52840 Sense, whereas the 3V pin is used to power AC and Nav LEDs as well as the VL53L0X TOF sensor.  Another significant change is the pin driving the NAV LEDs (now pin 6). Previously, pin 2 was used which created a conflict as it is shared with the antenna on the Bluetooth radio.  Pin 6 is connected to a terminal connector which allows for multiple NAV LEDs to be mounted on the frame of the quadcopter.  Library files were edited to reflect the pin change and LED count.

The unit is responding (crudely) to bluetooth control using the Bluefruit LE Connect app for iOS.  Currently, I am working to see if I can get the quad to lift off.  Once successful, I will focus on the code required for stabilizing and controlling the quad.  

I am now testing the quadcopter motor control again, having have made a few significant changes to the overall assembly.  Initially, I was using 15000kV 8520 motors with 55mm props, but I couldn't get the quad to lift off.  I determined that I was having an overall power issue as the Feather Sense would shut down completely at a PWM speed setting of around 110.  Even while connected to the USB, the motors would stop responding until reducing the PWM to 0 and then it would regain motor response.  Thinking that I needed to create more uplift, I installed 65mm props but still couldn't lift off.  In fact, the PWM level at which the motors would stop responding decreased to 100.  I was using Adafruit 1200mAh, 500mAh and 400mAh LiPo batteries, but I learned that these batteries are 2C rated batteries.  I ordered new 380Ah 1S 60C  batteries which hopefully will resolve my power issues. The overall weight of the quad was about 78 grams with the 400mAh battery.  Even printing new, lighter frames, the quad couldn't lift off with the 8520 motors.

I have modified the frame for 19000kV 615 motors and reduced the overall weight of the assemly with battery to 52g.  Still, not enough lift can be created before the Sense cuts out, although I can get up to a PWM setting of 250.  Hopefully the 60C batteries will do the trick.  Also, I am switching to tri blade props to see if that will help create more lift. 
