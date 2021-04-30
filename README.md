# Feather-nRF52840-Quadcopter

The AntiCollisionLight.h and .cpp files were edited and are now activating the LEDs on the Flight Control PCB.
  - I edited the source file adding some temporary debugging measures in begin() to verify pin and length was set correctly (now deleted)
  - I changed the order of the pin and length variables in establishing the neoPixel instance within the source file.  This corrected the pin assignment and led count - also added the type `NEO_GRB + NEO_KHZ800` to it (which made no difference but appears to be preferred)
  - In the source file, I edited the blink function by adding the `pixels->show()` pointer which was required.  I also added the pointer to the fill() call just for clarity.
  
 
