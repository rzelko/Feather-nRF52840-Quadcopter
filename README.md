# Feather-nRF52840-Quadcopter

The AntiCollisionLight.h and .cpp files were edited in effort to get troubleshoot why the sketch doesn't actually activate the LEDs on the Flight Control PCB.
  - I edited the .cpp file adding some debugging measures in begin() to verify pin and length was set correctly
  - I changed the order of the pin and length variables in establishing the neoPixel instance within the .cpp file.  This corrected the pin assignment and led count - also added the type `NEO_GRB + NEO_KHZ800` to it (which made no difference)
  - In the .cpp file, I edited the blink function by adding the `pixels->show()` pointer to see if that helped (but it didn't).  I also added the pointer to the fill() call but that made no difference.
  - I tried seeing if a new approach to blink() would make a difference (its commented out b/c it didn't)
 
