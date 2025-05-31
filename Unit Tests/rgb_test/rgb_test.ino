//Noah Dunleavy
//ENGS147 - Ski Jump Code

//Include libraries
#include "ENGS147_Ski_Jump.h"
#include <math.h>

void setup() {
  init_RGB();
}

void loop() {
  set_RGB(255, 0, 0);
  delay(1000);
  set_RGB(0, 255, 0);
  delay(1000);
  set_RGB(0, 0, 255);
  delay(1000);
  set_RGB(255, 255, 255); //should be white, but actually just flashing colors
  delay(1000);
  set_RGB(255, 255, 0); //should be yellow-ish, but actually just flashing green while red high
  delay(1000);
  set_RGB(0, 255, 255);
  delay(1000);
  set_RGB(255, 0, 255);
  delay(1000);
  set_RGB(0,0,0);
  delay(2000);

}
