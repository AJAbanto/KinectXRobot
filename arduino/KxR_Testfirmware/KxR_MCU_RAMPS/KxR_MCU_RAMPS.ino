

#include <gcode.h>
#include "ramps.h"
void setup() {
  setup_pins();
  Commands.begin((unsigned long) 9600); //initiate gcode communication
  Serial.println("Ready");
}

void loop() {
  
  if(Commands.available()){

    //Manual Writing steppers
    if(Commands.availableValue('X')){
      int angleX = Commands.GetValue('X');
      write_X_stepper(angleX);
    }
    
    if(Commands.availableValue('Y')){
      int angleY = Commands.GetValue('Y');
      write_Y_stepper(angleY);
      //write_Y_90accel();
    }
    
    if(Commands.availableValue('Z')){
      int angleZ = Commands.GetValue('Z');
      write_Z_stepper(angleZ);
    }
    
  }
}
