#include <gcode.h>
#include <pin_names.h>

//Notes:
//Gcode communication library used here can be found at:
//https://github.com/tinkersprojects/G-Code-Arduino-Library

//Instantiating GCODE streaming
void homing();
void clear_lcd();
commandscallback commands[2] = {{"G33",homing},{"G34",clear_lcd}};
gcode Commands(2,commands);
//Variables for coordinates sent via Gcode
double X,Y,Z;


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
