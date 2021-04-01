//RAMPS 1.4 PINS
#define X_STEP         54
#define X_DIR          55
#define X_ENABLE       38
 
#define Y_STEP         60
#define Y_DIR          61
#define Y_ENABLE       56


#define Z_STEP         46
#define Z_DIR          48
#define Z_ENABLE       62

//////////////////////////////////////////////////////////////
//Notes:                                                    //
//Gcode communication library used here can be found at:    //
//https://github.com/tinkersprojects/G-Code-Arduino-Library //
/////////////////////////////////////////////////////////////


//Gear box input to output ratio (1:36 and 1:6)
#define X_GEAR_MULTIPLIER 36
#define Y_GEAR_MULTIPLIER 36
#define Z_GEAR_MULTIPLIER 36

#define X_DEGREE_PER_STEP 1.8
#define Y_DEGREE_PER_STEP 1.8
#define Z_DEGREE_PER_STEP 1.8

//Either for Fullstep or Microstep (1 or 16 respectively)
#define X_STEPPING_MODE 16
#define Y_STEPPING_MODE 16
#define Z_STEPPING_MODE 16

//Pulse width (in microseconds) for a step using A4988 drivers
#define STEP_PULSE_DELAY 70


#define DELAY_BETWEEN_PULSE false
//Delay  (in miliseconds) between the falling edge of the previous pulse
//and  the rising edge of the next step pulse using A4988 drivers
#define STEP_DELAY 0
 
//Link lengths (in milimeters)
#define L1 300
#define L2 170
#define L3 170

//Joint angles (in degrees)
int THETA,ALPHA,BETA,THETA_0; 


//Setup pins
void setup_pins(){
  pinMode(X_STEP,OUTPUT);
  pinMode(Y_STEP,OUTPUT);
  pinMode(Z_STEP,OUTPUT);
  pinMode(X_DIR,OUTPUT);
  pinMode(Y_DIR,OUTPUT);
  pinMode(Z_DIR,OUTPUT);
  pinMode(X_ENABLE,OUTPUT);
  pinMode(Y_ENABLE,OUTPUT);
  pinMode(Z_ENABLE,OUTPUT);

  digitalWrite(X_ENABLE,LOW);
  digitalWrite(Y_ENABLE,LOW);
  digitalWrite(Z_ENABLE,LOW);
  
}


//GCODE Commands


//Function to write specific angle manually to Z stepper motor
void write_Z_stepper(int angle){
  int step_cnt = (abs(angle) / Z_DEGREE_PER_STEP) * Z_GEAR_MULTIPLIER  * Z_STEPPING_MODE;

  if(angle > 0) digitalWrite(Z_DIR,HIGH);  //going counter-clockwise atm
  else if(angle < 0) digitalWrite(Z_DIR,LOW);
   
  for(int i = 0; i < step_cnt;i++){
    digitalWrite(Z_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_DELAY);
    digitalWrite(Z_STEP, LOW);

    if(DELAY_BETWEEN_PULSE) delayMicroseconds(STEP_DELAY); 
  }
}

//Function to write specific angle manually to Y stepper motor
void write_Y_stepper(int angle){
  int step_cnt = (abs(angle) / Y_DEGREE_PER_STEP) * Y_GEAR_MULTIPLIER  * Y_STEPPING_MODE;

  if(angle > 0) digitalWrite(Y_DIR,HIGH);  //going counter-clockwise atm
  else if(angle < 0) digitalWrite(Y_DIR,LOW);
   
  for(int i = 0; i < step_cnt;i++){
    digitalWrite(Y_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_DELAY);
    digitalWrite(Y_STEP, LOW);
    
    if(DELAY_BETWEEN_PULSE) delayMicroseconds(STEP_DELAY);
  }
}

//Function to write specific angle manually to X stepper motor
void write_X_stepper(int angle){
  int step_cnt = (abs(angle) / X_DEGREE_PER_STEP) * X_GEAR_MULTIPLIER  * X_STEPPING_MODE;

  if(angle > 0) digitalWrite(X_DIR,HIGH);  //going counter-clockwise atm
  else if(angle < 0) digitalWrite(X_DIR,LOW);
   
  for(int i = 0; i < step_cnt;i++){
    digitalWrite(X_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_DELAY);
    digitalWrite(X_STEP, LOW);
    
    if(DELAY_BETWEEN_PULSE) delayMicroseconds(STEP_DELAY);
  }
}


//Enable motors
void enable_motors(){
  digitalWrite(X_ENABLE,LOW);
  digitalWrite(Y_ENABLE,LOW);
  digitalWrite(Z_ENABLE,LOW);
  Serial.println("Enabled motors");
}
//Disable motors
void disable_motors(){
  digitalWrite(X_ENABLE,HIGH);
  digitalWrite(Y_ENABLE,HIGH);
  digitalWrite(Z_ENABLE,HIGH);
  Serial.println("Disabled motors");
}


commandscallback commands[2] = {{"M17",enable_motors},{"M18",disable_motors}};
gcode Commands(2,commands);
