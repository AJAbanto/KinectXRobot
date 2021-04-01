//////////////////////////////////////////////////////////////
//Notes:                                                    //
//Gcode communication library used here can be found at:    //
//https://github.com/tinkersprojects/G-Code-Arduino-Library //
/////////////////////////////////////////////////////////////

#define X_STEP 2
#define Y_STEP 3
#define Z_STEP 4

#define X_DIR 5
#define Y_DIR 6
#define Z_DIR 7
#define EN 8

//Gear box input to output ratio (1:36 and 1:6)
#define X_GEAR_MULTIPLIER 36
#define Y_GEAR_MULTIPLIER 36
#define Z_GEAR_MULTIPLIER 36

#define X_DEGREE_PER_STEP 1.8
#define Y_DEGREE_PER_STEP 1.8
#define Z_DEGREE_PER_STEP 1.8

//Either for Fullstep or Microstep (1 or 16 respectively)
#define X_STEPPING_MODE 1
#define Y_STEPPING_MODE 1
#define Z_STEPPING_MODE 1

//Pulse width (in microseconds) for a step using A4988 drivers
#define STEP_PULSE_DELAY 27

//Delay  (in miliseconds) between each step pulse using A4988 drivers
#define STEP_DELAY 1
 
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
  pinMode(EN,OUTPUT);
  
}


//GCODE Commands

void write_Y_90accel(){
  int delays[1800];
  int STEPS = 1800;
  float c0 = 1500;
  float cn = c0;
  int cn_min = 100;

  digitalWrite(Y_DIR,HIGH);

  for(int i = 0; i < 1800; i++)
    Serial.println(i);
  /*
  //calculate delays
  for(int i = 0; i < STEPS ; i++){

    if( i > 0) cn = cn - ((2 * cn)/((4*i) + 1));
    if( cn < cn_min) cn = cn_min;
    delays[i] = cn;
    if(i < STEPS)Serial.println(i);
    if(i >= STEPS)("Done calculating");
  }
  // use delays from the array, forward
  for (int i = 0; i < STEPS; i++) {
    digitalWrite(Y_STEP, HIGH);
    delayMicroseconds( STEP_PULSE_DELAY );
    digitalWrite(Y_STEP, LOW);
    delayMicroseconds( delays[i] );
  }
  */

  Serial.println("Done");
}



//Function to write specific angle manually to Z stepper motor
void write_Z_stepper(int angle){
  int step_cnt = (abs(angle) / Z_DEGREE_PER_STEP) * Z_GEAR_MULTIPLIER  * Z_STEPPING_MODE;

  if(angle > 0) digitalWrite(Z_DIR,HIGH);  //going counter-clockwise atm
  else if(angle < 0) digitalWrite(Z_DIR,LOW);
   
  for(int i = 0; i < step_cnt;i++){
    digitalWrite(Z_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_DELAY);
    digitalWrite(Z_STEP, LOW);
    
    delay(STEP_DELAY); 
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
    
    delay(STEP_DELAY);
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
    
    delay(STEP_DELAY); 
  }
}


//Reset angle constants to pre-defined position
void reset_angles(){
  ALPHA = 90;
  BETA  = -180;
  THETA_0 = 0;
  THETA = 0;
}


commandscallback commands[1] = {{"R1",reset_angles}};
gcode Commands(1,commands);
