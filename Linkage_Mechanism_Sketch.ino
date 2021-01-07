// ME350 Disc Sorting Sketch - Version 01
// Based on Target Dropping Sketch V4.1 by Karthik Urs, Joe Saginaw, David Mark, Julia Slatin
// updated 10-29-2020 by Minna He

//////////////////////////////////////////////
// DEFINE CONSTANTS AND GLOBAL VARIABLES:   //
//////////////////////////////////////////////

//------------------------------ State Machine: ------------------------------//
// CONSTANTS: 
// Definition of states in the state machine
const int CALIBRATE                = 1;
const int PICKUP_DISC              = 2;
const int DETERMINE_DISC_COLOR     = 3;
const int MOVE_TO_ZONE             = 4;

// VARIABLES:
// Global variable that keeps track of the state:
// Start the state machine in calibration state:
int state = CALIBRATE;

//------------------------------ Color Sensor: ------------------------------//
// CONSTANTS: 
// Definition of disc colors:
const int RED = 1;
const int YELLOW = 2; 
const int BLUE = 3; 
const int NONE = 4; 

// VARIABLES: 
// Stores output frequency from the color sensor
int red = 0;
int green = 0;
int blue = 0;
int clr = 0;

// THRESHOLDS:
// Color sensor data compared against these to determine disc color

// RED DISC BOUNDS
int RED_R_LB = 6400;  // Lower bound for red channel
int RED_R_UB = 8000;  // Upper bound for red channel
int RED_G_LB = 4200;  // Lower bound for green channel
int RED_G_UB = 5100;  // Upper bound for green channel
int RED_B_LB = 5900;  // Lower bound for blue channel
int RED_B_UB = 7100;  // Upper bound for blue channel
int RED_CL_LB = 17000; // Lower bound for clear channel
int RED_CL_UB = 21000; // Upper bound for clear channel

// YELLOW DISC BOUNDS
int YELLOW_R_LB = 6600;  // Lower bound for red channel
int YELLOW_R_UB = 9600;  // Upper bound for red channel
int YELLOW_G_LB = 6000;  // Lower bound for green channel
int YELLOW_G_UB = 7700;  // Upper bound for green channel
int YELLOW_B_LB = 6500;  // Lower bound for blue channel
int YELLOW_B_UB = 7700;  // Upper bound for blue channel
int YELLOW_CL_LB = 21000; // Lower bound for clear channel
int YELLOW_CL_UB = 28000; // Upper bound for clear channel

// BLUE DISC BOUNDS
int BLUE_R_LB = 4600;  // Lower bound for red channel
int BLUE_R_UB = 5900;  // Upper bound for red channel
int BLUE_G_LB = 5200;  // Lower bound for green channel
int BLUE_G_UB = 6400;  // Upper bound for green channel
int BLUE_B_LB = 8500;  // Lower bound for blue channel
int BLUE_B_UB = 10001;  // Upper bound for blue channel
int BLUE_CL_LB = 19000; // Lower bound for clear channel
int BLUE_CL_UB = 25000; // Upper bound for clear channel

//------------------------------ Computation of position and velocity: ------------------------------//
// CONSTANTS: 
// Settings for velocity computation:
const int MIN_VEL_COMP_COUNT = 2;     // [encoder counts] Minimal change in motor position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME = 10000; // [microseconds] Minimal time that must pass between two velocity measurements
// VARIABLES:
volatile long motorPosition = 0; // [encoder counts] Current motor position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int encoderStatus = 0; // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).
float motorVelocity        = 0; // [encoder counts / seconds] Current motor velocity 
int previousMotorPosition  = 0; // [encoder counts] Motor position the last time a velocity was computed 
long previousVelCompTime   = 0; // [microseconds] System clock value the last time a velocity was computed 

//------------------------------ High-level behavior of the controller:  ------------------------------//
// CONSTANTS:
// Target positions:
const int CALIBRATION_VOLTAGE     = -3;                     // [Volt] Motor voltage used during the calibration process
const int DISC_PICKUP_POSITION    = 0;                     // [encoder counts] Motor position corresponding to disc pickup location
const int RED_ZONE_POSITION       = 325;                   // [encoder counts] Motor position corresponding to the red zone
const int YELLOW_ZONE_POSITION    = 155;                   // [encoder counts] Motor position corresponding to the yellow zone
const int BLUE_ZONE_POSITION      = 91;                   // [encoder counts] Motor position corresponding to the blue zone
const int WAIT_POSITION           = 0;                   // [encoder counts] Motor position corresponding to a wait position
const int LOWER_BOUND             = 0;                     // [encoder counts] Position of the left end stop
const int UPPER_BOUND             = 325;                  // [encoder counts] Position of the right end stop
const int TARGET_BAND             = 5;                     // [encoder counts] "Close enough" range when moving towards a target.

//------------------------------ PID Controller  ------------------------------//
// CONSTANTS:
const float KP                    = 0.07; // [Volt / encoder counts] P-Gain .08
const float KI                    = 0.011; // [Volt / (encoder counts * seconds)] I-Gain .005
const float KD                    = 0.006; // [Volt * seconds / encoder counts] D-Gain .004
const float SUPPLY_VOLTAGE        = 12;   // [Volt] Supply voltage at the HBridge
const float FRICTION_COMP_VOLTAGE = 2.15;     // [Volt] Voltage needed to overcome friction

// VARIABLES:
int desiredPosition  = 0; // [encoder counts] desired motor position
float positionError  = 0; // [encoder counts] Position error
float integralError  = 0; // [encoder counts * seconds] Integrated position error
float velocityError  = 0; // [encoder counts / seconds] Velocity error
float desiredVoltage = 0; // [Volt] Desired motor voltage
int   motorCommand   = 0; // [0-255] PWM signal sent to the motor
unsigned long executionDuration = 0; // [microseconds] Time between this and the previous loop execution.  Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0; // [microseconds] System clock value at the moment the loop was started the last time

//------------------------------ Electromagnet: ------------------------------//
//CONSTANTS:
const float DESIRED_PICKUP_VOLTAGE = 12.0; // [Volts]
const float DESIRED_MOVE_VOLTAGE = 12.0;   // [Volts]
const int   EM_PICKUP_DURATION = 2;        // [seconds]
const int   EM_DROP_DURATION = 2;          // [seconds]

//VARIABLES:
int pwm_voltage;  // pwm voltage delivered to electromagnet (0 to 255)
int dropT = 0;    // current time when drop starts
int dropT_c = 0;  // current time while dropping
float decayV = 0; // sinusoidal voltage 

//------------------------------ Pin assignment: ------------------------------//
// CONSTANTS:
const int PIN_NR_ENCODER_A        = 2;  // Never change these, since the interrupts are attached to pins 2 and 3
const int PIN_NR_ENCODER_B        = 3;  // Never change these, since the interrupts are attached to pins 2 and 3
const int PIN_NR_ON_OFF_SWITCH    = 9;  // Connected to toggle switch (turns mechanism on and off)
const int PIN_NRL_LIMIT_SWITCH    = 10; // Connected to limit switch (mechanism calibration)
const int PIN_NR_PWM_OUTPUT       = 11; // Connected to H Bridge (controls motor speed)
const int PIN_NR_PWM_DIRECTION_1  = 12; // Connected to H Bridge (controls motor direction)
const int PIN_NR_PWM_DIRECTION_2  = 13; // Connected to H Bridge (controls motor direction)
const int PIN_NR_PWM_ELECMAG      = 5;  // Connected to H Bridge (controls electromagnet voltage pwm)
const int PIN_NR_ELECMAG_DIR1     = 1;  // Connected to H Bridge (controls electromagnet direction)
const int PIN_NR_ELECMAG_DIR2     = 4;  // Connected to H Bridge (controls electromagnet direction)
const int PIN_COLORSENSE_S2       = 6;  // Connected to color sensor (determines color filter)
const int PIN_COLORSENSE_S3       = 7;  // Connected to color sensor (determines color filter)
const int PIN_COLORSENSE_OUT      = 8;  // Connected to color sensor (output signal from sensor)
const int PIN_COLORSENSE_S0       = A0; // Connected to color sensor (determines frequency scaling)
const int PIN_COLORSENSE_S1       = A1; // Connected to color sensor (determines frequency scaling)
const int PIN_COLORSENSE_LED      = A2; // Connected to color sensor (turns LEDS on/off)

// End of CONSTANTS AND GLOBAL VARIABLES


//////////////////////////////////////////////////////////////////////////////////////////
// The setup() function is called when a sketch starts. Use it to initialize variables, //
// pin modes, start using libraries, etc. The setup function will only run once, after  //
// each powerup or reset of the Arduino board:                                          //
//////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Declare which digital pins are inputs and which are outputs:
  // Toggle and Limit Switch
  pinMode(PIN_NR_ON_OFF_SWITCH,    INPUT);
  pinMode(PIN_NRL_LIMIT_SWITCH,    INPUT);
  // Motor encoder and power
  pinMode(PIN_NR_ENCODER_A,        INPUT_PULLUP);
  pinMode(PIN_NR_ENCODER_B,        INPUT_PULLUP);
  pinMode(PIN_NR_PWM_OUTPUT,       OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_1,  OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_2,  OUTPUT);
  // Electromagnet 
  pinMode(PIN_NR_ELECMAG_DIR1,     OUTPUT);
  pinMode(PIN_NR_ELECMAG_DIR2,     OUTPUT);
  pinMode(PIN_NR_PWM_ELECMAG,      OUTPUT);
  // Color Sensor 
  pinMode(PIN_COLORSENSE_S2,       OUTPUT);
  pinMode(PIN_COLORSENSE_S3,       OUTPUT);
  pinMode(PIN_COLORSENSE_OUT,      INPUT); 
  pinMode(PIN_COLORSENSE_S0,       OUTPUT);
  pinMode(PIN_COLORSENSE_S1,       OUTPUT);
  pinMode(PIN_COLORSENSE_LED,      OUTPUT);
  
  // Turn on the pullup resistors on the encoder channels
  digitalWrite(PIN_NR_ENCODER_A, HIGH);  
  digitalWrite(PIN_NR_ENCODER_B, HIGH);

  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updateMotorPosition' is called:
  attachInterrupt(0, updateMotorPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  attachInterrupt(1, updateMotorPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  // Color sensor settings
  digitalWrite(PIN_COLORSENSE_S0, HIGH); // Set to 20% scaling
  digitalWrite(PIN_COLORSENSE_S1, LOW);
  digitalWrite(PIN_COLORSENSE_LED, HIGH); // Turn on LEDS

  // Begin serial communication for monitoring.
  Serial.begin(115200);
  Serial.println("Start Executing Program.");
  

  // Set initial output to the motor to 0
  analogWrite(PIN_NR_PWM_OUTPUT, 0);

  // Set duty cycle of electromagnet. mapped to 0-255
  analogWrite(PIN_NR_PWM_ELECMAG, 0);
  digitalWrite(PIN_NR_ELECMAG_DIR1, LOW);
  digitalWrite(PIN_NR_ELECMAG_DIR2, LOW);
  dropDisc();
}
//------------------------------ End of function setup() ------------------------------//


////////////////////////////////////////////////////////////////////////////////////////////////
// After going through the setup() function, which initializes and sets the initial values,   //
// the loop() function does precisely what its name suggests, and loops consecutively,        //
// allowing your program to sense and respond. Use it to actively control the Arduino board.  //
//////////////////////////////////////////////////////////////////////////////////////////////// 
void loop() {
  // Determine the duration it took to execute the last loop. This time is used 
  // for integration and for monitoring the loop time via the serial monitor.
  executionDuration = micros() - lastExecutionTime;
  lastExecutionTime = micros();

  // Speed Computation:
  if ((abs(motorPosition - previousMotorPosition) > MIN_VEL_COMP_COUNT) || (micros() - previousVelCompTime) > MIN_VEL_COMP_TIME){
    // If at least a minimum time interval has elapsed or
    // the motor has travelled through at least a minimum angle ... 
    // .. compute a new value for speed:
    // (speed = delta angle [encoder counts] divided by delta time [seconds])
    motorVelocity = (double)(motorPosition - previousMotorPosition) * 1000000 / 
                            (micros() - previousVelCompTime);
    // Remember this encoder count and time for the next iteration:
    previousMotorPosition = motorPosition;
    previousVelCompTime   = micros();
  }
  
  //******************************************************************************//
  // The state machine:
  switch (state) {
    //****************************************************************************//
    // In the CALIBRATE state, we move the mechanism to a position outside of the 
    // work space (towards the limit switch).  Once the limit switch is on and 
    // the motor has stopped turning, we know that we are against the end stop
    case CALIBRATE:
      // We don't have to do anything here since this state is only used to set
      // a fixed output voltage.  This happens further below.
      
      // Decide what to do next:
      if (digitalRead(PIN_NRL_LIMIT_SWITCH)==HIGH && motorVelocity==0) { 
        // We reached the endstop.  Update the motor position to the limit:
        // (NOTE: If the limit switch is on the right, this must be UPPER_BOUND)
        motorPosition = LOWER_BOUND;  
        // Reset the error integrator:
        integralError = 0;
        // Calibration is finalized. Transition into DETERMINE_DISC_COLOR state
        Serial.println("State transition from CALIBRATE to DETERMINE_DISC_COLOR");
        state = PICKUP_DISC;
      } 
      // Otherwise we continue calibrating
      break;

    //****************************************************************************//
    // In the PICKUP_DISC state, we move to the disc pickup location
    case PICKUP_DISC:
      // The target position is the disc pickup location
      Serial.println("Moving to disc pickup"); // Comment out later to reduce lag
      desiredPosition = DISC_PICKUP_POSITION;

      if ((motorPosition <= desiredPosition + TARGET_BAND) && (motorPosition >= desiredPosition - TARGET_BAND) && motorVelocity==0) {
        //Transition into DETERMINE_DISC_COLOR
        Serial.println("State transition from PICKUP_DISC to DETERMINE_DISC_COLOR");
        state = DETERMINE_DISC_COLOR;
      }

      // Otherwise we continue moving towards the pickup position
      break;

    //****************************************************************************//
    // In the DETERMINE_DISC_COLOR state, we read the color sensor, lift the disc, and then move to the appropriate zone
    case DETERMINE_DISC_COLOR:

      if((motorPosition <= desiredPosition + TARGET_BAND) && (motorPosition >= desiredPosition - TARGET_BAND) && motorVelocity==0) {
      // We have reached the color sensor

        // Check for the disc color and output to serial:
        // If we get a color reading, set the position and move to that color zone
        // Consider taking an average of multiple readings
        delay(100);
        getColor();
        int discColor = evaluateColorSensor();
        Serial.print(" Disc color is: ");
        switch (discColor) {
           case RED:
              Serial.println("RED.");
              desiredPosition = RED_ZONE_POSITION;
              Serial.println("State transition from DETERMINE_DISC_COLOR to MOVE_TO_ZONE");
              // We have reached the pickup location
              // Turn on the electromagnet to pick up a disc
              Serial.println("Electromagnet is ON");
              setEM(DESIRED_PICKUP_VOLTAGE);
              delay(EM_PICKUP_DURATION*1000); // Wait for the electromagnet to pick up
              state = MOVE_TO_ZONE;
              break;
              
           case YELLOW:
              Serial.println("YELLOW.");
              desiredPosition = YELLOW_ZONE_POSITION;
              Serial.println("State transition from DETERMINE_DISC_COLOR to MOVE_TO_ZONE");
              // Turn on the electromagnet to pick up a disc
              Serial.println("Electromagnet is ON");
              setEM(DESIRED_PICKUP_VOLTAGE);
              delay(EM_PICKUP_DURATION*1000); // Wait for the electromagnet to pick up
              state = MOVE_TO_ZONE;
              break;
             
           case BLUE:
              Serial.println("BLUE.");
              desiredPosition = BLUE_ZONE_POSITION;
              Serial.println("State transition from DETERMINE_DISC_COLOR to MOVE_TO_ZONE");
              // Turn on the electromagnet to pick up a disc
              Serial.println("Electromagnet is ON");
              setEM(DESIRED_PICKUP_VOLTAGE);
              delay(EM_PICKUP_DURATION*1000); // Wait for the electromagnet to pick up
              state = MOVE_TO_ZONE;
              break;
              
           case NONE:  Serial.println("NONE.");  
              Serial.println("Electromagnet is OFF");
              digitalWrite(PIN_NR_ELECMAG_DIR1, LOW);
              digitalWrite(PIN_NR_ELECMAG_DIR2, LOW);
              break;
         }
      }

      // Otherwise, we stay in DETERMINE_DISC_COLOR
      break;

    //****************************************************************************//
    // In the MOVE_TO_ZONE state, we select an color zone and move toward it, or
    // move toward Target 3 (a default position) if there is no active target
    case MOVE_TO_ZONE:
      setEM(DESIRED_MOVE_VOLTAGE);

      //Comment out later to reduce lag
      if (desiredPosition == RED_ZONE_POSITION) {
        Serial.println("Moving to Red Zone");
      }
      else if (desiredPosition == YELLOW_ZONE_POSITION) {
        Serial.println("Moving to Yellow Zone");
      }
      else if (desiredPosition == BLUE_ZONE_POSITION) {
        Serial.println("Moving to Blue Zone");
      }
      
      if ((motorPosition <= desiredPosition + TARGET_BAND) && (motorPosition >= desiredPosition - TARGET_BAND) && motorVelocity==0) {
        // We have reached the desired disc drop off zone

        // Turn the electromagnet off to drop the disc
        Serial.println("Arrived at drop off zone, drop disc");
        dropDisc();
        delay (1000); // Wait 1 second for the disc to drop

        Serial.println("State transition from MOVE_TO_ZONE to PICKUP_DISC");
        state = PICKUP_DISC;
      }
      // Otherwise, we keep moving to the disc drop off zone
      break;

    //****************************************************************************//
    // We should never reach the next bit of code, which would mean that the state
    // we are currently in doesn't exist.  So if it happens, throw an error and 
    // stop the program:
    default: 
      Serial.println("Statemachine reached at state that it cannot handle.  ABORT!!!!");
      Serial.print("Found the following unknown state: ");
      Serial.println(state);
      while (1); // infinite loop to halt the program
    break;
  }
  // End of the state machine.
  //******************************************************************************//

  //******************************************************************************//
  // Recalibrate if we are in the leftmost position
  if (digitalRead(PIN_NRL_LIMIT_SWITCH)==HIGH && motorVelocity==0) { 
        // We reached the endstop.  Update the motor position to the limit:
        // (NOTE: If the limit switch is on the right, this must be UPPER_BOUND)
        motorPosition = LOWER_BOUND;  
        // Reset the error integrator:
        integralError = 0;
  } 
  
 
  //******************************************************************************//
  // Position Controller
  if (digitalRead(PIN_NR_ON_OFF_SWITCH)==HIGH) {
    // If the toggle switch is on, run the controller:

    //** PID control: **//  
    // Compute the position error [encoder counts]
    positionError = desiredPosition - motorPosition;
    // Compute the integral of the position error  [encoder counts * seconds]
    integralError = integralError + positionError * (float)(executionDuration) / 1000000; 
    // Compute the velocity error (desired velocity is 0) [encoder counts / seconds]
    velocityError = 0 - motorVelocity;
    // This is the actual controller function that uses the error in 
    // position and velocity and the integrated error and computes a
    // desired voltage that should be sent to the motor:
    desiredVoltage = KP * positionError +  
                     KI * integralError +
                     KD * velocityError;
 
    //** Friction Compensation terms: **//
    // Compensate for friction.  That is, if we now the direction of 
    // desired motion, add a base command that helps with moving in this
    // direction:
    if (positionError < -5) {
      desiredVoltage = desiredVoltage - FRICTION_COMP_VOLTAGE;
    }
    if (positionError > +5) {
      desiredVoltage = desiredVoltage + FRICTION_COMP_VOLTAGE;
    }

    // Anti-Wind-Up
    if (abs(desiredVoltage)>SUPPLY_VOLTAGE) {
      // If we are already saturating our output voltage, it does not make
      // sense to keep integrating the error (and thus ask for even higher
      // and higher output voltages).  Instead, stop the integrator if the 
      // output saturates. We do this by reversing the summation at the 
      // beginning of this function block:
      integralError = integralError - positionError * (float)(executionDuration) / 1000000; 
    }
    // End of 'if(onOffSwitch==HIGH)'
    
    // Override the computed voltage during calibration.  In this state, we simply apply a 
    // fixed voltage to move against one of the end-stops.
    if (state == CALIBRATE) {
      // add calibration code here
      desiredVoltage = CALIBRATION_VOLTAGE;
    }
  } else { 
    // Otherwise, the toggle switch is off, so do not run the controller, 
    // stop the motor...
    desiredVoltage = 0; 
    // .. and reset the integrator of the error:
    integralError = 0;
    
    //Also turn off the electromagnet
    digitalWrite(PIN_NR_ELECMAG_DIR1, LOW);
    digitalWrite(PIN_NR_ELECMAG_DIR2, LOW);
    
    // Produce some debugging output:
    Serial.println("The toggle switch is off.  Motor Stopped. Electromagnet off. ");
  } 
  // End of  else onOffSwitch==HIGH
  
  //** Send signal to motor **//
  // Convert from voltage to PWM cycle:
  motorCommand = int(abs(desiredVoltage * 255 / SUPPLY_VOLTAGE));
  // Clip values larger than 255
  if (motorCommand > 255) {
    motorCommand = 255;
  }
  // Send motor signals out
  analogWrite(PIN_NR_PWM_OUTPUT, motorCommand);
  // Determine rotation direction
  if (desiredVoltage >= 0) {
    // If voltage is positive ...
    // ... turn forward
    digitalWrite(PIN_NR_PWM_DIRECTION_1,LOW);  // rotate forward
    digitalWrite(PIN_NR_PWM_DIRECTION_2,HIGH); // rotate forward
  } else {
    // ... otherwise turn backward:
    digitalWrite(PIN_NR_PWM_DIRECTION_1,HIGH); // rotate backward
    digitalWrite(PIN_NR_PWM_DIRECTION_2,LOW);  // rotate backward
  }
  // End of Position Controller
  //*********************************************************************//
  
  // Print out current controller state to Serial Monitor.
  printStateToSerial();
}
//------------------------------ End of main loop ------------------------------//


//////////////////////////////////////////////////////////////////////
// This is a function to update the encoder count in the Arduino.   //
// It is called via an interrupt whenever the value on encoder      //
// channel A or B changes.                                          //
//////////////////////////////////////////////////////////////////////
void updateMotorPosition() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(2);   
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(3);
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  encoderStatus &= 15;
  if (encoderStatus==2 || encoderStatus==4 || encoderStatus==11 || encoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    motorPosition++;         // increase the encoder count by one
  } 
  else if (encoderStatus == 1 || encoderStatus == 7 || encoderStatus == 8 || encoderStatus == 14) {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    motorPosition--;         // decrease the encoder count by one
  }
}
//------------------------------ End of function updateMotorPosition() ------------------------------//


//////////////////////////////////////////////////////////////////////
// This function sends a status of the controller to the serial     //
// monitor.  Each character will take 85 microseconds to send, so   //
// be selective in what you write out:                              //
//////////////////////////////////////////////////////////////////////
void printStateToSerial() {
  //*********************************************************************//
  // Send a status of the controller to the serial monitor.  
  // Each character will take 85 microseconds to send, so be selective
  // in what you write out:

  //Serial.print("State Number:  [CALIBRATE = 1; MOVE_TO_DISC_PICKUP = 2; DETERMINE_DISC_COLOR = 3; MOVE_TO_ZONE = 4]: ");
  Serial.print("State#: "); 
  Serial.print(state);

  //Serial.print("      Motor Position [encoder counts]: ");
  Serial.print("  MP: "); 
  Serial.print(motorPosition);

  //Serial.print("      Motor Velocity [encoder counts / seconds]: ");
  Serial.print("  MV: "); 
  Serial.print(motorVelocity);

  //Serial.print("      Target Position [encoder counts]: ");
  Serial.print("  DP: "); 
  Serial.print(desiredPosition);

  //Serial.print("      Position Error [encoder counts]: ");
  Serial.print("  PE: "); 
  Serial.print(positionError);

  //Serial.print("      Integrated Error [encoder counts * seconds]: ");
  Serial.print("  IE: "); 
  Serial.print(integralError);

  //Serial.print("      Velocity Error [encoder counts / seconds]: ");
  Serial.print("  VE: "); 
  Serial.print(velocityError);

  //Serial.print("      Desired Output Voltage [Volt]: ");
  Serial.print("  DV: "); 
  Serial.print(desiredVoltage);
  
  
  // ALWAYS END WITH A NEWLINE.  SERIAL MONITOR WILL CRASH IF NOT
  Serial.println(); // new line
}
//------------------------------ End of Serial Out ------------------------------//


//////////////////////////////////////////////////////////////////////
// This function returns the RGB and Clr values read by             //
// the color sensor.                                                //
//////////////////////////////////////////////////////////////////////
int getColor(){
  // Setting RED (R) filtered photodiodes to be read
  digitalWrite(PIN_COLORSENSE_S2,LOW);  digitalWrite(PIN_COLORSENSE_S3,LOW);
  red = 1000000/pulseIn(PIN_COLORSENSE_OUT, LOW); // [micro sec] Time of one pulse from sensor
  Serial.print("R = ");  Serial.print(red); // Print red frequency value
  delay(100);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(PIN_COLORSENSE_S2,HIGH);  digitalWrite(PIN_COLORSENSE_S3,HIGH);
  green = 1000000/pulseIn(PIN_COLORSENSE_OUT, LOW); // [micro sec] Time of one pulse from sensor
  Serial.print(" G = ");  Serial.print(green); // Print green frequency value 
  delay(100);

  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(PIN_COLORSENSE_S2,LOW);  digitalWrite(PIN_COLORSENSE_S3,HIGH);
  blue = 1000000/pulseIn(PIN_COLORSENSE_OUT, LOW); // [micro sec] Time of one pulse from sensor
  Serial.print(" B = ");  Serial.print(blue); // Print blue frequency value
  delay(100);

  // Setting CLEAR (Cl) filtered photodiodes to be read
  digitalWrite(PIN_COLORSENSE_S2,HIGH);  digitalWrite(PIN_COLORSENSE_S3,LOW);
  clr = 1000000/pulseIn(PIN_COLORSENSE_OUT, LOW); // [micro sec] Time of one pulse from sensor
  Serial.print(" Clr = ");  Serial.print(clr); // Print clear frequency value
  delay(100);
}
//------------------------------ End of function getColor ------------------------------//

//////////////////////////////////////////////////////////////////////
// This function returns the color of the disc that is read         //
// based on the color calibration ranges set by the student         //
//////////////////////////////////////////////////////////////////////
int evaluateColorSensor(){
  // initialize disc type with 'NONE'.  Override later if a disc color was detected
  int discColor = NONE;
  
  // Check if the disc is RED
  if ((red > RED_R_LB) && (red < RED_R_UB) && (green > RED_G_LB) && (green < RED_G_UB) && (blue > RED_B_LB) && (blue < RED_B_UB) && (clr > RED_CL_LB) && (clr < RED_CL_UB)){
    discColor = RED;
  }
  // Check if the disc is YELLOW
  if ((red > YELLOW_R_LB) && (red < YELLOW_R_UB) && (green > YELLOW_G_LB) && (green < YELLOW_G_UB) && (blue > YELLOW_B_LB) && (blue < YELLOW_B_UB) && (clr > YELLOW_CL_LB) && (clr < YELLOW_CL_UB)){
    discColor = YELLOW;
  }
  // Check if the disc is BLUE
  if ((red > BLUE_R_LB) && (red < BLUE_R_UB) && (green > BLUE_G_LB) && (green < BLUE_G_UB) && (blue > BLUE_B_LB) && (blue < BLUE_B_UB) && (clr > BLUE_CL_LB) && (clr < BLUE_CL_UB)){
    discColor = BLUE;
  }
  
  // Note: If none of these if-statements is true, discColor remains 'NONE'
  return discColor;
} 
//------------------------------ End of function evaluateColorSensor() ------------------------------//


//////////////////////////////////////////////////////////////////////
// This function makes the electromagnet drop the disc using        //
// a sinusoidally decaying function to control the voltage          //
//////////////////////////////////////////////////////////////////////
void dropDisc() {
  dropT = millis();
  dropT_c = millis();
  while (dropT_c - dropT < EM_DROP_DURATION) {

    decayV = DESIRED_PICKUP_VOLTAGE*exp(-(dropT_c-dropT)/60)*sin((dropT_c-dropT)/6);
    setEM(decayV);
    Serial.println("Dropping");
    dropT_c = millis();
  }
}
//------------------------------ End of function dropDisc() ------------------------------//


//////////////////////////////////////////////////////////////////////
// This function sets the voltage of the electromagnet based        //
//////////////////////////////////////////////////////////////////////
void setEM(float involt) {
  int prev_volt = pwm_voltage;
  pwm_voltage = int(abs(involt/SUPPLY_VOLTAGE*255));
  if (prev_volt == pwm_voltage) return;
  analogWrite(PIN_NR_PWM_ELECMAG, pwm_voltage);
  if (involt > 0){
    digitalWrite(PIN_NR_ELECMAG_DIR1, HIGH);
    digitalWrite(PIN_NR_ELECMAG_DIR2, LOW);
  }
  else if (involt < 0){
    digitalWrite(PIN_NR_ELECMAG_DIR1, LOW);
    digitalWrite(PIN_NR_ELECMAG_DIR2, HIGH);
  }
  else {
    digitalWrite(PIN_NR_ELECMAG_DIR1, LOW);
    digitalWrite(PIN_NR_ELECMAG_DIR2, LOW);
  }
}
//------------------------------ End of function setEM() ------------------------------//
