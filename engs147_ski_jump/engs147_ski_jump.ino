//Noah Dunleavy
//ENGS147 - Ski Jump Code

//Include libraries
#include "AxisEncoderShield3.h"
#include "ArduinoMotorShieldR3.h"
#include <math.h> //for PI
#include "ENGS147_Utils.h"
#include "NAxisMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

//Pins used per shield
//Motor Shield: 
  //PWM: 12 (PIN_MOTOR_PWM)
  //Direction: 13 (PIN_MOTOR_DIR)
//Encoder Shield: (https://drive.google.com/file/d/1q7APIRsnUpQOD4YgCDmqQIS5qqpA4m3s/view)
  //X-Axis: 11
  //Y-Axis: 9
  //Z-Axis: 8
//LED:
  //Red: 6 (PIN_LED_RED)
  //Green: 4 (PIN_LED_GREEN)
  //Blue: 5 (PIN_LED_BLUE)
//Time of Flight: (https://www.adafruit.com/product/3316)
  //SDA: 20 (PIN_SDA) - Blue Line
  //SCL: 21 (PIN_SCL) - Yellow Line
  //GPIO: 22 (PIN_TOF_READY) - White Line
  //I2C Address: 0x29
//IMU: (https://github.com/adafruit/Adafruit_BNO055, https://www.adafruit.com/product/2472)
  //SDA: 20 (PIN_SDA) - Blue Line
  //SCL: 21 (PIN_SCL) - Yellow Line
  //GPIO: 23 (PIN_IMU_READY) - White Line
  //I2C Address: 0x28


//Definitions
#define ENCODER_X 1 //set up the xyz to channel mapping for readability later

#define RUN_TIME_MS 10000  //how long overall we will run for
#define DT_MOTOR_MS 500 //rate at which we control motor
#define DT_ORIENT_MS 1000 //how often we update the goal orientation / motor speed
#define US_TO_SEC 1000000 //microseconds in a second, for cleanliness later
#define NUM_SAMPLES 200 //allocate enough space

#define BUTTON_START 65 //also A11

#define RAMP_SPEED_RAD_S 100
#define IMPACT_ANGLE_RAD 0.25 //about 15 degrees

#define CPR 1440 //define the number of counts per motor encoder rotation

//Global Variables
ArduinoMotorShieldR3 motor_driver;  //initialize the motor shield

NAxisMotion mySensor;                 //Object that for the sensor

double goal_motor_rad_sec = 0.0;
double goal_orient_rad = 0.0;

//State tracking
enum State {IDLE, ON_RAMP, IN_AIR, LANDED};
State current_state, next_state = IDLE; //start idle

void setup() {
  Serial.begin(115200); //communicate at max speed

  //Setup motor driving
  motor_driver.init();  
  Serial.println("Shield Initialized!");
  initEncoderShield();  //void, so cannot do an if on it
  Serial.println("Encoder Initialized!");

  //Setup oreintation
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor. 
  //Sensor Initialization
  mySensor.initSensor(0x28);          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  mySensor.updateAccelConfig();

  //Setup GPIOs
  pinMode(BUTTON_START, INPUT_PULLUP);


  delay(500); //since cannot wait for init returns, just wait

}

void loop() {
 //Variable decleration
  unsigned long initial_start_time;  //keep track of when we kick things off, such to monitor total process time
  unsigned long current_time; //hold the most recent time update
  unsigned long previous_motor_start_time;  //previous motor control attempt
  unsigned long previous_orient_start_time; //previous orientation goal update
  long previous_encoder_value;
  bool running_motor = false; //flag to ensure we enter the loop on first iteration
  bool running_orient = false;
  
  double motor_speed[NUM_SAMPLES];  //[rad/sec] can go negative
  double car_pitch[NUM_SAMPLES];
  double command_voltage[NUM_SAMPLES];
  unsigned long motor_ndx = 0;
  unsigned long orient_ndx = 0;

  //Start up
  motor_driver.setM1Speed(0); //make sure motor is at rest
  delay(1000);

  while (current_state == IDLE){  //sit here until some boot out
    //Sit and do nothing
    set_RGB(255, 0, 0);

    if (digitalRead(BUTTON_START) == LOW){  //if button is pressed to start things
      next_state = ON_RAMP;
    }

    current_state = next_state;
  }

  //Begin timed processes
  current_time = micros();  //note here we are clcking off of us, but defines are in ms
  initial_start_time = current_time; //first start
  previous_encoder_value = getEncoderValue(ENCODER_X); //read where motor is when we start things  
  previous_motor_start_time = current_time; //start of whole process is also first loop
  previous_orient_start_time = current_time;

  while (current_time < (initial_start_time + RUN_TIME_MS*1000)){ //only run for specified time, convert to us //possibly add  && (times_ndx < NUM_SAMPLES)
    

    //Timing Loops
    //Update goal irentation at DT_ORIENT_MS
    current_time = micros();  
    if(!running_orient || (current_time - previous_orient_start_time) >= (DT_ORIENT_MS*1000)){ 
      if (current_state == ON_RAMP){
        goal_motor_rad_sec = RAMP_SPEED_RAD_S;  //if on ramp, just go for ramp speed
        break; //dont waste more time here than we need to
      }
      else if (current_state == IN_AIR){
        goal_orient_rad = IMPACT_ANGLE_RAD; //just for landing, we want this to be the goal - tricks will have this go through some array of orientations
      }

      //Time sensitive items first
      double dt_sec = (current_time - previous_orient_start_time) / (double)(US_TO_SEC);  
      double current_orient = get_pitch();

      //Compute and store
      car_pitch[orient_ndx] = current_orient;
      
      double e0 = goal_orient_rad - current_orient; //current error

      //Compute Goal Motor Speed to torque the car to desired orientation
        //Toruqe on motor one way will rotate car the other
        //Not largely concerned with error, largely just trasient - lead and proportional compensator
        //goal_motor_rad_sec = 0.1 * e0;

      //Set flags, update times and indexes
      running_orient = true; //only matters on first go through, 
      previous_orient_start_time = current_time; //the time which this loop started, which next time around will be the previous lop
      orient_ndx++;  //explictly do at end of use

      //Check data is ok

      //Update timing
      current_time = micros();  //update current time as these processes likely took some time

      //Check timing ok
    }

    //Drive a goal motor/car speed at DT_MOTOR_MS
    current_time = micros();  //update the top time
    if(!running_motor || (current_time - previous_motor_start_time) >= (DT_MOTOR_MS*1000)){ //if first pass through or time between loops exceeds our dt (to us)
      //Time sensitive items first
      double dt_sec = (current_time - previous_motor_start_time) / (double)(US_TO_SEC);  //not to be confused with defined dt, this is the recorded dt in sec
      long encoder_value = getEncoderValue(ENCODER_X); 

      //Compute and store
      motor_speed[motor_ndx] = running_motor ? get_motor_speed(encoder_value - previous_encoder_value, dt_sec) : 0.0;
      
      //Simple motor control loop, as seen in Lab #3
      double e0 = goal_motor_rad_sec - motor_speed[motor_ndx];  //current error
      double input = 0.1 * e0;

      //Apply Control
      command_voltage[motor_ndx] = input; 
      motor_driver.setM1Speed(voltage_to_pwm(input));

      //Set flags, update times and indexes
      running_motor = true; //only matters on first go through, 
      previous_motor_start_time = current_time; //the time which this loop started, which next time around will be the previous lop
      motor_ndx++;  //explictly do at end of use

      //Check data is ok

      //Update timing
      current_time = micros();  //update current time as these processes likely took some time

      //Check timing ok
    }


    //State Output
    if (current_state == ON_RAMP){
      set_RGB(0, 255, 0); 

      //Set motor speed goal
    } 
    else if (current_state == IN_AIR){
      set_RGB(0, 0, 255);
      //Update motor speed at DT_MOTOR_MS

      //Update motor goal orientation at rate DT_ORIENT_MS
    }
    else if (current_state == LANDED){
      set_RGB(255, 0, 255); //purple
      motor_driver.setM1Speed(0); //make sure motor is at rest
      delay(1000);
    }


    //Next State Logic
    if (current_state == ON_RAMP){
      if (get_dist_to_ground() > GROUND_THRESH){
        next_state = IN_AIR;
      }
    }
    else if (current_state == IN_AIR){
      if ((get_dist_to_ground() > GROUND_THRESH) && (get_pitch())){
        next_state = LANDED;
      }
    }
    else if (current_state == LANDED){
      //Just sit here
      next_state = IDLE;
    }

    //Change state
    current_state = next_state;
  }


  //Stop motor (either have been landed, or force a land)
  motor_driver.setM1Speed(0);

  if (current_state == IN_AIR){ //if we exitted loop because time elaosed, not because landed
    set_RGB(0, 255, 255);
  }
  else if (current_state == LANDED){
    set_RGB(0, 255, 0);
  }
  while(1){}
}

double get_pitch(){
  //[TODO] Check timing, maybe go continuous
  mySensor.updateEuler(); //force I2C to update data
  return mySensor.readEulerPitch(); //return the updated pitch

  //if at end of motor loop, have more time, maybe come to check this. And see if move ocntinuous
}

double get_dist_to_ground(){
  return 0.0;
}
