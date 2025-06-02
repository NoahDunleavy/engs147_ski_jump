//SETUP:
  //Table all the way down, 


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_VL6180X.h"
#include "AxisEncoderShield3.h"
#include "ENGS147_Ski_Jump.h"

#define VL6_ADDR 0x29 //time of flight i2c address (verified with i2c scanner)
#define BNO055_ADDRESS 0x28
#define BNO055_INT_EN 0x10
#define BNO055_INT_MSK 0x0F
#define BNO055_SYS_TRIGGER 0x3F

//Tweakable params
#define GROUND_THRESH 20  //mm threshold

//Pins
#define PIN_TOF_RESET 51  
#define PIN_IMU_READY 23
#define PIN_BUTTON 13
#define PIN_MOTOR_PWM 12

//Debug Pins
#define DBG_PIN_MOTOR 31
#define DBG_PIN_ORIENT 32
#define DBG_PIN_IMU 33
#define DBG_PIN_TOF 34
#define PIN_LED_RED 35
#define PIN_LED_GREEN 39
#define PIN_LED_BLUE 37

//Timing Parameters
#define RUN_TIME_MS 5000 //amount of time to give system before shsutdown AFTER on-ramp starts
#define MAX_AIR_TIME_MS 550
#define RAMP_TO_AIR 60
#define DT_ORIENT_MS 50
#define HANG_TIME_MS 200

#define TIMING_TOLERANCE_US 400  //how far off each timing loop can be from goal and still be 'ok'
#define IMU_ROUNDTRIP_MS 4  //stays under 4k us for orient and rad_sec

#define PWM_RAMP 160
#define PWM_RAMP_UP_TIME_MS 900

//Timing Structure
#define NUM_TIMERS 2  //one timer for oreint, motor, imu, and tof
#define NDX_ORIENT 0
#define NDX_MOTOR 1

//Control Constants
#define B_E0 10
#define B_E1 0
#define B_U1 0

double previous_error = 0;
double previous_input = 0;


//Structure to define timings
struct Timing {
  bool running = false;
  unsigned long previous_start_time_us = 0;
  unsigned long sample_period_us = 0;
  unsigned long next_start_us = 0;  //track when we think the next loop should be run
};

Timing control_timer[NUM_TIMERS];

volatile bool kicked = false;

uint8_t range = 0, status = 0;

double ref_angle = -PI;

Adafruit_BNO055 sensor_imu = Adafruit_BNO055(55, BNO055_ADDRESS, &Wire);
Adafruit_VL6180X sensor_tof = Adafruit_VL6180X();

enum State {IDLE, ON_RAMP, IN_AIR, LANDED};
State current_state = IDLE, next_state = IDLE, previous_state = IDLE; //start idle

void setup() {
  Serial.begin(115200);
  //Wire.begin();
  Serial.println("Serial initialized!");
  //Setup pins
  init_RGB();
  pinMode(PIN_BUTTON, INPUT_PULLUP);  //so if button pressed, goes low
  pinMode(PIN_TOF_RESET, OUTPUT);

  //TOF
  init_tof();

  //IMU
  if(!sensor_imu.begin()){
    set_RGB(255, 0, 0);
    Serial.println("IMU Failed!");
    //while(1){}  //if no imu, wait
  }

  //Motor
  init_motor();
  
  //Encoder
  initEncoderShield();

  //Setup timers
  control_timer[NDX_ORIENT].sample_period_us = DT_ORIENT_MS * 1000;

  set_RGB(0, 255, 0); //show we are all good
  delay(2000);  //then wait a bit
  Serial.println("Ready");
  set_RGB(0, 0, 0); //befor going off to free up indicators
}

void loop() {
  //Waiting idle
  //set_motor_speed(0); //make sure motor is at rest
  while(digitalRead(PIN_BUTTON) == HIGH){ //until button is pressed
    set_RGB(255, 0, 0);
  }
  delay(2000);
  init_watchdog(HANG_TIME_MS);
  unsigned long ramp_start_time = micros();
  uint8_t range = 0; //range in mm
  while(range < GROUND_THRESH){
    int ramp_speed = PWM_RAMP * min(double((micros() - ramp_start_time)) / double(PWM_RAMP_UP_TIME_MS * 1000), 1);
    set_motor_speed(ramp_speed);
    range = sensor_tof.readRange(); //range in mm
    sensor_tof.readRangeStatus();
    kicked = true;
  }
  current_state = IN_AIR;
  next_state = IN_AIR;

  //Sensor Readings
  sensors_event_t orientationData, angVelocityData, accelerometerData;
  double raw_pitch_rad = 0, raw_speed = 0, prev_processed_pitch_rad, processed_pitch_rad, absolute_pitch_rad = 0;
  int num_rotations = 0;


  //Timing
  unsigned long current_time = micros();
  unsigned long initial_start_time = current_time;
  unsigned long air_start_time = current_time;  

  while ( (current_time < (initial_start_time + RUN_TIME_MS*1000)) && (current_state != LANDED)){ //only for a certain amount of time
    if (air_start_time != 0){ //if we have assigned a value
      if (current_time > air_start_time + (MAX_AIR_TIME_MS * 1000)){
        set_motor_speed(0);
        set_RGB(255, 0, 0);
        //Serial.println(air_start_time);
        //Serial.println(current_time);
        Serial.println("Hard Stop!");
        current_state == LANDED;
        break;
      }
    }
    
    //Timer application
    for(int timer_num = 0; timer_num < NUM_TIMERS; timer_num++){
      current_time = micros();
      Timing current_timer = control_timer[timer_num];
      unsigned long dt_us = current_time - current_timer.previous_start_time_us;  //force dt to be 0 for first pass, mostly for timing checking
      if( !(current_timer.running) || (dt_us >= current_timer.sample_period_us) ){ 
        
        if (timer_num == NDX_ORIENT){
          double dt_sec = (dt_us) / (double)(1000000);  
          
          //Motor Runnings
          if (current_state == IN_AIR){
            double angle_error = ref_angle - absolute_pitch_rad;
            double motor_input = B_E0 * angle_error + B_E1 * previous_error + B_U1 * previous_input;
            set_motor_speed(voltage_to_pwm(motor_input));

            previous_input = motor_input;
            previous_error = angle_error;
          }
          else if (current_state == LANDED){
            set_motor_speed(0);
          }
          else{

          }

        }
        else{

        }

        //Set flags, update times and indexes
        current_timer.running = true; //only matters on first go through, 
        current_timer.previous_start_time_us = current_time; //the time which this loop started, which next time around will be the previous lop

        //Check data is ok

        //Update timing
        
        current_timer.next_start_us = current_timer.previous_start_time_us + current_timer.sample_period_us;  //when the next loop should be running

        
        //Check timing ok
        if (control_timer[timer_num].running){  //if the parent timer event was running (this avoids checking the first loop through, since not pointer logic)
          if ( (dt_us - current_timer.sample_period_us) > TIMING_TOLERANCE_US){ //if we took longer than tolerated
            //Serial.print("E"); Serial.print(timer_num); Serial.print(": "); Serial.println(dt_us - current_timer.sample_period_us);
          }
        }

        

        //update things as we have time
        unsigned long time_left_us = control_timer[NDX_ORIENT].next_start_us - micros();  //get how much time until the next loop
        if (time_left_us >= (IMU_ROUNDTRIP_MS) * 1000){ //if we have time 
          sensor_imu.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
          sensor_imu.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
          raw_pitch_rad = orientationData.orientation.z * PI / 180.0;
          raw_speed = angVelocityData.gyro.x;
          processed_pitch_rad = (raw_pitch_rad < 0 ? raw_pitch_rad + PI : raw_pitch_rad - PI);
          if ((prev_processed_pitch_rad > 0) && (processed_pitch_rad < 0)){
            if (raw_speed < 0){
              num_rotations++;
            }
          }
          else if ((prev_processed_pitch_rad < 0) && (processed_pitch_rad > 0)){
            if (raw_speed > 0){
              num_rotations--;
            }
          }
          prev_processed_pitch_rad = processed_pitch_rad;
          absolute_pitch_rad = processed_pitch_rad + num_rotations*2.0*PI;
        }

        control_timer[timer_num] = current_timer; //update the pointed timer
      }
    }

    
    //State Outputs
    if (current_state == IDLE){
      set_RGB(255, 0, 0);
    }
    else if (current_state == ON_RAMP){
      set_RGB(0, 255, 0);
    }
    else if (current_state == IN_AIR){
      set_RGB(0, 0, 255);
    }
    else if (current_state == LANDED){
      set_RGB(255, 0, 255);
    }
    else{

    }
    //State Next
    if (current_state == IDLE){
      if (range < GROUND_THRESH){
        next_state = ON_RAMP;
        ramp_start_time = micros();
        //Serial.println("On Ramp!");
      }
      else{
        next_state = current_state;
      }
    }
    else if (current_state == ON_RAMP){
      if (range > GROUND_THRESH){
        air_start_time = micros();
        next_state = IN_AIR;
        //Serial.println("In Air!");
      }
      else{
        next_state = current_state;
      }
    }
    else if (current_state == IN_AIR){
      if (range < GROUND_THRESH){
        //if () [TODO] add orientation check here
        //next_state = LANDED;
        //Serial.println("Landed!");
      }
      else{
        next_state = current_state;
      }
    }
    else if (current_state == LANDED){

    }
    else{
      next_state = IDLE;
    }

    //State Update
    previous_state = current_state;
    current_state = next_state;
    current_time = micros();  //force a time update at end for while loop logic
    kicked = true;  //kick watchdog
  }
  //Serial.println("Done While!");
  set_motor_speed(0);
  if (current_state != LANDED){
    set_RGB(255, 255, 255);
  }
  else{
    set_RGB(0, 255, 0);
  }
  while(digitalRead(PIN_BUTTON) == HIGH){
   
  }
  set_RGB(0, 0, 0);
  init_tof();
  delay(2000);  //debounce
}


void print_timing(Timing demo){
  //Serial.print("Prev Time [us]: "); //Serial.println(demo.previous_start_time_us);
  //Serial.print("dt [us]: "); //Serial.println(demo.sample_period_us);
  //Serial.print("Running [TF]: "); //Serial.println(demo.running);
  //Serial.println();
}

void imu_ISR(){
  //Serial.println("IMU Flag");
}



void TC5_Handler() {
  //Clear interrupt flag
  TC_GetStatus(TC1, 2);

  if (!kicked) {
    //Serial.println("!");
    analogWrite(PIN_MOTOR_PWM, 0);  //force the motor off
  }
  kicked = false; // Reset the flag for next round
}

void init_watchdog(int hang_time_ms) {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC5);

  TC_Configure(TC1, 2,
    TC_CMR_TCCLKS_TIMER_CLOCK4 | // MCK/128
    TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);

   uint32_t rc = (uint32_t)(hang_time_ms * 656.25);
  TC_SetRC(TC1, 2, rc); 

  TC_Start(TC1, 2);
  TC1->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;
  TC1->TC_CHANNEL[2].TC_IDR = ~TC_IER_CPCS;

  kicked = true;
  NVIC_EnableIRQ(TC5_IRQn);
  kicked = true;
}

void disable_watchdog() {
  TC_Stop(TC1, 2);
  NVIC_DisableIRQ(TC5_IRQn);
  pmc_disable_periph_clk(ID_TC5);
}

void init_tof(){
  digitalWrite(PIN_TOF_RESET, LOW); //reset is active low
  delay(10);
  digitalWrite(PIN_TOF_RESET, HIGH);
  delay(50);
  if (!sensor_tof.begin()) {
    Serial.println("Failed to initialize VL53L0X!");
    while (1);
  }

}

