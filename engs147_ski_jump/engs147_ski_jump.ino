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
#define MAX_AIR_TIME_MS 300
#define RAMP_TO_AIR 60
#define DT_ORIENT_MS 50
#define DT_MOTOR_MS 50
#define HANG_TIME_MS 500

#define TIMING_TOLERANCE_US 400  //how far off each timing loop can be from goal and still be 'ok'
#define IMU_ROUNDTRIP_MS 4  //stays under 4k us for orient and rad_sec
#define TOF_ROUNDTRIP_MS 3  //takes around 2.2ms to do the read, but .5 for function call

#define PWM_RAMP 130
#define PWM_RAMP_UP_TIME_MS 1050
#define PWM_AIR 250

//Timing Structure
#define NUM_TIMERS 2  //one timer for oreint, motor, imu, and tof
#define NDX_ORIENT 0
#define NDX_MOTOR 1


//Structure to define timings
struct Timing {
  bool running = false;
  unsigned long previous_start_time_us = 0;
  unsigned long sample_period_us = 0;
  unsigned long next_start_us = 0;  //track when we think the next loop should be run
};

Timing control_timer[NUM_TIMERS];

volatile bool new_imu_data = false;
volatile bool new_tof_data = false;
volatile bool button_pressed = false; 
volatile bool kicked = false;

unsigned long last_tof_reading = 0;

bool dbg_motor = false;
bool dbg_orient = false;

uint8_t range = 0, status = 0;

Adafruit_BNO055 sensor_imu = Adafruit_BNO055(55, BNO055_ADDRESS, &Wire);
Adafruit_VL6180X sensor_tof = Adafruit_VL6180X();


enum State {IDLE, ON_RAMP, IN_AIR, LANDED};
State current_state = IDLE, next_state = IDLE, previous_state = IDLE; //start idle

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Serial initialized!");
  //Setup pins
  init_RGB();
  pinMode(PIN_BUTTON, INPUT_PULLUP);  //so if button pressed, goes low
  pinMode(PIN_TOF_RESET, OUTPUT);
  pinMode(DBG_PIN_MOTOR, OUTPUT);
  pinMode(DBG_PIN_ORIENT, OUTPUT);
  pinMode(DBG_PIN_TOF, OUTPUT);
  pinMode(DBG_PIN_IMU, OUTPUT);

  //TOF
  digitalWrite(PIN_TOF_RESET, LOW); //reset is active low
  delay(10);
  digitalWrite(PIN_TOF_RESET, HIGH);
  delay(50);
  if(!sensor_tof.begin()) {
    Serial.println("TOF Failed!");
    set_RGB(0, 0, 255);
    while(1){}
  }
  sensor_tof.stopRangeContinuous();  //deal with it holding memory between power cycles
  delay(50);
  sensor_tof.startRange(); //ok to do this, itll just set a value ready in the buffer
  delay(50);

  //IMU
  if(!sensor_imu.begin()){
    set_RGB(255, 0, 0);
    Serial.println("IMU Failed!");
    while(1){}  //if no imu, wait
  }

  //Motor
  init_motor();
  
  //Encoder
  initEncoderShield();

  //Setup timers
  control_timer[NDX_ORIENT].sample_period_us = DT_ORIENT_MS * 1000;
  control_timer[NDX_MOTOR].sample_period_us = DT_MOTOR_MS * 1000;

  set_RGB(0, 255, 0); //show we are all good
  delay(2000);  //then wait a bit
  Serial.println("Ready");
  set_RGB(0, 0, 0); //befor going off to free up indicators
}

void loop() {
  //Waiting idle
  set_motor_speed(0); //make sure motor is at rest
  while(digitalRead(PIN_BUTTON) == HIGH){ //until button is pressed
    set_RGB(255, 0, 0);
  }
  delay(2000);
  init_watchdog(HANG_TIME_MS);
  Serial.println("Started");
  current_state = ON_RAMP;
  next_state = ON_RAMP;


  //Sensor Readings
  uint8_t range = 0; //range in mm
  sensors_event_t orientationData, angVelocityData, accelerometerData;
  double pitch_rad = 0, rot_rad_s = 0, accel_mag_ms2 = 0;


  //Timing
  unsigned long current_time = micros();
  unsigned long initial_start_time = micros();
  unsigned long air_start_time = 0;  //0 to flag
  unsigned long ramp_start_time = 0;

  Serial.println("Starting while!");
  while ( (current_time < (initial_start_time + RUN_TIME_MS*1000)) && (current_state != LANDED)){ //only for a certain amount of time
    if (air_start_time != 0){ //if we have assigned a value
      if (current_time > air_start_time + (MAX_AIR_TIME_MS * 1000)){
        set_motor_speed(0);
        set_RGB(255, 0, 0);
        Serial.println(air_start_time);
        Serial.println(current_time);
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
          
        }
        else if (timer_num == NDX_MOTOR){
          //Time sensitive items first
          double dt_sec = (dt_us) / (double)(1000000);  
          
          //Motor Runnings
          if (current_state == IDLE){
            set_motor_speed(0);
          }
          else if (current_state == ON_RAMP){
            int ramp_speed = PWM_RAMP * min(double((current_time - ramp_start_time)) / double(PWM_RAMP_UP_TIME_MS * 1000), 1);
            set_motor_speed(ramp_speed);
          }
          else if (current_state == IN_AIR){
            if (previous_state == IN_AIR){
              set_motor_speed(PWM_AIR);
            }
            else{
              set_motor_speed(0);
            }
            
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
        unsigned long time_left_us = min(control_timer[NDX_ORIENT].next_start_us, control_timer[NDX_MOTOR].next_start_us) - micros();  //get how much time until the next loop
        
        if (time_left_us >= (TOF_ROUNDTRIP_MS + IMU_ROUNDTRIP_MS + 1.5) * 1000){ //if we have time for both (+1.5 ms for switching devices)
          if (current_state == ON_RAMP){
            if (sensor_tof.isRangeComplete()){
              range = sensor_tof.readRange(); //range in mm
              sensor_tof.startRange();  //start next read
              //while(1){}  //simulate a hangup

            }
          }
          sensor_imu.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
          sensor_imu.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
          pitch_rad = orientationData.orientation.z * PI / 180.0; //could also do pitch roll or heading?
          rot_rad_s = angVelocityData.gyro.x;
        }
        else if (time_left_us >= (IMU_ROUNDTRIP_MS) * 1000){
          sensor_imu.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
          sensor_imu.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
          pitch_rad = orientationData.orientation.z * PI / 180.0; //could also do pitch roll or heading?
          rot_rad_s = angVelocityData.gyro.x;
          if (pitch_rad < 0){
            pitch_rad += 180.;
          }
          else{
            pitch_rad -= 180.0;
          }        
        }
        else if (time_left_us >= (TOF_ROUNDTRIP_MS) * 1000){
          if (current_state == ON_RAMP){
            if (sensor_tof.isRangeComplete()){
              range = sensor_tof.readRange(); //range in mm
              sensor_tof.startRange();  //start next read
              

            }
          }
        }
        else{
          //do nothing otherwise
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
        Serial.println("On Ramp!");
      }
      else{
        next_state = current_state;
      }
    }
    else if (current_state == ON_RAMP){
      if (range > GROUND_THRESH){
        air_start_time = micros();
        next_state = IN_AIR;
        Serial.println("In Air!");
      }
      else{
        next_state = current_state;
      }
    }
    else if (current_state == IN_AIR){
      if (range < GROUND_THRESH){
        //if () [TODO] add orientation check here
        next_state = LANDED;
        Serial.println("Landed!");
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
  Serial.println("Done While!");
  set_motor_speed(0);
  if (current_state != LANDED){
    set_RGB(255, 255, 255);
  }
  else{
    set_RGB(0, 255, 0);
  }
  disable_watchdog();
  while(digitalRead(PIN_BUTTON) == HIGH){
   
  }
  while(1){}
  set_RGB(0, 0, 0);
  delay(2000);  //debounce
}


void print_timing(Timing demo){
  Serial.print("Prev Time [us]: "); Serial.println(demo.previous_start_time_us);
  Serial.print("dt [us]: "); Serial.println(demo.sample_period_us);
  Serial.print("Running [TF]: "); Serial.println(demo.running);
  Serial.println();
}

void imu_ISR(){
  Serial.println("IMU Flag");
}



void TC5_Handler() {
  //Clear interrupt flag
  TC_GetStatus(TC1, 2);

  if (!kicked) {
    Serial.println("!");
    NVIC_SystemReset();
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



