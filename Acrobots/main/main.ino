//import libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>

//define pin connections to H bridge to control the two motors
#define Enable1 5
#define Enable2 6
#define Input1 3
#define Input2 4
#define Input3 7
#define Input4 8

//Define Variables we'll be connecting to for the PID object
double Setpoint, Input, Output;

//set controller constants. We won't be using the integral controller
double Kp=135, Ki=0, Kd=300;

//initialize the PID object, following declaration in example
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//initialize the IMU object
Adafruit_BNO055 bno = Adafruit_BNO055(55);
 
void setup(void) 
{
  //set the motor pins to output
  pinMode(Enable1, OUTPUT);
  pinMode(Input1, OUTPUT);
  pinMode(Input2, OUTPUT);
  pinMode(Enable2, OUTPUT);
  pinMode(Input3, OUTPUT);
  pinMode(Input4, OUTPUT);

  /* Initialise the imu sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    while(1);
  }
  bno.setExtCrystalUse(true);

  //set the input variable to 0
  //set the desired setpoint to 3 degrees for slanted surface
  //setpoint would be 0 degrees for flat surface. 
  Input = 0;
  Setpoint = 3;

  //set mode of PID controller, start system
  myPID.SetMode(AUTOMATIC);
  //set the output range of the PID controller to the input range of the PWM analog write, with positive and negative
  myPID.SetOutputLimits(-255, 255);
  //set the computing time of the PID controller to be as short as possible for the highest possible reactivity
  myPID.SetSampleTime(1);
}

void loop() {

  //update the sensor readings
  sensors_event_t event; 
  bno.getEvent(&event);

  //set the input to the PID controller to be the most recent pitch reading
  Input = event.orientation.z;
  //calculate the new PID output values
  myPID.Compute();

  //set the motor directions according to the direction of the output value
  if (Output < 0) {
    reverse();
  } else {
    forward();
  }

  //int out is the actual PWM value output to the motors
  int out = abs(Output);
  //write PWM values to motor enable pins
  analogWrite(Enable1, out);
  analogWrite(Enable2, out);
}

//helper function to set the motors to forward
void reverse() {
  digitalWrite(Input1, HIGH);
  digitalWrite(Input2, LOW);
  digitalWrite(Input3, HIGH);
  digitalWrite(Input4, LOW);
}


//helpfer function to set the motors to reverse
void forward() {
  digitalWrite(Input1, LOW);
  digitalWrite(Input2, HIGH);
  digitalWrite(Input3, LOW);
  digitalWrite(Input4, HIGH);  
}
