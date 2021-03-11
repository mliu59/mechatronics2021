//Include the Pixy library
#include <Pixy2.h>

//define motor pins
#define Enable1 3 //right motor enable and input pins
#define Input1 6
#define Input2 7

#define Enable2 5 //left motor enable and input pins
#define Input3 8
#define Input4 9

//define sensor pins
#define IR1 A4 //analog read pin for left IR sensor
#define IR2 A5 //analog read pin for right IR sensor
#define Ping 4 //digital pin for PING sensor

//define state variables for 5 distinct states
#define STOP 0 //stop: robot is within a stopdistance of a wall in front, but cannot find a color code
#define LEFT 1 //left: turning left
#define RIGHT 2 //right: turning right
#define UTURN 3 //uturn: making a uturn
#define FORWARD 4 //forward: moving forward, checking clearance with side and front walls, while checking for color tags

//define motor PWM levels (not a percentage, but out of 255 for use with analogWrite)
int motorLevelRight = 52; //equivalent of 20.7% motor power (52 + 1) / 256
int motorLevelLeft = 65;  //equivalent of 25.8% motor power (65 + 1) / 256

//set of boolean variables that help track whether or not a robot just finished a turn, 
//helps set strategy for following walls for a short period of time after turns
//see tooClose() and goForward() function
boolean uturned = false;
boolean leftTurned = false;
boolean rightTurned = false;

//robot distance variables, defines parameters of robot's navigation
#define stopDistance 8 //cm, space to leave in front of the robot
#define sideDistance 5.5 //cm, space to leave between the side of the robot and walls
#define adjustTime 150 //time that the motors are turned on for the adjustment maneuver
#define uturnTime 3500 //sets the amount of time that the robot should follow walls after a turning operation
                       //see tooClose() and goForward() function
#define cameraRange 30 //the upper range of camera's range when looking for a colored tag
#define expectedTurnTime 570 //a general estimate of how much time a turning operation uses
                             //does not need to be accurate, the adjustment maneuvers will correct for all cases

uint8_t smvar = STOP; //state variable
float frontDistance; //float value to hold the measurements of the front facing PING

Pixy2 pixy; //init pixy object

void setup() {

  //setup motor pins, default going forward.
  pinMode(Enable1, OUTPUT);
  pinMode(Enable2, OUTPUT);
  pinMode(Input1, OUTPUT);
  pinMode(Input2, OUTPUT);
  pinMode(Input3, OUTPUT);
  pinMode(Input4, OUTPUT);
  digitalWrite(Input1, HIGH);
  digitalWrite(Input2, LOW);
  digitalWrite(Input3, HIGH);
  digitalWrite(Input4, LOW);
  //initialize serial, pixy, and state variable after pausing for 10 seconds.
  Serial.begin(115200);
  pixy.init();
  stopMotors();
  delay(10000);
  smvar = FORWARD;
}

void loop() {
  //at each loop iteration, read the state variable and execute the correpsonding navigation scheme
  switch (smvar) {
    case STOP: //When the robot detects a wall in front of it but was not able to read a turn signal prior. 
               //This prompts the robot to reverse and try again.
      stopMotors();
      delay(1500);
      backUpAndReset();
      break;
    case LEFT: //Perform a timed left turn. 
      turnLeft();
      smvar = FORWARD;
      break;
    case RIGHT: //Perform a timed right turn. 
      turnRight();
      smvar = FORWARD;
      break;
    case UTURN: //Perform a timed U-turn. 
      uTurn();
      smvar = FORWARD;
      break;
    case FORWARD: //Default state of the robot, will go forward until it reaches a wall in front. 
                  //During the straight-line motion, it will also check for colored instructions as well as side wall distances for adjustments. 
      goForward();
      break;
  }
}

void backUpAndReset() {
  //function for reversing and resetting the robot
  //keep the motors in reverse state for approx the amount of time for a right/left turn, then return to default state: forward, and try again.
  setMotorsReverse();
  delay(expectedTurnTime);
  stopMotors();
  smvar = FORWARD;
}

int checkBlocks() {
  //check the blocks identified by the pixy, return the color/action detected
  pixy.ccc.getBlocks();
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_height < 140) { //(208 / 2)) filter out all blocks in the top half of the camrea's vision
      return pixy.ccc.blocks[i].m_signature; //green: right: return 1, red: left: return 2, purple: 180: return 3          
    }
  }
  return 0; //if no blocks are detected, return 0. 
}

void stopMotors() {
  //stop and pause motors
  analogWrite(Enable1, 0);
  analogWrite(Enable2, 0);
  delay(350);
}

float readPING() {
  //read the front distance of the robot (PING)
  //take 2 measurments and return the average (distance in cm)
  float a = 343.0 * measureDistance() / 1000000 / 2 * 100;
  delay(10);
  float b = 343.0 * measureDistance() / 1000000 / 2 * 100;
  //Serial.println((a+b)/2);
  return (a+b)/2;
}

void goForward() {
  
  //reset the variable for frent
  frontDistance = 200;
  //start moving the robot forward
  setMotorsForward();

  //initiate a timer for the wall hugging strategy, to track the time since the turn was complete
  unsigned long initT = millis();

  //check before moving if there is a color note within view
  //this is used mainly during reverse and reset actions
  if (frontDistance < cameraRange) {
    int var = checkBlocks();
    if (smvar == STOP || smvar == FORWARD || var != STOP) {
      smvar = var;
    }
  }

  //check if the fornt distance is within a threshold. If not, continue forward
  //if within threshold, stop and terminate the forward function.
  while (frontDistance > stopDistance) {


    //read the front distance
    frontDistance = readPING();
    //check the IR sensors, and determine if there needs to be an adjustment
    int cond = tooClose();

    //check the timer. For uturns, hug the left wall until the timer runs out. 
    //for rigtht and left turns, hug the wall for 1/3 of the timer for uturns.
    //when the timer runs out, the boolean variables will be flipped and the tooClose function will evaluate distances differently.
    if (millis() > initT + (uturnTime / 3)) {
      rightTurned = false;
      leftTurned = false;
    }
    
    if (millis() > initT + uturnTime) {
      uturned = false;
    }

    //check what status the tooClose function reports, and execute adjustment if needed.
    if (cond == 1) { //left too close, adjust right
      adjustRight();
    } else if (cond == 2) {
      adjustLeft();
    }
    
    //while the front distance is within a certain range for the camera's operation, read the first block detected, and change the state variable so that the
    //robot executes that action after the forward function is over.
    if (frontDistance < cameraRange) {
      int var = checkBlocks();
      if (smvar == STOP || smvar == FORWARD || var != STOP) {
        smvar = var;
      }
    }
    
  }
  //after the robot reaches the front wall, pause before moving on.
  stopMotors();
}

//set motor direction to left and perform a timed turn.
//During experimentation, I realized that the robot performs better with overturning than underturning
//thus the 1.2 factor
void turnLeft() {
  setMotorsLeft();
  delay(expectedTurnTime * 1.2);
  stopMotors();
  leftTurned = true;
}
//set motor direction to right and perform a timed turn.
void turnRight() {
  setMotorsRight();
  delay(expectedTurnTime * 1.2);
  stopMotors();
  rightTurned = true;
}
//set motor direction to right and perform a timed turn.
//by excperiemntation, uturn needs approx 1.9 times the default turning time
void uTurn() {
  setMotorsRight();
  delay(expectedTurnTime * 1.9);
  stopMotors();
  uturned = true;
}

//function for setting the motor direction to forward
void setMotorsForward() {
  analogWrite(Enable1, motorLevelRight);
  analogWrite(Enable2, motorLevelLeft);
  digitalWrite(Input1, HIGH);
  digitalWrite(Input2, LOW);
  digitalWrite(Input3, HIGH);
  digitalWrite(Input4, LOW);
}

//function for setting the motor direction to reverse
void setMotorsReverse() {
  analogWrite(Enable1, motorLevelRight);
  analogWrite(Enable2, motorLevelLeft);
  digitalWrite(Input1, LOW);
  digitalWrite(Input2, HIGH);
  digitalWrite(Input3, LOW);
  digitalWrite(Input4, HIGH);
}

//take the pulse length and return the measured distance
unsigned long measureDistance()
{
  // set pin as output so we can send a pulse
  pinMode(Ping, OUTPUT);
  // set output to LOW
  digitalWrite(Ping, LOW);
  delayMicroseconds(5);

  // now send the 5uS pulse out to activate Ping)))
  digitalWrite(Ping, HIGH);
  delayMicroseconds(5);
  digitalWrite(Ping, LOW);

  // now we need to change the digital pin
  // to input to read the incoming pulse
  pinMode(Ping, INPUT);

  // finally, measure the length of the incoming pulse
  return pulseIn(Ping, HIGH);
}

//short burst of left turn to adjust to the left
//move straight for a short burst after
void adjustLeft() {
  stopMotors();
  //Serial.println("Adjusting Left");
  setMotorsLeft();
  delay(adjustTime);
  stopMotors();
  setMotorsForward();
  delay(adjustTime * 0.9);
}

//short burst of right turn to adjust to the left
//move straight for a short burst after
void adjustRight() {
  stopMotors();
  //Serial.println("Adjusting Right");
  setMotorsRight();
  delay(adjustTime);
  stopMotors();
  setMotorsForward();
  delay(adjustTime * 0.9);
}


//function to return to the goForward function whether or not an adjustment is needed
int tooClose() {
  //read the IR sensors, get voltages for both sides
  float left = analogRead(IR1) * 0.0048828125; // value from sensor * (5/1024)
  float right = analogRead(IR2) * 0.0048828125;

  //below: return 1 is right adjustment
  //       return 2 is left adjustment
  //       return 0 no adjustment needed


  //regardless of whether the robot just finished a turn, if too close to the side walls, make adjustment
  if (convertIRtoDistance(left) < sideDistance) {
    return 1;
  } else if (convertIRtoDistance(right) < sideDistance) {
    return 2;
  }

  //function will reach here if the side distances are not too close
  //check if the robot just finished a right turn or uturn
  //follow left wall    
  //this is done by checking for the distance to the left wall, if the distance is too high, adjustment to the left
  if ((uturned || rightTurned) && convertIRtoDistance(left) > sideDistance * 1.9) {
    return 2;
  }

  //check if the robot just finished a left turn
  //follow right wall    
  //this is done by checking for the distance to the right wall, if the distance is too high, adjustment to the right
  if (leftTurned && convertIRtoDistance(right) > sideDistance * 1.9) {
    return 1;
  }

  //if no conditions are met, that means that no adjustment is needed
  return 0;
}

//convert the IR voltage to a distance value, using the 2nd order polynomial fit generated from calibration
float convertIRtoDistance(float volt) {
  return 2.3228 * volt * volt - 17.969 * volt + 38.839;
}

//function for setting the motor direction to left turn/adjustment
void setMotorsLeft() {
  analogWrite(Enable1, motorLevelRight);
  analogWrite(Enable2, motorLevelLeft);
  digitalWrite(Input1, HIGH);
  digitalWrite(Input2, LOW);
  digitalWrite(Input3, LOW);
  digitalWrite(Input4, HIGH);
}
//function for setting the motor direction to right turn/adjustment
void setMotorsRight() {
  analogWrite(Enable1, motorLevelRight);
  analogWrite(Enable2, motorLevelLeft);
  digitalWrite(Input1, LOW);
  digitalWrite(Input2, HIGH);
  digitalWrite(Input3, HIGH);
  digitalWrite(Input4, LOW);
}
