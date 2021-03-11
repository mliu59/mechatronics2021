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

  stopMotors();
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
  Serial.begin(115200);
  pixy.init();
  delay(10000);
  smvar = FORWARD;
}

void loop() {
  switch (smvar) {
    case STOP:
      stopMotors();
      delay(1500);
      backUpAndReset();
      break;
    case LEFT:
      turnLeft();
      smvar = FORWARD;
      break;
    case RIGHT:
      turnRight();
      smvar = FORWARD;
      break;
    case UTURN:
      uTurn();
      smvar = FORWARD;
      break;
    case FORWARD:
      goForward();
      break;
  }
}

void backUpAndReset() {
  setMotorsReverse();
  delay(expectedTurnTime);
  stopMotors();
  goForward();
}

int checkBlocks() {
  pixy.ccc.getBlocks();
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_height < 140) { //(208 / 2)) {
      return pixy.ccc.blocks[i].m_signature; //green: right, blue: left, purple: 180          
    }
                           //blue: 1, green: 2, purple: 3;
  }
  return 0;
}

void stopMotors() {
  analogWrite(Enable1, 0);
  analogWrite(Enable2, 0);
  delay(350);
}

float readPING() {
  float a = 343.0 * measureDistance() / 1000000 / 2 * 100;
  delay(10);
  float b = 343.0 * measureDistance() / 1000000 / 2 * 100;
  Serial.println((a+b)/2);
  return (a+b)/2;
}

void goForward() {
  frontDistance = 200;
  setMotorsForward();
  unsigned long initT = millis();

  if (frontDistance < cameraRange) {
    int var = checkBlocks();
    if (smvar == STOP || smvar == FORWARD || var != STOP) {
      smvar = var;
    }
  }
  
  while (frontDistance > stopDistance) {
    
    frontDistance = readPING();
    int cond = tooClose();

    if (millis() > initT + (uturnTime / 4)) {
      rightTurned = false;
      leftTurned = false;
    }
    
    if (millis() > initT + uturnTime) {
      uturned = false;
    }
    
    if (cond == 1) { //left too close, adjust right
      adjustRight();
    } else if (cond == 2) {
      adjustLeft();
    }
    
        
    if (frontDistance < cameraRange) {
      int var = checkBlocks();
      if (smvar == STOP || smvar == FORWARD || var != STOP) {
        smvar = var;
      }
    }
    
  }
  stopMotors();
}

void turnLeft() {
  setMotorsLeft();
  delay(expectedTurnTime * 1.2);
  stopMotors();
  leftTurned = true;
}

void turnRight() {
  setMotorsRight();
  delay(expectedTurnTime * 1.2);
  stopMotors();
  rightTurned = true;
}

void uTurn() {
  setMotorsRight();
  delay(expectedTurnTime * 1.9);
  stopMotors();
  uturned = true;
}

void setMotorsForward() {
  analogWrite(Enable1, motorLevelRight);
  analogWrite(Enable2, motorLevelLeft);
  digitalWrite(Input1, HIGH);
  digitalWrite(Input2, LOW);
  digitalWrite(Input3, HIGH);
  digitalWrite(Input4, LOW);
}

void setMotorsReverse() {
  analogWrite(Enable1, motorLevelRight);
  analogWrite(Enable2, motorLevelLeft);
  digitalWrite(Input1, LOW);
  digitalWrite(Input2, HIGH);
  digitalWrite(Input3, LOW);
  digitalWrite(Input4, HIGH);
}

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

void adjustLeft() {
  stopMotors();
  Serial.println("Adjusting Left");
  setMotorsLeft();
  delay(adjustTime);
  stopMotors();
  setMotorsForward();
  delay(adjustTime * 0.9);
}

void adjustRight() {
  stopMotors();
  Serial.println("Adjusting Right");
  setMotorsRight();
  delay(adjustTime);
  stopMotors();
  setMotorsForward();
  delay(adjustTime * 0.9);
}

int tooClose() {

  float left = analogRead(IR1) * 0.0048828125; // value from sensor * (5/1024)
  float right = analogRead(IR2) * 0.0048828125;
  if (uturned || rightTurned) {
    //follow left wall
    if (convertIRtoDistance(left) < sideDistance) {
      return 1;
    } else if (convertIRtoDistance(right) < sideDistance) {
      return 2;
    } else if (convertIRtoDistance(left) > sideDistance * 1.9) {
      return 2;
    } else {
      return 0;
    }
  }

  if (leftTurned) {
    //follow right wall
    if (convertIRtoDistance(left) < sideDistance) {
      return 1;
    } else if (convertIRtoDistance(right) < sideDistance) {
      return 2;
    } else if (convertIRtoDistance(right) > sideDistance * 1.9) {
      return 1;
    } else {
      return 0;
    }
  }
  
  if (convertIRtoDistance(left) < sideDistance) {
    return 1;
  } else if (convertIRtoDistance(right) < sideDistance) {
    return 2;
  } else {
    return 0;
  }
}

float convertIRtoDistance(float volt) {
  return 2.3228 * volt * volt - 17.969 * volt + 38.839;
}

void setMotorsLeft() {
  analogWrite(Enable1, motorLevelRight);
  analogWrite(Enable2, motorLevelLeft);
  digitalWrite(Input1, HIGH);
  digitalWrite(Input2, LOW);
  digitalWrite(Input3, LOW);
  digitalWrite(Input4, HIGH);
}

void setMotorsRight() {
  analogWrite(Enable1, motorLevelRight);
  analogWrite(Enable2, motorLevelLeft);
  digitalWrite(Input1, LOW);
  digitalWrite(Input2, HIGH);
  digitalWrite(Input3, HIGH);
  digitalWrite(Input4, LOW);
}
