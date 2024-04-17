#include <robot.h>
#include <Chassis.h>
#include <ir_codes.h>
#include <event_timer.h>
#include <MQTT_comm.h>

int lastError = 0;

bool onRamp = false;
bool isClose = false;

int x = 0;
int y = 0;
int heading = 0;

int xDestination = 0;
int yDestination = 0;
int goalHeading = 0;

ROBOT_STATE robotState = ROBOT_IDLE;
Chassis chassis;

unsigned long TIME_TO_CENTER = 500;
EventTimer navigationTimer(TIME_TO_CENTER);

EventTimer lineTimer(5);

const double TURN_SPEED = 5; //CM/SEC

int timeToTurn(double speed, double angle){
  return int((angle*PI*7.0*1020.0/180.0)/speed);
}

EventTimer turnTimer(timeToTurn(TURN_SPEED, 90));
bool checkTurnTimer(void) {
  return turnTimer.checkExpired();
}

bool checkLineTimer(void)
{
  return lineTimer.checkExpired();
}

void idle(void)
{
    Serial.println("Idling!");
    chassis.setMotorEfforts(0, 0);
    robotState = ROBOT_IDLE;
}

String keyCodeString;
void handleKeyCode(int16_t keyCode, MQTT& mqtt)
{ 
    Serial.println(keyCode);

  if(keyCode == ENTER_SAVE) idle();

  switch(keyCode)
  {
    case UP_ARROW:
      chassis.setMotorTargetSpeeds(10, 10);
      robotState = ROBOT_DRIVING;
      break;
    case RIGHT_ARROW:
      chassis.setMotorTargetSpeeds(10, -10);
      robotState = ROBOT_RIGHT;
      break;
    case DOWN_ARROW:
      chassis.setMotorTargetSpeeds(-10, -10);
      robotState = ROBOT_DRIVING;
      break;
    case LEFT_ARROW:
      chassis.setMotorTargetSpeeds(-10, 10);
      robotState = ROBOT_LEFT;
      break;
    case NUM_1:
      robotState = ROBOT_LINING;
      Serial.println("Number 1 Pressed");
      mqtt.sendMessage("team19", "Advanced Intelligence Sequence Initiated");
      break;
    case NUM_2:
      robotState = ROBOT_TURNING;
      chassis.setWheelTargetSpeeds(5, -5);
      break;
    default:
      break;
  }
}

//change to robot looking
void handleAprilTagReading(OpenMV& camera, openMVISqC& mv, MQTT& mqtt){
  if(robotState == ROBOT_LOOKING){
    int id = -1;
    int rot = -1;
    mv.FindAprilTags(camera, nullptr, nullptr, nullptr, nullptr, &rot, &id);
    if(id != -1 && rot != -1){
      mqtt.sendMessage("team19", "ID:" + String(id) + ", ROT:" + String(rot));
      idle();
    }
  }
}


void handlePitchUpdate(float pitchAngle)
{
#ifdef __DEBUG_IMU__
    Serial.println(pitchAngle);
#endif
#ifdef __DEBUG_STATE__
    Serial.println(robotState);
#endif
    if(robotState == ROBOT_CLIMBING)
    {
      if(pitchAngle > -1.5){
        robotState = ROBOT_APRIL_CENTERING;
        navigationTimer.start(1500);
        digitalWrite(30, 1);
        onRamp = false;
      }
    }
    else if(robotState == ROBOT_LINING)
    {
      if(pitchAngle < -7.0){
        robotState = ROBOT_CLIMBING;
        digitalWrite(30, 0);
        onRamp = true;
      }
    }
    else if(robotState == ROBOT_BRIDGING)
    {
      chassis.setMotorTargetSpeeds(15, 16);
      if(pitchAngle > 7.0){
        robotState = ROBOT_DESCENT;
        chassis.setMotorTargetSpeeds(0, 0);
        digitalWrite(30, 0);
        onRamp = true;
      }
    }
    else if(robotState == ROBOT_DESCENT)
    {
      if(pitchAngle < 3.5){
        idle();
        robotState = ROBOT_IDLE;
        chassis.setMotorEfforts(0, 0);
        digitalWrite(30, 1);
        onRamp = false;
      }
    }
}

void handleLineTimer(Lining& line, bool isBlackLine)
{
  if(robotState == ROBOT_LINING || robotState == ROBOT_CLIMBING || robotState == ROBOT_DESCENT || robotState == ROBOT_CENTERING)
  {
    line.lineFollow(isBlackLine);
  }
  lineTimer.restart();
}


void handleIntersection(void)
{  
  if(robotState == ROBOT_LINING){
    robotState = ROBOT_CENTERING;
    Serial.println("LINING -> CENTERING");
    navigationTimer.start(TIME_TO_CENTER);
  }
}

bool checkNavigationTimer(void){
  return navigationTimer.checkExpired();
}

void handleNavigationTimer(void){
  if(robotState == ROBOT_CENTERING){
    if(isClose){
      turnDeadreckon(T_LEFT, 90.0, TURN_SPEED);
      Serial.println("Centering -> Turning");
    }
    else{
      robotState = ROBOT_LINING;
      Serial.println("Centering -> Lining");
      navigationTimer.restart();
    }
  }
  if(robotState == ROBOT_APRIL_CENTERING){
    robotState = ROBOT_LOOKING;
    chassis.setMotorEfforts(0, 0);
  }
}

void turnDeadreckon(int direction, double angle, double speed){
  robotState = ROBOT_DEAD_TURN;
  chassis.setWheelTargetSpeeds(speed*direction, -speed*direction);
  turnTimer.start(timeToTurn(speed, angle));
  navigationTimer.restart();
}

void handleTurnTimer(void) {
  if(robotState == ROBOT_DEAD_TURN){
    robotState = ROBOT_LINING;
    turnTimer.restart();
  }
  else {
    turnTimer.restart();
  }
}

void handleGetDistance(float& distance){
  if((distance > farValue) && isClose){
    isClose = false;
  }
  else if((distance < closeValue) && !isClose){
    isClose = true;
  }
}

void handleMQTT(MQTT& mqtt){
  mqtt.returnCoordinates(xDestination, yDestination);
}