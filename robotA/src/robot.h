#pragma once

#include <Arduino.h>
#include <Chassis.h>
#include "Lining.h"
#include <MQTT_comm.h>
#include "openmv.h"
#include "openMV_I2C.h"

enum ROBOT_STATE
{   
    ROBOT_IDLE, 
    ROBOT_LEFT,
    ROBOT_RIGHT,
    ROBOT_DRIVING,
    ROBOT_STANDOFF, 
    ROBOT_LINING,
    ROBOT_WALLING,
    ROBOT_CLIMBING,
    ROBOT_DESCENT,
    ROBOT_BRIDGING,
    ROBOT_CENTERING,
    ROBOT_TURNING,
    ROBOT_LOOKING,
    ROBOT_DEAD_TURN,
    ROBOT_APRIL_CENTERING,
};

const int closeValue = 19;
const int farValue = 24;

const int T_RIGHT = 1;
const int T_LEFT = -1;

void turnDeadreckon(int direction, double angle, double speed);

void handleAprilTagReading(OpenMV& camara, openMVISqC& mv, MQTT& mqtt);

void handleKeyCode(int16_t keyCode, MQTT& mqtt);

bool checkLineTimer(void);
void handleLineTimer(Lining& line, bool isBlackLine);

bool checkTurnTimer(void);
void handleTurnTimer(void);

bool checkIntersection(void);
void handleIntersection(void);

bool checkNavigationTimer(void);
void handleNavigationTimer(void);

void handlePitchUpdate(float pitchAngle);
void handleYawUpdate(float& yawAngle);

bool checkTurn(void);
void handleTurn(int16_t gyroReading);

void handleGetDistance(float& distance);

void handleMQTT(MQTT& mqtt);