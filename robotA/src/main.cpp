#include <Arduino.h>
#include <robot.h>
#include <LSM6.h>
#include "Lining.h"
#include <MQTT_comm.h>
#include <openmv.h>
#include <openMV_I2C.h>


#include <ir_codes.h>
#include <IRdecoder.h>
#include <Sharp-IR.h>

#define IR_PIN 14
IRDecoder decoder(IR_PIN);
LSM6 imu;
Lining line;

OpenMV camara;
openMVISqC mv;

MQTT mqtt;

SharpIR ir(A0);

float yawAngle = 0;

void setup() 
{
    Serial.begin(115200);
    delay(500);
    pinMode(30, OUTPUT);
    Serial.println("setup()");
    imu.init();
    imu.calibrate();
    ir.init();
    mqtt.init();
    chassis.init();
    decoder.init();
    Serial.println("/setup()");
    line.calibrateLineSensor();
    mv.init();
}


void loop() 
{

    if(chassis.loop()){}

    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) handleKeyCode(keyCode, mqtt);

    //if(mqtt.checkMQTT()) handleMQTT(mqtt);

    if(line.checkIntersection()) handleIntersection();

    if(checkLineTimer()) handleLineTimer(line, false);

    float distance;
    if(ir.getDistance(distance)) handleGetDistance(distance);

    if(checkNavigationTimer()) handleNavigationTimer();

    if(mv.readytoRead()) handleAprilTagReading(camara, mv, mqtt);

    if(checkTurnTimer()) handleTurnTimer();
    
    float pitchAngle = 0;
    float xAcc = 0;
    if(imu.checkForIMUUpdate(pitchAngle, yawAngle, xAcc)){
        handlePitchUpdate(pitchAngle);
    }
}