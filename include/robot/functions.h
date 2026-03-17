#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"

void conveyorControl(int rpm);
void scraperChange();
void trapdoorChange();
void wingChange();
void ptoChange();
void doubleParkChange();
void odomLiftChange();
void tank();
struct ChainToPointParams {
    /** whether the robot should move forwards or backwards. True by default */
    bool forwards = true;
    /** the maximum speed the robot can travel at. Value between 0-127. 127 by default */
    float maxSpeed = 127;
};
void chainToPoint(float x, float y, int timeout, ChainToPointParams params = {}, bool async = true);