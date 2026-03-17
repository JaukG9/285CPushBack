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
void chainToPoint(float x, float y, int timeout, ChainToPointParams params = {}, bool async = true);