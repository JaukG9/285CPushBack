#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"

void conveyorControl(int rpm);
void conveyorBrake();
void scraperChange();
void trapdoorChange();
void wingChange();
void doubleParkChange();
void odomLiftChange();
void tank();