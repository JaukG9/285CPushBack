#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"

void conveyorControl(int rpm);
void conveyorBrake();
void ptoChange();
void scraperChange();
void trapdoorChange();
void wingChange();
void intakeFunnelChange();
void doubleParkChange();
void odomLiftChange();
void tank();