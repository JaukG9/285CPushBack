#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "robot-config.h"

void conveyerControl(int rpm);
void scraperChange(bool scraperVal);
void trapdoorChange(bool midtakeVal);
void wingChange(bool wingVal);
void ptoChange(bool ptoVal);
void doubleParkChange(bool doubleParkVal);
void odomLiftChange(bool odomLiftVal);
void tank();