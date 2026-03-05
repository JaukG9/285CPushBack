#pragma once

#include "main.h"
#include "lemlib/api.hpp"

extern pros::Controller controller;
extern pros::Motor conveyer1;
extern pros::Motor conveyer2;
extern pros::adi::DigitalOut scraper;
extern pros::adi::DigitalOut trapdoor;
extern pros::adi::DigitalOut wing;
extern pros::adi::DigitalOut pto;
extern pros::adi::DigitalOut intakeFunnel;
extern pros::adi::DigitalOut doublePark;
extern pros::adi::DigitalOut odomLift;

extern bool scraperActivated;
extern bool trapdoorActivated;
extern bool wingActivated;
extern bool ptoActivated;
extern bool intakeFunnelActivated;
extern bool doubleParkActivated;
extern bool odomLiftActivated;