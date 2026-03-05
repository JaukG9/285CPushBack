#include "main.h"
#include "lemlib/api.hpp"
#include "robot-config.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* initialization of motors & pistons */
pros::Motor conveyer1(-1, pros::MotorGearset::blue);
pros::Motor conveyer2(10, pros::MotorGearset::blue);
pros::ADIDigitalOut scraper('H'); // extends scraper piston
pros::ADIDigitalOut trapdoor('G'); // extends midtake piston
pros::ADIDigitalOut wing('F'); // extends wing
pros::ADIDigitalOut pto('E'); // extends pto
pros::ADIDigitalOut intakeFunnel('D'); //extends intakeFunnel
pros::ADIDigitalOut doublePark('C'); // extends double park
pros::ADIDigitalOut odomLift('B'); // odom lift

bool scraperActivated = false;
bool trapdoorActivated = false;
bool wingActivated = false;
bool ptoActivated = false;
bool intakeFunnelActivated = false;
bool doubleParkActivated = false;
bool odomLiftActivated = false;