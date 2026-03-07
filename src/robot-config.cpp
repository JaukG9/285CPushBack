#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* initialization of motors & pistons */
pros::Motor conveyer1(-1, pros::MotorGearset::blue);
pros::Motor conveyer2(10, pros::MotorGearset::blue);
pros::adi::DigitalOut scraper('H'); // extends scraper piston
pros::adi::DigitalOut trapdoor('G'); // extends midtake piston
pros::adi::DigitalOut wing('F'); // extends wing
pros::adi::DigitalOut pto('E'); // extends pto
pros::adi::DigitalOut intakeFunnel('D'); //extends intakeFunnel
pros::adi::DigitalOut doublePark('C'); // extends double park
pros::adi::DigitalOut odomLift('B'); // odom lift

bool scraperActivated = false;
bool trapdoorActivated = false;
bool wingActivated = false;
bool ptoActivated = false;
bool intakeFunnelActivated = false;
bool doubleParkActivated = false;
bool odomLiftActivated = false;