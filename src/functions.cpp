#include "main.h"
#include "lemlib/api.hpp"
#include "drivetrain.h"
#include "robot-config.h"
#include "odometry.h"
#include "functions.h"

void conveyerControl(int rpm){
    conveyer1.move_velocity(rpm);
    conveyer2.move_velocity(rpm);
}

void scraperChange(bool scraperVal){
    scraper.set_value(!scraperVal);
}

void trapdoorChange(bool midtakeVal){
    trapdoor.set_value(!midtakeVal);
}

void wingChange(bool wingVal){
    wing.set_value(!wingVal);
}

void ptoChange(bool ptoVal){
    pto.set_value(!ptoVal);
}

void doubleParkChange(bool doubleParkVal){
    doublePark.set_value(!doubleParkVal);
}

void odomLiftChange(bool odomLiftVal){
    odomLift.set_value(!odomLiftVal);
}

void tank(){
    int axisL = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
    int axisR = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y); // Gets the turn left/right from right joystick
    
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        axisL = 127;
        axisR = 127;
    }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        axisL = -127;
        axisR = -127;
    }

    chassis.tank(axisL, axisR); // tank drive
}