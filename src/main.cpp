#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"
#include "robot/drivetrain.h"
#include "robot/functions.h"
#include "robot/odometry.h"
#include "robot/autos.h"

void initialize(){
	pros::lcd::initialize();
	chassis.calibrate();

    scraper.set_value(false);
    trapdoor.set_value(false);
    wing.set_value(false);
    pto.set_value(false);
    doublePark.set_value(false);
    odomLift.set_value(false);
}

void disabled(){}

void autonomous(){
    
}

void opcontrol(){
    bool runningConveyor = false;
    int sCount = 0;
    int tCount = 0;
    int wCount = 0;
    int ptoCount = 0;
    int odomCount = 0;

    odomLift.set_value(true);

	while(true){
        tank();

        // R1 = wing
        // R2 = full conveyor
        // L1 = reverse conveyor
        // L2 = half conveyor
        // A = trapdoor
        // X = matchloader

        runningConveyor = false;
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            if(!ptoActivated){
                pto.set_value(true);
                ptoActivated = true;
            }
            conveyorControl(600);
            runningConveyor = true;
        }else{
            if(ptoActivated){
                pto.set_value(false);
                ptoActivated = false;
            }
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            conveyorControl(600);
            runningConveyor = true;
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            conveyorControl(-600);
            runningConveyor = true;
        }

        if(!runningConveyor){
            conveyorBrake();
        }

        /* pistons (scraper & midtake) */
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && tCount > 25){
            trapdoorChange();
            tCount = 0;
        }else if(tCount <= 25){tCount++;}
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) && sCount > 25){
            scraperChange();
            sCount = 0;
        }else if(sCount <= 25){sCount++;}
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && wCount > 25){
            wingChange();
            wCount = 0;
        }else if(wCount <= 25){wCount++;}

		pros::delay(20); // 20 ms downtime
	}
}