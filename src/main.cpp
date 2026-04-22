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

    pros::Task selector(autonSelector);
}

void disabled(){}

void autonomous(){
    pros::screen::erase();

    switch(autonomousSelection){
        case 0: skip(); break;
        case 1: left_4Rush(); break;
        case 2: right_4Rush(); break;
        case 3: left_7Rush(); break;
        case 4: right_7Rush(); break;
        case 5: left_43Split(); break;
        case 6: right_43Split(); break;
        case 7: left_awp(); break;
        case 8: right_awp(); break;
        case 9: skills(); break;
        default: break;
    }

    //pros::lcd::print(0, "%f %f %f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
}

void opcontrol(){
    scraper.set_value(false);
    trapdoor.set_value(false);
    pto.set_value(false);
    doublePark.set_value(false);
    odomLift.set_value(true);
    wing.set_value(true);
    wingActivated = true;

    // boolean variable to 
    bool runningConveyor = false;

    // integer variables to add buffers between button presses for piston changes
    int sCount = 0;
    int tCount = 0;
    int wCount = 0;
    int ptoCount = 0;
    int odomCount = 0;

    // extends the odomLift piston
    odomLift.set_value(true);

	while(true){
        pros::lcd::print(0, "%f", conveyor.get_actual_velocity());

        tank();

        runningConveyor = false;
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            if(trapdoorActivated){
                conveyorControl(300);
            }else{
                if(!ptoActivated){ptoChange();}
                conveyorControl(600);
            }
            runningConveyor = true;
        }else if(ptoActivated){
            ptoChange();
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            conveyorControl(600);
            runningConveyor = true;
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            if(!intakeFunnelActivated){
                intakeFunnelChange();
            }
            conveyorControl(-600);
            runningConveyor = true;
        }else if(intakeFunnelActivated){
            intakeFunnelChange();
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