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
	int axisL = 0;
	int axisR = 0;
    bool scraperExtended = false;
    bool midtakeExtended = false;
    bool rabbitExtended = false;
    bool odomExtended = false;
    int sCount = 0;
    int mCount = 0;
    int rCount = 0;
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

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            conveyorControl(600);
        }else{conveyorBrake();}

        /* pistons (scraper & midtake) 
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && mCount > 25){
            midtakeExtended = !midtakeExtended;
            trapdoor.set_value(midtakeExtended);
            mCount = 0;
        }else if(mCount <= 25){mCount++;}
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && sCount > 25){
            scraperExtended = !scraperExtended;
            scraper.set_value(scraperExtended);
            sCount = 0;
        }else if(sCount <= 25){sCount++;}
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) && rCount > 25){
            rabbitExtended = !rabbitExtended;
            wing.set_value(rabbitExtended);
            rCount = 0;
        }else if(rCount <= 25){rCount++;} */

		pros::delay(20); // 20 ms downtime
	}
}