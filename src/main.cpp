#include "main.h"
#include "lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* initialization of motors & pistons */
pros::Motor intake(-1, pros::MotorGearset::blue); // spins intake clockwise
pros::Motor extakeT(20); // spins extake counterclockwise
pros::MotorGroup left_mg({-3, -4, -5}, pros::MotorGearset::blue); // left motor group (clockwise --> omni = counterclockwise)
pros::MotorGroup right_mg({8, 9, 10}, pros::MotorGearset::blue); // right motor group (counterclockwise --> omni = clockwise)
pros::ADIDigitalOut scraper('H'); // extends scraper piston
pros::ADIDigitalOut midtake('G'); // extends midtake piston*/

/* creation of drivetrain */
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.22, // 11.22 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450 (600 * (36 / 48))
                              2 // horizontal drift is 2 (for now)
);

/* odometry */
pros::Imu imu(11); // imu

/*
pros::Rotation horizontal_encoder(20); // horizontal tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D', true); // vertical tracking wheel encoder
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75); // horizontal tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5); // vertical tracking wheel
*/

// odometry settings
lemlib::OdomSensors sensors(
    nullptr,  // vertical tracking wheel 1
    nullptr,  // vertical tracking wheel 2
    nullptr,  // horizontal tracking wheel 1
    nullptr,  // horizontal tracking wheel 2
    &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve steer_curve(10, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &steer_curve,
                        &steer_curve
);


void on_center_button(){}


void initialize(){
	pros::lcd::initialize();
	chassis.calibrate();

    midtake.set_value(true);
}

void disabled(){}

void autonomous(){
    /* revised right
    chassis.setPose(-48.32, -16.57, 102);
    chassis.moveToPose(-14.86, -24.39, 102, 2000);
    pros::delay(600);
    intake.move_velocity(600);
    pros::delay(500);
    chassis.moveToPose(-5, -5, 45, 1200);
    intake.move_velocity(-600);
    pros::delay(1800);
    chassis.moveToPose(-36, -42, 0, 2000, {.forwards = false});
    chassis.turnToHeading(270, 800);
    intake.move_velocity(600);
    scraper.set_value(true);
    chassis.moveToPose(-64, -42, 270, 1000);
    pros::delay(2000);
    chassis.moveToPose(-5.55, -42, 270, 1000, {.forwards = false});
    pros::delay(800);
    extakeT.move_velocity(600); */

    /* left auton
    chassis.setPose(-48.32, 16.57, 78);
    chassis.moveToPose(-14.86, 24.39, 78, 2000);
    pros::delay(600);
    intake.move_velocity(600);
    pros::delay(800);
    chassis.moveToPose(-60.61, 47.33, 270, 2000);
    pros::delay(200);
    chassis.moveToPose(-5.55, 47.33, 270, 3000, {.forwards = false});
    pros::delay(3000);
    extakeT.move_velocity(600); */
    
    /* right auton
    chassis.setPose(-48.32, -16.57, 102);
    chassis.moveToPose(-14.86, -24.39, 102, 2000);
    pros::delay(200);
    intake.move_velocity(600);
    chassis.moveToPose(-31, -31, 45, 2000, {.forwards = false});
    pros::delay(300);
    intake.brake();
    chassis.moveToPose(-5, -5, 45, 1500);
    pros::delay(800);
    intake.move_velocity(-600);
    pros::delay(1000);
    intake.brake(); */
    

    /* skills auton - left start  -------------  maximum total time to take */
        //** left alliance side /
    chassis.setPose(-48, 16, 0);
    scraper.set_value(true);
    chassis.moveToPose(-64, 47, 270, 2000);                         // 2 sec
    intake.move_velocity(600);
    pros::delay(4000);                                              // 6 sec
    scraper.set_value(false);
    chassis.moveToPoint(-42, 47, 270, {.forwards = false});         // 7 sec
    chassis.turnToHeading(45, 600);                                 // 7.6 sec
    chassis.moveToPoint(-28.497, 59.905, 90);                       
    chassis.moveToPoint(45, 59.905, 90);                            // 10.4 sec
    chassis.turnToHeading(45, 600);                                 // 11 sec
    chassis.moveToPose(30, 47, 90, 1000, {.forwards = false});      // 12 sec
    pros::delay(600);                                               // 12.6 sec
    extakeT.move_velocity(600);
    pros::delay(2000);                                              // 14.6 sec
    extakeT.brake();
        //** left opponent side /
    scraper.set_value(true);
    chassis.moveToPose(64, 47, 90, 1000);                           // 15.6 sec
    pros::delay(4000);                                              // 19.6 sec
    scraper.set_value(false);
    chassis.moveToPose(30, 47, 90, 1000, {.forwards = false});      // 20.6 sec
    pros::delay(600);                                               // 21.2 sec
    extakeT.move_velocity(600);
    pros::delay(2000);                                              // 23.2 sec
    intake.brake();
    extakeT.brake();
        //** right opponent side /
    chassis.moveToPoint(40.624, -16.735, 180);                      // 26 sec
    scraper.set_value(true);
    chassis.moveToPose(64, -47, 90, 1000);                          // 27 sec
    intake.move_velocity(600);
    pros::delay(4000);                                              // 31 sec
    scraper.set_value(false);
    chassis.moveToPose(42, -47, 90, 1000, {.forwards = false});     // 32 sec
    chassis.turnToHeading(215, 600);                                // 32.6 sec
    chassis.moveToPoint(28.497, -59.905, 270);
    chassis.moveToPoint(-45, -59.905, 270);                         // 35.4 sec
    chassis.turnToHeading(215, 600);                                // 36 sec
    chassis.moveToPose(-30, -47, 270, 1000);                        // 37 sec
    pros::delay(600);                                               // 37.6 sec
    extakeT.move_velocity(600);
    pros::delay(2000);                                              // 39.6 sec
    extakeT.brake();
        //** right alliance side /
    scraper.set_value(true);
    chassis.moveToPose(-64, -47, 270, 1000);                        // 40.6 sec
    pros::delay(4000);                                              // 44.6 sec
    scraper.set_value(false);
    chassis.moveToPose(-30, -47, 270, 1000, {.forwards = false});   // 45.6 sec
    pros::delay(600);                                               // 46.2 sec
    extakeT.move_velocity(600);
    pros::delay(2000);                                              // 48.2 sec
    intake.brake();
    extakeT.brake();
        //** center goals /
    chassis.moveToPoint(-47, -47, 270);                             // 50.25 sec
    chassis.turnToHeading(45, 750);                                 // 51 sec
    intake.move_velocity(600);
    chassis.moveToPose(-13, -13, 45, 2000);                         // 53 sec
    pros::delay(600);                                               // 53.6 sec
    intake.move_velocity(-600);
    pros::delay(2400);                                              // 56 sec
    intake.brake();
        //** clear and park /
    chassis.moveToPose(-14.5, -24, 340, 800, {.forwards = false});  // 56.8 sec
    chassis.moveToPoint(-45, 0, 270);                               // 58 sec
    scraper.set_value(true);
    chassis.moveToPose(-62, 0, 270, 10000);                         // until end
    intake.move_velocity(600);
    extakeT.move_velocity(600);                                     // end
        //** ideally 79 points /
    // */


    /* skip auton
    chassis.setPose(0, -15, 0);
    intake.move_velocity(-600);
    chassis.moveToPoint(0, 15, 10000);
    chassis.moveToPoint(0, 0, 10000, {.forwards = false}); */

    /* 
    
    */
}

void intakeConveyer(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intake.move_velocity(600); // intake motors forward
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intake.move_velocity(-600); // low extake
    else intake.brake();
}
void extake(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        extakeT.move_velocity(600);
        intake.move_velocity(600);
    }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) extakeT.move_velocity(-600);
    else{extakeT.brake();}
}

void opcontrol(){
	int axisL = 0;
	int axisR = 0;
    bool scraperExtended = false;
    bool midtakeExtended = false;
    int sCount = 0;
    int mCount = 0;

	while (true) {
		pros::lcd::print(0, "%d %d %d", 0.3, axisL, axisR);
		
		/* drive */
		axisL = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
		axisR = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y); // Gets the turn left/right from right joystick
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            left_mg.move_velocity(450);
            right_mg.move_velocity(450);
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            left_mg.move_velocity(-450);
            right_mg.move_velocity(-450);
        }else{
            chassis.tank(axisL, axisR); // arcade drive
        }

        /* intake + conveyer */
        intakeConveyer();

        /* extake (top & mid) */
        extake();

        /* pistons (scraper & midtake) */
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && mCount > 25){
            midtakeExtended = !midtakeExtended;
            midtake.set_value(midtakeExtended);
            mCount = 0;
        }else if(mCount <= 25){mCount++;}
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && sCount > 25){
            scraperExtended = !scraperExtended;
            scraper.set_value(scraperExtended);
            sCount = 0;
        }else if(sCount <= 25){sCount++;}

		pros::delay(20); // 20 ms downtime
	}
}