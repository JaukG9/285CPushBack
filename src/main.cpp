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

/*
// lateral PID controller
lemlib::ControllerSettings lateral_controller(11, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              80, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              75 // maximum acceleration (slew)
); */

lemlib::ControllerSettings lateral_controller(11, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              80, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              25, // derivative gain (kD)
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
    /* PID Tuning 
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 10000); 
        // */

    /* left auton 
    chassis.setPose(-48.32, 16.57, 78);
    chassis.moveToPoint(-14.86, 24.39, 1000);                       // 1
    intake.move_velocity(600);
    pros::delay(800);                                               // 1.8
    chassis.moveToPoint(-24, -24, 1500, {.forwards = false});       // 3.3
    chassis.turnToHeading(315, 700);                                // 4
    chassis.moveToPoint(-10, 10, 1000);                             // 5
    pros::delay(1000);                                              // 6
    extakeT.move_velocity(600);
    pros::delay(1000);                                              // 7
    scraper.set_value(true);
    chassis.moveToPose(-62, 48, 270, 2000);                         // 9
    pros::delay(3000);                                              // 12
    chassis.moveToPoint(-25, 48, 1500, {.forwards = false});        // 13.5
    pros::delay(1500);                                              // 15
    extakeT.move_velocity(600);
        // */

    /* skills auton - left start  -------------  maximum total time to take   
        //** left alliance side /
    chassis.setPose(-48, 16, 0);
    scraper.set_value(true);
    chassis.moveToPoint(-48, 52, 1500);                             // 1.5
    chassis.turnToHeading(270, 750);                                // 2.25
    chassis.moveToPoint(-62, 48, 1000);                             // 3.25
    intake.move_velocity(600);
    pros::delay(3000);                                              // 6.25
    scraper.set_value(false);
    chassis.moveToPoint(-48, 48, 1000, {.forwards = false});        // 7.25
    chassis.turnToHeading(45, 750);                                 // 9
    chassis.moveToPoint(-36.095, 59.905, 1000);                     // 10
    chassis.turnToHeading(90, 750);                                 // 10.75
    chassis.moveToPoint(50, 59.905, 1500);                          // 12.25
    chassis.turnToHeading(45, 750);                                 // 13
    chassis.moveToPose(25, 48, 90, 1500, {.forwards = false});      // 14.5
    pros::delay(2000);                                              // 16.5
    extakeT.move_velocity(600);
    pros::delay(2000);                                              // 18.5
    extakeT.brake();
        //** left opponent side /
    scraper.set_value(true);
    chassis.moveToPoint(62, 48, 1000);                              // 19.5
    pros::delay(3000);                                              // 22.5
    scraper.set_value(false);
    chassis.moveToPoint(25, 48, 1500, {.forwards = false});         // 24
    pros::delay(2000);                                              // 26
    extakeT.move_velocity(600);
    pros::delay(2000);                                              // 28
    intake.brake();
    extakeT.brake();
        //** right opponent side /
    chassis.moveToPoint(41, -52, 3000);                             // 31
    scraper.set_value(true);
    chassis.turnToHeading(90, 750);                                 // 31.75
    chassis.moveToPoint(62, -48, 1500);                             // 33.25
    intake.move_velocity(600);
    pros::delay(3000);                                              // 36.25
    scraper.set_value(false);
    chassis.moveToPoint(48, -48, 1000, {.forwards = false});        // 37.25
    chassis.turnToHeading(215, 750);                                // 38
    chassis.moveToPoint(36.095, -59.905, 1000);                     // 41
    chassis.turnToHeading(270, 750);                                // 41.75
    chassis.moveToPoint(-50, -59.905, 1500);                        // 43.25
    chassis.turnToHeading(215, 750);                                // 44
    chassis.moveToPose(-25, -48, 270, 1500);                        // 45.5
    pros::delay(2000);                                              // 47.5
    extakeT.move_velocity(600); 
    pros::delay(2000);                                              // 49.5
    extakeT.brake();
        //** right alliance side /
    scraper.set_value(true);
    chassis.moveToPoint(-62, -48, 1000);                            // 50.5
    pros::delay(3000);                                              // 53.5
    scraper.set_value(false);
    chassis.moveToPoint(-25, -48, 1500, {.forwards = false});       // 55
    pros::delay(2000);                                              // 57
    extakeT.move_velocity(600);
    pros::delay(2000);                                              // 59
    intake.brake();
    extakeT.brake();
        //** center goals /
    chassis.moveToPoint(-48, -48, 1000);                            // 60
    chassis.turnToHeading(45, 750);                                 // 60.75
    intake.move_velocity(600);
    chassis.moveToPoint(-10, -10, 1500);                            // 62.25
    pros::delay(2000);                                              // 64.25
    intake.move_velocity(-600);
    extakeT.move_velocity(-600);
    pros::delay(1750);                                              // 66
    intake.brake();
    extakeT.brake();
        //** clear and park /
    chassis.moveToPoint(-22, -22, 1200, {.forwards = false});       // 67.2
    chassis.turnToHeading(315, 800);                                // 69
    chassis.moveToPoint(-44, 0, 1000);                              // 70
    chassis.turnToHeading(270, 500);                                // 70.5
    scraper.set_value(true);
    chassis.moveToPoint(-62, 0, 10000);                             // until end
    intake.move_velocity(600);
    extakeT.move_velocity(600);                                     // end
        //** ideally 79 points /
    // */


    /* skip auton
    chassis.setPose(0, 0, 0);
    intake.move_velocity(600);
    chassis.moveToPoint(0, 3, 10000);
        // */
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
            axisL = 127;
            axisR = 127;
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            axisL = -127;
            axisR = -127;
        }

        chassis.tank(axisL, axisR); // arcade drive

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