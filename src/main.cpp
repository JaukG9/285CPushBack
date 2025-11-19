#include "main.h"
#include "lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* initialization of motors & pistons */
pros::Motor intake(-1, pros::MotorGearset::blue); // spins intake clockwise
pros::Motor extakeT(20); // spins extake counterclockwise - may need to combine extakes due to power limits
pros::Motor extakeM(19); // spins extake counterclockwise - may need to combine extakes due to power limits
pros::MotorGroup left_mg({-3, -4, -5}, pros::MotorGearset::blue); // left motor group (clockwise --> omni = counterclockwise)
pros::MotorGroup right_mg({8, 9, 10}, pros::MotorGearset::blue); // right motor group (counterclockwise --> omni = clockwise)
//pros::ADIDigitalOut scraper(11); // extends scraper piston
pros::ADIDigitalOut midtake('H'); // extends midtake piston*/

/* creation of drivetrain */
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.22, // 11.22 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450 (600 * (36 / 48))
                              2 // horizontal drift is 2 (for now)
);

/* odometry */
pros::Imu imu(2); // imu
pros::Rotation horizontal_encoder(20); // horizontal tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D', true); // vertical tracking wheel encoder
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75); // horizontal tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5); // vertical tracking wheel

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
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
                        &steer_curve
);


void on_center_button(){}


void initialize(){
	pros::lcd::initialize();
	chassis.calibrate();
}


void disabled(){}


void autonomous(){
    /* left intake */
    /*chassis.setPose(-48.32, 16.57, 78);
    intake.move_velocity(600);
    chassis.moveToPose(-24, 22, 68, 2000);
    chassis.moveToPose(-13.61, 27.5, 330, 1200);
    intake.brake();
    chassis.moveToPose(-30.28, 36.79, 290, 1800);
    chassis.moveToPose(-42.5, 43.5, 315, 1200);
    chassis.moveToPose(-18, 18, 315, 2000);*/

    /* right intake */
    chassis.setPose(-48.32, -16.57, 102);
    intake.move_velocity(600);
    chassis.moveToPose(-14.86, -24.39, 102, 2000);
    chassis.moveToPose(-37.21, -36.27, 45, 2000);
    chassis.moveToPose(-18, -18, 45, 1500);
    intake.move_velocity(-600);
}


void opcontrol(){
	int axisL = 0;
	int axisR = 0;
    bool scraperExtended = false;
    bool midtakeExtended = false;
    int sCount = 0;

	while (true) {
		pros::lcd::print(0, "%d %d", axisL, axisR);
		
		/* drive */
		axisL = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
		axisR = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y); // Gets the turn left/right from right joystick
		chassis.tank(axisL, axisR); // tank drive

        /* intake + conveyer */
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intake.move_velocity(600); // intake motors forward
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intake.move_velocity(-600); // low extake
        else intake.brake();

        /* extake (may need to change due to wattage limitations) */
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            extakeT.move_velocity(600);
            intake.move_velocity(600);
        }else extakeT.brake();
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) extakeM.move_velocity(600);
        else extakeM.brake();

        /* pistons - will finish with a toggle later
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            scraperExtended = !scraperExtended;
            scraper.set_value(scraperExtended);
        }*/
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && sCount > 50){
            midtakeExtended = !midtakeExtended;
            midtake.set_value(midtakeExtended);
            sCount = 0;
        }else if(sCount <= 50){sCount++;}

		pros::delay(20); // 20 ms downtime
	}
}