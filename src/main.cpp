/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ayaan                                                     */
/*    Created:      10/19/2025, 3:26:09 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;


// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here
controller Controller = controller();
int axisLS, axisLT, axisRS, axisRT;

motor Motor1 = motor(PORT1, ratio6_1, false);
motor Motor2 = motor(PORT2, ratio6_1, false);
motor Motor3 = motor(PORT3, ratio6_1, false);
motor Motor4 = motor(PORT11, ratio6_1, true);
motor Motor5 = motor(PORT12, ratio6_1, true);
motor Motor6 = motor(PORT13, ratio6_1, true);
motor_group leftMotors = motor_group(Motor1, Motor2, Motor3);
motor_group rightMotors = motor_group(Motor4, Motor5, Motor6);

drivetrain DriveTrain = drivetrain(leftMotors, rightMotors, 260, 285, 252, mm, 48/36);

// button a
void aPressed(){
    //DriveTrain.drive(forward, 600, rpm);
}
void aReleased(){
    DriveTrain.stop();
}

void axisLTChanged(){
    leftMotors.spin(forward, 6 * Controller.Axis3.position(), rpm);
}
void axisRTChanged(){
    rightMotors.spin(forward, 6 * Controller.Axis3.position(), rpm);
}
/*
void axisLSChanged(){
    leftDrive(axisLS);
}
void axisRSChanged(){
    leftDrive(axisRS);
}*/

// main
int main() {
    while(1) {
        Brain.Screen.clearScreen();
        Controller.ButtonA.pressed(aPressed);
        Controller.ButtonA.released(aReleased);

        Controller.Axis3.changed(axisLTChanged);
        Controller.Axis2.changed(axisRTChanged);

        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
