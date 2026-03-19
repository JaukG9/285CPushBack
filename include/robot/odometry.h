#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "robot/drivetrain.h"
#include "robot/odometry.h"

class CustomChassis : public lemlib::Chassis{
    public:
    using lemlib::Chassis::Chassis;

    struct ChainToPointParams {
        /** whether the robot should move forwards or backwards. True by default */
        bool forwards = true;
        /** the maximum speed the robot can travel at. Value between 0-127. 127 by default */
        float maxSpeed = 127;
    };
    void chainToPoint(float x, float y, int timeout, ChainToPointParams params = {}, bool async = true){
        float currentX = chassis.getPose().x;
        float currentY = chassis.getPose().y;
        double distance = std::sqrt(std::pow(x - currentX, 2) + std::pow(y - currentY, 2));
        float actualMax = std::min((double) params.maxSpeed, sqrt(1000 * distance));
        float minSpeed = actualMax / 2;
        float earlyExitRange = distance * 15 / 100 * actualMax / 127;

        chassis.moveToPoint(x, y, timeout, 
                            {.forwards = params.forwards, 
                            .maxSpeed = params.maxSpeed,
                            .minSpeed = minSpeed, 
                            .earlyExitRange = earlyExitRange}, 
                            async);
    }
    struct ChainToHeadingParams{
        /** the direction the robot should turn in. AUTO by default */
        AngularDirection direction = AngularDirection::AUTO;
        /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
        int maxSpeed = 127;
    };
    void chainToHeading(float theta, int timeout, ChainToHeadingParams params = {}, bool async = true){
        float currentHeading = chassis.getPose().theta;
        float distance = std::abs(currentHeading - theta);
        float actualMax = std::min((double) params.maxSpeed, sqrt(500 * distance));
        int minSpeed = actualMax / 2;
        float earlyExitRange = distance * 8 / 100 * actualMax / 127;

        chassis.turnToHeading(theta, timeout,
                            {.direction = params.direction,
                            .maxSpeed = params.maxSpeed},
                            async);
    }
};

extern CustomChassis chassis;