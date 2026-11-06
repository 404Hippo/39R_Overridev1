#pragma once
#include "api.h"
#include "lemlib/api.hpp"

extern double computeY(double x, double t);

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern pros::Imu imu;

extern pros::Rotation verticalEnc;

extern lemlib::Chassis chassis;

extern pros::Controller controller;

extern double driveConst;

extern double leftValue;
extern double rightValue;

extern void updateDistanceValues();