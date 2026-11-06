#include "main.h"
#include <cmath>
/*
LemLib robot configuration of drivetrain, sensors, autonomous, and drive curve settings.
*/

// Drivetrain Motor Groups
pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue); // left motor group
pros::MotorGroup rightMotors({18, 19, 20}, pros::MotorGearset::blue); // right motor group

pros::Imu imu(15); // inertial sensor

// Tracking Sensors
pros::Rotation verticalEnc(-14);

lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -0.5);

// Drivetrain Settings
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              11.4375,
                              lemlib::Omniwheel::NEW_275,
                              450,
                              8 
);

// Linear Motion
lemlib::ControllerSettings linearController(7, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            20, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            30, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            127 // maximum acceleration (slew)
);

// Angular Motion
lemlib::ControllerSettings angularController(7, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             50, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// Sensor Configuration
lemlib::OdomSensors sensors(&vertical, 
                            nullptr,
                            nullptr, 
                            nullptr, 
                            &imu
);

// Linear Drive Curve
// curve set to 1 because we use a custom curve equation
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// Angular Drive Curve
// curve set to 1 because we use a custom curve equation
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);

// Chassis Assembly
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// Custom Drive Curve
double driveConst = 0.3295;

double computeY(double x, double t){
    double exp_neg_t = std::exp(-t / 10.0);
    double exp_term = std::exp((std::abs(x) - 127.0) / 10.0); // absolute value using std::abs
    double y = (exp_neg_t + exp_term * (1.0 - exp_neg_t)) * x;
    return y;
}

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Distance Reset
double leftValue;
double rightValue;

void updateDistanceValues() {
        leftValue = 72 - ((leftdistance.get_distance()/25.4) + 6.09375);
        rightValue = 72 - ((rightdistance.get_distance()/25.4) + 6.09375);
}