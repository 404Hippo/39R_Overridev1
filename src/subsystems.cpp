#include "main.h"
#include <algorithm>
#include <cmath>
/*
Subsystems and globals such as motors, sensors, pneumatics, etc.
*/

// Motors
pros::Motor stage1(-4, pros::v5::MotorGears::green);
pros::Motor stage2(-2, pros::v5::MotorGears::green);
pros::Motor dr4bLeftMotor(-9, pros::v5::MotorGears::green);
pros::Motor dr4bRightMotor(10, pros::v5::MotorGears::green);

// Intake
int Intake = 0;

void setIntake(int intakePower) {
    stage1.move_voltage(intakePower);
    stage2.move_voltage(intakePower);
}

// Pneumatics
pros::adi::Pneumatics clawtop('C', false);
pros::adi::Pneumatics clawbottom('A', false);
pros::adi::Pneumatics clawpivot('B', false);

// Distance Sensors
pros::Distance leftdistance(8);
pros::Distance rightdistance(3);

// Rotation Sensors
pros::Rotation dr4bLeft(16);
pros::Rotation dr4bRight(-17);

// PID constants
double dr4bKp = 150;
double dr4bKi = 0;
double dr4bKd = 0;

// Configuration
double dr4bTarget = 0;
int DR4B = 0;

double dr4bMin = 0;
double dr4bMax = 70;  // Conservative until full travel is verified

// Manual control
double dr4bManualVoltage = 5000; // Tune after verifying directions
double dr4bSyncKp = 200;         // mV per degree of side difference

// Safety
double dr4bMaxDifference = 5;    // Stop if sides differ by >5 degrees

// PID state
double leftIntegral = 0;
double rightIntegral = 0;
double leftPrevError = 0;
double rightPrevError = 0;

bool dr4bWasManual = false;
bool dr4bFault = false;

// Physical output angle (12T drives 84T)
double getLeftAngle() {
    return dr4bLeft.get_position() / 700.0;
}

double getRightAngle() {
    return dr4bRight.get_position() / 700.0;
}

// Autonomous position command
void setDR4B(double angle) {
    dr4bTarget = std::clamp(angle, dr4bMin, dr4bMax);
}

// Continuous controller
void controlDR4B() {
    constexpr double dt = 0.01;

    double leftPos = getLeftAngle();
    double rightPos = getRightAngle();
    double averagePos = (leftPos + rightPos) / 2.0;
    double difference = leftPos - rightPos;

    // Stop both motors if the sides become too misaligned
    if (std::abs(difference) > dr4bMaxDifference) {
        dr4bFault = true;
    }

    if (dr4bFault) {
        dr4bLeftMotor.move_voltage(0);
        dr4bRightMotor.move_voltage(0);
        return;
    }

    bool manual = !pros::competition::is_autonomous() && DR4B != 0;

    if (manual) {
        double direction = DR4B > 0 ? 1.0 : -1.0;

        // Enforce position limits
        if ((direction > 0 && averagePos >= dr4bMax) ||
            (direction < 0 && averagePos <= dr4bMin)) {
            dr4bLeftMotor.move_voltage(0);
            dr4bRightMotor.move_voltage(0);
            dr4bWasManual = true;
            return;
        }

        // Base manual voltage plus synchronization correction
        double base = direction * dr4bManualVoltage;
        double correction = difference * dr4bSyncKp;

        double leftOutput = std::clamp(base - correction, -12000.0, 12000.0);
        double rightOutput = std::clamp(base + correction, -12000.0, 12000.0);

        dr4bLeftMotor.move_voltage(static_cast<int32_t>(leftOutput));
        dr4bRightMotor.move_voltage(static_cast<int32_t>(rightOutput));

        dr4bWasManual = true;
        return;
    }

    // On release, hold the current physical position
    if (dr4bWasManual) {
        dr4bTarget = std::clamp(averagePos, dr4bMin, dr4bMax);

        leftIntegral = 0;
        rightIntegral = 0;
        leftPrevError = dr4bTarget - leftPos;
        rightPrevError = dr4bTarget - rightPos;

        dr4bWasManual = false;
    }

    // Independent PID for both sides
    double leftError = dr4bTarget - leftPos;
    double rightError = dr4bTarget - rightPos;

    leftIntegral += leftError * dt;
    rightIntegral += rightError * dt;

    double leftDerivative = (leftError - leftPrevError) / dt;
    double rightDerivative = (rightError - rightPrevError) / dt;

    double leftOutput =
        dr4bKp * leftError +
        dr4bKi * leftIntegral +
        dr4bKd * leftDerivative;

    double rightOutput =
        dr4bKp * rightError +
        dr4bKi * rightIntegral +
        dr4bKd * rightDerivative;

    leftOutput = std::clamp(leftOutput, -12000.0, 12000.0);
    rightOutput = std::clamp(rightOutput, -12000.0, 12000.0);

    dr4bLeftMotor.move_voltage(static_cast<int32_t>(leftOutput));
    dr4bRightMotor.move_voltage(static_cast<int32_t>(rightOutput));

    leftPrevError = leftError;
    rightPrevError = rightError;
}