#include "main.h"
#include <algorithm>
#include <cmath>
/*
Subsystems and globals such as motors, sensors, pneumatics, etc.
*/

// Motors
pros::Motor dr4bLeftMotor(-9, pros::v5::MotorGears::green);
pros::Motor dr4bRightMotor(10, pros::v5::MotorGears::green);

// Pneumatics
pros::adi::Pneumatics claw('B', false);
pros::adi::Pneumatics willy('A', false);
pros::adi::Pneumatics toggle('C', false);

// Distance Sensors
pros::Distance leftdistance(8);
pros::Distance rightdistance(3);

// Rotation Sensors
pros::Rotation dr4bLeft(16);
pros::Rotation dr4bRight(-17);

// PID constants
double dr4bKp = 300;
double dr4bKi = 0;
double dr4bKd = 0;
double dr4bKgLeft = 800;
double dr4bKgRight = 3000;

// Configuration
double dr4bTarget = 0;
int DR4B = 0;

double dr4bMin = 0;
double dr4bMax = 95;  // Conservative until full travel is verified

// Manual control
double dr4bManualVoltage = 12000; // Tune after verifying directions
double dr4bSyncKp = 20;
double dr4bSyncDeadband = 1.5;

// PID state
double leftIntegral = 0;
double rightIntegral = 0;
double leftPrevError = 0;
double rightPrevError = 0;

bool dr4bWasManual = false;

bool dr4bHoming = false;

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

    if (dr4bHoming) {
        return;
    }
    constexpr double dt = 0.01;

    double leftPos = getLeftAngle();
    double rightPos = getRightAngle();
    double averagePos = (leftPos + rightPos) / 2.0;
    double difference = leftPos - rightPos;

    static int debugCounter = 0;
    if (++debugCounter >= 20) {
        debugCounter = 0;
        printf("L:%.1f R:%.1f Avg:%.1f Input:%d\n", leftPos, rightPos, averagePos, DR4B);
        printf(
        "Avg:%.1f L:%.1f R:%.1f "
        "CurrentL:%d CurrentR:%d "
        "TempL:%.1f TempR:%.1f\n",
        averagePos, leftPos, rightPos,
        dr4bLeftMotor.get_current_draw(),
        dr4bRightMotor.get_current_draw(),
        dr4bLeftMotor.get_temperature(),
        dr4bRightMotor.get_temperature()
        );
    }

    // Stop both motors if the sides become too misaligned
    static int misalignmentCount = 0;

    bool manual = !pros::competition::is_autonomous() && DR4B != 0;

    if (manual) {
        double direction = DR4B > 0 ? 1.0 : -1.0;

        // Smooth synchronization correction
        static double filteredDifference = 0;

        filteredDifference =
            0.85 * filteredDifference + 0.15 * difference;

        double correction = 0;

        if (std::abs(filteredDifference) > dr4bSyncDeadband) {
            correction =
                (std::abs(filteredDifference) - dr4bSyncDeadband)
                * dr4bSyncKp
                * (filteredDifference > 0 ? 1.0 : -1.0);
        }

        // Calculate motor voltages
        double base = direction * dr4bManualVoltage;

        double leftOutput = std::clamp(
            base - correction, -12000.0, 12000.0
        );

        double rightOutput = std::clamp(
            base + correction, -12000.0, 12000.0
        );

        // Reduce speed near the upper and lower limits
        constexpr double slowZone = 20.0;

        auto limitOutput = [&](double pos, double output) {
            if (output > 0) {
                double remaining = dr4bMax - pos;

                if (remaining <= 0)
                    return 0.0;

                double scale = std::clamp(remaining / slowZone, 0.0, 1.0);
                output *= scale;
            }
            else if (output < 0) {
                double remaining = pos - dr4bMin;

                if (remaining <= 0)
                    return 0.0;

                double scale = std::clamp(remaining / slowZone, 0.0, 1.0);
                output *= scale;
            }

            return std::clamp(output, -12000.0, 12000.0);
        };

        leftOutput = limitOutput(leftPos, leftOutput);
        rightOutput = limitOutput(rightPos, rightOutput);

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
        dr4bKd * leftDerivative +
        dr4bKgLeft;

    double rightOutput =
        dr4bKp * rightError +
        dr4bKi * rightIntegral +
        dr4bKd * rightDerivative +
        dr4bKgRight;

    leftOutput = std::clamp(leftOutput, -12000.0, 12000.0);
    rightOutput = std::clamp(rightOutput, -12000.0, 12000.0);

    if (leftPos >= dr4bMax && leftOutput > 0)
        leftOutput = 0;

    if (rightPos >= dr4bMax && rightOutput > 0)
        rightOutput = 0;

    if (leftPos <= dr4bMin && leftOutput < 0)
        leftOutput = 0;

    if (rightPos <= dr4bMin && rightOutput < 0)
        rightOutput = 0;

    dr4bLeftMotor.move_voltage(static_cast<int32_t>(leftOutput));
    dr4bRightMotor.move_voltage(static_cast<int32_t>(rightOutput));

    leftPrevError = leftError;
    rightPrevError = rightError;
}

bool dr4bHomeRequested = false;

bool homeDR4B() {
    if (dr4bHoming)
        return false;

    dr4bHoming = true;

    constexpr int homingVoltage = -12000;
    constexpr int sampleMs = 50;
    constexpr int timeoutMs = 1000;

    constexpr double movementThreshold = 0.15;
    constexpr double minTravel = 1.0;
    constexpr int stableSamplesRequired = 5;

    double startLeft = getLeftAngle();
    double startRight = getRightAngle();

    double lastLeft = startLeft;
    double lastRight = startRight;

    int leftStable = 0;
    int rightStable = 0;

    bool leftStopped = false;
    bool rightStopped = false;

    bool success = false;

    for (int elapsed = 0; elapsed < timeoutMs; elapsed += sampleMs) {
        double left = getLeftAngle();
        double right = getRightAngle();

        bool leftMoved = std::abs(left - startLeft) >= minTravel;
        bool rightMoved = std::abs(right - startRight) >= minTravel;

        if (leftMoved &&
            std::abs(left - lastLeft) < movementThreshold)
            leftStable++;
        else
            leftStable = 0;

        if (rightMoved &&
            std::abs(right - lastRight) < movementThreshold)
            rightStable++;
        else
            rightStable = 0;

        leftStopped = leftStable >= stableSamplesRequired;
        rightStopped = rightStable >= stableSamplesRequired;

        if (leftStopped && rightStopped) {
            success = true;

            break;
        }

        dr4bLeftMotor.move_voltage(leftStopped ? 0 : homingVoltage);
        dr4bRightMotor.move_voltage(rightStopped ? 0 : homingVoltage);

        lastLeft = left;
        lastRight = right;

        pros::delay(sampleMs);
    }

    dr4bLeftMotor.move_voltage(0);
    dr4bRightMotor.move_voltage(0);

    if (success) {
        // Only valid if both sides are confirmed at the physical bottom
        dr4bLeft.reset_position();
        dr4bRight.reset_position();
        dr4bTarget = 0;
    }

    dr4bHoming = false;

    return success;
}

void DR4Bhome() {
    dr4bHomeRequested = true;
}