#include "main.h"
/*
Match and skills autonomous programs
*/

void verticalShort() {
    chassis.setPose(-4, -62.5, 180);
    claw.set_value(true);
    chassis.moveToPose(-4, -50, 180, 1500, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveToPose(-4, -72, 180, 1000, {.minSpeed = 100});
    chassis.waitUntilDone();
    chassis.moveToPose(-4, -50, 180, 1500, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveToPose(-4, -72, 180, 1000, {.minSpeed = 100});
    chassis.waitUntilDone();
    chassis.moveToPose(-4, -48, 180, 1500, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    chassis.setPose(-4, -rightValue, 90);
    setDR4B(10);
    chassis.moveToPose(-24, -48, 90, 1000, {.forwards = false, .minSpeed = 40});
    chassis.waitUntilDone();
    chassis.setPose(-18, -rightValue, chassis.getPose().theta);
    setDR4B(0);
    DR4Bhome();
    chassis.moveToPoint(-15.5, -48, 1000);
    chassis.waitUntilDone();
    pros::delay(500);
    claw.toggle();
    pros::delay(500);
    chassis.moveToPose(-5, -47, 90, 1500, {.minSpeed = 60});
    chassis.waitUntilDone();
    chassis.turnToHeading(135, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-48, 48, 2000, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();

    
}

void skills() {
    chassis.setPose(4, -64.5, 180);
    claw.set_value(true);
    toggle.set_value(false);
    chassis.moveToPoint(4, -65, 500);
    chassis.waitUntilDone();
    chassis.moveToPose(4, -48, 180, 1500, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 750);
    chassis.waitUntilDone();
    chassis.setPose(4, -leftValue, -90);
    setDR4B(15);
    chassis.moveToPose(24, -49, -90, 1000, {.forwards = false, .minSpeed = 40});
    chassis.waitUntilDone();
    chassis.setPose(18, -leftValue, chassis.getPose().theta);
    setDR4B(0);
    chassis.moveToPoint(15.5, -48, 1000);
    chassis.waitUntilDone();
    claw.set_value(false);
    pros::delay(500);
    chassis.moveToPose(5, -48, -90, 1000, {.minSpeed = 60});
    chassis.waitUntilDone();
    chassis.turnToHeading(-135, 500);
    chassis.waitUntilDone();
    setDR4B(0);
    DR4Bhome();
    chassis.moveToPose(25, -28, -135, 1500, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveToPose(25, -28, -135, 1000, {.forwards = false, .maxSpeed = 20});
    claw.set_value(true);
    pros::delay(500);
    setDR4B(30);
    chassis.waitUntilDone();
    chassis.turnToHeading(-5, 1000);
    chassis.waitUntilDone();
    chassis.moveToPose(27, -50, 0, 1000, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    chassis.setPose(24, -46, 0);
    setDR4B(0);
    pros::delay(500);
    claw.set_value(false);
    pros::delay(500);
    chassis.moveToPoint(25, -35, 1000, {.minSpeed = 60});
    chassis.waitUntilDone();
    chassis.turnToHeading(-45, 500);
    chassis.waitUntilDone();
    setDR4B(0);
    DR4Bhome();
    chassis.moveToPose(44, -53, -45, 1500, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveToPose(45, -54, -45, 1000, {.forwards = false, .minSpeed = 20});
    chassis.waitUntil(3);
    claw.set_value(true);
    pros::delay(500);
    setDR4B(45);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    chassis.moveToPose(23, -56, 90, 1000, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    claw.set_value(false);
    pros::delay(500);
    chassis.moveToPoint(30, -55, 1500);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 0, 2000);
    chassis.waitUntilDone();
    

}

void test() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    setDR4B(70);

}