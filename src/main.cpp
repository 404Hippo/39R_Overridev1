#include "main.h"

void initialize() {
	chassis.calibrate(); // calibrate sensors
    pros::lcd::initialize(); // initialize lcd

    pros::Task distanceTask([]{
        while (true) {
            updateDistanceValues();
            pros::delay(10);
        }
    });

    pros::Task intakeTask([]{
        while (true) {
            setIntake(Intake); 
            pros::delay(10);
        }
    });

    dr4bLeft.reset_position();
    dr4bRight.reset_position();

    pros::Task dr4bTask([] {
    while (true) {
            controlDR4B();
            pros::delay(10);
        }
    });

    clawpivot.set_value(true);

}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    test();
}

void opcontrol() {

	while (true) {
		// Drivetrain Control
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        leftMotors.move(computeY(leftY, driveConst));
        rightMotors.move(computeY(rightY, driveConst));

        // Intake Control
        Intake = (controller.get_digital(DIGITAL_R1) - controller.get_digital(DIGITAL_R2)) * 12000;

        DR4B = (controller.get_digital(DIGITAL_L1) - controller.get_digital(DIGITAL_L2)) * 12000;

        if (controller.get_digital_new_press(DIGITAL_Y)) {
            clawtop.toggle();
            clawbottom.toggle();
        }


        pros::delay(10); // delay to save resources
	}
}
