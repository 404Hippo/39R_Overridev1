#pragma once
#include "api.h"

extern pros::Motor dr4bLeftMotor;
extern pros::Motor dr4bRightMotor;

extern pros::adi::Pneumatics claw;
extern pros::adi::Pneumatics willy;
extern pros::adi::Pneumatics toggle;

extern pros::Distance leftdistance;
extern pros::Distance rightdistance;

extern pros::Rotation dr4bLeft;
extern pros::Rotation dr4bRight;

extern int DR4B;
extern double dr4bTarget;

double getLeftAngle();
double getRightAngle();

extern double dr4bKp;
extern double dr4bKi;
extern double dr4bKd;

void setDR4B(double angle);
void controlDR4B();

extern bool dr4bHomeRequested;

bool homeDR4B();

extern void DR4Bhome();