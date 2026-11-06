#pragma once
#include "api.h"

extern pros::Motor stage1;
extern pros::Motor stage2;
extern pros::Motor dr4bLeftMotor;
extern pros::Motor dr4bRightMotor;

extern pros::adi::Pneumatics clawtop;
extern pros::adi::Pneumatics clawbottom;
extern pros::adi::Pneumatics clawpivot;

extern int Intake;
extern void setIntake(int intakePower);

extern pros::Distance leftdistance;
extern pros::Distance rightdistance;

extern pros::Rotation dr4bLeft;
extern pros::Rotation dr4bRight;

extern int DR4B;
extern double dr4bTarget;

extern double dr4bKp;
extern double dr4bKi;
extern double dr4bKd;

void setDR4B(double angle);
void controlDR4B();