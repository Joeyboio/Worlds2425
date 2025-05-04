#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::adi::Pneumatics Clamp('B', false);
inline pros::adi::Pneumatics Doinker('H', false);
inline pros::adi::Button IntLim('G');
inline pros::Rotation Rotation(4);
inline pros::Optical Optical(10);

inline pros::Motor Intake(11,pros::v5::MotorGears::blue);
inline pros::Motor LB(15,pros::v5::MotorGears::green);
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
/*
const int numStates = 4;
int states[numStates] = {0, 31900, 28000, 14000};
int curState = 0;
int Target = 0;

void NextState(){
    curState +=1;
    if (curState == 4){
        curState=0;
    }
    Target=states[curState];
}

void LiftControl(){
    double KP = 3;
    double error= Target- Rotation.get_position();
    double velocity = KP *error;
    LB.move(velocity);
}

*/