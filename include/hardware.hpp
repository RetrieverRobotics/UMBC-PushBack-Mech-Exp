#pragma once

#include "api.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "config.hpp"

struct Hardware {
    // Drive
    pros::MotorGroup frontLeft;
    pros::MotorGroup frontRight;
    pros::MotorGroup backLeft;
    pros::MotorGroup backRight;

    // Intake
    pros::Motor topRoller;
    pros::Motor backRoller;
    pros::Motor bottomOutsideRoller;
    pros::Motor middleRoller;
    pros::Motor bottomRoller;
    pros::Motor feeder;

    // Sensors
    pros::Optical optical;
    pros::ADIPort pneumatic;

    Hardware() :
        frontLeft(std::vector<int8_t>{
            ports::drivetrain::frontLeft::front_left_front,
            ports::drivetrain::frontLeft::front_left_middle,
            ports::drivetrain::frontLeft::front_left_back}),
        frontRight(std::vector<int8_t>{
            ports::drivetrain::frontRight::front_right_front,
            ports::drivetrain::frontRight::front_right_middle,
            ports::drivetrain::frontRight::front_right_back}),
        backLeft(std::vector<int8_t>{
            ports::drivetrain::backLeft::back_left_top,
            ports::drivetrain::backLeft::back_left_middle,
            ports::drivetrain::backLeft::back_left_bottom}),
        backRight(std::vector<int8_t>{
            ports::drivetrain::backRight::back_right_top,
            ports::drivetrain::backRight::back_right_middle,
            ports::drivetrain::backRight::back_right_bottom}),
        topRoller           (ports::intake::top_roller),
        backRoller          (ports::intake::back_roller),
        bottomOutsideRoller (ports::intake::bottom_outside_roller),
        middleRoller        (ports::intake::middle_roller),
        bottomRoller        (ports::intake::bottom_roller),
        feeder              (ports::feeder),
        optical             (ports::optical),
        pneumatic      (ports::pneumatic, E_ADI_DIGITAL_OUT)
    {}

    void configure() {
        for (auto *g : {&frontLeft, &frontRight, &backLeft, &backRight}) {
            g->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            g->set_gearing(constants::driveGearColor);
        }

        for (auto *m : {&topRoller, &backRoller, &bottomOutsideRoller, &middleRoller, &bottomRoller, &feeder}) {
            m->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            m->set_gearing(constants::intakeGearColor);
        }
        pneumatic.set_value(0);
        optical.set_led_pwm(100);
    }
};