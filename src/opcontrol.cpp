/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

#include "api.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "umbc.h"
#include "umbc/robot.hpp"
#include "BallTracker.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

using namespace pros;
using namespace umbc;
using namespace std;

void umbc::Robot::opcontrol()
{

    Hardware hw;
    hw.configure();
    BallTracker balltracker;

    double lf, rf, lb, rb;
    intake::intakeState iState = intake::intakeState::OFF;
    uint32_t lastMotorCheck = 0;
    
    while (1)
    {
        // Joystick input
        double x = (controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / constants::controllerMax);
        double y = (controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / constants::controllerMax);
        double turn = (controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / constants::controllerMax);

        lf = (y - x + turn);
        rf = (y + x - turn);
        lb = (y + x + turn);
        rb = (y - x - turn);

        hw.frontLeft.move_velocity(lf * constants::driveGearMultiplier);
        hw.frontRight.move_velocity(rf * constants::driveGearMultiplier);
        hw.backLeft.move_velocity(lb * constants::driveGearMultiplier);
        hw.backRight.move_velocity(rb * constants::driveGearMultiplier);
        balltracker.update(hw.optical, iState, hw);
        if (iState != intake::intakeState::REJECT and iState != intake::intakeState::REJECTWITHFEED)
        {
            // Button input handling: toggle intake/scoring modesx
            if (controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_B))
            {
                apply_intake_config(iState, hw, intake::SCORING_BOTTOM);
            }

            else if (controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_Y))
            {
                apply_intake_config(iState, hw, intake::SCORING_MID);
            }

            else if (controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_X))
            {
                apply_intake_config(iState, hw, intake::SCORING_TOP);
            }

            else if (controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_A))
            {
                apply_intake_config(iState, hw, intake::CYCLE_MODE);
            }

            else if (controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            {
                apply_intake_config(iState, hw, intake::INTAKE_FEED_MODE);
            }
            else
            {
                apply_intake_config(iState, hw, intake::OFF_MODE);
            }
        }
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            balltracker.toggleColor();
        }
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            balltracker.toggleSort();
        }
        if (pros::millis() - lastMotorCheck > 500)
        {
            lastMotorCheck = pros::millis();
            for (int i = 1; i <= 20; i++)
            {
                if (i != 15)
                {
                    pros::Motor motor(i);
                    motor.get_faults();
                    if (errno == ENODEV)
                    {
                        controller_master->print(0, 0, "DISCONNECT PORT: %d", i);
                        controller_master->rumble(".");
                    }
                }
            }
        }
        // delay
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}