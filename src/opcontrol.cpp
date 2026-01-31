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

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

using namespace pros;
using namespace umbc;
using namespace std;

// Example motor port definitions (replace with your actual ports)
#define FL1 3
#define FL2 -1
#define FL3 2
#define FR1 8
#define FR2 -10
#define FR3 9
#define BL1 11
#define BL2 -12
#define BL3 13
#define BR1 20
#define BR2 -19
#define BR3 18
#define i1 -17
#define i2 5
#define i3 7
#define i4 4
#define i5 6
#define i6 70
#define i7 40
#define JOYSTICK_MAX 127.0
#define gearMult 600
double lf, rf, lb, rb;

// Intake states
enum class intakeState
{
    INTAKE,
    SCORING_BOTTOM,
    SCORING_MID,
    SCORING_TOP,
    OFF
};
void umbc::Robot::opcontrol()
{
    // nice names for controllers (do not edit)
    umbc::Controller *controller_master = this->controller_master;
    umbc::Controller *controller_partner = this->controller_partner;

    // Motor groups for each wheel set
    std::vector<int8_t> frontLeft{FL1, FL2, FL3};
    std::vector<int8_t> frontRight{FR1, FR2, FR3};
    std::vector<int8_t> backLeft{BL1, BL2, BL3};
    std::vector<int8_t> backRight{BR1, BR2, BR3};
    // Create motor groups
    Motor_Group frontLeftGroup(frontLeft);
    Motor_Group frontRightGroup(frontRight);
    Motor_Group backLeftGroup(backLeft);
    Motor_Group backRightGroup(backRight);
    
    //Pneumatics
    pros::ADIPort right (1, E_ADI_DIGITAL_OUT);
    pros::ADIPort left (2, E_ADI_DIGITAL_OUT);
    frontLeftGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    frontRightGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    backLeftGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    backRightGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    // Intake motors
    pros::Motor ig1(i1);
    pros::Motor ig2(i2);
    pros::Motor ig3(i3);
    pros::Motor ig4(i4);
    pros::Motor ig5(i5);
    pros::Motor ig6(i6);
    pros::Motor ig7(i7);
    // Set gearing if needed
    pros::motor_gearset_e gearColor = pros::E_MOTOR_GEAR_BLUE;
    frontLeftGroup.set_gearing(gearColor);
    frontRightGroup.set_gearing(gearColor);
    backLeftGroup.set_gearing(gearColor);
    backRightGroup.set_gearing(gearColor);
    // Intake state
    intakeState iState = intakeState::OFF;
    while (1)
    {

        // Joystick input
        double x = controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        double y = -controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Cubic scaling for finer control at low speeds
        x = pow((x/JOYSTICK_MAX),3);
        y = pow((y/JOYSTICK_MAX),3);
        turn = pow((turn/JOYSTICK_MAX),3);
        
        // calculate motor velocities (mecanum mixing)
        lf = (y - x - turn);
        rf = (y + x + turn);
        lb = (y + x - turn);
        rb = (y - x + turn);
        // Normalize wheel speeds
        frontRightGroup.move_velocity(rf * gearMult);
        backRightGroup.move_velocity(rb * gearMult);
        frontLeftGroup.move_velocity(lf * gearMult);
        backLeftGroup.move_velocity(lb * gearMult);
        // Intake controls  
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
        {
            if (iState == intakeState::INTAKE)
            {
                iState = intakeState::OFF;
                ig1.move_velocity(0);
                ig2.move_velocity(0);
                ig3.move_velocity(0);
                ig4.move_velocity(0);
                ig5.move_velocity(0);
            }
            else
            {
                iState = intakeState::INTAKE;
                ig1.move_velocity(-600);
                ig2.move_velocity(-600);
                ig3.move_velocity(-600);
                ig4.move_velocity(-600);
                ig5.move_velocity(600);                
            }
        }
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
        {
            if (iState == intakeState::SCORING_BOTTOM)
            {
                iState = intakeState::OFF;
                ig1.move_velocity(0);
                ig2.move_velocity(0);
                ig3.move_velocity(0);
                ig4.move_velocity(0);
                ig5.move_velocity(0);
            }
            else
            {                
                iState = intakeState::SCORING_BOTTOM;
                ig1.move_velocity(600);
                ig2.move_velocity(600);
                ig3.move_velocity(-600);
                ig4.move_velocity(0);
                ig5.move_velocity(0);
            }
        }
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
        {
            if (iState == intakeState::SCORING_MID)
            {
                iState = intakeState::OFF;
                ig1.move_velocity(0);
                ig2.move_velocity(0);
                ig3.move_velocity(0);
                ig4.move_velocity(0);
                ig5.move_velocity(0);
            }
            else
            {
                iState = intakeState::SCORING_MID;
                ig1.move_velocity(600);
                ig2.move_velocity(-600);
                ig3.move_velocity(-600);
                ig4.move_velocity(600);
                ig5.move_velocity(0);
            }
        }
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
        {
            if (iState == intakeState::SCORING_TOP)
            {
                iState = intakeState::OFF;
                ig1.move_velocity(0);
                ig2.move_velocity(0);
                ig3.move_velocity(0);
                ig4.move_velocity(0);
                ig5.move_velocity(0);
                
            }
            else
            {
                iState = intakeState::SCORING_TOP;
                ig1.move_velocity(600);
                ig2.move_velocity(-600);
                ig3.move_velocity(-600);
                ig4.move_velocity(-600);
                ig5.move_velocity(-600);   
            }
        }
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
        {
            right.set_value(true);
            left.set_value(true);
            ig7.move_velocity(600);
            ig6.move_velocity(-600);

        }
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            right.set_value(false);
            left.set_value(false);
            ig7.move_velocity(0);
            ig6.move_velocity(-0);
        }
        // Delay for opcontrol loop
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}