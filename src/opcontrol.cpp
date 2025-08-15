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
#define FL1 1
#define FL2 -2
#define FL3 3
#define FR1 4
#define FR2 -5
#define FR3 6
#define ML1 7
#define ML2 -8
#define ML3 9
#define MR1 10
#define MR2 -11
#define MR3 12

void umbc::Robot::opcontrol() {
    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    // Motor groups for each wheel set
    std::vector<int8_t> frontLeft{FL1, FL2, FL3};
    std::vector<int8_t> frontRight{FR1, FR2, FR3};
    std::vector<int8_t> middleLeft{ML1, ML2, ML3};
    std::vector<int8_t> middleRight{MR1, MR2, MR3};

    Motor_Group frontLeftGroup(frontLeft);
    Motor_Group frontRightGroup(frontRight);
    Motor_Group middleLeftGroup(middleLeft);
    Motor_Group middleRightGroup(middleRight);

    // Set gearing if needed
    pros::motor_gearset_e gearColor = pros::E_MOTOR_GEAR_BLUE;
    frontLeftGroup.set_gearing(gearColor);
    frontRightGroup.set_gearing(gearColor);
    middleLeftGroup.set_gearing(gearColor);
    middleRightGroup.set_gearing(gearColor);


    extern pros::Imu imu_sensor; // Use the IMU initialized in main.cpp
    double desired_heading = imu_sensor.get_heading();

    while (1) {

    // Joystick input
    double x = controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    double y = -controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double turn = controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // Field-relative drive: rotate joystick vector by negative robot heading
    double heading_deg = imu_sensor.get_heading();
    double heading_rad = heading_deg * M_PI / 180.0;
    double cosA = cos(-heading_rad);
    double sinA = sin(-heading_rad);
    double field_x = x * cosA - y * sinA;
    double field_y = x * sinA + y * cosA;

    // Use field-relative x/y in drive calculations
    double fl = field_y + field_x + turn;
    double fr = field_y - field_x - turn;
    double ml = field_y + turn;
    double mr = field_y - turn;

        // Send velocities to motor groups
        frontLeftGroup.move_velocity(fl);
        frontRightGroup.move_velocity(fr);
        middleLeftGroup.move_velocity(ml);
        middleRightGroup.move_velocity(mr);

        pros::Task::delay(this->opcontrol_delay_ms);
    }
}