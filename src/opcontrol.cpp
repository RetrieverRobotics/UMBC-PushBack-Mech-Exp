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

// Motor port definitions for chassis (FL=front-left, FR=front-right, BL=back-left, BR=back-right)
#define FL1 -3
#define FL2 1
#define FL3 -2
#define FR1 8
#define FR2 -10
#define FR3 9
#define BL1 -11
#define BL2 12
#define BL3 -13
#define BR1 20
#define BR2 -19
#define BR3 18
#define i1 -17
#define i2 5
#define i3 7
#define i4 4
#define i5 6
#define i6 14
#define i7 15

#define OPTICAL_PORT 16
#define JOYSTICK_MAX 127.0
#define gearMult 600
#define driveGearColor pros::E_MOTOR_GEAR_BLUE 
#define intakeSensorDelay 500
#define intakeGearColor pros::E_MOTOR_GEAR_GREEN
#define defaultColor teamColor::RED

#define blue make_pair(200, 255)
#define red make_pair(1, 30)
 
double lf, rf, lb, rb;

// Enumeration for intake/scoring modes
enum class intakeState
{
    INTAKE,
    CYCLE,
    SCORING_BOTTOM,
    SCORING_MID,
    SCORING_TOP,
    REJECT,
    OFF
};
enum class teamColor
{
    RED,
    BLUE,
    UNDEFINED
};
struct ball {
    teamColor color;
    uint32_t timeDetected;
};

// ===== Mode Functions =====
// These functions set motor velocities and update intake state.
// Can be called multiple times from different contexts.

/// Sets motors for intake mode (draws in balls)
static void set_intake_mode(intakeState &iState,
                            pros::Motor &ig1,
                            pros::Motor &ig2,
                            pros::Motor &ig3,
                            pros::Motor &ig4,
                            pros::Motor &ig5)
{
    iState = intakeState::INTAKE;
    ig1.move_velocity(-600 );
    ig2.move_velocity(-600);
    ig3.move_velocity(-600);
    ig4.move_velocity(-600);
    ig5.move_velocity(600);
}

static void set_cycle_mode(intakeState &iState,
                            pros::Motor &ig1,
                            pros::Motor &ig2,
                            pros::Motor &ig3,
                            pros::Motor &ig4,
                            pros::Motor &ig5)
{
    iState = intakeState::CYCLE;
    ig1.move_velocity(600 );
    ig2.move_velocity(-600);
    ig3.move_velocity(-600);
    ig4.move_velocity(-600);
    ig5.move_velocity(600);
}

/// Sets motors for bottom goal scoring
static void set_scoring_bottom_mode(intakeState &iState,
                                    pros::Motor &ig1,
                                    pros::Motor &ig2,
                                    pros::Motor &ig3,
                                    pros::Motor &ig4,
                                    pros::Motor &ig5)
{
    iState = intakeState::SCORING_BOTTOM;
    ig1.move_velocity(300);
    ig2.move_velocity(300);
    ig3.move_velocity(-300);
    ig4.move_velocity(-0);
    ig5.move_velocity(-0);
}

/// Sets motors for mid goal scoring
static void set_scoring_mid_mode(intakeState &iState,
                                 pros::Motor &ig1,
                                 pros::Motor &ig2,
                                 pros::Motor &ig3,
                                 pros::Motor &ig4,
                                 pros::Motor &ig5)
{
    iState = intakeState::SCORING_MID;
    ig1.move_velocity(600);
    ig2.move_velocity(-600);
    ig3.move_velocity(-600);
    ig4.move_velocity(600);
    ig5.move_velocity(0);
}

/// Sets motors for top goal scoring
static void set_scoring_top_mode(intakeState &iState,
                                 pros::Motor &ig1,
                                 pros::Motor &ig2,
                                 pros::Motor &ig3,
                                 pros::Motor &ig4,
                                 pros::Motor &ig5)
{
    iState = intakeState::SCORING_TOP;
    ig1.move_velocity(300);
    ig2.move_velocity(-300);
    ig3.move_velocity(-300);
    ig4.move_velocity(-300);
    ig5.move_velocity(-300);
}

/// Stops all intake/scoring motors
static void set_off_mode(intakeState &iState,
                         pros::Motor &ig1,
                         pros::Motor &ig2,
                         pros::Motor &ig3,
                         pros::Motor &ig4,
                         pros::Motor &ig5)
{
    iState = intakeState::OFF;
    ig1.move_velocity(0);
    ig2.move_velocity(0);
    ig3.move_velocity(0);
    ig4.move_velocity(0);
    ig5.move_velocity(0);
}
void umbc::Robot::opcontrol()
{
    

    // Controller references
    umbc::Controller *controller_master = this->controller_master;
    umbc::Controller *controller_partner = this->controller_partner;

    // Chassis motor groups organized by wheel position
    std::vector<int8_t> frontLeft{FL1, FL2, FL3};
    std::vector<int8_t> frontRight{FR1, FR2, FR3};
    std::vector<int8_t> backLeft{BL1, BL2, BL3};
    std::vector<int8_t> backRight{BR1, BR2, BR3};

    // Initialize motor groups from ports
    MotorGroup frontLeftGroup(frontLeft);
    MotorGroup frontRightGroup(frontRight);
    MotorGroup backLeftGroup(backLeft);
    MotorGroup backRightGroup(backRight);
    pros::Motor ig1(i1);
    pros::Motor ig2(i2);
    pros::Motor ig3(i3);
    pros::Motor ig4(i4);
    pros::Motor ig5(i5);
    pros::Motor ig6(i6);
    pros::Motor ig7(i7);

    // Configure motor brake modes
    frontLeftGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    frontRightGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    backLeftGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    backRightGroup.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    ig1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    ig2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    ig3.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    ig4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    ig5.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    ig6.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ig7.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    

    // Configure motor gearing
    frontLeftGroup.set_gearing(driveGearColor);
    frontRightGroup.set_gearing(driveGearColor);
    backLeftGroup.set_gearing(driveGearColor);
    backRightGroup.set_gearing(driveGearColor);
    ig1.set_gearing(intakeGearColor);
    ig2.set_gearing(intakeGearColor);
    ig3.set_gearing(intakeGearColor);
    ig4.set_gearing(intakeGearColor);
    ig5.set_gearing(intakeGearColor);
    ig6.set_gearing(intakeGearColor);
    ig7.set_gearing(intakeGearColor);
    ig1.set_current_limit(750);
    // Optical sensor and ball tracking
    std::queue<ball> ballQueue;
    teamColor teamcolor = defaultColor;
    pros::Optical optical_sensor(OPTICAL_PORT);
    intakeState currState = intakeState::OFF;
    int colorValue = 100;
    intakeState iState = intakeState::OFF;
    optical_sensor.set_led_pwm(100);
    bool sortDisabled = false;
    
    // Pneumatics
    pros::ADIPort right(1, E_ADI_DIGITAL_OUT);
    pros::ADIPort left(2, E_ADI_DIGITAL_OUT);
    

    while (1) {
        // Joystick input
        double x = controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        double y = -controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = -controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Normalize joystick
        x /= JOYSTICK_MAX;
        y /= JOYSTICK_MAX;
        turn /= JOYSTICK_MAX;

        // Cubic scaling
        x = x * x * x;
        y = y * y * y;
        turn = turn * turn * turn;

        lf = (y - x + turn);
        rf = (y + x - turn);
        lb = (y + x + turn);
        rb = (y - x - turn);

        // Normalize
        // double maxVal = std::max({fabs(lf), fabs(rf), fabs(lb), fabs(rb)});
        // if (maxVal > 1.0) {
        //     lf /= maxVal;
        //     rf /= maxVal;
        //     lb /= maxVal;
        //     rb /= maxVal;
        // }

        // Drive
        frontLeftGroup.move_velocity(lf * gearMult);
        frontRightGroup.move_velocity(rf * gearMult);
        backLeftGroup.move_velocity(lb * gearMult);
        backRightGroup.move_velocity(rb * gearMult);

        // Button input handling: toggle intake/scoring modes
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            if (iState == intakeState::INTAKE) {
                set_off_mode(iState, ig1, ig2, ig3, ig4, ig5);
            } else {
                set_intake_mode(iState, ig1, ig2, ig3, ig4, ig5);
            }
        }

        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (iState == intakeState::SCORING_BOTTOM) {
                set_off_mode(iState, ig1, ig2, ig3, ig4, ig5);
            } else {
                set_scoring_bottom_mode(iState, ig1, ig2, ig3, ig4, ig5);
            }
        }

        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (iState == intakeState::SCORING_MID) {
                set_off_mode(iState, ig1, ig2, ig3, ig4, ig5);
            } else {
                set_scoring_mid_mode(iState, ig1, ig2, ig3, ig4, ig5);
            }
        }

        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (iState == intakeState::SCORING_TOP) {
                set_off_mode(iState, ig1, ig2, ig3, ig4, ig5);
            } else {
                set_scoring_top_mode(iState, ig1, ig2, ig3, ig4, ig5);
            }
        }

        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            right.set_value(true);
            left.set_value(true);
            ig7.move_velocity(600);
            ig6.move_velocity(-600);
        }

        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            right.set_value(false);
            left.set_value(false);
            ig7.move_velocity(600);
            ig6.move_velocity(-600);
        }

        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            if (iState == intakeState::CYCLE) {
                set_off_mode(iState, ig1, ig2, ig3, ig4, ig5);
            } else {
                set_cycle_mode(iState, ig1, ig2, ig3, ig4, ig5);
            }
        }

        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            sortDisabled = !sortDisabled;
            if(sortDisabled){
                if(iState == intakeState::REJECT){
                    set_intake_mode(iState, ig1, ig2, ig3, ig4, ig5);
                }
            }
        }

        //Read optical sensor hue value
        
        {
            colorValue = optical_sensor.get_hue();

            // Ball detection: queue balls by detected color
            if (colorValue >= get<0>(red) && colorValue < get<1>(red)) {
                ballQueue.push({teamColor::RED, pros::millis()});
            }
            if (colorValue >= get<0>(blue) && colorValue < get<1>(blue)) {
                ballQueue.push({teamColor::BLUE, pros::millis()});
            }
            // Process queued balls after 500ms delay
            // Only act on optical detections while in INTAKE mode so scoring motor states
            // aren't overridden by sensor events.
            if(!sortDisabled){
            while (!ballQueue.empty() && (pros::millis() - ballQueue.front().timeDetected) > intakeSensorDelay) {
                if (iState == intakeState::INTAKE) {
                    // Sort balls by team color and adjust intake accordingly
                    if (ballQueue.front().color != teamcolor) {
                        // Opponent ball detected: drive eject/aux motor to expel
                        iState = intakeState::REJECT; // Temporary state to trigger motor update without changing intake state
                        ig5.move_velocity(-600);
                    } else {
                        // Friendly ball detected: ensure intake motors run
                        set_intake_mode(iState, ig1, ig2, ig3, ig4, ig5);
                    }
                }
                else if(iState == intakeState::CYCLE){
                    if (ballQueue.front().color != teamcolor) {
                        // Opponent ball detected: drive eject/aux motor to expel
                        set_cycle_mode(iState, ig1, ig2, ig3, ig4, ig5);
                        ig5.move_velocity(-600);
                    } else {
                        // Friendly ball detected: ensure intake motors run
                        set_cycle_mode(iState, ig1, ig2, ig3, ig4, ig5);
                    }
                }
                ballQueue.pop();
            }
        }
        }

        if(controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            if(teamcolor == teamColor::RED){
                teamcolor = teamColor::BLUE;
            } else {
                teamcolor = teamColor::RED;
            }
        }
        //Control loop delay (ms)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}