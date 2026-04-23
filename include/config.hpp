#include "api.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"

#include <vector>

using namespace pros;
using namespace std;

namespace ports
{
    constexpr int radio = 21;
    constexpr int optical = 17;
    constexpr int feeder = 18;
    constexpr int pneumatic = 1;
    namespace intake
    {
        constexpr int top_roller = 11;
        constexpr int back_roller = 12;
        constexpr int bottom_outside_roller = 13;
        constexpr int middle_roller = 14;
        constexpr int bottom_roller = 16;
    }
    namespace drivetrain
    {
        namespace frontLeft
        {
            constexpr int front_left_front = -1;
            constexpr int front_left_middle = 2;
            constexpr int front_left_back = -3;
        }
        namespace frontRight
        {
            constexpr int front_right_front = 9;
            constexpr int front_right_middle = -8;
            constexpr int front_right_back = 7;
        }
        namespace backLeft
        {
            constexpr int back_left_top = -4;
            constexpr int back_left_middle = 5;
            constexpr int back_left_bottom = -6;
        }
        namespace backRight
        {
            constexpr int back_right_top = 20;
            constexpr int back_right_middle = -19;
            constexpr int back_right_bottom = 10;
        }
    }
}

namespace constants
{
    constexpr double driveGearRatio = 1.5;
    constexpr double driveWheelDiameter = 4.0;
    constexpr double driveWheelCircumference = driveWheelDiameter * M_PI;
    constexpr double idealSpeed = driveWheelCircumference * driveGearRatio * 200.0 / 60.0;
    constexpr pros::MotorGears driveGearColor = pros::v5::MotorGears::green;
    constexpr pros::MotorGears intakeGearColor = pros::v5::MotorGears::blue;
    constexpr double driveGearMultiplier = 200;
    constexpr double intakeGearMultiplier = 600;
    constexpr double controllerMax = 127.0;
    constexpr int opcontrol_delay_ms = 20;
    constexpr int detectionThreshold = 3;
    constexpr uint32_t travelDelay = 500;
    constexpr uint32_t rejectDuration = 200;
    constexpr std::pair<int, int> redRange = {1, 30};
    constexpr std::pair<int, int> blueRange = {200, 255};
}

namespace intake
{
    enum class intakeState
    {
        INTAKE,
        INTAKEFEEDER,
        CYCLE,
        SCORING_BOTTOM,
        SCORING_MID,
        SCORING_TOP,
        REJECT,
        REJECTWITHFEED,
        OFF
    };
    struct IntakeConfig
    {
        intakeState state;
        bool p1;
        int v1, v2, v3, v4, v5, v6;
    };

    constexpr IntakeConfig INTAKE_MODE = {intakeState::INTAKE, 0, -600, -600, -600, -600, 600, 0};
    constexpr IntakeConfig INTAKE_FEED_MODE = {intakeState::INTAKEFEEDER, 1, -600, -600, -600, -600, 600, 600};
    constexpr IntakeConfig CYCLE_MODE = {intakeState::CYCLE, 0, 600, -600, -600, -600, 600, 0};
    constexpr IntakeConfig SCORING_BOTTOM = {intakeState::SCORING_BOTTOM, 0, 300, 300, -300, 0, 0, 0};
    constexpr IntakeConfig SCORING_MID = {intakeState::SCORING_MID, 0, 600, -600, -600, 600, 0, 0};
    constexpr IntakeConfig SCORING_TOP = {intakeState::SCORING_TOP, 0, 300, -300, -300, -300, -300, 0};
    constexpr IntakeConfig REJECT_MODE = {intakeState::REJECT, 0, 600, -600, -600, -600, 600, 0};
    constexpr IntakeConfig REJECT_FEED_MODE = {intakeState::REJECTWITHFEED, 1, 600, -600, -600, -600, 600, 600};
    constexpr IntakeConfig OFF_MODE = {intakeState::OFF, 0, 0, 0, 0, 0, 0, 0};

}