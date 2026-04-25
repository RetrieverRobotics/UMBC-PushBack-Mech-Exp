#pragma once

#include "hardware.hpp"

static void apply_intake_config(intake::intakeState &iState, Hardware &hw, const intake::IntakeConfig &cfg)
{
    iState = cfg.state;
    hw.pneumatic.set_value(cfg.p1);
    hw.feeder.move_velocity(cfg.v6);
    hw.topRoller.move_velocity(cfg.v1);
    hw.backRoller.move_velocity(cfg.v2);
    hw.bottomOutsideRoller.move_velocity(cfg.v3);
    hw.middleRoller.move_velocity(cfg.v4);
    hw.bottomRoller.move_velocity(cfg.v5);
}