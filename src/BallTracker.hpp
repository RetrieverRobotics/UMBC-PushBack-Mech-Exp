#pragma once

#include "api.h"
#include "pros/optical.hpp"
#include "intake.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"

#include <queue>

using namespace pros;
enum class teamColor {
    RED,
    BLUE,
    UNDEFINED
};

struct ball {
    teamColor color;
    uint32_t timeDetected;
};

struct BallTracker {
    std::queue<ball> ballQueue;
    teamColor teamcolor;
    bool sortDisabled;
    int consecutiveDetections;
    bool wasOpponentColor;

    BallTracker() :
        teamcolor(teamColor::RED),
        sortDisabled(false),
        consecutiveDetections(0),
        wasOpponentColor(false)
    {}

    void update(pros::Optical &optical, intake::intakeState &iState, Hardware &hw) {
        if (sortDisabled) return;

        int hue = optical.get_hue();
        int distance = optical.get_proximity();
        bool isOpponentColor = isOpponent(hue);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Distance: %d", distance);
        // Confidence + edge detection: only push when we hit the threshold for the first time
        if (isOpponentColor && distance == 255) {
            consecutiveDetections++;
        } else {
            consecutiveDetections = 0;
            wasOpponentColor = false;
        }

        if (consecutiveDetections >= constants::detectionThreshold && !wasOpponentColor) {
            ballQueue.push({teamcolor == teamColor::RED ? teamColor::BLUE : teamColor::RED, pros::millis()});
            wasOpponentColor = true;
        }

        // Process queue front only
        if (!ballQueue.empty()) {
            ball &front = ballQueue.front();
            uint32_t now = pros::millis();
            uint32_t rejectStart = front.timeDetected + constants::travelDelay;
            uint32_t rejectEnd   = rejectStart + constants::rejectDuration;

            if (now >= rejectStart && now < rejectEnd) {
                // Rejection window — apply correct reject config based on current state
                if (iState == intake::intakeState::INTAKEFEEDER) {
                    apply_intake_config(iState, hw, intake::REJECT_FEED_MODE);
                } else {
                    apply_intake_config(iState, hw, intake::REJECT_MODE);
                }
            } else if (now >= rejectEnd) {
                // Rejection window over — pop and restore
                ballQueue.pop();
                if (ballQueue.empty()) {
                    if (iState == intake::intakeState::REJECT) {
                        apply_intake_config(iState, hw, intake::INTAKE_MODE);
                    } else if (iState == intake::intakeState::REJECTWITHFEED) {
                        apply_intake_config(iState, hw, intake::INTAKE_FEED_MODE);
                    }
                }
            }
        }
    }

    void toggleSort() {
        sortDisabled = !sortDisabled;
        if (sortDisabled) {
            // Clear any active rejection state
            while (!ballQueue.empty()) ballQueue.pop();
            consecutiveDetections = 0;
            wasOpponentColor = false;
        }
    }

    void toggleColor() {
        teamcolor = (teamcolor == teamColor::RED) ? teamColor::BLUE : teamColor::RED;
    }

private:
    bool isOpponent(int hue) {
        if (teamcolor == teamColor::RED) {
            return hue >= std::get<0>(constants::blueRange) && hue < std::get<1>(constants::blueRange);
        } else {
            return hue >= std::get<0>(constants::redRange) && hue < std::get<1>(constants::redRange);
        }
    }
};