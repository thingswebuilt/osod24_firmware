//
// Created by markm on 11/11/2023.
//

#ifndef OSOD_MOTOR_2040_ACKERMANN_STRATEGY_H
#define OSOD_MOTOR_2040_ACKERMANN_STRATEGY_H

#include "drivetrain_config.h"
#include "mixer_strategy.h"
#include "types.h"

namespace MIXER {
    using namespace COMMON;
    struct SteeringAngle {
        float raw;
        float constrained;
        float slip;
    };

    class AckermannMixer : public MixerStrategy {
    private:
        float wheelTrack;         // Distance between the left and right wheels (m)
        float wheelBase;          // Distance between the front and rear wheels (m)
        float maxSteeringAngle;   // Max angle from straight ahead that the steerable wheels can pivot (radians)
        float turnRadius;         // calculated turn radius from inputs, m to centreline

    public:
        explicit AckermannMixer(float track = CONFIG::WHEEL_TRACK,
                    float base = CONFIG::WHEEL_BASE,
                    float angle = CONFIG::MAX_STEERING_ANGLE); // constructor

        DriveTrainState mix(float velocity, float angularVelocity) override; //mixing function

        [[nodiscard]] float
        getFrontWheelSpeed(float angularVelocity, float wheelTurnRadius, float slipAngle,
                           CONFIG::Handedness side) const;

        [[nodiscard]] float getRearWheelSpeed(float velocity, float wheelTurnRadius, CONFIG::Handedness side) const;

        [[nodiscard]] SteeringAngle
        getWheelAngle(float wheelTurnRadius, float velocity, CONFIG::Handedness side) const;

        [[nodiscard]] float constrainSteeringAngle(float angle) const;

    };
} // namespace MIXER
#endif //OSOD_MOTOR_2040_ACKERMANN_STRATEGY_H
