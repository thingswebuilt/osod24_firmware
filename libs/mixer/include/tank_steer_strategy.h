//
// Created by markmellors on 03/12/2023.
//

#ifndef OSOD_MOTOR_2040_TANK_STEER_STRATEGY_H
#define OSOD_MOTOR_2040_TANK_STEER_STRATEGY_H
#include "mixer_strategy.h"
#include "types.h"

namespace MIXER {

    class TankSteerStrategy : public MixerStrategy {
        // This class implements a steering strategy for a tank-like robot.
        // It calculates the motor speeds based on the given linear and angular velocities.
    public:
        DriveTrainState mix(float velocity, float angularVelocity) override;
    };
}


#endif //OSOD_MOTOR_2040_TANK_STEER_STRATEGY_H
