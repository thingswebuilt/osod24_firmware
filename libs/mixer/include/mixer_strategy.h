//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_MIXER_STRATEGY_H
#define OSOD_MOTOR_2040_MIXER_STRATEGY_H

#include "types.h"

namespace MIXER {
    using namespace COMMON;
    class MixerStrategy {
    public:
        virtual DriveTrainState mix(float velocity, float angularVelocity) = 0;
    };
}

#endif //OSOD_MOTOR_2040_MIXER_STRATEGY_H
