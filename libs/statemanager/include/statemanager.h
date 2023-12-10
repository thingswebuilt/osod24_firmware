//
// Created by robbe on 02/12/2023.
//

#ifndef OSOD_MOTOR_2040_STATEMANAGER_H
#define OSOD_MOTOR_2040_STATEMANAGER_H

#include "receiver.h"
#include "state_estimator.h"
#include "stoker.h"
#include "mixer_strategy.h"

namespace STATEMANAGER {

    // define a RequestedState struct containing the requested state parameters: velocity and angular velocity
    struct RequestedState {
        float velocity;
        float angularVelocity;
    };

    struct Stokers {
        STOKER::Stoker* FRONT_LEFT;
        STOKER::Stoker* FRONT_RIGHT;
        STOKER::Stoker* REAR_LEFT;
        STOKER::Stoker* REAR_RIGHT;
    };

    class StateManager {
    public:
        explicit StateManager(MIXER::MixerStrategy *mixerStrategy, STATE_ESTIMATOR::StateEstimator *stateEstimator);

        void requestState(RequestedState requestedState);
    private:
        MIXER::MixerStrategy *mixerStrategy;
        STATE_ESTIMATOR::StateEstimator *stateEstimator;
        Stokers stokers{};
        // max speed factor - scale the speed of the motors down to this value
        static constexpr float SPEED_EXTENT = 1.0f;

        void setSpeeds(MIXER::MotorSpeeds motorSpeeds) const;
    };

} // StateManager

#endif //OSOD_MOTOR_2040_STATEMANAGER_H
